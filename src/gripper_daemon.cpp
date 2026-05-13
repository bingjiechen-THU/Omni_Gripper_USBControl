#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "protocol/pub_user.h"

namespace {

constexpr uint8_t kChannel = 0;
constexpr float kQMax = 12.5f;
constexpr float kDqMax = 30.0f;
constexpr float kTauMax = 10.0f;
constexpr double kPi = 3.14159265358979323846;

struct Config {
    std::string host = "127.0.0.1";
    int port = 5055;
    std::string sn;
    uint16_t can_id = 0x01;
    uint32_t nom_baud = 1000000;
    uint32_t data_baud = 5000000;
    float close_position_rad = 0.05f;
    float open_position_rad = 0.80f;
    float max_width_m = 0.08f;
    float kp = 5.0f;
    float kd = 0.3f;
    int control_period_ms = 10;
};

struct State {
    float q = 0.0f;
    float dq = 0.0f;
    float tau = 0.0f;
    bool received = false;
    std::chrono::steady_clock::time_point updated_at{};
};

std::atomic<bool> g_running{true};

float clampf(float value, float low, float high) {
    return std::min(std::max(value, low), high);
}

float uint_to_float(uint16_t value, float min_value, float max_value, uint8_t bits) {
    const float span = max_value - min_value;
    return static_cast<float>(value) / static_cast<float>((1 << bits) - 1) * span + min_value;
}

uint16_t float_to_uint(float value, float min_value, float max_value, uint8_t bits) {
    value = clampf(value, min_value, max_value);
    const float span = max_value - min_value;
    return static_cast<uint16_t>((value - min_value) / span * static_cast<float>((1 << bits) - 1));
}

std::vector<uint8_t> mit_payload(float kp, float kd, float q, float dq, float tau) {
    const uint16_t q_uint = float_to_uint(q, -kQMax, kQMax, 16);
    const uint16_t dq_uint = float_to_uint(dq, -kDqMax, kDqMax, 12);
    const uint16_t tau_uint = float_to_uint(tau, -kTauMax, kTauMax, 12);
    const uint16_t kp_uint = float_to_uint(kp, 0.0f, 500.0f, 12);
    const uint16_t kd_uint = float_to_uint(kd, 0.0f, 5.0f, 12);

    return {
        static_cast<uint8_t>((q_uint >> 8) & 0xFF),
        static_cast<uint8_t>(q_uint & 0xFF),
        static_cast<uint8_t>((dq_uint >> 4) & 0xFF),
        static_cast<uint8_t>(((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)),
        static_cast<uint8_t>(kp_uint & 0xFF),
        static_cast<uint8_t>((kd_uint >> 4) & 0xFF),
        static_cast<uint8_t>(((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)),
        static_cast<uint8_t>(tau_uint & 0xFF),
    };
}

bool find_number(const std::string& body, const std::string& key, double* out) {
    const std::regex pattern("\"" + key + R"("\s*:\s*(-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?))");
    std::smatch match;
    if (!std::regex_search(body, match, pattern)) {
        return false;
    }
    *out = std::stod(match[1].str());
    return true;
}

std::string http_path(const std::string& request) {
    std::istringstream stream(request);
    std::string method;
    std::string path;
    stream >> method >> path;
    const auto query_pos = path.find('?');
    return query_pos == std::string::npos ? path : path.substr(0, query_pos);
}

std::string http_body(const std::string& request) {
    const std::string marker = "\r\n\r\n";
    const auto pos = request.find(marker);
    if (pos == std::string::npos) {
        return "";
    }
    return request.substr(pos + marker.size());
}

std::string json_escape(const std::string& value) {
    std::ostringstream out;
    for (char c : value) {
        switch (c) {
            case '"':
                out << "\\\"";
                break;
            case '\\':
                out << "\\\\";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                out << c;
                break;
        }
    }
    return out.str();
}

class OmniGripperDaemon {
public:
    explicit OmniGripperDaemon(Config config) : config_(std::move(config)) {
        connect_device();
        send_refresh();
        control_thread_ = std::thread([this] { control_loop(); });
    }

    ~OmniGripperDaemon() {
        stop();
    }

    device_handle* device() const {
        return dev_;
    }

    void stop() {
        bool expected = false;
        if (!stopped_.compare_exchange_strong(expected, true)) {
            return;
        }
        keep_control_.store(false);
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
        if (dev_ != nullptr) {
            send_disable();
            device_close_channel(dev_, kChannel);
            device_close(dev_);
            dev_ = nullptr;
        }
        if (handle_ != nullptr) {
            damiao_handle_destroy(handle_);
            handle_ = nullptr;
        }
    }

    void on_frame(usb_rx_frame_t* frame) {
        if (frame == nullptr) {
            return;
        }
        const uint16_t frame_id = static_cast<uint16_t>(frame->head.can_id);
        const uint16_t master_id = static_cast<uint16_t>(config_.can_id + 0x10);
        if (frame_id != config_.can_id && frame_id != master_id && frame->payload[0] != config_.can_id) {
            return;
        }
        if (frame->payload[2] == 0x33 || frame->payload[2] == 0x55 || frame->payload[2] == 0xAA) {
            return;
        }

        const uint16_t q_uint = (static_cast<uint16_t>(frame->payload[1]) << 8) | frame->payload[2];
        const uint16_t dq_uint = (static_cast<uint16_t>(frame->payload[3]) << 4) | (frame->payload[4] >> 4);
        const uint16_t tau_uint = (static_cast<uint16_t>(frame->payload[4] & 0x0F) << 8) | frame->payload[5];

        std::lock_guard<std::mutex> lock(mutex_);
        state_.q = uint_to_float(q_uint, -kQMax, kQMax, 16);
        state_.dq = uint_to_float(dq_uint, -kDqMax, kDqMax, 12);
        state_.tau = uint_to_float(tau_uint, -kTauMax, kTauMax, 12);
        state_.received = true;
        state_.updated_at = std::chrono::steady_clock::now();
    }

    std::string state_json(bool ok = true, const std::string& message = "") {
        send_refresh();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        State state;
        float target = 0.0f;
        bool enabled = enabled_.load();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state = state_;
            target = target_rad_;
        }

        const float width_norm = normalized_from_position(state.q);
        const float target_width_norm = normalized_from_position(target);
        const double age_s = state.received
            ? std::chrono::duration<double>(std::chrono::steady_clock::now() - state.updated_at).count()
            : -1.0;

        std::ostringstream out;
        out << std::fixed << std::setprecision(6)
            << "{"
            << "\"ok\":" << (ok ? "true" : "false")
            << ",\"message\":\"" << json_escape(message) << "\""
            << ",\"can_id\":" << config_.can_id
            << ",\"master_id\":" << (config_.can_id + 0x10)
            << ",\"enabled\":" << (enabled ? "true" : "false")
            << ",\"state_ready\":" << (state.received ? "true" : "false")
            << ",\"position_rad\":" << state.q
            << ",\"velocity_rad_s\":" << state.dq
            << ",\"torque\":" << state.tau
            << ",\"target_position_rad\":" << target
            << ",\"width\":" << width_norm
            << ",\"width_m\":" << width_norm * config_.max_width_m
            << ",\"gripper_pos\":" << static_cast<int>(std::lround(width_norm * 255.0f))
            << ",\"target_width\":" << target_width_norm
            << ",\"target_width_m\":" << target_width_norm * config_.max_width_m
            << ",\"target_gripper_pos\":" << static_cast<int>(std::lround(target_width_norm * 255.0f))
            << ",\"open_position_rad\":" << config_.open_position_rad
            << ",\"close_position_rad\":" << config_.close_position_rad
            << ",\"max_width_m\":" << config_.max_width_m
            << ",\"state_age_s\":" << age_s
            << "}";
        return out.str();
    }

    std::string open() {
        move_to(config_.open_position_rad);
        return state_json(true, "opened");
    }

    std::string close() {
        move_to(config_.close_position_rad);
        return state_json(true, "closed");
    }

    std::string activate() {
        ensure_enabled();
        return state_json(true, "activated");
    }

    std::string disable() {
        enabled_.store(false);
        keep_control_.store(false);
        send_disable();
        return state_json(true, "disabled");
    }

    std::string move_from_body(const std::string& body) {
        double value = 0.0;
        if (find_number(body, "position_rad", &value) || find_number(body, "target_rad", &value)) {
            move_to(static_cast<float>(value));
            return state_json(true, "moved_position_rad");
        }
        if (find_number(body, "width_m", &value)) {
            const double normalized = config_.max_width_m > 0.0f ? value / config_.max_width_m : 0.0;
            move_to(position_from_normalized(static_cast<float>(normalized)));
            return state_json(true, "moved_width_m");
        }
        if (find_number(body, "width_mm", &value)) {
            const double normalized = config_.max_width_m > 0.0f ? (value / 1000.0) / config_.max_width_m : 0.0;
            move_to(position_from_normalized(static_cast<float>(normalized)));
            return state_json(true, "moved_width_mm");
        }
        if (find_number(body, "width", &value)) {
            move_to(position_from_normalized(static_cast<float>(value)));
            return state_json(true, "moved_width");
        }
        if (find_number(body, "gripper_pos", &value) || find_number(body, "command", &value)) {
            move_to(position_from_normalized(static_cast<float>(value / 255.0)));
            return state_json(true, "moved_command");
        }
        return error_json("Expected one of position_rad, target_rad, width, width_m, width_mm, gripper_pos, command.");
    }

    std::string error_json(const std::string& message) {
        return state_json(false, message);
    }

private:
    void connect_device() {
        handle_ = damiao_handle_create(DEV_USB2CANFD);
        damiao_print_version(handle_);

        int device_count = damiao_handle_find_devices(handle_);
        if (device_count <= 0) {
            throw std::runtime_error("No DM USB-CANFD devices found.");
        }

        device_handle* devices[16] = {nullptr};
        int handle_count = 0;
        damiao_handle_get_devices(handle_, devices, &handle_count);
        for (int i = 0; i < handle_count; ++i) {
            int pid = 0;
            int vid = 0;
            device_get_pid_vid(devices[i], &pid, &vid);
            char serial[255] = {0};
            device_get_serial_number(devices[i], serial, sizeof(serial));
            if (config_.sn.empty() || config_.sn == serial) {
                dev_ = devices[i];
                config_.sn = serial;
                break;
            }
        }

        if (dev_ == nullptr) {
            throw std::runtime_error("Requested USB-CANFD device was not found.");
        }
        if (!device_open(dev_)) {
            throw std::runtime_error("Failed to open USB-CANFD device.");
        }
        if (!device_channel_set_baud_with_sp(
                dev_, kChannel, true, config_.nom_baud, config_.data_baud, 0.75f, 0.75f)) {
            throw std::runtime_error("Failed to set CANFD baudrate.");
        }
        if (!device_open_channel(dev_, kChannel)) {
            throw std::runtime_error("Failed to open CANFD channel.");
        }
    }

    void send_frame(uint32_t can_id, const std::vector<uint8_t>& bytes) {
        if (dev_ == nullptr) {
            return;
        }
        uint8_t payload[8] = {0};
        const size_t n = bytes.size() < 8 ? bytes.size() : 8;
        if (n > 0) {
            std::memcpy(payload, bytes.data(), n);
        }
        device_channel_send_fast(dev_, kChannel, can_id, 1, false, true, true, 8, payload);
    }

    void send_refresh() {
        send_frame(0x7FF, {
            static_cast<uint8_t>(config_.can_id & 0xFF),
            static_cast<uint8_t>((config_.can_id >> 8) & 0xFF),
            0xCC, 0x00, 0x00, 0x00, 0x00, 0x00,
        });
    }

    void write_mit_mode() {
        send_frame(0x7FF, {
            static_cast<uint8_t>(config_.can_id & 0xFF),
            static_cast<uint8_t>((config_.can_id >> 8) & 0xFF),
            0x55, 0x0A, 0x01, 0x00, 0x00, 0x00,
        });
    }

    void send_enable() {
        send_frame(config_.can_id, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC});
    }

    void send_disable() {
        send_frame(config_.can_id, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD});
    }

    void ensure_enabled() {
        if (enabled_.load()) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!keep_control_.load() && state_.received) {
                target_rad_ = clamp_position(state_.q);
            }
        }
        write_mit_mode();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        send_enable();
        enabled_.store(true);
        keep_control_.store(true);
    }

    float position_from_normalized(float normalized) const {
        normalized = clampf(normalized, 0.0f, 1.0f);
        return config_.close_position_rad + normalized * (config_.open_position_rad - config_.close_position_rad);
    }

    float normalized_from_position(float position) const {
        const float span = config_.open_position_rad - config_.close_position_rad;
        if (std::fabs(span) < 1e-6f) {
            return 0.0f;
        }
        return clampf((position - config_.close_position_rad) / span, 0.0f, 1.0f);
    }

    float clamp_position(float position) const {
        const float low = std::min(config_.close_position_rad, config_.open_position_rad);
        const float high = std::max(config_.close_position_rad, config_.open_position_rad);
        return clampf(position, low, high);
    }

    void move_to(float position_rad) {
        keep_control_.store(true);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            target_rad_ = clamp_position(position_rad);
        }
        ensure_enabled();
    }

    void control_loop() {
        while (!stopped_.load()) {
            if (enabled_.load() && keep_control_.load()) {
                float target = 0.0f;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    target = target_rad_;
                }
                send_frame(config_.can_id, mit_payload(config_.kp, config_.kd, target, 0.0f, 0.0f));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(config_.control_period_ms));
        }
    }

    Config config_;
    damiao_handle* handle_ = nullptr;
    device_handle* dev_ = nullptr;
    std::mutex mutex_;
    State state_;
    float target_rad_ = 0.0f;
    std::atomic<bool> enabled_{false};
    std::atomic<bool> keep_control_{false};
    std::atomic<bool> stopped_{false};
    std::thread control_thread_;
};

OmniGripperDaemon* g_daemon = nullptr;

void frame_callback(usb_rx_frame_t* frame) {
    if (g_daemon != nullptr) {
        g_daemon->on_frame(frame);
    }
}

void signal_handler(int) {
    g_running.store(false);
}

void send_http_response(int client_fd, int status, const std::string& body) {
    const char* reason = status == 200 ? "OK" : "Bad Request";
    std::ostringstream response;
    response << "HTTP/1.1 " << status << " " << reason << "\r\n"
             << "Content-Type: application/json\r\n"
             << "Content-Length: " << body.size() << "\r\n"
             << "Connection: close\r\n\r\n"
             << body;
    const std::string text = response.str();
    send(client_fd, text.data(), text.size(), 0);
}

std::string read_http_request(int client_fd) {
    std::string request;
    char buffer[4096];
    while (true) {
        const ssize_t n = recv(client_fd, buffer, sizeof(buffer), 0);
        if (n <= 0) {
            break;
        }
        request.append(buffer, buffer + n);
        const auto header_end = request.find("\r\n\r\n");
        if (header_end == std::string::npos) {
            continue;
        }
        std::smatch match;
        size_t content_length = 0;
        const std::string headers = request.substr(0, header_end);
        if (std::regex_search(headers, match, std::regex("Content-Length:\\s*(\\d+)", std::regex::icase))) {
            content_length = static_cast<size_t>(std::stoul(match[1].str()));
        }
        if (request.size() >= header_end + 4 + content_length) {
            break;
        }
    }
    return request;
}

void serve_http(OmniGripperDaemon& daemon, const Config& config) {
    const int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        throw std::runtime_error("Failed to create server socket.");
    }
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(config.port));
    if (inet_pton(AF_INET, config.host.c_str(), &address.sin_addr) != 1) {
        close(server_fd);
        throw std::runtime_error("Invalid bind host: " + config.host);
    }
    if (bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        close(server_fd);
        throw std::runtime_error("Failed to bind HTTP server.");
    }
    if (listen(server_fd, 16) < 0) {
        close(server_fd);
        throw std::runtime_error("Failed to listen on HTTP server socket.");
    }

    std::cerr << "OmniGripper daemon listening on http://" << config.host << ":" << config.port << std::endl;
    while (g_running.load()) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(server_fd, &fds);
        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 200000;
        const int ready = select(server_fd + 1, &fds, nullptr, nullptr, &timeout);
        if (ready <= 0) {
            continue;
        }

        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        const int client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
        if (client_fd < 0) {
            continue;
        }

        const std::string request = read_http_request(client_fd);
        const std::string path = http_path(request);
        const std::string body = http_body(request);

        std::string response_body;
        int status = 200;
        try {
            if (path == "/health") {
                response_body = "{\"ok\":true}";
            } else if (path == "/state" || path == "/getstate") {
                response_body = daemon.state_json();
            } else if (path == "/open") {
                response_body = daemon.open();
            } else if (path == "/close") {
                response_body = daemon.close();
            } else if (path == "/activate") {
                response_body = daemon.activate();
            } else if (path == "/disable") {
                response_body = daemon.disable();
            } else if (path == "/move" || path == "/move_width") {
                response_body = daemon.move_from_body(body);
                if (response_body.find("\"ok\":false") != std::string::npos) {
                    status = 400;
                }
            } else {
                status = 400;
                response_body = daemon.error_json("Unknown endpoint: " + path);
            }
        } catch (const std::exception& exc) {
            status = 400;
            response_body = daemon.error_json(exc.what());
        }
        send_http_response(client_fd, status, response_body);
        close(client_fd);
    }
    close(server_fd);
}

Config parse_args(int argc, char** argv) {
    Config config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + arg);
            }
            return argv[++i];
        };
        if (arg == "--host") {
            config.host = next();
        } else if (arg == "--port") {
            config.port = std::stoi(next());
        } else if (arg == "--sn") {
            config.sn = next();
        } else if (arg == "--can-id") {
            config.can_id = static_cast<uint16_t>(std::stoul(next(), nullptr, 0));
        } else if (arg == "--nom-baud") {
            config.nom_baud = static_cast<uint32_t>(std::stoul(next(), nullptr, 0));
        } else if (arg == "--data-baud") {
            config.data_baud = static_cast<uint32_t>(std::stoul(next(), nullptr, 0));
        } else if (arg == "--close-position-rad") {
            config.close_position_rad = std::stof(next());
        } else if (arg == "--open-position-rad") {
            config.open_position_rad = std::stof(next());
        } else if (arg == "--max-width-m") {
            config.max_width_m = std::stof(next());
        } else if (arg == "--kp") {
            config.kp = std::stof(next());
        } else if (arg == "--kd") {
            config.kd = std::stof(next());
        } else if (arg == "--control-period-ms") {
            config.control_period_ms = std::stoi(next());
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    return config;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        Config config = parse_args(argc, argv);
        OmniGripperDaemon daemon(config);
        g_daemon = &daemon;
        device_hook_to_rec(daemon.device(), frame_callback);
        serve_http(daemon, config);
        g_daemon = nullptr;
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "Fatal error: " << exc.what() << std::endl;
        return 1;
    }
}

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "protocol/pub_user.h"

namespace {

constexpr uint8_t kChannel = 0;
constexpr float kQMax = 12.5f;
constexpr float kDqMax = 30.0f;
constexpr float kTauMax = 10.0f;

struct MotorState {
    uint16_t can_id = 0;
    uint16_t master_id = 0;
    float q = 0.0f;
    float dq = 0.0f;
    float tau = 0.0f;
    bool received = false;
};

std::map<uint16_t, MotorState> g_states;

float uint_to_float(uint16_t value, float min_value, float max_value, uint8_t bits) {
    const float span = max_value - min_value;
    return static_cast<float>(value) / static_cast<float>((1 << bits) - 1) * span + min_value;
}

uint16_t float_to_uint(float value, float min_value, float max_value, uint8_t bits) {
    if (value < min_value) {
        value = min_value;
    } else if (value > max_value) {
        value = max_value;
    }
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

void register_motor(uint16_t can_id) {
    MotorState state;
    state.can_id = can_id;
    state.master_id = can_id + 0x10;
    g_states[can_id] = state;
    g_states[state.master_id] = state;
}

MotorState* find_state(usb_rx_frame_t* frame) {
    auto it = g_states.find(static_cast<uint16_t>(frame->head.can_id));
    if (it != g_states.end()) {
        return &it->second;
    }

    const uint16_t payload_id = frame->payload[0];
    it = g_states.find(payload_id);
    if (it != g_states.end()) {
        return &it->second;
    }

    const uint16_t master_low_nibble = frame->payload[0] & 0x0F;
    it = g_states.find(master_low_nibble + 0x10);
    if (it != g_states.end()) {
        return &it->second;
    }

    return nullptr;
}

void on_frame(usb_rx_frame_t* frame) {
    std::cerr << "[RX] id=0x" << std::hex << std::uppercase << frame->head.can_id
              << " ch=" << std::dec << static_cast<int>(frame->head.channel)
              << " dlc=" << static_cast<int>(frame->head.dlc)
              << " data=";
    std::cerr << std::hex << std::uppercase << std::setfill('0');
    for (int i = 0; i < 8; ++i) {
        std::cerr << std::setw(2) << static_cast<int>(frame->payload[i]);
        if (i != 7) {
            std::cerr << " ";
        }
    }
    std::cerr << std::dec << std::setfill(' ') << std::endl;

    MotorState* state = find_state(frame);
    if (!state) {
        return;
    }
    if (frame->payload[2] == 0x33 || frame->payload[2] == 0x55 || frame->payload[2] == 0xAA) {
        return;
    }

    const uint16_t q_uint = (static_cast<uint16_t>(frame->payload[1]) << 8) | frame->payload[2];
    const uint16_t dq_uint = (static_cast<uint16_t>(frame->payload[3]) << 4) | (frame->payload[4] >> 4);
    const uint16_t tau_uint = (static_cast<uint16_t>(frame->payload[4] & 0x0F) << 8) | frame->payload[5];

    state->q = uint_to_float(q_uint, -kQMax, kQMax, 16);
    state->dq = uint_to_float(dq_uint, -kDqMax, kDqMax, 12);
    state->tau = uint_to_float(tau_uint, -kTauMax, kTauMax, 12);
    state->received = true;

    g_states[state->can_id] = *state;
    g_states[state->master_id] = *state;
}

void send_frame(device_handle* dev, uint32_t can_id, const std::vector<uint8_t>& bytes) {
    uint8_t payload[8] = {0};
    const size_t n = bytes.size() < 8 ? bytes.size() : 8;
    std::memcpy(payload, bytes.data(), n);
    device_channel_send_fast(dev, kChannel, can_id, 1, false, true, true, 8, payload);
}

void refresh(device_handle* dev, uint16_t can_id) {
    send_frame(dev, 0x7FF, {static_cast<uint8_t>(can_id & 0xFF), static_cast<uint8_t>((can_id >> 8) & 0xFF),
                            0xCC, 0x00, 0x00, 0x00, 0x00, 0x00});
}

void write_mit_mode(device_handle* dev, uint16_t can_id) {
    send_frame(dev, 0x7FF, {static_cast<uint8_t>(can_id & 0xFF), static_cast<uint8_t>((can_id >> 8) & 0xFF),
                            0x55, 0x0A, 0x01, 0x00, 0x00, 0x00});
}

void motor_command(device_handle* dev, uint16_t can_id, uint8_t command) {
    send_frame(dev, can_id, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command});
}

std::vector<uint16_t> parse_ids(const std::string& text) {
    std::vector<uint16_t> ids;
    std::stringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ',')) {
        if (!item.empty()) {
            ids.push_back(static_cast<uint16_t>(std::stoul(item, nullptr, 0)));
        }
    }
    return ids;
}

}  // namespace

int main(int argc, char** argv) {
    std::string sn = "02A361119DB9A49659AF543D75712B65";
    std::string mode = "status";
    std::vector<uint16_t> ids = {0x01};
    double duration = 1.0;
    double amplitude = 0.03;
    double frequency = 0.25;
    float kp = 1.5f;
    float kd = 0.2f;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--sn" && i + 1 < argc) {
            sn = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            mode = argv[++i];
        } else if (arg == "--can-id" && i + 1 < argc) {
            ids = parse_ids(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration = std::stod(argv[++i]);
        } else if (arg == "--amplitude" && i + 1 < argc) {
            amplitude = std::stod(argv[++i]);
        } else if (arg == "--frequency" && i + 1 < argc) {
            frequency = std::stod(argv[++i]);
        } else if (arg == "--kp" && i + 1 < argc) {
            kp = std::stof(argv[++i]);
        } else if (arg == "--kd" && i + 1 < argc) {
            kd = std::stof(argv[++i]);
        }
    }

    for (uint16_t id : ids) {
        register_motor(id);
    }

    damiao_handle* handle = damiao_handle_create(DEV_USB2CANFD);
    damiao_print_version(handle);

    const int found = damiao_handle_find_devices(handle);
    std::cerr << "device_cnt " << found << std::endl;

    device_handle* dev_list[16] = {nullptr};
    int handle_cnt = 0;
    damiao_handle_get_devices(handle, dev_list, &handle_cnt);

    device_handle* dev = nullptr;
    for (int i = 0; i < handle_cnt; ++i) {
        int pid = 0;
        int vid = 0;
        device_get_pid_vid(dev_list[i], &pid, &vid);
        char serial[255] = {0};
        device_get_serial_number(dev_list[i], serial, sizeof(serial));
        std::cerr << "Found device VID=0x" << std::hex << vid << " PID=0x" << pid
                  << std::dec << " SN=" << serial << std::endl;
        if (sn == serial) {
            dev = dev_list[i];
        }
    }

    if (!dev) {
        std::cerr << "Requested USB-CANFD SN not found: " << sn << std::endl;
        damiao_handle_destroy(handle);
        return 1;
    }
    if (!device_open(dev)) {
        std::cerr << "device_open failed" << std::endl;
        damiao_handle_destroy(handle);
        return 1;
    }

    bool ok = device_channel_set_baud_with_sp(dev, kChannel, true, 1000000, 5000000, 0.75f, 0.75f);
    std::cerr << "set_baud_1M_5M=" << (ok ? "ok" : "fail") << std::endl;
    device_baud_t baud = {0};
    if (device_channel_get_baudrate(dev, kChannel, &baud)) {
        std::cerr << "baud can=" << baud.can_baudrate << " canfd=" << baud.canfd_baudrate
                  << " sp=" << baud.can_sp << " fd_sp=" << baud.canfd_sp << std::endl;
    }

    if (!device_open_channel(dev, kChannel)) {
        std::cerr << "device_open_channel failed" << std::endl;
        device_close(dev);
        damiao_handle_destroy(handle);
        return 1;
    }
    device_hook_to_rec(dev, on_frame);

    for (uint16_t id : ids) {
        refresh(dev, id);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    for (uint16_t id : ids) {
        const MotorState& state = g_states[id];
        std::cerr << "state id=0x" << std::hex << id << std::dec
                  << " received=" << state.received
                  << " q=" << state.q
                  << " dq=" << state.dq
                  << " tau=" << state.tau << std::endl;
    }

    if (mode == "hold" || mode == "nudge") {
        bool all_received = true;
        for (uint16_t id : ids) {
            all_received = all_received && g_states[id].received;
        }
        if (!all_received) {
            std::cerr << "No complete feedback for all motors; refusing to enable." << std::endl;
        } else {
            for (uint16_t id : ids) {
                write_mit_mode(dev, id);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                motor_command(dev, id, 0xFC);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

            std::map<uint16_t, float> centers;
            for (uint16_t id : ids) {
                centers[id] = g_states[id].q;
            }

            const auto start = std::chrono::steady_clock::now();
            double last_print = -1.0;
            while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < duration) {
                const double elapsed =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                for (uint16_t id : ids) {
                    float target = centers[id];
                    if (mode == "nudge") {
                        target += static_cast<float>(
                            amplitude * std::sin(static_cast<float>(2.0 * M_PI * frequency * elapsed)));
                    }
                    send_frame(dev, id, mit_payload(kp, kd, target, 0.0f, 0.0f));
                    if (elapsed - last_print >= 0.2) {
                        std::cerr << "cmd id=0x" << std::hex << id << std::dec
                                  << " target=" << target
                                  << " feedback=" << g_states[id].q
                                  << " vel=" << g_states[id].dq
                                  << " tau=" << g_states[id].tau << std::endl;
                    }
                }
                if (elapsed - last_print >= 0.2) {
                    last_print = elapsed;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            for (uint16_t id : ids) {
                motor_command(dev, id, 0xFD);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
    } else if (mode != "status") {
        std::cerr << "Unknown mode: " << mode << std::endl;
    }

    device_close_channel(dev, kChannel);
    device_close(dev);
    damiao_handle_destroy(handle);
    return 0;
}


# 使用USB转CANFD设备控制电机

此程序用于通过**USB转CANFD设备**控制OmniGripper夹爪（DM4310电机）。程序将电机控制模式设置为MIT模式，启用电机输出正弦波轨迹控制实现夹爪开合往复运动，电机波特率设置为5M。如果连接了多个电机，在5M波特率下，末端电机需要接入一个120欧的电阻。

## 软件架构
- 使用C++语言编写，不使用ROS。
- 测试环境： ubuntu 22.04

## 安装和编译

### 1. 确保系统安装libusb库

版本需为**1.0.29**或更高版本。如果没有安装，可以使用以下命令安装：
```bash
wget https://github.com/libusb/libusb/releases/download/v1.0.29/libusb-1.0.29.tar.bz2
tar -xf libusb-1.0.29.tar.bz2
cd libusb-1.0.29
./configure --prefix=/usr
make
sudo make install
```

### 2. 安装GCC 13

此项目需要使用 **GCC 13** 进行编译。确保安装了GCC 13，并可以通过以下方式设置编译器：

#### 方法 1：通过 `CMakeLists.txt` 文件设置

打开 `CMakeLists.txt` 文件（位于项目根目录下），在文件的顶部添加以下行来指定编译器为 **GCC 13**：

```cmake 
# 设置C++编译器为GCC 13
set(CMAKE_CXX_COMPILER "/usr/local/bin/g++-13")
set(CMAKE_C_COMPILER "/usr/local/bin/gcc-13")
```

#### 方法 2：在终端中设置编译器

如果不想修改 `CMakeLists.txt` 文件，也可以直接在终端中设置编译器：

```bash 
export CC=/usr/local/bin/gcc-13
export CXX=/usr/local/bin/g++-13
```

然后继续执行构建命令：

```bash 
cmake ..
make
```

#### 方法 3：使用 `update-alternatives` 命令设置默认编译器

如果系统中已安装多个版本的GCC，可以使用 `update-alternatives` 命令将 **GCC 13** 设置为默认编译器：

```bash 
sudo update-alternatives --install /usr/bin/gcc gcc /usr/local/bin/gcc-13 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/local/bin/g++-13 100
```

然后选择默认的GCC版本：

```bash 
sudo update-alternatives --config gcc
sudo update-alternatives --config g++
```

选择 **GCC 13** 作为默认选项。

### 3. 创建工作空间并进入该目录

```bash 
mkdir -p ~/catkin_ws
cd ~/catkin_ws
```

### 4. 将 `u2canfdC++` 文件夹放入 `catkin_ws` 目录下

### 5. 进入 `u2canfdC++` 文件夹

```bash 
cd ~/catkin_ws/u2canfdC++
```

### 6. 创建构建目录、配置并编译程序

```bash 
cd ~/catkin_ws/u2canfdC++
rm -rf build
mkdir build
cd build
cmake ..
make -j$(nproc)
```

## 设置和配置

### 步骤 1：确认电机波特率为5M（出场默认波特率为5M，请勿修改）

使用最新的上位机软件将电机的波特率设置为 **5M**。

### 步骤 2：设置USB转CANFD设备权限

为了确保程序能够正常访问USB设备，需要设置USB转CANFD设备的权限：

1. 编辑udev规则文件：
   ```bash
   sudo nano /etc/udev/rules.d/99-usb.rules
   ```

2. 添加以下规则：

   对于**USB转单路CANFD设备**：
   ```
   SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666"
   ```


3. 重新加载并触发udev规则：
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

   > **注意：** 这个权限设置只需要做一次，之后不需要在重启或拔插设备时重新设置。

### 步骤 3：查找USB转CANFD设备的序列号

1. 进入到工作空间的 `build` 目录：
   ```bash
   cd ~/catkin_ws/u2canfdC++/build
   ```

2. 运行 `dev_sn` 工具查找设备的**序列号**：
   ```bash
   ./dev_sn
   ```

   输出中的 `SN` 后面的一串数字就是该设备的序列号。请复制该序列号。

### 步骤 4：修改并编译代码

1. 打开 `main.cpp` 文件，将其中的序列号替换为步骤3中获取的序列号。

2. 根据实际使用的设备选择 **单路CANFD** 。

3. 修改完成后，重新编译程序：
   ```bash
   cd ~/catkin_ws/u2canfd C++/build
   make
   ```

### 步骤 5：运行程序

1. 编译完成后，运行电机控制程序：
   ```bash
   cd ~/catkin_ws/u2canfd C++/build
   ./dm_main
   ```
   此时夹爪开始做往复运动，控制台会输出类似以下信息：
   ```
   Motor CAN ID: 1 | Target Position: 0.5 | Actual Position: 0.52 | Velocity: 0.0 | Torque: 0.0 | Time Interval: 0.001
   Motor CAN ID: 1 | Target Position: 0.5 | Actual Position: 0.51 | Velocity: 0.0 | Torque: 0.0 | Time Interval: 0.001
   Motor CAN ID: 1 | Target Position: 0.5 | Actual Position: 0.51 | Velocity: 0.0 | Torque: 0.0 | Time Interval: 0.001
   ```
   输出包括了电机的目标位置、实际位置、速度、力矩和时间间隔等信息。这些数据将帮助你了解电机的控制状态及其与目标位置的偏差。随着夹爪做往复运动，目标位置会持续变化

## 注意事项

- **5M波特率**：如果多个电机连接在5M波特率（出厂已设置为5M波特率）下，请确保末端电机接入一个 **120欧的电阻**，以避免数据传输问题。
- 该程序会与USB转CANFD设备进行通信，发送命令控制电机的动作。

## 故障排除

- 如果设备权限出现问题，确保udev规则已经正确配置，并且设备能够被系统识别。
- 如果电机没有响应，请检查 **波特率** 和 **控制模式** 是否设置正确。

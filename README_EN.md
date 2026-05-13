
# Control Motor Using a USB-to-CANFD Device

This program is used to control the OmniGripper gripper (DM4310 motor) through a **USB-to-CANFD device**. The program sets the motor control mode to MIT mode, enables the motor, and outputs a sinusoidal trajectory command to realize the reciprocating opening and closing motion of the gripper. The motor baud rate is set to 5M. If multiple motors are connected under the 5M baud rate, a 120-ohm resistor needs to be connected to the motor at the end of the bus.

## Software Architecture
- Written in C++ without ROS.

## Installation and Build

### 1. Make sure libusb is installed on the system

The required version is **1.0.29** or higher. If it is not installed, you can install it using the following commands:

```bash
wget https://github.com/libusb/libusb/releases/download/v1.0.29/libusb-1.0.29.tar.bz2
tar -xf libusb-1.0.29.tar.bz2
cd libusb-1.0.29
./configure --prefix=/usr
make
sudo make install
````

### 2. Install GCC 13

This project needs **GCC 13** for compilation. Make sure GCC 13 is installed, and you can set the compiler in one of the following ways:

#### Method 1: Set it in the `CMakeLists.txt` file

Open the `CMakeLists.txt` file (located in the project root directory), and add the following lines at the top of the file to specify **GCC 13** as the compiler:

```cmake
# Set C++ compiler to GCC 13
set(CMAKE_CXX_COMPILER "/usr/local/bin/g++-13")
set(CMAKE_C_COMPILER "/usr/local/bin/gcc-13")
```

#### Method 2: Set the compiler in the terminal

If you do not want to modify the `CMakeLists.txt` file, you can also set the compiler directly in the terminal:

```bash
export CC=/usr/local/bin/gcc-13
export CXX=/usr/local/bin/g++-13
```

Then continue with the build commands:

```bash
cmake ..
make
```

#### Method 3: Use the `update-alternatives` command to set the default compiler

If multiple GCC versions are installed on the system, you can use the `update-alternatives` command to set **GCC 13** as the default compiler:

```bash
sudo update-alternatives --install /usr/bin/gcc gcc /usr/local/bin/gcc-13 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/local/bin/g++-13 100
```

Then select the default GCC version:

```bash
sudo update-alternatives --config gcc
sudo update-alternatives --config g++
```

Choose **GCC 13** as the default option.

### 3. Create a workspace and enter the directory

```bash
mkdir -p ~/catkin_ws
cd ~/catkin_ws
```

### 4. Place the `u2canfdC++` folder into the `catkin_ws` directory

### 5. Enter the `u2canfdC++` folder

```bash
cd ~/catkin_ws/u2canfdC++
```

### 6. Create the build directory, configure, and compile the program

```bash
mkdir build
cd build
cmake ..
make
```

## Setup and Configuration

### Step 1: Confirm that the motor baud rate is 5M (the factory default baud rate is 5M, please do not change it)

Use the latest host computer software to set the motor baud rate to **5M**.

### Step 2: Set permissions for the USB-to-CANFD device

To ensure that the program can access the USB device correctly, you need to set permissions for the USB-to-CANFD device:

1. Edit the udev rules file:

   ```bash
   sudo nano /etc/udev/rules.d/99-usb.rules
   ```

2. Add the following rule:

   For the **USB-to-CANFD device**:

   ```text
   SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666"
   ```

3. Reload and trigger the udev rules:

   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

   > **Note:** This permission setting only needs to be done once. After that, it does not need to be set again after rebooting or unplugging/replugging the device.

### Step 3: Find the serial number of the USB-to-CANFD device

1. Enter the `build` directory in the workspace:

   ```bash
   cd ~/catkin_ws/u2canfdC++/build
   ```

2. Run the `dev_sn` tool to find the **serial number** of the device:

   ```bash
   ./dev_sn
   ```

   The string following `SN` in the output is the serial number of the device. Please copy this serial number.

### Step 4: Modify and compile the code

1. Open the `main.cpp` file and replace the serial number in it with the serial number obtained in Step 3.

2. Check the relevant configuration according to the actual device in use.

3. After the modification is completed, recompile the program:

   ```bash
   cd ~/catkin_ws/u2canfdC++/build
   make
   ```

### Step 5: Run the program

1. After compilation is completed, run the motor control program:

   ```bash
   cd ~/catkin_ws/u2canfdC++/build
   ./dm_main
   ```

   At this point, the gripper will start reciprocating, and the console will output information similar to the following:

   ```text
   Motor CAN ID: 1 | Target Position: 0.5 | Actual Position: 0.52 | Velocity: 0.0 | Torque: 0.0 | Time Interval: 0.001
   Motor CAN ID: 1 | Target Position: 0.5 | Actual Position: 0.51 | Velocity: 0.0 | Torque: 0.0 | Time Interval: 0.001
   Motor CAN ID: 1 | Target Position: 0.5 | Actual Position: 0.51 | Velocity: 0.0 | Torque: 0.0 | Time Interval: 0.001
   ```

   The output includes the motor target position, actual position, velocity, torque, and time interval. These data help you understand the motor control status and its deviation from the target position. As the gripper performs reciprocating motion, the target position will continue to change.

## Notes

* **5M baud rate**: If multiple motors are connected under the 5M baud rate (the factory default baud rate is already set to 5M), please make sure that a **120-ohm resistor** is connected to the motor at the end of the bus to avoid data transmission issues.
* The program communicates with the USB-to-CANFD device and sends commands to control the motor motion.

## Troubleshooting

* If there is a device permission problem, make sure the udev rules are configured correctly and that the device can be recognized by the system.
* If the motor does not respond, please check whether the **baud rate** and **control mode** are set correctly.


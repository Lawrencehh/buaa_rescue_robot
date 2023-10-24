# BUAA Rescue Robot

## Overview

This repository contains the codebase for a rescue robot control system, developed using ROS 2 (Humble).

## Table of Contents

1. [Installation](#installation)
2. [Usage](#usage)
3. [Contributing](#contributing)
4. [License](#license)

## Installation

### Prerequisites

- ROS 2 Humble
- C++14 or higher

### Build and Install

```bash
# Clone the repository
git clone https://github.com/Lawrencehh/buaa_rescue_robot.git

# Navigate to the workspace
cd buaa_rescue_robot

# install requirements
rosdep install --from-paths src --ignore-src -r -y

# Build the project
colcon build

# Source the setup script
source install/setup.bash

# remove the brltty
sudo apt remove brltty
```

### Serial Devices Rename
```bash
sudo gedit /etc/udev/rules.d/99-usb-serial.rules
```
```bash
 KERNEL=="ttyUSB[0-9]*", MODE="0666"
# Set a fixed name for specific serial devices
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK+="ttyRobomaster1"
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="23a3", MODE="0666", SYMLINK+="ttyPullPushSensors1"
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", SYMLINK+="ttyElevatorLinearModules"
```

```bash
sudo gedit /etc/udev/rules.d/99-force-dimension.rules
```
```bash
# Add permissions
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1451", ATTRS{idProduct}=="0402", MODE="0666", GROUP="plugdev"
```
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```



## Usage
### launch the gui
```bash
ros2 launch buaa_rescue_robot_msgs control_by_keyboard.launch.py 
```
### keyboard_controller  
Run the keyboard_publisher node:
```bash
ros2 run keyboard_controller keyboard_publisher
```


The communication protocol for the robot system can be found in the link:  
https://docs.google.com/document/d/15l8EZoLQ8ltLROfFUyf_mJ9b2J9PZ0hl/edit?usp=sharing&ouid=102832733841320281280&rtpof=true&sd=true

## Contributing
The current code contributors are Dr. Huang Hao and Xiang Yan.

## License
This project is licensed under the MIT License.

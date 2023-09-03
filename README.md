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

# Build the project
colcon build

# Source the setup script
source install/setup.bash
```

## Usage
### Talker and Listener  
Run the talker node:

```bash
ros2 run cpp_pubsub talker
```

Run the listener node:
```bash
ros2 run cpp_pubsub listener
```

### Serial Communication
Run the serial sender node:
```bash
ros2 run serial serial_sender
```

Run the serial receiver node:
```bash
ros2 run serial serial_receiver
```
The communication protocol for the robot system can be found in the link:  
https://docs.google.com/document/d/15l8EZoLQ8ltLROfFUyf_mJ9b2J9PZ0hl/edit?usp=sharing&ouid=102832733841320281280&rtpof=true&sd=true

## Contributing
The current code contributor is Dr. Huang Hao.

## License
This project is licensed under the MIT License.

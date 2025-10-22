# Virtual Tactile Sensor Using Wrench Sensing

This repository contains the implementation of a virtual tactile sensor system that utilizes wrench sensing capabilities. The system is designed to work with a Franka Emika Panda robotic arm and an ATI Gamma Force/Torque sensor.

## System Requirements

### Hardware Components
- [Franka Emika Panda](https://franka.de/) robotic arm
- [ATI Gamma FT sensor](https://www.ati-ia.com/products/ft/ft_models.aspx?id=gamma) for force/torque measurements
- Ubuntu 20.04 (required for ROS Noetic)

### Software Prerequisites
- ROS Noetic
- Docker (for containerized deployment)

## Network Configuration

The system requires specific network settings to function properly:
- Robot subnet: 172.16.0.1
- ATI Gamma FT sensor: 128.178.145.248

Ensure all devices are on the same subnet for proper communication.

## Docker Setup

Navigate to the docker directory from the project root:
```bash
cd docker
```

1. Install Docker:
```bash
bash install_docker.sh
```

2. Build the Docker container:
```bash
bash build_docker.sh
```

3. Start the virtual tactile pad environment:
```bash
bash start_tactile_pad.sh
```

## ROS Usage

The project is built on ROS Noetic and includes several launch files for different functionalities.

To run a launch file:
```bash
roslaunch virtual_tactile_pad <launch_file_name>.launch
```

To view available launch files:
```bash
ls src/virtual_tactile_pad/launch/
```

## Getting Started

1. Clone this repository
2. Set up your network configuration
3. Install Docker and build the container
4. Launch the ROS environment
5. Run the desired launch file for your specific use case
   
(Demo videos are located on this [drive](https://drive.google.com/drive/u/1/folders/1xjCZFb9rxKVa-lkfhxeg2w6x7sw5DxhQ))

## Troubleshooting

If you encounter network connectivity issues:
- Verify that all devices are on the same subnet
- Check the IP configurations of both the robot and the FT sensor
- Ensure no firewall rules are blocking the required ports

## License

This project is licensed under the Apache License, Version 2.0 - see below for details:

```
Copyright 2025

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

For the full license text, please see the [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0) website.



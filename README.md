# virtual_tactile_pad

# ROS and Docker Command Reference Guide

## Network Setup
- Need to be on same sub-network with the robot (172.16.0.1) and the ATI Gamma FT sensor if using it (xxx.xx.x.x)

## Docker Commands
### Container Management
```bash
# List all docker images
docker images

# Remove a docker image
docker rmi <image>

# List running and stopped containers
docker ps -a

# Create and start a new container in terminal
docker run -it <container>

# Start a stopped container
docker start <container>

# Restart a container
docker restart <container>

# Remove a stopped container
docker rm <container>

# Connect to container (AICA specific)
aica-docker connect <container>
```

### AICA Docker Launch Command
```bash
aica-docker interactive panda_controllers:noetic -u ros --net host --no-hostname \
    -v $(pwd)/panda_controllers_adapted:/home/ros/ros_ws/src/panda_controllers \
    --cap-add=sys_nice --ulimit rtprio=99
```

## ROS Commands
### Build and Source
```bash
# Compile changes
catkin_make

# Source ROS setup files
source devel/setup.bash
source /opt/ros/noetic/setup.bash
```

### Launch Commands
```bash
# Launch F/T sensor
roslaunch netft_rdt_driver ft_sensor.launch

# Launch Cartesian impedance controller
roslaunch franka_example_controllers cartesian_impedance_example_controller.launch robot_ip:=172.16.0.1

# Launch rqt_plot
rosrun rqt_plot rqt_plot

# Monitor F/T sensor data
rostopic echo /ft_sensor/netft_data
```

## Important Configuration Changes

### Update Frequency Rate
1. In `/home/ros/ros_ws/src/franka_ros/franka_control/config/default_controllers.yaml`:
```yaml
publish_rate: 1000  # [Hz]
```

2. In `/home/ros/ros_ws/src/franka_ros/franka_control/launch/franka_control.launch`:
```xml
<param name="rate" value="1000"/>
<arg name="load_gripper" default="false" />
```

## Important Topics
- Topic to subscribe: `/franka_state_controller/franka_states`

### Sample Data Format
```
tau_ext: [-0.00071146 -0.00700912 -0.00227974 -0.00110497 -0.00086796 -0.00085522 -0.00304101]
q: [0.81605056 0.60174385 -1.10007806 -1.38739981 -1.88743462 3.36594903 2.15684093]
```

## Docker Session Management
- To exit container: type `exit` or use `Ctrl+P, Ctrl+Q`
# Run rosbridge (ROS2 Humble)

## Install
sudo apt update
sudo apt install ros-humble-rosbridge-server

## Launch
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml   # ws://<IP>:9090

## Test
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /joint_states

# Optional demo
ros2 run turtlesim turtlesim_node
# remap or bridge /cmd_vel if needed

#!/bin/bash

# Source the ROS 2 workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Trap SIGINT to kill background processes when the script is terminated
trap 'kill $(jobs -p)' SIGINT

# Launch rm_driver in the background
echo "Starting RealMan Driver..."
ros2 launch rm_driver rm_75_driver.launch.py &

# Wait for a moment to ensure driver starts up
sleep 5

# Launch ethercat_bringup in the foreground
echo "Starting EtherCAT Bringup..."
ros2 launch ethercat_device_control ethercat_bringup.launch.py

# Wait for background processes to finish
wait

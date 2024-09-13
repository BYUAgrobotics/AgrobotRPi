#!/bin/bash

##########################################################
# STARTS CONTROLLERS AND RECORDS A ROS2 BAG FILE
# - Run this after running the 'start.sh' and 'test.sh'
#   scripts
# - Log files are saved in '~/bag' on the host machine
#   running the docker container
##########################################################

echo ""
echo "Enter a descriptive folder name for the rosbag: "
read FOLDER
echo ""

cd ~/ros2_ws
source install/setup.bash

# Publish to init
ros2 topic pub /init std_msgs/msg/Empty -1

FOLDER=$FOLDER-$(date +"%Y-%m-%d-%H-%M-%S")
ros2 bag record -o /home/agrobot/ros2_ws/bag/$FOLDER -s mcap -a

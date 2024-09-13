#!/bin/bash

##########################################################
# TESTS EXPECTED ROS TOPICS
# - Run this after running the 'start.sh' script
##########################################################

cd ~/ros2_ws
source install/setup.bash

echo ""
echo "LISTING FOUND TOPICS..."
ros2 topic list

# echo ""
# echo "LISTENING TO TOPIC 'BATTERY_DATA'..."
# ros2 topic echo --once /battery_data

echo ""
echo "LISTENING TO TOPIC 'RANGE_DATA'..."
ros2 topic echo --once /range_data

echo ""
echo "PUBLISHING TO 'DRIVE_COMMAND'..."
ros2 topic pub -1 /drive_command agrobot_interfaces/msg/DriveCommand '{fr_motor: 10.0, fl_motor: 10.0, br_motor: 10.0, bl_motor: 10.0}'

echo ""
echo "PUBLISHING TO 'DRIVE_COMMAND'..."
ros2 topic pub -1 /drive_command agrobot_interfaces/msg/DriveCommand '{fr_motor: 0.0, fl_motor: 0.0, br_motor: 0.0, bl_motor: 0.0}'

echo ""
echo "TEST COMPLETE"

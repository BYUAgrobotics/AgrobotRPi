#!/bin/bash

##########################################################
# TESTS EACH OF THE EXPECTED MICROROS TOPICS
# - Use this after setting up a new PCB to test the agent
#   and Teensy board connections
##########################################################

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

cleanup() {

    killall micro_ros_agent
    wait
    
    exit 0
}
trap cleanup SIGINT

if [ -z "$(tycmd list | grep Teensy)" ]; then
    echo ""
    printError "No Teensy boards avaliable to connect to"
    echo ""

    exit 1

else 
    cd ~/microros_ws
    source install/setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &
fi

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

cleanup

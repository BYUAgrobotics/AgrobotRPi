#!/bin/bash
# Created by Nelson Durrant, Sep 2024

##########################################################
# STARTS THE AGENT AND RUNS A SPECIFIED LAUNCH FILE
# - Specify a start configuration using 'bash start.sh 
#   <launch>' (ex. 'bash start.sh base')
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

echo ""
echo "#################################################"
echo "# BYU AGRICULTURAL ROBOTICS TEAM - START SCRIPT #"
echo "#################################################"
echo ""

# Start the micro-ROS agent
if [ -z "$(tycmd list | grep Teensy)" ]; then
    printError "No Teensy boards avaliable to connect to"
    echo ""

else 
    cd ~/microros_ws
    source install/setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &

    sleep 5
    echo ""
fi

# Start the ROS 2 launch files
cd ~/ros2_ws
source install/setup.bash

case $1 in
    "base")
        ros2 launch agrobot_py base_launch.py
        ;;
    *)
        printWarning "No start configuration specified"
        printWarning "Specify a start configuration using 'bash start.sh <config>' (ex. 'bash start.sh base')"
        echo ""
        ;;
esac

cleanup

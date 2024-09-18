#!/bin/bash
# Created by Nelson Durrant, Sep 2024

##########################################################
# SYNCS INTERFACE CHANGES WITH THE TEENSY_WS
# - After running this script, recompile micro-ROS in 
#   PlatformIO using 'msg_update.sh'
# - Don't run this in the Docker image -- run it on your 
#   own machine or locally on the the Pi
##########################################################

rsync -avc --delete ~/AgrobotRPi/src/agrobot_interfaces ~/AgrobotTeensy/agrobot/extra_packages

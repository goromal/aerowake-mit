#!/bin/bash

# Sync clock with uavswarm computer
sudo ntpdate -u 192.168.0.19

# Set up ROS network
source ROS_ENVS.sh
ros_raven


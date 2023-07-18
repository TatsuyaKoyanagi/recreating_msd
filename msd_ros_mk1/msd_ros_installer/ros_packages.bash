#!/bin/bash

# for catkin build
sudo apt install -y python3-catkin-tools

# ds4drv 
sudo pip install ds4drv
# ros joy_node
sudo apt install ros-noetic-joy

# for cartographer navigation
sudo apt-get install -y ros-noetic-ddynamic-reconfigure
sudo apt-get install -y ros-noetic-tf2-sensor-msgs
sudo apt-get install -y ros-noetic-move-base
sudo apt-get install -y ros-noetic-gmapping ros-noetic-amcl ros-noetic-map-server
sudo apt-get install -y ros-noetic-dwa-local-planner
sudo apt-get install -y ros-noetic-robot-localization
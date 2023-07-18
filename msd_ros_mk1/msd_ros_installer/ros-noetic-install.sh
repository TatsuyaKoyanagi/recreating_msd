#!/bin/bash

#sources.listを設定
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#鍵の設定
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key      C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

#インストール
sudo apt install ros-noetic-desktop-full

#環境設定
source /opt/ros/noetic/setup.bash

#.bashrcに書き込む
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

#ROSのツールのインストール
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install -y python3-rosdep

sudo rosdep init

rosdep update
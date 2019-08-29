#!/bin/bash

apt-get update && apt-get install -y wget jq python3-rosdistro python3-rosinstall python3-colcon-common-extensions ros-dashing-ros2bag ros-dashing-rosbag2 ros-dashing-rosbag2-storage-default-plugins ros-dashing-ros2pkg ros-dashing-ros2launch ros-dashing-ros2topic
pip3 install awscli --upgrade


#!/bin/bash
set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting install"

# get the current commit SHA
SHA=`git rev-parse HEAD`

# get the current package name
PACKAGE_NAME=${PWD##*/}

sudo apt-get -y update -qq
sudo apt-mark hold openssh-server

# 20.04 problem fix
sudo apt-get -y install grub-efi
sudo update-grub

# the "gce-compute-image-packages" package often freezes the installation at some point
# the installation freezes when it tries to manage some systemd services
# this attempts to install the package and stop the problematic service during the process
((sleep 90 && (sudo systemctl stop google-instance-setup.service && echo "gce service stoped" || echo "gce service not stoped")) & (sudo timeout 120s apt-get -y install gce-compute-image-packages)) || echo "\e[1;31mInstallation of gce-compute-image-packages failed\e[0m"

ACCEPT_EULA=Y && sudo apt-get -y upgrade --fix-missing

sudo apt-get -y install dpkg git

echo "clone uav_ros_stack"
cd
git clone https://github.com/lmark1/uav_ros_stack.git
cd uav_ros_stack

echo "running the main install.sh"
./installation/install.sh

gitman update

# checkout the SHA
cd ~/uav_ros_stack/.gitman/$PACKAGE_NAME
git checkout "$SHA"

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/uav_ros_stack
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws

echo "install ended"
#!/bin/bash
cat <<EOF
********************************************************************************
                 Installing ROS2 Crystal from binaries.
********************************************************************************

This script will configure a fresh AMD64 Ubuntu 18+ install with ROS2.

Please perform the following steps:

1. Install Ubuntu.

2. Place this script into a directory.

3. Run this script by typing "source <script name>".

Press enter to continue...
EOF
read

# abort on error
set -e

sudo apt update && sudo apt install curl gnupg2 lsb-release
curl http://repo.ros2.org/repos.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
export CHOOSE_ROS_DISTRO=crystal
sudo apt update
sudo apt install ros-$CHOOSE_ROS_DISTRO-desktop

# argcomplete
sudo apt install python3-argcomplete

# done
echo Installation complete.
echo Your AMD64 Ubuntu is now configured to support ROS Crystal with Security enabled.
echo
echo Be sure to set up your .bashrc file as required to access ROS2 Crystal.


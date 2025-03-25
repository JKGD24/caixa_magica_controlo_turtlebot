#!/bin/bash

# Set the Gazebo version (recommended: harmonic)
export GZ_VERSION=harmonic

echo "Installing Gazebo Harmonic and setting up ROS 2 dependencies..."

# Install wget if not already installed
sudo apt update
sudo apt install -y wget

# Add Gazebo APT sources
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update package lists
sudo apt update

# Add Gazebo sources to rosdep for the non-default pairing of ROS 2 Humble and Gazebo Harmonic
wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list
rosdep update

# Clone required repositories using vcstool
cd ~/ardu_ws || exit
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src

# Update ROS and Gazebo dependencies
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build the workspace
colcon build --packages-up-to ardupilot_gz_bringup

# Run tests
source install/setup.bash
colcon test --packages-select ardupilot_sitl ardupilot_dds_tests ardupilot_gazebo ardupilot_gz_applications ardupilot_gz_description ardupilot_gz_gazebo ardupilot_gz_bringup
colcon test-result --all --verbose

echo "Installation and setup complete."

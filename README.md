#Installation instructions

Tested on:

ROS 2 Jazzy
Gazebo Harmonic
Linux (Ubuntu recommended)

Other ROS 2 distros may work but are not guaranteed.

System dependencies
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-jazzy-geometry-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-tf2-msgs \
  ros-jazzy-rosgraph-msgs \
  ros-jazzy-actuator-msgs \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-interfaces

Python dependencies
python3 -m pip install --user numpy scipy pygame


If pygame fails:

sudo apt install -y \
  libsdl2-dev libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev

Build & run
git clone https://github.com/soapmansantos/salvos_sim.git

source /opt/ros/jazzy/setup.bash
cd salvos_sim
colcon build
source install/setup.bash

ros2 launch salvos_gz salvos_sim.launch.py

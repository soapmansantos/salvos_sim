# Salvos Technologies Simulation and Control Stack


`salvos_gz`

- ROS 2 and Gazebo simulation of a **263 kg heavy-lift tailsitter eVTOL pentarotor**, developed to design and test its custom flight control system.  



`salvos_control`

- A model-based, cascaded nonlinear thrust-vectoring controller (differential thrust only).



Control challenge:
- The vehicle is designed for efficient forward flight and high payload capacity, which results in large inertia and aerodynamic sensitivity at low speeds. This makes position control during hover and landing especially challenging.<br><br>







## Installation instructions
Tested on:
- ROS 2 Jazzy
- Gazebo Harmonic
- Ubuntu 24.04

### System dependencies

```bash
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
```

### Python dependencies
```bash
python3 -m pip install --user numpy scipy pygame
```

If pygame fails:
```bash
sudo apt install -y \
  libsdl2-dev libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev
```

### Build & run

```bash
git clone https://github.com/soapmansantos/salvos_sim.git
source /opt/ros/jazzy/setup.bash
cd salvos_sim
colcon build
source install/setup.bash
ros2 launch salvos_gz salvos_sim.launch.py
```


## Controls (keyboard)

A small pygame window opens and listens for keyboard input.

### Mode & system
- **m** — Toggle manual control  
  - When switching back to auto, the target is reset to the current position and hovers there
- **r** — Reset simulation  
  - Stops motors, resets the world, reinitialises the controller, randomises wind

### Landing / takeoff
- **n** — Start landing sequence  
  - Manual control disabled  
  - Holds current position, then descends
- **t** — Take off again (only after landing)  

### Manual control inputs
- **w, a, s, d** — Manual control inputs  
  - Only active when manual mode is enabled (`m`)  
  - Height and heading set by controller

# TUCAN ROS2 Package Stack
Repository for the TU-Delft team of the IMAV 2024 Indoor competition. Works with ROS2 Humble. WIP

## 1. Installation

1. Clone this repository into the following folder structure:
```
├── ros2_ws                    
│   ├── src          
│   │   ├── tucan_ros2     # This repository
```
2. Initialize submodules.
```
git submodule update --init --recursive
```

3. Install dependencies using rosdep:
```
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace.
```
colcon build
```

## 2. Run Simulation

This assumes ubuntu 22.04, on different ubuntu gz-garden has to be installed separately.

1. Clone our px4 repository in the correct location and build:
```
cd ~
git clone git@github.com:IMAV-MAVLab-2024/PX4-Autopilot.git
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot
make px4_sitl
```

2. Update gazebo bridge.
```
sudo apt remove ros-humble-ros-gz-bridge
sudo apt-get install ros-humble-ros-gzgarden
```

3. Build the micro-XRCE agent:
```
cd ~    # or any  other directory you want
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

4. Build
```
colcon build
```

5. Run simulation
```
source install/local_setup.sh
ros2 launch tucan_sim tucan_sim.launch.py
```

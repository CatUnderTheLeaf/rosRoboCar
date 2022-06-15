# rosRoboCar
Robocar uses ROS2 Humble Hawksbill and Gazebo 11.

### Installation

> Windows users read first [Instruction for Windows users](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Instruction-for-Windows-users) and then continue

Install [Gazebo](https://classic.gazebosim.org/download) (download compatible with ROS2 Humble version 11). Install `gazebo_ros_pkgs` if not installed:

```
sudo apt install ros-humble-gazebo-ros-pkgs
```

Download this repository

```
git clone --recurse-submodules https://github.com/CatUnderTheLeaf/rosRoboCar.git

source /opt/ros/humble/local_setup.bash
cd /path/to/robocar_ws

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble

# Building packages
# It takes about 18 minutes
colcon build

# Source this workspace
source install/local_setup.bash

# Copy Gazebo models, else it will be downloaded from internet without my changes 
cp -r /src/robo_car/world ~/.gazebo/models/
```
### Launch

Launch a car simulation example based on [Prius Hybrid in Sonoma Raceway](https://app.gazebosim.org/chapulina/fuel/worlds/Prius%20on%20Sonoma%20Raceway)

```
# In one terminal launch Gazebo with race world and a car in it
gazebo --verbose src/robo_car/world/robocar.world

# In another terminal launch ros nodes
ros2 launch robo_car robocar_launch.py
```

To be continued...

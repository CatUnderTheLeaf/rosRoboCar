# rosRoboCar
Now Robocar uses [Webots ROS2 Interface](https://github.com/cyberbotics/webots_ros2) with [Webots](https://github.com/cyberbotics/webots) simulator.

### Installation

> Windows users read first [Instruction for Windows users](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Instruction-for-Windows-users) and then continue

Download this repository

```
git clone --recurse-submodules https://github.com/CatUnderTheLeaf/rosRoboCar.git
```

Install webots_ros2 packages.

```
source /opt/ros/$ROS_DISTRO/local_setup.bash

# Retrieve the sources
cd /path/to/robocar_ws
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# Building packages
# It takes about 18 minutes
colcon build

# Source this workspace (careful when also sourcing others)
source install/local_setup.bash
```

### Launch

Launch a car simulation example based on Tesla Model 3. If Webots simulator R2022a is not installed, it can be downloaded when you launch a Robocar for the first time.

```
ros2 launch robo_car robocar_launch.py
```

To be continued...

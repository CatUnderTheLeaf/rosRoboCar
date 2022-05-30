# rosRoboCar
Now Robocar uses [Webots ROS2 Interface](https://github.com/cyberbotics/webots_ros2) with [Webots](https://github.com/cyberbotics/webots) simulator.

### Installation

Download this repository

```
git clone --recurse-submodules https://github.com/CatUnderTheLeaf/rosRoboCar.git
```
In order to launch Robocar please install Webots ROS2 Interface and Webots simulator. Webots can be downloaded automatically by clicking Automatic in a window that appears when you launch a Robocar.

##### Linux
```
source /opt/ros/$ROS_DISTRO/local_setup.bash

# Retrieve the sources
cd /path/to/robocar_ws
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# Building packages
colcon build

# Source this workspace (careful when also sourcing others)
source install/local_setup.bash
```

##### Windows
Make sure to run the following commands from the `x64 Native Tools Command Prompt for VS 2019` as an Administrator:
```
# Install pip dependencies
pip install rosinstall_generator colcon-common-extensions lark vcstool

# Source ROS 2 environment, change if you have another location
call C:\dev\ros2_galactic\local_setup.bat

# Retrieve the sources
cd \path\to\robocar_ws
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2

# Install ROS 2 dependencies, change ros2 path if needed
rosinstall_generator --deps-only --deps --from-path=src --exclude-path=C:\dev\ros2_galactic --exclude=moveit > deps.repos
vcs import src < deps.repos

# Building packages
colcon build

# Source this workspace (careful when also sourcing others)
call install\local_setup.bat
```

### Launch

Launch a car simulation example based on Tesla Model 3.

```
ros2 launch robo_car robocar_launch.py
```

To be continued...

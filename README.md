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
### Launch Simulation

Launch a car simulation example based on [Prius Hybrid in Sonoma Raceway](https://app.gazebosim.org/chapulina/fuel/worlds/Prius%20on%20Sonoma%20Raceway)

```
# In one terminal launch Gazebo with race world and a car in it
gazebo --verbose src/robo_car/world/robocar.world

# In another terminal launch ros nodes
ros2 launch robo_car robocar_launch.py
```

### Real car

I have a simple [Donkey car](https://docs.donkeycar.com/) with RaspberryPi and camera. On RaspberryPi I installed [Ubuntu Server 22.04](https://ubuntu.com/download/raspberry-pi) and ROS2 Humble Hawksbill.

If you currently have the 2GB version, you can run all your code on a car. But if you have the 1 GB version for Ubuntu Server + ROS2 it is enough, but you’ll probably be limited in the future if you try to start more than a few nodes and launch files. In that case you may run your ROS2 nodes across multiple machines, your PC and your car:
* make sure you don’t have a firewall blocking your communications on the network. If you have a firewall, allow UDP multicasting, or disable the firewall at least during your first tests.
* connect all your machines in the same network. <b>This is very important, otherwise they simply won’t be able to find each other</b>.
* check if the machines can reach out to each other. First, get the IP address of each machine inside the network by running 'hostname -I.' Then ping each other.
* on one machine 'ros2 run demo_nodes_cpp talker', on another 'ros2 run demo_nodes_py listener'
* if there is no communication between machines then read [this](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/How-to-setup-ROS2-Fast-DDS-Discovery-Server) article.

To be continued...

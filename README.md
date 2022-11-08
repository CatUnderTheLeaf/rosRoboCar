# rosRoboCar
Robocar uses ROS1 Noetic and [DonkeyCar](https://docs.donkeycar.com/). It is a multiple machine project, where a car sends camera images and on computer I can visualize and work with them.

### Installation on PC

ROS1 Noetic should be installed.

> Windows users read first [Instruction for Windows users](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Instruction-for-Windows-users) and then continue

```
# Download this repository
git clone --recurse-submodules https://github.com/CatUnderTheLeaf/rosRoboCar.git

source /opt/ros/noetic/setup.bash
cd /path/to/robocar_ws

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic

# Building packages
# Add CATKIN_IGNORE to robot_only packages, e.g. raspicam_node
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# Source this workspace
source devel/setup.bash

```

### Installation on a car

1. A DonkeyCar has RaspberryPi, so first make it [work](https://docs.donkeycar.com/guide/robot_sbc/setup_raspberry_pi/).
2. Install ROS1 Noetic from [sources](http://wiki.ros.org/noetic/Installation/Source). In the `catkin_make_isolated` step add `-j2` flag, so it will not stuck.
3. Download this repository.
4. Unfortunately dependencies can not be resolved, as there are no binary packages. So download in `~/ros_catkin_ws/src` following repositories: [image_transport](https://github.com/ros-perception/image_transport_plugins.git), replace [image_common](https://github.com/ros-perception/image_common.git).
5. Install `compressed_image_transport`, `image_transport_plugins`, `camera_calibration_parsers`, `camera_info_manager` with a command
```
./src/catkin/bin/catkin_make_isolated -j2 --install -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 --install-space /opt/ros/noetic --pkg YOUR_PACKAGE_NAME
```

### Running ROS across multiple machines

[Connect](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) your car and PC. Don't forget to export `ROS_IP` and `ROS_MASTER_URI`

Now on a car `raspicam_node` can be launched with
```
roslaunch raspicam_node camerav2_1280x960.launch
```

On PC:
```
 rosrun image_view image_view image:=/raspicam_node/image _image_transport:=compressed
```


Next step: camera calibration, sending PWM signals.

# rosRoboCar for the "Autonomous Driving Competition''
[Competition task](https://www.meetup.com/autonomous-robots-berlin/): A real small car has to do at least 3 loops (a long loop to keep the lane or a loop to avoid obstacles) in a row without human intervention.

Robocar uses ROS1 Noetic and [DonkeyCar](https://docs.donkeycar.com/). For testing and training purposes it can be run across multiple machines, when a car sends camera images and on computer I can visualize and work with them.

### Setup

1. [Installation](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Installation) on PC and on a car

2. (optional) [Running ROS across multiple machines](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Running-ROS-across-multiple-machines)

3. [Camera calibration](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Camera-calibration). All data should be added to `robocar_ws/src/donkeycar/config/camera_info.yaml`

4. Steering and throttle [calibration](https://docs.donkeycar.com/guide/calibrate/). All channels and PWM values should be added to `robocar_ws/src/donkey_actuator/config/servos.yaml`

5. [Setup](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Setup-joystick) joystick control

6. [Record](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Make-bagfiles) bagfiles

### Nodes and topics

![rqtgraph](https://github.com/CatUnderTheLeaf/rosRoboCar/blob/main/additional_files/rosgraph.png)

### TF frames tree

![frames](https://github.com/CatUnderTheLeaf/rosRoboCar/blob/main/additional_files/tf_frames.jpg)

### Launch

On PC
```
# mock camera will publish only one image with 30Hz rate
roslaunch donkeycar donkey.launch simulation:=1

# to play recorded bag
roslaunch donkeycar donkey.launch simulation:=1 playbag:=1
```

On a car
```
roslaunch donkeycar donkey.launch simulation:=0
```

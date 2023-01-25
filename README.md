# rosRoboCar for the "Autonomous Driving Competition''
[Competition task](https://www.meetup.com/autonomous-robots-berlin/): A real small car has to do at least 3 loops (a long loop to keep the lane or a loop to avoid obstacles) in a row without human intervention.

Robocar uses ROS1 Noetic and [DonkeyCar](https://docs.donkeycar.com/). For testing and training purposes it can be run across multiple machines, when a car sends camera images and on computer I can visualize and work with them.

### Current configuration

* Sensors:
  * RaspberryPi Camera
* Motors (recieve only PWM signals):
  * Throttle
  * Steering

### Run on PC

Currently on PC without a car a bagfile can be used.

1. [Install](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Installation) on PC

2. Download [bagfiles](https://drive.google.com/drive/folders/1vaq_QgQXcNmLYTS5K2Qk5rRrib_qC0az?usp=share_link) directory to `robocar_ws/src/path_from_image`

3. Install tensorflow 2.2 (not the latest) with `pip install tensorflow==2.2`

4. For GPU acceleration also install [CUDA Toolkit 10.1](https://developer.nvidia.com/cuda-10.1-download-archive-base) and [CuDNN](https://developer.nvidia.com/rdp/cudnn-archive). Unzip to a suitable location and copy files to corresponding folders in `NVIDIA GPU Computing Toolkit\CUDA\v10.1` Check with `python -c "import tensorflow as tf; print(tf.config.list_physical_devices('GPU'))"`

5. I could possibly forget some dependancies to be installed, like `scikit-learn` etc.

6. Make all `.py` files executable. (Some files could be commited from Windows PC)

7. Add empty `CATKIN_IGNORE` file to `raspicam_node` so it will be not build

8. Launch in `robocar_ws`
```
# make sure catkin_make was executed prior
source devel/setup.bash

# launch bag-play, image-view and rviz
# to change bag-file just edit src/donkeycar/launch/bagdonkey.launch file
roslaunch donkeycar bagdonkey.launch

# in separate terminal 
source devel/setup.bash
# and launch lane detection, which needs GPU acceleration
roslaunch donkeycar lanedetect.launch
```

---
### To be updated later...

### Steps to make a car

1. [Installation](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Installation) on PC and on a car

   * [Resolve](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Connectivity-problem) connectivity problems if there are any.

2. (optional) [Running ROS across multiple machines](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Running-ROS-across-multiple-machines)

3. [Camera calibration](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Camera-calibration). All data should be added to `robocar_ws/src/donkeycar/config/camera_info.yaml`

4. Steering and throttle [calibration](https://docs.donkeycar.com/guide/calibrate/). All channels and PWM values should be added to `robocar_ws/src/donkey_actuator/config/servos.yaml`

5. [Setup](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Setup-joystick) joystick control

6. [Record](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Make-bagfiles) bagfiles

### Lane Keeping Pipeline

  - [ ] !!! On the competition site [resolve](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Connectivity-problem) all connectivity problems
  - [x] publish TF messages from URDF model
     - [x] URDF model of a car is in `src/donkeycar/config/car.xacro`
     - [x] add `optical_camera_link` in the model due to [different axis](http://wiki.ros.org/image_pipeline/CameraInfo) orientation
     - [x] add `joint_state_publisher` and `robot_state_publisher` nodes to the launch file to publish static TF messages. Transfomation between `base_footprint` and `camera_link_optical` should be possible
  - [x] send image from camera
     - [x] `raspicam_node` publishes `ImageCompressed` and `CameraInfo` messages with 30Hz rate. With a flag can also publish `ImageRaw`
  - [x] image processing
     - [x] make undistortion faster as 20Hz, because it slows down everything else
  - [ ] Lane detection, 2 variants
     - [ ] Neural network
        - [ ] get lots of images (bag)
        - [ ] train network
     - [ ] Traditional Computer Vision approach
        - [x] get matrices for transformation into 'top-view'
        - [x] make a binary treshold
        - [ ] with a sliding window get points of lanes, make it faster then 25 Hz
        - [ ] lanes are not always seen in the image
        - [x] fit polynomials
        - [x] draw a filled polygon
        - [x] get middle line points and draw them
        - [x] unwarp polygon image
        - [x] combine with weights original image with a filled polygon
     - [x] publish image with lane polygon and middle line points in the `camera_link_optical` frame
     - [x] transform middle line points from `camera_link_optical` to `base_footprint` and publish `Path` message
  - [x] `controller_node` get first point from `Path` message and transform to `Twist` message (linear.x and angular.z)  
  - [ ] `donkey_actuator_node` transforms `Twist` message to PWM signals

### Nodes and topics

![rqtgraph](https://github.com/CatUnderTheLeaf/rosRoboCar/blob/main/additional_files/rosgraph.png)

### TF frames tree

![frames](https://github.com/CatUnderTheLeaf/rosRoboCar/blob/main/additional_files/tf_frames.jpg)

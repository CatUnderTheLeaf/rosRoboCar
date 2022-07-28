import numpy as np
import cv2
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class MockCamera(Node):

    def __init__(self):
        super().__init__('mock_camera')      
        
        # mock of CameraInfo
        self.info = CameraInfo()
        self.info.header.frame_id = 'front_camera_sensor'
        self.info.height = 800
        self.info.width = 800
        self.info.distortion_model = 'plumb_bob'
        self.info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.info.k = [476.7030836014194, 0.0, 400.5, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 1.0]
        self.info.p = [476.7030836014194, 0.0, 400.5, -33.36921585209936, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.info.binning_x = 0
        self.info.binning_y = 0
        self.info.roi.x_offset = 0
        self.info.roi.y_offset = 0
        self.info.roi.height = 0
        self.info.roi.width = 0
        self.info.roi.do_rectify = False

        self.bridge = CvBridge()

        # Publishers and subscribers
        # Get topic names from ROS params
        self.declare_parameter('image_raw', '/vehicle/front_camera/image_raw')
        self.declare_parameter('camera_info', '/vehicle/front_camera/camera_info')
       
        self.camera_pub = self.create_publisher(
            Image,
            self.get_parameter('image_raw').get_parameter_value().string_value,
            10)
        self.camera_pub

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter('camera_info').get_parameter_value().string_value,
            10)
        self.camera_info_pub

        self._output_timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            test_dir_path = '/home/catundertheleaf/robocar/robocar_ws/src/path_from_image/resource'
            camera_img = cv2.imread(os.path.join(test_dir_path, 'cv_image.jpg'))
            # make image message and publish it
            # img type is 8UC4 not compatible with bgr8
            camera_img_msg = self.bridge.cv2_to_imgmsg(camera_img, "bgr8")
            self.camera_pub.publish(camera_img_msg)
            self.camera_info_pub.publish(self.info)
                
        except CvBridgeError as e:
            self.get_logger().info(e)
        

def main(args=None):
    rclpy.init(args=args)
    mock_camera = MockCamera()

    rclpy.spin(mock_camera)

    mock_camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
from sensor_msgs.msg import CameraInfo

# mock of CameraInfo
info = CameraInfo()
info.header.frame_id = 'front_camera_sensor'
info.height = 800
info.width = 800
info.distortion_model = 'plumb_bob'
info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
info.k = [476.7030836014194, 0.0, 400.5, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 1.0]
info.p = [476.7030836014194, 0.0, 400.5, -33.36921585209936, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 0.0, 1.0, 0.0]
info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
info.binning_x = 0
info.binning_y = 0
info.roi.x_offset = 0
info.roi.y_offset = 0
info.roi.height = 0
info.roi.width = 0
info.roi.do_rectify = False
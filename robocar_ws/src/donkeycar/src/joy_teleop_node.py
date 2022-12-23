#!/usr/bin/env python3
import sys
import rospy
from i2cpwm_board.msg import ServoConfig
from i2cpwm_board.srv import ServosConfig, ServosConfigRequest, StopServos
from i2cpwm_board.msg import ServoArray, Servo
from sensor_msgs.msg import Joy

class JoyToServoPublisher():

    def __init__(self):
        servo_config = rospy.get_param("servos")
        rospy.loginfo(servo_config)
        self.config_servos(servo_config)

        #rospy.wait_for_service('stop_servos')
        #self.stop = rospy.ServiceProxy('stop_servos', StopServos)

        self.steering = 0
        self.steering_servo = servo_config[1]['servo']
        self.steering_axis = rospy.get_param("axis_angular")

        self.throttle = 0
        self.throttle_servo = servo_config[0]['servo']
        self.throttle_axis = rospy.get_param("axis_linear")

        self.offset = 0.1

        self.stop_button = rospy.get_param("stop_button")

        self.joy_sub = rospy.Subscriber(
             'joy',
             Joy,
             self.joy_callback,
             queue_size=1)

        self.servo_pub = rospy.Publisher(
             'servo_topic',
             ServoArray,
             queue_size=1)

     #   rospy.on_shutdown(self.stop)

        rospy.spin()

    def config_servos(self, config):
        rospy.wait_for_service('config_servos')
        try:
            configure = rospy.ServiceProxy('config_servos', ServosConfig)
            msg = ServosConfigRequest()
            for servo in config:
                serv = ServoConfig()
                serv.servo = servo['servo']
                serv.center = servo['center']
                serv.range = servo['range']
                serv.direction = servo['direction']
                msg.servos.append(serv)
            configure(msg)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def joy_callback(self, data):
        if (data.buttons[self.stop_button]==1):
            self.stop()
        else:
            msg = ServoArray()

            cur_throttle = round(data.axes[self.throttle_axis], 2)
          #  rospy.loginfo(cur_throttle)
            if (abs(cur_throttle-self.throttle) > self.offset):
                self.throttle = cur_throttle
                serv = Servo()
                serv.servo = self.throttle_servo
                serv.value = cur_throttle
                msg.servos.append(serv)

            cur_steer = round(data.axes[self.steering_axis], 2)
            if (abs(cur_steer-self.steering) > self.offset):
                self.steering = cur_steer
                serv = Servo()
                serv.servo = self.steering_servo
                serv.value = cur_steer
                msg.servos.append(serv)
            if (len(msg.servos)>0):
                self.servo_pub.publish(msg)

    def stop(self):
        """
        Turn off servos when shutting down
        """
        #rospy.loginfo("hahah")
        rospy.wait_for_service('stop_servos')
        try:
            stop = rospy.ServiceProxy('stop_servos', StopServos)
            stop()
            rospy.loginfo("stopped")
            rospy.signal_shutdown("stopped motors")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

def main(args):
    rospy.init_node('joy_teleop_node', anonymous=True, log_level=rospy.INFO)
    node = JoyToServoPublisher()

    try:
        print("running joy_teleop_node node")
    except KeyboardInterrupt:
        print("Shutting down ROS joy_teleop_node node")

if __name__ == '__main__':
    main(sys.argv)

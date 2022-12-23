#!/usr/bin/env python3
import sys
import rospy
# from i2cpwm_board.msg import ServoArray, Servo
# from sensor_msgs.msg import Joy

class JoyToServoPublisher():

    def __init__(self):
        servo_config = rospy.get_param("servos")
        rospy.loginfo(servo_config)
        
        self.steering = 0
        self.steering_servo = servo_config[1]['servo']
        self.steering_axis = rospy.get_param("axis_angular")

        self.throttle = 0
        self.throttle_servo = servo_config[0]['servo']
        self.throttle_axis = rospy.get_param("axis_linear")

        self.offset = 0.03

        # self.joy_sub = rospy.Subscriber(
        #     rospy.get_param('~joy'),
        #     Joy,
        #     self.joy_callback,
        #     queue_size=1)
       
        # self.servo_pub = rospy.Publisher(
        #     rospy.get_param('~servo_topic'),
        #     ServoArray,
        #     queue_size=1)

        rospy.spin()

    def config_servos(self, config):
        rospy.wait_for_service('config_servos')
        try:
            configure = rospy.ServiceProxy('config_servos', config_servos)
            configure(config)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # def joy_callback(self, data):
    #     msg = ServoArray()

    #     if (abs(data.axes[self.throttle_axis]-self.throttle) > self.offset):
    #         self.throttle = data.axes[self.throttle_axis.axis]
    #         serv = Servo()
    #         serv.servo = self.throttle_servo
    #         serv.value = self.throttle
    #         msg.servos.append(serv)
        
    #     if (abs(data.axes[self.steering_axis]-self.steering) > self.offset):
    #         self.steering = data.axes[self.steering_axis]
    #         serv = Servo()
    #         serv.servo = self.steering_servo
    #         serv.value = self.steering
    #         msg.servos.append(serv)
 
    #     self.servo_pub.publish(msg)

def main(args):
    rospy.init_node('joy_teleop_node', anonymous=True, log_level=rospy.INFO)
    node = JoyToServoPublisher()

    try:
        print("running joy_teleop_node node")
    except KeyboardInterrupt:
        print("Shutting down ROS joy_teleop_node node")

if __name__ == '__main__':
    main(sys.argv)
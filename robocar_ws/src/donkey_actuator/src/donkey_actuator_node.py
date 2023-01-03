#!/usr/bin/env python3
import sys
import rospy
from i2cpwm_board.msg import ServoConfig
from i2cpwm_board.srv import ServosConfig, ServosConfigRequest
from i2cpwm_board.msg import ServoArray, Servo
from geometry_msgs.msg import Twist

class DonkeyActuator():

    def __init__(self):
        servo_config = rospy.get_param("servos")
        self.config_servos(servo_config)

        self.steering = 0
        self.steering_servo = servo_config[1]['servo']

        self.throttle = 0
        self.throttle_servo = servo_config[0]['servo']
        # init car with zero values
        self.publish_servo(self.throttle, self.steering)

        self.offset = 0.1

        # self.stop_button = rospy.get_param("stop_button")

        self.twist_sub = rospy.Subscriber(
             'donkey/drive',
             Twist,
             self.twist_callback,
             queue_size=1)

        self.servo_pub = rospy.Publisher(
             'servo_topic',
             ServoArray,
             queue_size=1)

        rospy.spin()

    def config_servos(self, config):
        """Call to `config_servos` service for use of proportional servo controller

        Args:
            config: yaml configuration loaded on params server

        """        
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

    def twist_callback(self, data):
        """Callback on each change in twist message.
        if throttle or steering are not the same as previous - publish it

        Args:
            data (geometry_msgs.Twist): twist message

        """            
        cur_throttle = round(data.linear.x, 2)
        cur_steer = round(data.angular.z, 2)

        self.publish_servo(cur_throttle, cur_steer)           

    def publish_servo(self, cur_throttle, cur_steer):
        """Publish servo or/and throttle as i2cpwm_board.ServoArray message

        Args:
            cur_throttle (float): current throttle
            cur_steer (float): current steering

        """    
        msg = ServoArray()

        if (abs(cur_throttle-self.throttle) > self.offset):
            self.throttle = cur_throttle
            serv = Servo()
            serv.servo = self.throttle_servo
            serv.value = cur_throttle
            msg.servos.append(serv)

        if (abs(cur_steer-self.steering) > self.offset):
            self.steering = cur_steer
            serv = Servo()
            serv.servo = self.steering_servo
            serv.value = cur_steer
            msg.servos.append(serv)
        if (len(msg.servos)>0):
            self.servo_pub.publish(msg)

def main(args):
    rospy.init_node('donkey_actuator_node', anonymous=True, log_level=rospy.INFO)
    node = DonkeyActuator()

    try:
        print("running donkey_actuator node")
    except KeyboardInterrupt:
        print("Shutting down ROS donkey_actuator node")

if __name__ == '__main__':
    main(sys.argv)

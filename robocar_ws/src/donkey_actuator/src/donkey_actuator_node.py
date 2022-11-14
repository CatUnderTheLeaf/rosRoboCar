#!/usr/bin/env python

# The MIT License (MIT)

# Copyright (c) 2018 Liam Bowers <liamondrop@gmail.com> (https://gitub.com/liamondrop)

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import Adafruit_PCA9685
import rospy
# import donkey_msgs.msg
import geometry_msgs.msg


def map_value_to_pwm_(servo, value):
    if value < -1.0001 or value > 1.0001:
        rospy.logerr('({}) value must be between -1.0 and 1.0'.format(value))
        return 0
    pulse = servo['direction'] * 0.5 * servo['range'] * value + servo['center']
    return int(pulse)


class ActuatorException(Exception):
    pass


class Actuator:
    def __init__(self):
        if not rospy.has_param('servos'):
            raise ActuatorException('Servos not configured')

        self.servos = rospy.get_param('servos')
        self.controller = Adafruit_PCA9685.PCA9685()
        self.controller.set_pwm_freq(self.servos.get('pwm_frequency', 50))

        # send center pulse to throttle servo to calibrate ESC
        self.set_servo_center_(self.servos['throttle'])
        rospy.sleep(1)

        # initiate subscribers
        # rospy.Subscriber('donkey/servo_pulse', donkey_msgs.msg.ServoPulse,
        #                  self.servo_pulse_cb_)
        rospy.Subscriber('donkey/drive', geometry_msgs.msg.Twist,
                         self.drive_cb_)

    # def servo_pulse_cb_(self, msg):
    #     """
    #     Callback function for the donkey/servo_pulse topic. Intended as a utility
    #     to help properly configure a servo's range of pulse values.
    #     The following are an example of finding the steering servo's center, i.e. 333:
    #     $ rostopic pub /donkey/servo_pulse donkey_msgs/ServoPulse "{name: steering, value: 300}"
    #     $ rostopic pub /donkey/servo_pulse donkey_msgs/ServoPulse "{name: steering, value: 350}"
    #     $ rostopic pub /donkey/servo_pulse donkey_msgs/ServoPulse "{name: steering, value: 330}"
    #     $ rostopic pub /donkey/servo_pulse donkey_msgs/ServoPulse "{name: steering, value: 335}"
    #     $ rostopic pub /donkey/servo_pulse donkey_msgs/ServoPulse "{name: steering, value: 333}"
    #     """
    #     servo = self.servos[msg.name]
    #     self.set_servo_pulse_(servo, int(msg.value))
    #     rospy.loginfo('servo: {}, channel: {}, value: {}'.format(
    #         msg.name, servo['channel'], int(msg.value)))

    def drive_cb_(self, msg):
        """
        Callback function for the donkey/drive topic
        Sets the values for the steering and throttle servos using s standard
        geometry_msgs/Twist message. The linear.x component controls the
        throttle, and the angular.z component controls the steering.
        The following is an example of a command to drive straight forward
        at 75% throttle:
        $ rostopic pub /donkey/drive geometry_msgs/Twist "{linear: {x: 0.75}, angular: {z: 0.0}}"
        """
        self.set_servo_proportional_(self.servos['steering'], msg.angular.z)
        rospy.loginfo('servo: steering, value: {}'.format(msg.angular.z))
        self.set_servo_proportional_(self.servos['throttle'], msg.linear.x)
        rospy.loginfo('servo: throttle, value: {}'.format(msg.linear.x))

    def set_servo_center_(self, servo):
        self.set_servo_pulse_(servo, servo['center'])

    def set_servo_pulse_(self, servo, value):
        self.controller.set_pwm(servo['channel'], 0, value)
        rospy.logdebug('channel: {}, value: {}'.format(servo['channel'], value))

    def set_servo_proportional_(self, servo, value):
        pulse = map_value_to_pwm_(servo, value)
        self.set_servo_pulse_(servo, pulse)

    def __del__(self):
        """
        Turn off servos when shutting down
        """
        self.set_servo_pulse_(self.servos['steering'], 0)
        self.set_servo_pulse_(self.servos['throttle'], 0)

if __name__ == '__main__':
    try:
        rospy.init_node('donkey_actuator', log_level=rospy.INFO)
        a = Actuator()
        rospy.spin()
    except ActuatorException as e:
        rospy.logfatal('{}. Shutting down actuator node'.format(e))
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

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

# import Adafruit_PCA9685

# import rospy
# import geometry_msgs.msg


# def map_value_to_pwm_(servo, value):
#     if value < -1.0001 or value > 1.0001:
#         rospy.logerr('({}) value must be between -1.0 and 1.0'.format(value))
#         return 0
#     pulse = servo['direction'] * 0.5 * servo['range'] * value + servo['center']
#     return int(pulse)


# class ActuatorException(Exception):
#     pass


# class Actuator:
#     def __init__(self):
#         if not rospy.has_param('servos'):
#             raise ActuatorException('Servos not configured')

#         self.servos = rospy.get_param('servos')
#         self.controller = Adafruit_PCA9685.PCA9685(address=0x40)
#         self.controller.set_pwm_freq(self.servos.get('pwm_frequency', 60))

#         # send center pulse to throttle servo to calibrate ESC
#         self.set_servo_center_(self.servos['throttle'])
#         rospy.sleep(1)

#         # initiate subscriber
#         rospy.Subscriber('donkey/drive', geometry_msgs.msg.Twist,
#                          self.drive_cb_)
    
#     def drive_cb_(self, msg):
#         """
#         Callback function for the donkey/drive topic
#         Sets the values for the steering and throttle servos using s standard
#         geometry_msgs/Twist message. The linear.x component controls the
#         throttle, and the angular.z component controls the steering.
#         The following is an example of a command to drive straight forward
#         at 75% throttle:
#         $ rostopic pub /donkey/drive geometry_msgs/Twist "{linear: {x: 0.75}, angular: {z: 0.0}}"
#         """
#         self.set_servo_proportional_(self.servos['steering'], msg.angular.z)
#         rospy.loginfo('servo: steering, value: {}'.format(msg.angular.z))
#         self.set_servo_proportional_(self.servos['throttle'], msg.linear.x)
#         rospy.loginfo('servo: throttle, value: {}'.format(msg.linear.x))

#     def set_servo_center_(self, servo):
#         self.set_servo_pulse_(servo, servo['center'])

#     def set_servo_pulse_(self, servo, value):
#         self.controller.set_pwm(servo['channel'], 0, value)
#         rospy.logdebug('channel: {}, value: {}'.format(servo['channel'], value))

#     def set_servo_proportional_(self, servo, value):
#         pulse = map_value_to_pwm_(servo, value)
#         self.set_servo_pulse_(servo, pulse)

#     def __del__(self):
#         """
#         Turn off servos when shutting down
#         """
#         self.set_servo_pulse_(self.servos['steering'], 0)
#         self.set_servo_pulse_(self.servos['throttle'], 0)

# if __name__ == '__main__':
#     try:
#         rospy.init_node('donkey_actuator_node', log_level=rospy.DEBUG)
#         a = Actuator()
#         rospy.spin()
#     except ActuatorException as e:
#         rospy.logfatal('{}. Shutting down actuator node'.format(e))
#     except rospy.ROSInterruptException:
#         pass



"""
Code written by Tiziano Fiorenzani
https://github.com/tizianofiorenzani/ros_tutorials.git

I have made minor changes in class/node names

Class for low level control of our car. It assumes ros-12cpwmboard has been
installed
"""
import rospy
from i2cpwm_board.msg import Servo, ServoArray
from geometry_msgs.msg import Twist
import time


class ServoConvert():
    def __init__(self, id=1, center_value=333, range=90, direction=1):
        self.value      = 0.0
        self.value_out  = center_value
        self._center    = center_value
        self._range     = range
        self._half_range= 0.5*range
        self._dir       = direction
        self.id         = id

        #--- Convert its range in [-1, 1]
        self._sf        = 1.0/self._half_range

    def get_value_out(self, value_in):
        #--- value is in [-1, 1]
        self.value      = value_in
        self.value_out  = int(self._dir*value_in*self._half_range + self._center)
        print(self.id, self.value_out)
        return(self.value_out)

class DkLowLevelCtrl():
    def __init__(self):
        # rospy.loginfo("Setting Up the Node...")

        # rospy.init_node('dk_llc')

        self.actuators = {}
        self.actuators['throttle']  = ServoConvert(id=1)
        self.actuators['steering']  = ServoConvert(id=2, direction=1) #-- positive left
        rospy.loginfo("> Actuators corrrectly initialized")

        self._servo_msg       = ServoArray()
        for i in range(2): self._servo_msg.servos.append(Servo())

        #--- Create the servo array publisher
        self.ros_pub_servo_array    = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        rospy.loginfo("> Publisher corrrectly initialized")

        #--- Create the Subscriber to Twist commands
        self.ros_sub_twist          = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
        rospy.loginfo("> Subscriber corrrectly initialized")

        #--- Get the last time e got a commands
        self._last_time_cmd_rcv     = time.time()
        self._timeout_s             = 5

        rospy.loginfo("Initialization complete")

    def set_actuators_from_cmdvel(self, message):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        #-- Save the time
        self._last_time_cmd_rcv = time.time()

        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(message.linear.x)
        self.actuators['steering'].get_value_out(message.angular.z)
        rospy.loginfo("Got a command v = %2.1f  s = %2.1f"%(message.linear.x, message.angular.z))
        self.send_servo_msg()

    def set_actuators_idle(self):
        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(0)
        self.actuators['steering'].get_value_out(0)
        rospy.loginfo("Setting actutors to idle")
        self.send_servo_msg()

    def send_servo_msg(self):
        for actuator_name, servo_obj in self.actuators.items():
            self._servo_msg.servos[servo_obj.id-1].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id-1].value = servo_obj.value_out
            rospy.loginfo("Sending to %s command %d"%(actuator_name, servo_obj.value_out))

        self.ros_pub_servo_array.publish(self._servo_msg)

    @property
    def is_controller_connected(self):
        print(time.time() - self._last_time_cmd_rcv)
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def run(self):

        #--- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            print(self._last_time_cmd_rcv, self.is_controller_connected)
            if not self.is_controller_connected:
                self.set_actuators_idle()

            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('donkey_actuator_node', log_level=rospy.DEBUG)
        dk_llc     = DkLowLevelCtrl()
        dk_llc.run()
#         rospy.spin()
    except rospy.ROSInterruptException:
        pass

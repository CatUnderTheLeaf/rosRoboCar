#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from pid import PID

import math
import sys

class Controller():

    def __init__(self):
        self.pid = PID(rospy.get_param('~kp'), rospy.get_param('~ki'), rospy.get_param('~kd'), mn=-1.0, mx=1.0)
        self.velocity = rospy.get_param('~velocity')
        self.last_time = rospy.Time.now()
        self.cmd_pub = rospy.Publisher(
            'donkey/drive', 
            Twist, 
            queue_size=1)
        
        self.img_waypoints_sub = rospy.Subscriber(
            rospy.get_param('~path_waypoints'),
            Path,
            self.path_waypoints_callback,
            queue_size=1)

        rospy.spin()  
    
    def path_waypoints_callback(self, msg):
        
        heading_angle = self.get_heading_angle(msg.poses[0].pose.position)

        vel_msg = Twist()
        vel_msg.linear.x = self.velocity
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        # global last_time
        current_time = rospy.Time.now()
        dt = current_time.to_time() - self.last_time.to_time()
        vel_msg.angular.z = self.pid.calculate(heading_angle, dt)
        rospy.logdebug("vel_msg.angular.z is ({})".format(vel_msg.angular.z))  
        self.last_time = rospy.Time.now()

        # Publishing our vel_msg
        self.cmd_pub.publish(vel_msg)

    def get_heading_angle(self, point):
        # as point lays on the same plane as robot frame
        # we can ignore z value
        return math.atan2(point.y, point.x)

def main(args):
    rospy.init_node('controller_node', anonymous=True, log_level=rospy.INFO)
    node = Controller()

    try:
        print("running controller_node")
    except KeyboardInterrupt:
        print("Shutting down ROS controller_node")

if __name__ == '__main__':
    main(sys.argv)
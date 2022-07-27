#!/usr/bin/env python
import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import Point#, TwistStamped
# from ackermann_msgs.msg import AckermannDriveStamped

# Node for publishing markers for RVIZ
class MarkerPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        # publish markers for steering angle
        # self.publish_steering()
        # publish markers for target angular velocity
        # self.publish_angular_velocity()
        # publish lane waypoints
        self.publish_waypoints()

    # def publish_steering(self):
    #     self.ack_pub = rospy.Publisher('/visualize/steering', Marker, queue_size=1)
    #     self.ack_sub = rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.ackermann_cb)        

    # def ackermann_cb(self, msg):
    #     steering = msg.drive.steering_angle
    #     # publish red arrow
    #     self.publishMarker(self.ack_pub, marker_type = "arrow", angle=steering, color=(1,0,0))

    # def publish_angular_velocity(self):
    #     self.ang_pub = rospy.Publisher('/visualize/target_angular', Marker, queue_size=1)
    #     self.ang_sub = rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)

    # def twist_cb(self, msg):
    #     angular_vel = msg.twist.angular.z
    #     # publish blue arrow
    #     self.publishMarker(self.ang_pub, marker_type="arrow", angle=angular_vel, color=(0,0,1))

    def publish_waypoints(self):
        self.declare_parameter('path_waypoints', '/path/path_waypoints')
        self.declare_parameter('viz_waypoints', '/visualize/waypoints')
        
        self.path_sub = self.create_subscription(
            Path,
            self.get_parameter('path_waypoints').get_parameter_value().string_value,
            self.waypoint_cb,
            1)
        self.path_sub
        self.marker_pub = self.create_publisher(
            Marker,
            self.get_parameter('viz_waypoints').get_parameter_value().string_value,
            1)
        self.marker_pub

    def waypoint_cb(self, msg):
        waypoints = msg.poses
        points = [x.pose.position for x in waypoints]
        self.publishMarker(self.waypoint_pub, marker_type="line", points=points, color=(0,1,0), frame=msg.header.frame_id)

    def publishMarker(self, pub, marker_type, color, angle=0, points=None, frame="base_link"):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "/"
        marker.id = 0        
        marker.action = Marker.ADD        
        if (marker_type == "arrow"):
            marker.type = Marker.ARROW
            marker.points.append(Point(0, 0, 0))
            marker.points.append(Point(math.cos(angle), math.sin(angle), 0))
        else:
            marker.type = Marker.LINE_STRIP
            marker.points.extend(points)
        marker.scale.x = 0.02
        marker.scale.y = 0.05
        marker.scale.z = 0.1
        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()

    rclpy.spin(marker_publisher)

    marker_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
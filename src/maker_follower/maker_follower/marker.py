#!/usr/bin/env python

import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher_node')
        self.marker_pub = self.create_publisher(Marker, 'marker_topic', 10)
        self.tf_sub = self.create_subscription(TransformStamped, 'tf_topic', self.tf_callback, 10)
        self.tf_sub  # prevent unused variable warning
        self.marker = Marker()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    def tf_callback(self, msg):
        try:
            # Get the latest transform between the frame you want to follow and the fixed frame
            transform = self.tf_buffer.lookup_transform('fixed_frame', '/hotend_link', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        self.marker.header.frame_id = 'fixed_frame'  # Set the fixed frame ID for the marker
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose = transform.transform
        self.marker.scale.x = 0.2  # Set the scale of the marker
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.a = 1.0  # Set the color and transparency of the marker
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

        self.marker_pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    rclpy.spin(marker_publisher)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

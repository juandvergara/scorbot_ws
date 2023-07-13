import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'marker_topic', 10)
        self.marker_timer_ = self.create_timer(0.08, self.publish_marker)
        self.marker_msg = Marker()
        self.config_marker()
        self.init_tf_listener()

    def init_tf_listener(self):
        self.first_name_ = "hotend_link"
        self.second_name_ = "world"
        self.get_logger().info("Transforming from {} to {}".format(self.second_name_, self.first_name_))
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def config_marker(self):
        self.marker_msg.header.frame_id = 'world'
        self.marker_msg.type = Marker.LINE_STRIP
        self.marker_msg.action = Marker.ADD
        self.marker_msg.pose.position.x = 0.0
        self.marker_msg.pose.position.y = 0.0
        self.marker_msg.pose.position.z = 0.0  # Add 0.01 to the Z-axis
        self.marker_msg.pose.orientation.w = 1.0
        self.marker_msg.scale.x = 0.0004  
        self.marker_msg.color.a = 0.5
        self.marker_msg.color.g = 1.0

    def publish_marker(self):
        try:
            trans = self._tf_buffer.lookup_transform(self.second_name_, self.first_name_, rclpy.time.Time())
            point = Point()
            point.x = trans.transform.translation.x
            point.y = trans.transform.translation.y
            point.z = trans.transform.translation.z

            self.marker_msg.points.append(point)

            self.publisher_.publish(self.marker_msg)
            self.get_logger().info('Marker published')

        except LookupException as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    rclpy.spin(marker_publisher)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

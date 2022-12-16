import time
from custom_interfaces.srv import JointTarget # type: ignore
import rclpy
from math import pi, cos, sin, radians
import serial

from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

class RobotControlNode(Node):

    def __init__(self):
        super().__init__('rpi_robot_control_node')  # type: ignore

        self.declare_parameter('pico_port', '/dev/ttyACM0')

        self.joint_states_target_publisher = self.create_publisher(
            String,
            'joint_states_target',
            10
        )
        self.srv = self.create_service(JointTarget, 'joint_target', self.joint_target_callback)

        time.sleep(0.2)
        self.port = self.get_parameter('pico_port').get_parameter_value().string_value
        self.ser = serial.Serial(self.port)
        self.get_logger().info(f'Using serial port {self.ser.name}')

        self.joint_state_target = String()
        
    def joint_target_callback(self, request, response):
        data_send = request.data + "\n"
        self.joint_state_target.data = data_send
        self.joint_states_target_publisher.publish(self.joint_state_target)
        self.get_logger().info('Sending serial data\n')
        print(data_send)
        command = data_send.encode('utf-8')
        self.get_logger().info(f'Sending command: {command}')
        self.ser.write(command)
        return response


def main(args=None):
    rclpy.init(args=args)
    robot_control_node = RobotControlNode()
    rclpy.spin(robot_control_node)

    robot_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

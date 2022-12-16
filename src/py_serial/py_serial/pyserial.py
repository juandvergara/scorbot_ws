import time

import rclpy
from math import pi, cos, sin, radians
import serial

from rclpy.node import Node
from sensor_msgs.msg import JointState


class RobotFeedbackNode(Node):

    def __init__(self):
        super().__init__('rpi_robot_feedback_node')  # type: ignore

        self.declare_parameter('pico_port', '/dev/ttyACM1')

        self.joint_states_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        time.sleep(0.2)
        self.port = self.get_parameter(
            'pico_port').get_parameter_value().string_value
        self.ser = serial.Serial(self.port)
        self.get_logger().info(f'Using serial port {self.ser.name}')
        # set timer
        self.pub_period = 0.001  # 0.02 seconds = 50 hz = pid rate for robot
        self.pub_timer = self.create_timer(
            self.pub_period, self.timer_callback)
        self.joint_state_position = JointState()

    def timer_callback(self):
        while self.ser.in_waiting == 0:
            pass

        res = self.ser.read(self.ser.in_waiting).decode('UTF-8')
        self.get_logger().debug(f'data: "{res}", bytes: {len(res)}')

        if res == '0' or len(res) < 30 or len(res) > (34 + 13):
            self.get_logger().warn(f'Bad data: "{res}", {len(res)}')
            return None

        raw_list = res.split(',')
        base_position = radians(float(raw_list[0]))
        body_position = radians(-float(raw_list[1]))
        shoulder_position = radians(float(raw_list[2]))
        elbow_position = radians(-float(raw_list[3]) - float(raw_list[2]))
        wrist_position = -(radians(float(raw_list[4])) - radians(
            float(raw_list[5])))/2 + radians(float(raw_list[3]))
        wrist_yaw = (
            radians(float(raw_list[4])) + radians(float(raw_list[5])))/2

        self.joint_state_position.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_position.name = [
            "slide_base_joint", "body_joint", "shoulder_joint", "elbow_joint", "wrist_joint", "roll_wrist_joint"]
        self.joint_state_position.position = [base_position, body_position, shoulder_position, elbow_position,
                                              wrist_position, wrist_yaw]
        self.joint_states_publisher.publish(self.joint_state_position)
        self.get_logger().warn(f'Receiving: "{raw_list[1]}", "{raw_list[2]}", "{raw_list[3]}", "{raw_list[4]}", "{raw_list[5]}"')


def main(args=None):
    rclpy.init(args=args)
    robot_feedback_node = RobotFeedbackNode()
    rclpy.spin(robot_feedback_node)

    robot_feedback_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

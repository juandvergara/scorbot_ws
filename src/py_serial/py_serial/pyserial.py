import time
from custom_interfaces.srv import JointTarget # type: ignore
import rclpy
from math import pi, cos, sin, radians
import serial

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class RobotFeedbackNode(Node):

    def __init__(self):
        super().__init__('rpi_robot_feedback_node')  # type: ignore

        self.declare_parameter('pico_port_slave', '/dev/ttyACM1')
        self.declare_parameter('pico_port_master', '/dev/ttyACM0')

        self.joint_states_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        self.joint_states_target_publisher = self.create_publisher(
            String,
            'joint_states_target',
            10
        )
        self.srv = self.create_service(JointTarget, 'joint_target', self.joint_target_callback)

        time.sleep(0.2)
        self.port_slave = self.get_parameter(
            'pico_port_slave').get_parameter_value().string_value
        self.port_master = self.get_parameter(
            'pico_port_master').get_parameter_value().string_value   

        self.serial_slave = serial.Serial(self.port_slave, baudrate=115200)
        self.serial_master = serial.Serial(self.port_master, baudrate=115200)

        self.get_logger().info(f'Using serial master port {self.serial_master.name}')
        self.get_logger().info(f'Using serial slave port {self.serial_slave.name}')
        # set timer
        self.pub_period = 0.05  # 0.02 seconds = 50 hz = pid rate for robot
        self.pub_timer = self.create_timer(
            self.pub_period, self.timer_callback)
        self.joint_state_position = JointState()
        self.joint_state_target = String()

    def timer_callback(self):
        data_send = "e\n"
        command = data_send.encode('utf-8')
        self.get_logger().info(f'Sending command: {command}')
        self.serial_master.write(command)

        while self.serial_slave.in_waiting == 0:
            pass

        res = self.serial_slave.read(self.serial_slave.in_waiting).decode('UTF-8')
        self.get_logger().debug(f'data: "{res}", bytes: {len(res)}')

        if res == '0' or len(res) < 30 or len(res) > (38 + 13):
            self.get_logger().warn(f'Bad data: "{res}", {len(res)}')
            return None

        raw_list = res.split(',')
        base_position = radians(float(raw_list[0]))
        body_position = radians(float(raw_list[1]))
        shoulder_position = radians(float(raw_list[2]))
        elbow_position = radians(float(raw_list[3]))
        wrist_position = radians(float(raw_list[4]))
        wrist_yaw = radians(float(raw_list[5]))

        self.joint_state_position.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_position.name = [
            "slide_base_joint", "body_joint", "shoulder_joint", "elbow_joint", "wrist_joint", "roll_wrist_joint"]
        self.joint_state_position.position = [base_position, body_position, shoulder_position, elbow_position,
                                              wrist_position, wrist_yaw]
        self.joint_states_publisher.publish(self.joint_state_position)
        self.get_logger().warn(f'Receiving: "{raw_list[1]}", "{raw_list[2]}", "{raw_list[3]}", "{raw_list[4]}", "{raw_list[5]}"')


    def joint_target_callback(self, request, response):
        data_send = request.data + "\n"
        self.joint_state_target.data = data_send
        self.joint_states_target_publisher.publish(self.joint_state_target)
        self.get_logger().info('Sending serial data\n')
        print(data_send)
        command = data_send.encode('utf-8')
        self.get_logger().info(f'Sending command: {command}')
        self.serial_master.write(command)
        return response


def main(args=None):
    rclpy.init(args=args)
    robot_feedback_node = RobotFeedbackNode()
    rclpy.spin(robot_feedback_node)

    robot_feedback_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

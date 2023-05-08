import time
from custom_interfaces.srv import JointTarget  # type: ignore
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
        self.srv = self.create_service(
            JointTarget, 'joint_target', self.joint_target_callback)

        time.sleep(0.2)
        self.port_slave = self.get_parameter(
            'pico_port_slave').get_parameter_value().string_value
        self.port_master = self.get_parameter(
            'pico_port_master').get_parameter_value().string_value

        self.serial_slave = serial.Serial(self.port_slave, baudrate=115200)
        self.serial_master = serial.Serial(self.port_master, baudrate=115200)

        self.get_logger().info(
            f'Using serial master port {self.serial_master.name}')
        self.get_logger().info(
            f'Using serial slave port {self.serial_slave.name}')
        # set timer
        self.pub_period = 0.05  # 0.02 seconds = 50 hz = pid rate for robot
        self.pub_timer = self.create_timer(
            self.pub_period, self.timer_callback)
        self.joint_state_position = JointState()
        self.joint_state_target = String()

    def timer_callback(self):
        data_send = "e\n"
        command = data_send.encode('utf-8')
        # self.get_logger().info(f'Sending command to controllers: {command}')

        self.serial_master.write(command)
        while self.serial_master.in_waiting == 0:
            pass
        res_master = self.serial_master.read(
            self.serial_master.in_waiting).decode('UTF-8')
        # self.get_logger().debug(f'data: "{res_master}", bytes: {len(res_master)}')

        self.serial_slave.write(command)
        while self.serial_slave.in_waiting == 0:
            pass
        res_slave = self.serial_slave.read(
            self.serial_slave.in_waiting).decode('UTF-8')
        # self.get_logger().debug(f'data: "{res_slave}", bytes: {len(res_slave)}')

        # if res == '0' or len(res) < 30 or len(res) > (38 + 13):
        #     self.get_logger().warn(f'Bad data: "{res}", {len(res)}')
        #     return None

        raw_list_master = res_master.split(',')
        raw_list_slave = res_slave.split(',')
        base_position = float(raw_list_master[0]) / 100.0
        body_position = -radians(float(raw_list_master[1]))
        shoulder_position = radians(float(raw_list_master[2]))
        elbow_position = -radians(float(raw_list_slave[0])) - shoulder_position
        wrist_position = radians(-float(raw_list_slave[1]) + float(raw_list_slave[2])) / 2.0 + radians(float(raw_list_slave[0]))
        wrist_yaw = radians(float(raw_list_slave[1]) + float(raw_list_slave[2])) / 2.0

        self.joint_state_position.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_position.name = [
            "slide_base_joint", "body_joint", "shoulder_joint", "elbow_joint", "wrist_joint", "roll_wrist_joint", "extruder_screw_joint"]
        self.joint_state_position.position = [base_position, body_position, shoulder_position, elbow_position,
                                              wrist_position, wrist_yaw, 0.0]
        self.joint_states_publisher.publish(self.joint_state_position)
        # self.get_logger().warn(f'Receiving: "{raw_list_master[1]}", "{raw_list_master[2]}", "{raw_list_slave[0]}", "{raw_list_slave[1]}", "{raw_list_slave[2]}"')

    def joint_target_callback(self, request, response):
        data_send = request.data
        self.joint_state_target.data = data_send
        self.joint_states_target_publisher.publish(self.joint_state_target)
        self.get_logger().info(f'Sending serial data {data_send}\n')
        data_to_control = data_send.split(',')

        body = -float(data_to_control[1])
        elbow = -float(data_to_control[2]) - float(data_to_control[3])
        wrist_left = float(data_to_control[5]) - float(data_to_control[4]) + elbow
        wrist_right = float(data_to_control[4]) + float(data_to_control[5]) - elbow

        data_master = "p " + data_to_control[0] + "," + str(body) + "," + data_to_control[2] + "\n"
        data_slave = "p " + str(elbow) + "," + str(wrist_left) + "," + str(wrist_right) + "\n"

        command_master = data_master.encode('utf-8')
        time.sleep(0.01)
        command_slave = data_slave.encode('utf-8')
        self.serial_master.write(command_master)
        self.serial_slave.write(command_slave)

        # self.get_logger().info(f'Sending command to master: {command_master}')
        # self.get_logger().info(f'Sending command to slave: {command_slave}')
        return response


def main(args=None):
    rclpy.init(args=args)
    robot_feedback_node = RobotFeedbackNode()
    rclpy.spin(robot_feedback_node)

    robot_feedback_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import time
from custom_interfaces.srv import JointTarget  # type: ignore
import rclpy
from builtin_interfaces.msg import Duration

from math import cos, sin, atan2, sqrt, pi, radians
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

x_real = 0.200
y_real = 0.200
z_real = 0.200

position = [x_real, y_real, z_real]
rotation = [0, 0, 0]


def inverse_kinematics_scorbot():
    global position, rotation
    rotation = [radians(rotation[0]), radians(
        rotation[1]), radians(rotation[2])]
    position_t = np.array([[position[0]],
                           [position[1]],
                           [position[2]]])

    scale_perception = np.array([0, 0, 0, 1])

    Rx = np.array([[1, 0, 0],
                   [0, cos(rotation[0]), -sin(rotation[0])],
                   [0, sin(rotation[0]), cos(rotation[0])]])

    Ry = np.array([[cos(rotation[1]), 0, sin(rotation[1])],
                   [0, 1, 0],
                   [-sin(rotation[1]), 0, cos(rotation[1])]])

    Rz = np.array([[cos(rotation[2]), -sin(rotation[2]), 0],
                   [sin(rotation[2]), cos(rotation[2]), 0],
                   [0, 0, 1]])

    R = (Rz.dot(Ry)).dot(Rx)
    T = np.vstack((np.hstack((R, position_t)), scale_perception))

    l_1 = 0.450
    l_2 = 0.220
    l_3 = 0.220
    a_2 = 0.025

    nx = R[0, 0]
    ny = R[1, 0]
    sx = R[0, 1]
    sy = R[1, 1]
    ax = R[0, 2]
    ay = R[1, 2]
    az = R[2, 2]

    if position[1] < 0.4:
        banda = position[1] + 0.1
    else:
        banda = 0.100 * ((position[1] - 0.300) // 0.100) + 0.05

    offset_banda = 0.17

    if position[1] < 0:
        banda = 0
    elif banda > 0.95:
        banda = 0.95

    d_1 = offset_banda + banda

    x = position[0]
    y = position[1] - d_1
    z = position[2] - l_1

    theta_1 = atan2(-x, y)
    theta_5 = atan2(nx*cos(theta_1) + ny*sin(theta_1),
                    sx*cos(theta_1) + sy*sin(theta_1))

    w = sqrt(x**2 + y**2) - a_2
    c3 = (z**2 + w**2 - l_2**2 - l_3**2)/(2*l_2*l_3)

    if c3 > 1:
        print("\n OUT OF WORKSPACE ROBOT! \n")
        return -1.0, -1.0, -1.0, -1.0, -1.0, -1.0
    else:
        s3 = -sqrt(1-c3**2)

    theta_3 = atan2(s3, c3)

    k1 = l_2 + l_3*c3
    k2 = l_3*s3

    theta_2 = atan2(z, w) - atan2(k2, k1)

    theta_4 = atan2(az, -ax*sin(theta_1) + ay*cos(theta_1)) - \
        (theta_2 + theta_3)

    print("Matrix translation and roatation resultant \n")
    print(T)

    print("\n Value each joint \n")
    print(banda, theta_1, theta_2, theta_3, theta_4, theta_5)
    print("Success pose! \n")

    return float(banda), theta_1, theta_2, theta_3, theta_4, theta_5


class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics_node')  # type: ignore

        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            'arm_controller/joint_trajectory',
            10
        )

        self.srv = self.create_service(
            JointTarget, 'pose_target', self.pose_target_callback)

        time.sleep(0.2)


        self.get_logger().info('Waiting goal pose ...\n')

    def pose_target_callback(self, request, response):
        global position, rotation

        joint_trajectory_msg = JointTrajectory()
        trajectory_point = JointTrajectoryPoint()
        joint_trajectory_msg.joint_names = ["slide_base_joint", "body_joint", "shoulder_joint", "elbow_joint",
                                                 "wrist_joint", "roll_wrist_joint", "extruder_screw_joint"]

        data = (request.data).split(",")
        position = [float(i) for i in data[0:3]]
        rotation = [float(i) for i in data[3:6]]

        base_position, body_position, shoulder_position, elbow_position, wrist_position, wrist_yaw = inverse_kinematics_scorbot()

        if base_position == -1.0:
            self.get_logger().error('\n OUT WORKSPACE ROBOT! \n')
            return response

        joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_point.positions = [base_position, body_position, shoulder_position, elbow_position,
                                           wrist_position, wrist_yaw, 0.0]
        trajectory_point.time_from_start = Duration(sec=5)
        joint_trajectory_msg.points.append(trajectory_point)

        self.joint_trajectory_publisher.publish(joint_trajectory_msg)

        self.get_logger().info('\n Waiting next goal pose ...\n')
        return response


def main(args=None):
    rclpy.init(args=args)
    inverse_kinematics_node = InverseKinematics()

    rclpy.spin(inverse_kinematics_node)
    inverse_kinematics_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

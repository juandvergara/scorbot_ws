import time
from custom_interfaces.srv import JointTarget  # type: ignore
import rclpy
from builtin_interfaces.msg import Duration

from math import cos, sin, atan2, sqrt, pi, radians
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

x_real = 0.200
y_real = 0.200
z_real = 0.200

position = [x_real, y_real, z_real]
rotation = [0, 0, 0]
limits = [(-1000, 100), (-3.15, 2.09), (-0.52, 2.09),
          (-1000, 1000), (-1000, 1000), (-1000, 1000)]

rotation_y = radians(-90)

wrist_roll2hotend = np.array([[cos(rotation_y),  0, sin(rotation_y), -0.049],
                              [0,  1,          0, -0.0137],
                              [-sin(rotation_y), 0, cos(rotation_y), 0.132],
                              [0, 0,       0,           1]])

hotend2wrist_roll = np.linalg.inv(wrist_roll2hotend)


def check_variable_limits(variables, limits):
    for i, var in enumerate(variables):
        if var < limits[i][0] or var > limits[i][1]:
            return True
    return False


def inverse_kinematics_scorbot(position_goal, rotation_goal, wrist):
    global limits, hotend2wrist_roll
    rotation = [radians(rotation_goal[0]),
                radians(rotation_goal[1]),
                radians(rotation_goal[2])]
    position_t = np.array([[position_goal[0]],
                           [position_goal[1]],
                           [position_goal[2]]])

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
    # R = np.matmul(np.matmul(Rz,Ry), Rx)
    T = np.vstack((np.hstack((R, position_t)), scale_perception))
    arm_transform = np.matmul(T, hotend2wrist_roll) if wrist else T
    # arm_transform = T.dot(hotend2wrist_roll) if wrist else T

    link_1 = 0.450
    link_2 = 0.220
    link_3 = 0.220
    a_2 = 0.025

    nx = arm_transform[0, 0]
    ny = arm_transform[1, 0]
    sx = arm_transform[0, 1]
    sy = arm_transform[1, 1]
    ax = arm_transform[0, 2]
    ay = arm_transform[1, 2]
    az = arm_transform[2, 2]

    # if position[1] < 0.4:
    #     slide_base = position[1] + 0.1
    # else:
    #     slide_base = 0.100 * ((position[1] - 0.300) // 0.100) + 0.05

    offset_slide_base = 0.17
    slide_base = 0.0

    # if position[1] < 0:
    #     slide_base = 0
    # elif slide_base > 0.95:
    #     slide_base = 0.95

    d_1 = offset_slide_base + slide_base

    x = arm_transform[0, 3]
    y = arm_transform[1, 3] - d_1
    z = arm_transform[2, 3] - link_1

    theta_1 = atan2(-x, y)
    theta_5 = atan2(nx*cos(theta_1) + ny*sin(theta_1),
                    sx*cos(theta_1) + sy*sin(theta_1))

    w = sqrt(x**2 + y**2) - a_2
    cos_theta_3 = (z**2 + w**2 - link_2**2 - link_3**2)/(2*link_2*link_3)

    if cos_theta_3 > 1:
        print("\n OUT OF WORKSPACE ROBOT! BY THETA3\n")
        return -1.0, -1.0, -1.0, -1.0, -1.0, -1.0
    else:
        sin_theta_3 = -sqrt(1-cos_theta_3**2)

    theta_3 = atan2(sin_theta_3, cos_theta_3)

    k1 = link_2 + link_3*cos_theta_3
    k2 = link_3*sin_theta_3

    theta_2 = atan2(z, w) - atan2(k2, k1)

    theta_4 = atan2(az, -ax*sin(theta_1) + ay*cos(theta_1)) - \
        (theta_2 + theta_3)

    np.set_printoptions(precision=5, suppress=True)
    print("Matrix translation and rotation resultant \n")
    print(arm_transform)

    print("\nMatrix hotend2wrist \n")
    print(hotend2wrist_roll)

    print("\nMatrix desired \n")
    print(T)

    print("X:")
    print(arm_transform[0, 3])

    print("\n Value each joint \n")
    print(f"{slide_base:.6f}, {theta_1:.6f}, {theta_2:.6f}, {theta_3:.6f}, {theta_4:.6f}, {theta_5:.6f}")
    print("Success pose! \n")

    result = [slide_base, theta_1, theta_2, theta_3, theta_4, theta_5]

    # if check_variable_limits(result, limits):
    #     print("\n OUT LIMITS ROBOT! \n")
    #     return -1.0, -1.0, -1.0, -1.0, -1.0, -1.0

    return result

class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics_node')  # type: ignore

        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            # arm_controller/joint_trajectory    /    joint_trajectory_position_controller/joint_trajectory
            'joint_trajectory_position_controller/joint_trajectory',
            10
        )

        self.marker_object_listener = self.create_publisher(
            Marker, '/marker_hotend', 10)

        self.srv = self.create_service(
            JointTarget, 'pose_target', self.pose_target_callback)

        time.sleep(0.2)

        self.marker_object = Marker()
        self.marker_object.header.frame_id = '/hotend_link'
        self.marker_object.id = 0
        self.marker_object.type = Marker.LINE_STRIP
        self.marker_object.action = Marker.ADD

        self.get_logger().info('Waiting goal pose ...\n')
        self.extruder_pos = 0

    def pose_target_callback(self, request, response):
        global position, rotation

        joint_trajectory_msg = JointTrajectory()
        trajectory_point = JointTrajectoryPoint()
        joint_trajectory_msg.joint_names = ["slide_base_joint", "body_joint", "shoulder_joint", "elbow_joint",
                                            "wrist_joint", "roll_wrist_joint", "extruder_screw_joint"]

        data = (request.data).split(",")
        position = [float(i) for i in data[0:3]]
        rotation = [float(i) for i in data[3:6]]
        self.extruder_pos += float(data[6])

        base_position, body_position, shoulder_position, elbow_position, wrist_position, wrist_yaw = inverse_kinematics_scorbot(
            position, rotation, True)

        if base_position == -1.0:
            self.get_logger().error('\n OUT WORKSPACE ROBOT! \n')
            return response

        joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_point.positions = [base_position, body_position, shoulder_position, elbow_position,
                                      wrist_position, wrist_yaw, self.extruder_pos]
        trajectory_point.time_from_start = Duration(nanosec=2000000000)
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

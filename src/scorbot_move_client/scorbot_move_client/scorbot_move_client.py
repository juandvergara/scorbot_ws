import time
from custom_interfaces.srv import JointTarget  # type: ignore
import rclpy
from builtin_interfaces.msg import Duration

import numpy as np
from math import radians, cos, sin, atan2, sqrt, degrees

from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

wrist_roll2hotend = np.array([[cos(radians(-90)),  0, sin(radians(-90)), -0.049],
                              [0,  1,          0, -0.0137],
                              [-sin(radians(-90)), 0, cos(radians(-90)), 0.132],
                              [0, 0,       0,           1]])

hotend2wrist_roll = np.linalg.inv(wrist_roll2hotend)


gcode_filename = '/home/juanmadrid/Escritorio/Shape-Box.gcode'

def extract_values_from_gcode(filename):
    data = []
    last_values = [0, 0, 0, 0, 0, 0, 0]  # Initial values for X, Y, Z, E, R, P, Y, NANOSEC
    time_sum = 0  # Variable to store the cumulative sum of time_nanosec
    time_sum_nano = 0

    with open(filename, 'r') as file:
        for line in file:
            if line.startswith('G1'):
                words = line.split()
                x = next((float(word[1:]) for word in words if word.startswith('X')), last_values[0]) / 1000
                y = next((float(word[1:]) for word in words if word.startswith('Y')), last_values[1]) / 1000
                z = next((float(word[1:]) for word in words if word.startswith('Z')), last_values[2])
                e = next((float(word[1:]) for word in words if word.startswith('E')), last_values[3])
                f = next((float(word[1:]) for word in words if word.startswith('F')), last_values[4])

                if f != last_values[4]:
                    f /= 60 * 1000  # Divide F value by 60
                if z != last_values[2]:
                    z /= 1000

                if last_values == [0, 0, 0, 0, 0, 0, 0]:
                    last_values = [x, y, z, e, f, 180, -90]
                    continue

                distance = sqrt((x - last_values[0]) ** 2 + (y - last_values[1]) ** 2 + (z - last_values[2]) ** 2)
                time_sec = int((distance / f))
                time_nanosec = (((distance / f) - time_sec) * 1e9)
                atan2_base = -degrees(atan2(x, y - 0.170)) - 90

                last_values = [x, y, z, e, f, 180, atan2_base]
                row = np.array(last_values[:3] + [0] + last_values[5:] + last_values[3:4] + [time_nanosec], dtype=np.float64)  # Create the modified row as a NumPy array

                time_sum += time_sec  # Update the cumulative sum of time_nanosec
                time_sum_nano += time_nanosec
                row[-1] = [time_sum] + [time_sum_nano]  # Assign the cumulative sum to the last element of the row

                data.append(row)  # Append the modified row to data

    return np.array(data)

def inverse_kinematics_scorbot(position_goal, rotation_goal, extruder_pos, sec_time_between_points, nanosec_time_between_points, wrist):
    global hotend2wrist_roll
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
    T = np.vstack((np.hstack((R, position_t)), scale_perception))
    arm_transform = T.dot(hotend2wrist_roll) if wrist else T

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

    offset_slide_base = 0.17
    slide_base = 0.0

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
    print("Matrix translation and roatation resultant \n")
    print(arm_transform)

    print("\n Value each joint \n")
    print(f"{slide_base:.6f}, {theta_1:.6f}, {theta_2:.6f}, {theta_3:.6f}, {theta_4:.6f}, {theta_5:.6f}")
    print("Success pose! \n")

    result = [slide_base, theta_1, theta_2, theta_3, theta_4, theta_5, extruder_pos, sec_time_between_points, nanosec_time_between_points,]

    return result

class ScorbotActionClient(Node):

    def __init__(self):
        super().__init__('scorbot_action_client')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_position_controller/follow_joint_trajectory')
        self.goal_msg = FollowJointTrajectory.Goal()
        self.goal_msg.trajectory.joint_names = ["slide_base_joint", "body_joint", "shoulder_joint", "elbow_joint",
                                                "wrist_joint", "roll_wrist_joint", "extruder_screw_joint"]

    def calculateTrajectory(self):
        time_points = 0
        time_between_points = 2
        num_rows = 3

        points_xyz_rpy = extract_values_from_gcode(gcode_filename)

        post_process_xyz_rpy = []

        for i in range(len(points_xyz_rpy) - 1):
            current_row = points_xyz_rpy[i]
            next_row = points_xyz_rpy[i + 1]
            post_process_xyz_rpy.append(current_row)

            spacing = (next_row - current_row) / (num_rows + 1)

            for j in range(1, num_rows + 1):
                new_row = current_row + (j * spacing)
                post_process_xyz_rpy.append(new_row)

        post_process_xyz_rpy.append(points_xyz_rpy[-1])

        trajectory_points = []

        for data_point in post_process_xyz_rpy:
            position = [float(i) for i in data_point[0:3]]
            rotation = [float(i) for i in data_point[3:6]]
            extruder_pos = float(data_point[6])
            sec_time_between_points = int(data_point[7])
            nanosec_time_between_points = int(data_point[7])
            result = inverse_kinematics_scorbot(position, rotation, extruder_pos, sec_time_between_points, nanosec_time_between_points, True)
            trajectory_points.append(result)

        for positions in trajectory_points:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            print(positions[7:8])
            trajectory_point.time_from_start = Duration(sec=int(positions[7:8][0]),nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
            time_points += time_between_points

    def startTrajectory(self):

        self.calculateTrajectory()

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.error_code))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.actual.positions))


def main(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()

import time
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

base2station = np.array([[1,  0, 0, 0.0],
                         [0,  1, 0, 0.0],
                         [0,  0, 1, 0.2],
                         [0,  0, 0, 1]])

gcode_filename = '/home/juanmadrid/Escritorio/Vase.gcode'


def post_process_coordenates(data, resolution, lower_limit, upper_limit):
    columns = [[] for _ in range(len(data[0]))]

    # Append the first point to the respective column list
    for value, column in zip(data[0], columns):
        column.append(value)

    # Iterate over the rows in the data array
    for i in range(len(data) - 1):
        point1 = data[i]
        point2 = data[i + 1]

        x1, y1, z1, roll1, pitch1, yaw1, e1, time1 = point1
        x2, y2, z2, roll2, pitch2, yaw2, e2, time2 = point2

        # Calculate the Euclidean distance between the two points
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

        if distance > upper_limit:
            # Calculate the number of intermediate points based on the spacing
            num_points = int(distance / resolution)

            # Calculate the increments for each column
            x_increment = (x2 - x1) / num_points
            y_increment = (y2 - y1) / num_points
            z_increment = (z2 - z1) / num_points
            roll_increment = (roll2 - roll1) / num_points
            pitch_increment = (pitch2 - pitch1) / num_points
            yaw_increment = (yaw2 - yaw1) / num_points
            extruder_increment = (e2 - e1) / num_points
            time_increment = (time2 - time1) / num_points

            # Iterate and interpolate values for each column
            for j in range(num_points):
                x = x1 + j * x_increment
                y = y1 + j * y_increment
                z = z1 + j * z_increment
                roll = roll1 + j * roll_increment
                pitch = pitch1 + j * pitch_increment
                yaw = yaw1 + j * yaw_increment
                e = e1 + j * extruder_increment
                t = time1 + j * time_increment

                # Append the interpolated values to the respective column list
                if j != 0:
                    columns[0].append(x)
                    columns[1].append(y)
                    columns[2].append(z)
                    columns[3].append(roll)
                    columns[4].append(pitch)
                    columns[5].append(yaw)
                    columns[6].append(e)
                    columns[7].append(t)
        elif distance >= lower_limit:
            # Append the original point to the respective column list
            for value, column in zip(point1, columns):
                column.append(value)

    # Add the last point of the original data
    for value, column in zip(data[-1], columns):
        column.append(value)

    # Transpose the column lists to get the interpolated data in columns
    return np.transpose(columns)

def extract_values_from_gcode(filename):
    data = []
    # Initial values for X, Y, Z, E, R, P, Y, NANOSEC
    last_values = [0, 0, 0, 0, 0, 0, 0]
    time_sum_sec = 0  # Variable to store the cumulative sum of time_nanosec

    with open(filename, 'r') as file:
        print("Processing GCode...")
        for line in file:
            if line.startswith('G1'):
                words = line.split()
                x = next(
                    (float(word[1:]) for word in words if word.startswith('X')), last_values[0])
                y = next(
                    (float(word[1:]) for word in words if word.startswith('Y')), last_values[1])
                z = next(
                    (float(word[1:]) for word in words if word.startswith('Z')), last_values[2])
                e = next(
                    (float(word[1:]) for word in words if word.startswith('E')), last_values[3])
                f = next(
                    (float(word[1:]) for word in words if word.startswith('F')), last_values[4])

                if f != last_values[4]:
                    f /= 60 * 1000  # Divide F value by 60
                if x != last_values[0]:
                    x /= 1000
                if y != last_values[1]:
                    y /= 1000
                if z != last_values[2]:
                    z /= 1000

                if last_values == [0, 0, 0, 0, 0, 0, 0]:
                    last_values = [x, y, z, e, f, 180, -90]
                    continue

                distance = sqrt(
                    (x - last_values[0]) ** 2 + (y - last_values[1]) ** 2 + (z - last_values[2]) ** 2)
                time_sum_sec += (distance / f)
                atan2_base = -degrees(atan2(x, y - 0.170)) - 90

                last_values = [x, y, z, e, f, 180, atan2_base]
                row = np.array(last_values[:3] + [0] + last_values[5:] + last_values[3:4] + [
                               time_sum_sec], dtype=np.float64)  # Create the modified row as a NumPy array

                data.append(row)  # Append the modified row to data

    return np.array(data)

def inverse_kinematics_scorbot(position_goal, rotation_goal, extruder_pos, sec_time_between_points, nanosec_time_between_points, wrist):
    global hotend2wrist_roll, base2station
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
    arm_transform = np.matmul(base2station, np.matmul(
        T, hotend2wrist_roll)) if wrist else T

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
    # print("Matrix translation and roatation resultant \n")
    # print(arm_transform)

    # print("\n Value each joint \n")
    # print(f"{slide_base:.6f}, {theta_1:.6f}, {theta_2:.6f}, {theta_3:.6f}, {theta_4:.6f}, {theta_5:.6f}")
    # print("Success pose! \n")

    result = [slide_base, theta_1, theta_2, theta_3, theta_4, theta_5,
              extruder_pos, sec_time_between_points, nanosec_time_between_points, ]

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

        points_xyz_rpy = extract_values_from_gcode(gcode_filename)
        # print(points_xyz_rpy)

        post_process_xyz_rpy = post_process_coordenates(
            points_xyz_rpy, 0.0002, 0.0001, 0.0005)

        trajectory_points = []

        for data_point in post_process_xyz_rpy:
            position = [float(i) for i in data_point[0:3]]
            rotation = [float(i) for i in data_point[3:6]]
            extruder_pos = float(data_point[6])
            absolute_time = data_point[7]
            sec_time_between_points = int(absolute_time)
            nanosec_time_between_points = int(
                (absolute_time - sec_time_between_points) * 1e9)
            result = inverse_kinematics_scorbot(
                position, rotation, extruder_pos, sec_time_between_points, nanosec_time_between_points, True)
            trajectory_points.append(result)

        for positions in trajectory_points:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            # print(positions[7:])
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)

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

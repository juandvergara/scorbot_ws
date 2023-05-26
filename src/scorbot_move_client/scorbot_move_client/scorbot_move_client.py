import time
from custom_interfaces.srv import JointTarget  # type: ignore
import rclpy
from builtin_interfaces.msg import Duration

from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

class ScorbotActionClient(Node):

    def __init__(self):
        super().__init__('scorbot_action_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_position_controller/follow_joint_trajectory')
      
    def startTrajectory(self):
        goal_msg = FollowJointTrajectory.Goal()
        time_btw_points = 5
        time_from_start = 0

        goal_msg.trajectory.joint_names = ["slide_base_joint", "body_joint", "shoulder_joint", "elbow_joint",
                                            "wrist_joint", "roll_wrist_joint", "extruder_screw_joint"]

        trajectory_point_1 = JointTrajectoryPoint()   # X400 Y100 X450
        trajectory_point_1.positions = [0.0, -1.7440419932472615, 0.5234731600245995, -1.0469463200491989,
                                      0.5234731600245996, -3.4989908547067257e-16, 0.0]
        trajectory_point_1.time_from_start = Duration(sec=time_btw_points)
        goal_msg.trajectory.points.append(trajectory_point_1)
        time_btw_points += 5

        trajectory_point_2 = JointTrajectoryPoint()   # X400 Y200 Z450
        trajectory_point_2.positions = [0.0, -1.4959364790841299, 0.5455809721574817, -1.0911619443149632,
                                      0.5455809721574822, 8.164311994315704e-16, 0.0]
        trajectory_point_2.time_from_start = Duration(sec=time_btw_points)
        goal_msg.trajectory.points.append(trajectory_point_2)
        time_btw_points += 5

        trajectory_point_3 = JointTrajectoryPoint()   # X300 Y200 Z450
        trajectory_point_3.positions = [0.0, -1.4711276743037347, 0.8913008885610343, -1.7826017771220686,
                                      0.891300888561035, 6.123233995736776e-16, 0.0]
        trajectory_point_3.time_from_start = Duration(sec=time_btw_points)
        goal_msg.trajectory.points.append(trajectory_point_3)
        time_btw_points += 5

        trajectory_point_4 = JointTrajectoryPoint()   # X300 Y100 Z450 
        trajectory_point_4.positions = [0.0, -1.800028260071892, 0.8719764546094321, -1.7439529092188641,
                                        0.8719764546094323, -2.624243141030043e-16, 0.0]
        trajectory_point_4.time_from_start = Duration(sec=time_btw_points)
        goal_msg.trajectory.points.append(trajectory_point_4)
        time_btw_points += 5

        trajectory_point_5 = JointTrajectoryPoint()   # X400 Y100 Z450
        trajectory_point_5.positions = [0.0, -1.7440419932472615, 0.5234731600245995, -1.0469463200491989,
                                      0.5234731600245996, -3.4989908547067257e-16, 0.0]
        trajectory_point_5.time_from_start = Duration(sec=time_btw_points)
        goal_msg.trajectory.points.append(trajectory_point_5)
        

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

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
        self.get_logger().info('Received feedback: {0}'.format(feedback.actual.positions))


def main(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState

class ErrorCalculator(Node):

    def __init__(self):
        super().__init__('error_calculator_node')  # type: ignore

        self.joint_state_listener = self.create_subscription(
            JointTrajectoryControllerState, 
            '/joint_trajectory_position_controller/state', 
            self.save_error_callback, 10)
        self.joint_state_listener

        self.error_values = {
            'error_body_joint': [],
            'error_shoulder_joint': [],
            'error_elbow_joint': [],
            'error_wrist_joint': [],
            'error_roll_wrist_joint': []
        }

    def save_error_callback(self, joint_states):
        self.error_values['error_body_joint'].append(joint_states.error.positiions[1])
        self.error_values['error_shoulder_joint'].append(joint_states.error.positiions[2])
        self.error_values['error_elbow_joint'].append(joint_states.error.positiions[3])
        self.error_values['error_wrist_joint'].append(joint_states.error.positiions[4])
        self.error_values['error_roll_wrist_joint'].append(joint_states.error.positiions[5])
        
    def calculate_mse(self):
        mse_values = {}
        total_squared_errors = []
        for key, error_list in self.error_values.items():
            if not error_list:
                mse_values[key] = 0.0
            else:
                squared_errors = [error ** 2 for error in error_list]
                mean_square_error = sum(squared_errors) / len(squared_errors)
                mse_values[key] = mean_square_error
                total_squared_errors.extend(squared_errors)

        if not total_squared_errors:
            mse_values['total_error'] = 0.0
        else:
            total_mean_square_error = sum(total_squared_errors) / len(total_squared_errors)
            mse_values['total_error'] = total_mean_square_error

        return mse_values

def main(args=None):
    rclpy.init(args=args)
    error_calculator_node = ErrorCalculator()

    try:
        rclpy.spin(error_calculator_node)
    except KeyboardInterrupt:
        pass

    mse_values = error_calculator_node.calculate_mse()
    for key, mse in mse_values.items():
        print(f"Mean Square Error ({key}): {mse}")

    error_calculator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

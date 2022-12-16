import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    my_package_dir = get_package_share_directory('scorbot_er_v')

    urdf_file_name = 'scorbot_printer.urdf'
    urdf = os.path.join(my_package_dir,
                        'urdf',
                        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_description = {'robot_description': robot_desc}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("scorbot_er_v"),
            "config",
            "scorbot_er_v_controllers.yaml",
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=[urdf]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
                '-d', os.path.join(my_package_dir, 'config', 'view_robot.rviz')],
        name='rviz2',
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["arm_controller"],
    )

    joint_foward_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller"],
    )

    spawn_entity_gazebo = Node(package='gazebo_ros', executable='spawn_entity.py',
                               arguments=['-topic', 'robot_description',
                                          '-entity', 'my_scorbot'],
                               output='screen')

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        gazebo,
        spawn_entity_gazebo,
        joint_state_broadcaster_spawner,
        robot_state_pub_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)

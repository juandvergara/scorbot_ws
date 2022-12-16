import os
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    my_package_dir = get_package_share_directory('scorbot_er_v')

    urdf_file_name = 'scorbot_printer.urdf'
    urdf = os.path.join(my_package_dir,
                        'urdf',
                        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', os.path.join(my_package_dir, 'config', 'view_robot.rviz')],
        name='rviz2',
        output='screen',
    )

    arm_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["arm_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
    )
    
    joint_foward_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller"],
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity_gazebo = Node(package='gazebo_ros', executable='spawn_entity.py',
                               arguments=['-topic', 'robot_description',
                                          '-entity', 'my_scorbot'],
                               output='screen')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                             description='Flag to enable joint_state_publisher_gui'),
        
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        gazebo,
        spawn_entity_gazebo,
        arm_drive_spawner, 
        joint_broad_spawner
    ])
'''
joint_state_publisher_gui_node,


        '''
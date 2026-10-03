import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = get_package_share_directory('swarmbot_description')
    xacro_file = os.path.join(description_pkg, 'urdf', 'swarmbot.urdf.xacro')
    rviz_config = os.path.join(description_pkg, 'rviz', 'swarmbot.rviz')

    robot_name = LaunchConfiguration('robot_name')

    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    }
    frame_prefix = {'frame_prefix': [robot_name, '/']}

    namespaced_nodes = GroupAction([
        PushRosNamespace(robot_name),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, frame_prefix],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot_01',
            description=(
                'Robot name. Used as ROS namespace and as the TF frame_prefix, '
                'so several robots can run this launch file at once without '
                'clashing (e.g. robot_01, robot_02, ...).'
            ),
        ),
        namespaced_nodes,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])

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
    # Config aparte de la de sim.launch.py: Fixed Frame = <robot_name>/odom
    # en vez de <robot_name>/base_link, porque aqui SI hay algo publicando
    # esa transformada (ver imu_orientation_tf.py mas abajo) y queremos que
    # la camara quede anclada a un frame que no rote con el robot.
    rviz_config = os.path.join(description_pkg, 'rviz', 'swarmbot_real.rviz')

    robot_name = LaunchConfiguration('robot_name')
    agent_port = LaunchConfiguration('agent_port')

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
        # No encoder data yet: this keeps the (still undriven) wheel joints
        # at a default pose so TF stays complete. Once the firmware starts
        # publishing real joint states on this topic, drop this node.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot_01',
            description=(
                'Robot name. Used as ROS namespace and as the TF frame_prefix, '
                'matching the node name (and /<robot_name>/... topics) already '
                'used by the ESP32-S3 firmware.'
            ),
        ),
        DeclareLaunchArgument(
            'agent_port',
            default_value='8888',
            description='UDP port micro_ros_agent listens on (must match the firmware AGENT_PORT).',
        ),
        # Same command documented in ~/swarmbot_firmware/README.md.
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['udp4', '--port', agent_port],
        ),
        # Fuera del GroupAction namespaceado a proposito: el firmware ya
        # hornea "robot_01/" en los nombres de topic que publica (no usa
        # namespacing de ROS), y este nodo debe escribir en el /tf global
        # (no en /robot_01/tf) para que RViz y robot_state_publisher lo
        # vean. Ver comentario dentro de imu_orientation_tf.py.
        Node(
            package='swarmbot_bringup',
            executable='imu_orientation_tf.py',
            name='imu_orientation_tf',
            output='screen',
            parameters=[{'robot_name': robot_name}],
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

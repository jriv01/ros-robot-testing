import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'rdt_legacy_robot'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'robot.urdf.xacro'
    urdf = os.path.join(
        get_package_share_directory(pkg_name),
        urdf_file_name
    )
    robot_desc = xacro.process_file(urdf).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,'robot_description': robot_desc}],
        arguments=[urdf]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        node_robot_state_publisher
    ])
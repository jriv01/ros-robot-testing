import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

    package_name = "tinkerbot_gazebo_sim"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "rsp.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory(package_name), "worlds", "apriltag.world"
        ),
        description="World to load",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "tinkerbot", "-z", "0.5"],
        output="screen",
    )

    # Launch the ROS-Gazebo bridge for normal topics
    bridge_params = os.path.join(
        get_package_share_directory(package_name), "gz_bridge.yaml"
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    # Bridge images from webcam
    ros_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/front_webcam/image_raw"],
        output="screen",
    )

    return LaunchDescription(
        [
            rsp,
            world_arg,
            gazebo,
            spawn_entity,
            ros_gz_bridge,
            ros_image_bridge,
        ]
    )

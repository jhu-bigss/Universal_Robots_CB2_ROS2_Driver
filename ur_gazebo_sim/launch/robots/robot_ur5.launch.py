#!/usr/bin/env -S ros2 launch
"""Launch script for spawning UR5 into Ignition Gazebo world"""

from typing import List

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    robot_sdf_path = PathJoinSubstitution([
                        FindPackageShare("ur_cb2_description"),
                        "sdf",
                        "ur5.sdf",
                    ])
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # List of nodes to be launched
    nodes = [
        # ros_ign_gazebo_create
        Node(
            package="ros_ign_gazebo",
            executable="create",
            output="log",
            arguments=["-file", robot_sdf_path, "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]

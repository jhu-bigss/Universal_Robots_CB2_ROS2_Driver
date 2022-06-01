#!/usr/bin/env -S ros2 launch
# modified from https://github.com/AndrejOrsula/ur_gazebo_sim/blob/master/launch/default.launch.py

from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world_type = LaunchConfiguration("world_type")
    ur_type = LaunchConfiguration("ur_type")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    log_level = LaunchConfiguration("log_level")

    # Determine what world/robot combination to launch
    declared_arguments.append(
        DeclareLaunchArgument(
            "__world_launch_basename",
            default_value=["world_", world_type, ".launch.py"],
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "__robot_launch_basename",
            default_value=["robot_", ur_type, ".launch.py"],
        ),
    )

    # List of included launch descriptions
    launch_descriptions = [
        # Launch Ignition Gazebo with the required ROS<->IGN bridges
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_gazebo_sim"),
                        "launch",
                        "worlds",
                        LaunchConfiguration("__world_launch_basename"),
                    ]
                )
            ),
            launch_arguments=[
                ("use_sim_time", use_sim_time),
                ("ign_verbosity", ign_verbosity),
                ("log_level", log_level),
            ],
        ),
        # Spawn robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_gazebo_sim"),
                        "launch",
                        "robots",
                        LaunchConfiguration("__robot_launch_basename"),
                    ]
                )
            ),
            launch_arguments=[
                ("use_sim_time", use_sim_time),
                ("ign_verbosity", ign_verbosity),
                ("log_level", log_level),
            ],
        ),
        # Launch move_group of MoveIt 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_moveit_config"),
                        "launch",
                        "ur_moveit.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("ur_type", ur_type),
                ("ros2_control_plugin", "ign"),
                ("ros2_control_command_interface", "effort"),
                # TODO: Re-enable colligion geometry for manipulator arm once spawning with specific joint configuration is enabled
                ("collision_arm", "false"),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("log_level", log_level),
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # World selection
        DeclareLaunchArgument(
            "world_type",
            default_value="default",
            description="Name of the world configuration to load.",
        ),
        # Robot selection
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5",
            description="Name of the robot type to use.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("ur_gazebo_sim"),
                "rviz",
                "ur_gazebo_sim.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="2",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
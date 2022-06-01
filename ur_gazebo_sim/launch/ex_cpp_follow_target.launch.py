#!/usr/bin/env -S ros2 launch
# modified from https://github.com/AndrejOrsula/ign_moveit2_examples/blob/master/launch/ex_cpp_follow_target.launch.py

from os import path
from typing import List

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = "ur_moveit_config"
    ur_type = LaunchConfiguration("ur_type")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    log_level = LaunchConfiguration("log_level")

    # URDF
    _robot_description_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=",
            ur_type,
        ]
    )
    robot_description = {"robot_description": _robot_description_xml}

    # SRDF
    _robot_description_semantic_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(moveit_config_package),
                    "srdf",
                    "ur.srdf.xacro",
                ]
            ),
            " ",
            "name:=",
            ur_type,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": _robot_description_semantic_xml
    }

    # Rviz configuration file
    rviz_config = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    # List of included launch descriptions
    launch_descriptions = [
        # Launch world with robot (configured for this example)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur_gazebo_sim"),
                        "launch",
                        "default.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("world_type", "follow_target"),
                ("ur_type", ur_type),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("ign_verbosity", ign_verbosity),
                ("log_level", log_level),
            ],
        ),
    ]

    # List of nodes to be launched
    nodes = [
        # Run the example node (C++)
        Node(
            package="ur_gazebo_sim",
            executable="ex_follow_target",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Robot type
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5",
            description="Type of the robot to use.",
        ),
        # Locations of robot resources
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Robot description package.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        # Miscellaneous
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
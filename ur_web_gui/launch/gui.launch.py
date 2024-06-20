from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur5", "ur10"],
        )
    )

    ur_type = LaunchConfiguration("ur_type")

    return LaunchDescription(declared_arguments + [
        Node(
            package='ur_web_gui',
            executable='ur_web_gui',
            parameters=[{
                'ur_type': ur_type,
                'robot_description_pkg_name': 'ur_description',
            }],
        ),
    ])
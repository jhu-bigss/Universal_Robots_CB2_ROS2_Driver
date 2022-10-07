
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    velocity_cmds = PathJoinSubstitution(
        [FindPackageShare("ur_cb2_bringup"), "config", "test_goal_publishers_config.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="ur_cb2_robot_driver",
                executable="publisher_forward_velocity_controller.py",
                name="publisher_forward_velocity_controller",
                parameters=[velocity_cmds],
                output="screen",
            )
        ]
    )

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur_web_gui',
            executable='ur_web_gui',
            parameters=[{
                'ur_type': 'ur5',
                'robot_description_pkg_name': 'ur_description',
            }],
        ),
    ])
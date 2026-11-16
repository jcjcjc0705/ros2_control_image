"""Two-joint MIT demo on mock hardware, and the template for a robot's own bringup launch.

    ros2 launch robot_bringup mit_demo.launch.py
    ros2 topic pub -r 50 /mit_controller/commands mit_msgs/msg/MitCommand \\
        "{position: [0.5, -0.5], velocity: [0, 0], kp: [20, 20], kd: [1, 1], effort: [0, 0]}"

A robot's bringup package copies this file, points the two paths at its own share
directory, and lists its own controllers. Everything else comes from bringup.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('robot_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='mock', description='real, mock, ...'),
        DeclareLaunchArgument('namespace', default_value=''),
        IncludeLaunchDescription(
            os.path.join(share, 'launch', 'bringup.launch.py'),
            launch_arguments={
                'description_file': os.path.join(share, 'examples', 'mit_demo.urdf.xacro'),
                'controllers_file': os.path.join(share, 'examples', 'mit_demo_controllers.yaml'),
                'controllers': 'joint_state_broadcaster mit_controller',
                'hardware': LaunchConfiguration('hardware'),
                'namespace': LaunchConfiguration('namespace'),
            }.items(),
        ),
    ])

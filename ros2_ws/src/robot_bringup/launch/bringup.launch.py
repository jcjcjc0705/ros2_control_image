"""Generic bring-up for any robot image built on ros2_control_image.

Starts robot_state_publisher and ros2_control_node from a robot's xacro and controllers
file, then spawns the listed controllers. A robot's own bringup package just includes
this launch file with its own files (see mit_demo.launch.py), so it carries no launch
logic of its own.

    ros2 launch robot_bringup bringup.launch.py \\
        description_file:=/path/to/robot.urdf.xacro \\
        controllers_file:=/path/to/controllers.yaml \\
        controllers:="joint_state_broadcaster mit_controller" \\
        hardware:=mock
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

# (name, default or None if required, description)
ARGUMENTS = [
    ('description_file', None, 'Robot xacro or URDF file (absolute path).'),
    ('description_args', '', 'Extra xacro arguments, e.g. "prefix:=left_ port:=can1".'),
    ('hardware', 'real',
     'Backend, passed to the xacro as hardware:=<value>. Convention: real, mock, isaac.'),
    ('controllers_file', None,
     'controller_manager parameters: update_rate, controller types and their parameters.'),
    ('controllers', 'joint_state_broadcaster',
     'Controllers to load and activate, space-separated.'),
    ('inactive_controllers', '', 'Controllers to load but leave inactive, space-separated.'),
    ('namespace', '', 'ROS namespace for every node.'),
    ('use_sim_time', 'false', 'Follow /clock (true when running against a simulator).'),
]


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    use_sim_time = arg('use_sim_time').lower() in ('true', '1', 'yes')
    xacro = f"xacro {arg('description_file')} hardware:={arg('hardware')} {arg('description_args')}"
    robot_description = {
        'robot_description': ParameterValue(Command(xacro), value_type=str),
    }

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='both',
        ),
        # controller_manager reads the URDF from robot_state_publisher's topic.
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[arg('controllers_file'), {'use_sim_time': use_sim_time}],
            remappings=[('~/robot_description', 'robot_description')],
            output='both',
        ),
    ]

    active = arg('controllers').split()
    if active:
        nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            arguments=active,
            output='both',
        ))
    inactive = arg('inactive_controllers').split()
    if inactive:
        nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            arguments=inactive + ['--inactive'],
            output='both',
        ))

    namespace = arg('namespace')
    return [GroupAction([PushRosNamespace(namespace)] + nodes)] if namespace else nodes


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(name, description=description)
        if default is None else
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]
    return LaunchDescription(declared + [OpaqueFunction(function=launch_setup)])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ultrasonic_node = Node(
        package='hardware_interfaces',
        executable='ultrasonic',
        name='ultrasonic',
        output='screen'
    )

    manipulator_node = Node(
        package='hardware_interfaces',
        executable='manipulator',
        name='manipulator',
        output='screen'
    )

    speaker_node = Node(
        package='hardware_interfaces',
        executable='speaker',
        name='speaker',
        output='screen'
    )

    return LaunchDescription([
        ultrasonic_node,
        manipulator_node,
        speaker_node,
    ])

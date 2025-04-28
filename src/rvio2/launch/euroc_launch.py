import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rvio2')
    params_file = os.path.join(pkg_share, 'config', 'rvio2_euroc.yaml')

    return LaunchDescription([
        Node(
            package='rvio2',
            executable='rvio2_mono',
            name='rvio2_mono',
            output='screen',
            parameters=[params_file]
        )
    ])

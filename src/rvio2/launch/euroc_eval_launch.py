import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rvio2')
    # Default configuration and rosbag file path. (You can switch between different rosbag paths by commenting/uncommenting.)
    default_config = os.path.join(pkg_share, 'config', 'rvio2_euroc.yaml') + " " + "/home/zheng/ROS/datasets/eth_dataset/euroc/V1_01_easy.bag"
    # For example, to use a different bag file, change the string accordingly:
    # default_config = os.path.join(pkg_share, 'config', 'rvio2_euroc.yaml') + " " + "/home/zheng/ROS/datasets/eth_dataset/euroc/V1_02_medium.bag"

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to configuration file and rosbag file (separated by a space)'
    )

    rvio2_mono_eval_node = Node(
        package='rvio2',
        executable='rvio2_mono_eval',
        name='rvio2_mono_eval',
        output='screen',
        arguments=[LaunchConfiguration('config')]
    )

    return LaunchDescription([
        config_arg,
        rvio2_mono_eval_node
    ])

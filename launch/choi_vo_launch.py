import os

from click import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    launch_description = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('choi_vo'),
        'config',
        'config.yaml'
    )

    vo_node = Node(
        package    = 'choi_vo',
        executable = 'stereo_vo',
        name       = 'stereo_vo',
        parameters = [{"config" : "duasd"}]
    )

    launch_description.add_action(vo_node)

    return launch_description
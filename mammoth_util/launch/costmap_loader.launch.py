import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    # ROS packages
    pkg_mammoth_util = get_package_share_directory('mammoth_util')

    # LOADED COSTMAP HERE
    costmap_img_filename = 'costmap.pgm' # just the file name no path
    # LOADED COSTMAP HERE

    config_dir = os.path.join(pkg_mammoth_util, 'config')
    costmaps_dir = os.path.join(config_dir, 'costmaps')
    costmap_filepath = os.path.join(costmaps_dir, costmap_img_filename)

    # Nodes
    costmap_loader = Node(
        package='mammoth_util',
        executable='costmap_loader',
        name='costmap_loader',
        output='screen',
        parameters=[{
            'costmap_filepath': costmap_filepath,
        }]
    )

    return LaunchDescription([
        # Nodes
        costmap_loader
    ])
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cartographer_dir = get_package_share_directory('cartographer_ros')
    cartographer_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(cartographer_dir + '/launch/offline_backpack_2d.launch.py'),
      launch_arguments = {
          'bag_filenames': '~/share/rosbag2_2022_10_22-12_21_39/rosbag2_2022_10_22-12_21_39_0.db3',
      }.items())
      
    ld = LaunchDescription()
    ld.add_action(cartographer_launch)

    return ld

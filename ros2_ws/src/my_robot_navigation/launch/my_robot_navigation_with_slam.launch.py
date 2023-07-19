import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments #
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz2_file = LaunchConfiguration('rviz2_file')

    declare_arg_map = DeclareLaunchArgument(
        'map',
        default_value = os.path.join(
            get_package_share_directory('my_robot_navigation'),
            'map',
            'my_map.yaml'
        ),
        description = 'The full path to the map yaml file.'
    )

    declare_arg_params_file = DeclareLaunchArgument(
        'params_file',
        default_value = os.path.join(
            get_package_share_directory('my_robot_navigation'),
            'params',
            'nav2_params.yaml'),
        description = 'The full path to the param file.'
    )

    declare_arg_rviz2_config_path = DeclareLaunchArgument(
        'rviz2_file',
        default_value = os.path.join(
            get_package_share_directory('my_robot_navigation'),
            'rviz',
            'nav2_view.rviz'),
        description = 'The full path to the rviz file'
    )

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )
    
    cartographer_launc_file_dir = os.path.join(
        get_package_share_directory('cartographer_slam'),
        'launch'
    )

    # Launch files and Nodes #
    # start navigation
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_launch_file_dir, '/navigation_launch.py']), # bringup without amcl, map_server
        launch_arguments = {
            #'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time}.items(),
    )

    # start the visualization
    rviz2_cmd = Node(
        name = 'rviz2',
        package = 'rviz2', executable = 'rviz2', output = 'screen',
        arguments = ['-d', rviz2_file],
        parameters = [{'use_sim_time': use_sim_time}]
    )
    
    # start SLAM
    cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            cartographer_launc_file_dir, '/cartographer_publish_odom.launch.py']),
    )
    
    # start inspection
    inspection_cmd = Node(
        package = 'my_robot_navigation',
        executable = 'inspection',
        #emulate_tty = True,
        output = 'screen')
    
    ld = LaunchDescription()
    ld.add_action(declare_arg_map)
    ld.add_action(declare_arg_params_file)
    ld.add_action(declare_arg_rviz2_config_path)

    ld.add_action(nav2_cmd)
    ld.add_action(cartographer_cmd)
    ld.add_action(rviz2_cmd)
    #ld.add_action(inspection_cmd)

    return ld

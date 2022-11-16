import os
import launch
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    log_info = launch.actions.LogInfo(
            msg = "Launch odometry_publisher node and IMU node."
        )

    timer_action = launch.actions.TimerAction(period=3.0, actions=[
            launch.actions.LogInfo(
                msg = "It's been three seconds since the launch"
            )
        ])
        
    rosbag2_node = launch.actions.ExecuteProcess(
        cmd = ['ros2', 'bag', 'record', '-a'],
        output = 'screen'
    )

    unity_node = Node(
        package = 'ros_tcp_endpoint',
        executable = 'default_server_endpoint',
        namespace = '',
        output = 'screen',
        prefix = 'xterm -e',
        parameters = [{'ROS_IP': '192.168.11.90'},
                      {'use_sim_time': False}],
    )

    micro_ros_node = Node(
        package = 'micro_ros_agent',
        executable = 'micro_ros_agent',
        namespace = '',
        output = 'screen',
        prefix = 'xterm -e',
        arguments = ['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        parameters=[{'use_sim_time': False}],
    )
    # sudo usermod -a -G dialout ubuntu
    # groups ubuntu
    # sudo reboot
    
    pkg_share = FindPackageShare(package='my_robot_description').find('my_robot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/my_robot_description.urdf')
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )

    imu_dir = get_package_share_directory('bno055')
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_dir + '/launch/bno055.launch.py'))
        
    imu_filter_node = Node(
        package = 'imu_complementary_filter',
        executable = 'complementary_filter_node',
        namespace = '',
        output = 'screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings = [
            ('/imu/data_raw', '/bno055/imu_raw'),
            ('/imu/mag', '/bno055/mag')
        ],
        #remappings = [
        #    ('/bno055/imu_raw', '/imu/data_raw'),
        #    ('/bno055/mag', '/imu/mag')
        #],
    )
    
    odometry_node = Node(
        package = 'odometry_publisher',
        executable = 'odometry_publisher',
        namespace = '',
        output = 'screen',
        parameters=[{'use_sim_time': False}],
        #prefix = 'xterm -e',
    )
    
    ekf_node = Node(
        package = 'robot_localization',
        executable = 'ekf_node',
        namespace = '',
        output = 'screen',
        parameters = [os.path.join(get_package_share_directory('robot_localization'), 'params', 'my_ekf.yaml')],
        remappings = [
            ('odometry/filtered', '/odom')
        ],
    )
    
    #lidar_dir = get_package_share_directory('ldlidar_stl_ros2')
    #lidar_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(lidar_dir + '/launch/ld06.launch.py'))
    
    #lidar_dir = get_package_share_directory('ldlidar')
    #lidar_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(lidar_dir + '/launch/ldlidar.launch.py'),
    #    launch_arguments = {
    #        #'use_sim_time': True,
    #        'serial_port': '/dev/lidar',
    #        'topic_name': 'scan',
    #        'lidar_frame': 'lidar_link',
    #        'range_threshold': '0.005'
    #        }.items())
    
    lidar_node = Node(
        package = 'ldlidar_stl_ros2',
        executable = 'ldlidar_stl_ros2_node',
        namespace = '',
        #output='screen',
        parameters = [
            {'product_name': 'LDLiDAR_LD06'},
            {'topic_name': 'scan'},
            {'port_name': '/dev/lidar'},
            {'frame_id': 'lidar_link'},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0},
            #{'use_sim_time': False}
        ]
    )
    
    navigation_dir = get_package_share_directory('my_robot_navigation')
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_dir + '/launch/my_robot_navigation.launch.py'),
        launch_arguments = {
            'use_sim_time': 'false',
            }.items())
    
    '''
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_dir + '/launch/online_async_launch.py'),
        launch_arguments = {
            'use_sim_time': 'false',
            'slam_params_file': 'my_mapper_params_online_async.yaml',
            }.items())
    '''
    '''
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_dir + '/launch/offline_launch.py'),
        launch_arguments = {
            'use_sim_time': 'false',
            'slam_params_file': 'my_mapper_params_offline.yaml',
            }.items())
    '''
    
    ''' 
    cartographer_slam_dir = get_package_share_directory('cartographer_slam')
    cartographer_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cartographer_slam_dir + '/launch/cartographer.launch.py'))
    '''
    
    camera_node = Node(
        package = 'oak_d_lite',
        executable = 'detection',
        namespace = '',
        #output = 'screen',
        parameters=[{'use_sim_time': False}],
        #prefix = 'xterm -e',
    )

    image_compress_node = Node(
        package = 'image_transport',
        executable = 'republish',
        namespace = '',
        arguments = ['raw', 'compressed'],
        remappings=[
            ('in', '/image_raw'),
            ('out/compressed', '/image_raw/compressed')
        ],
    )
    
    

    ld = LaunchDescription()
    #ld.add_action(rosbag2_node)
    ld.add_action(log_info)
    ld.add_action(timer_action)
    ld.add_action(unity_node)
    ld.add_action(micro_ros_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(imu_launch)
    ##ld.add_action(imu_filter_node)
    ##ld.add_action(odometry_launch)
    ld.add_action(odometry_node)
    ld.add_action(ekf_node)
    ##ld.add_action(lidar_launch)
    ld.add_action(lidar_node)
    ld.add_action(camera_node)
    ld.add_action(image_compress_node)
    ##ld.add_action(slam_toolbox_launch)
    ##ld.add_action(cartographer_slam_launch)
    #ld.add_action(navigation_launch)

    return ld

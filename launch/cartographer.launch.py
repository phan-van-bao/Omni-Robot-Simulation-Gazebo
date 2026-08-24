import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Sử dụng sim time = true làm mặc định cho Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    project_prefix = get_package_share_directory('project_omni')

    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(project_prefix, 'config')
    )

    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='omni_base_2d.lua' # Đã sửa thành tên file lua mình hướng dẫn ở bước trước
    )

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # Đường dẫn tới file config RViz
    rviz_config_dir = os.path.join(
        get_package_share_directory('project_omni'),
        'rviz',
        'project.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('cartographer_config_dir', default_value=cartographer_config_dir),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('resolution', default_value=resolution),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec),

        # Node chính chạy Cartographer
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/scan', '/scan_front_raw'),
                ('/imu', '/base_imu'),
                ('/odom', '/mobile_base_controller/odometry')
            ],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ]
        ),

        # Gọi trực tiếp Node Occupancy Grid thay vì gọi qua file launch khác
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-resolution', resolution, 
                '-publish_period_sec', publish_period_sec
            ]
        ),

        # Node RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])
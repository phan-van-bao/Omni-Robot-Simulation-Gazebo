import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('project_omni')

    urdf_file = os.path.join(pkg, 'urdf', 'omni_base.urdf')
    bridge_config = os.path.join(pkg, 'config', 'bridge_config.yaml')
    controller_config = os.path.join(pkg, 'config', 'configuration.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    combined_resource_path = [
        os.path.join(pkg, 'models'),
        os.path.dirname(pkg),
        pkg
    ]

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(combined_resource_path)
    )

    set_ros_args = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_ARGS',
        value=f'--ros-args --params-file {controller_config}'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'omni_base',
            '-x', '0.0',
            '-y', '10',
            '-z', '0.1'
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    delayed_bridge = TimerAction(
        period=5.0,
        actions=[bridge]
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    mobile_base_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mobile_base_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    delayed_controllers = TimerAction(
        period=8.0,
        actions=[
            joint_state_broadcaster,
            mobile_base_controller
        ]
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ros_args,
        robot_state_publisher,
        delayed_spawn,
        delayed_bridge,
        delayed_controllers,
    ])
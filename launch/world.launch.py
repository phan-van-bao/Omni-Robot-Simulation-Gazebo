import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('project_omni')
    world_file = os.path.join(pkg, 'worlds', 'hospital_full.world')

    combined_resource_path = [
        os.path.join(pkg, 'models'),
        os.path.dirname(pkg),
        pkg
    ]

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(combined_resource_path)
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items(),
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim
    ])
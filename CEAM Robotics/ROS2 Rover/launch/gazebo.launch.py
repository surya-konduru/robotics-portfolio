import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    package_dir = get_package_share_directory('rover_control')
    world_path = os.path.join(package_dir, 'worlds', 'empty.world')

    gazebo = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-r',
            '-s',
            world_path
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    calib_file = os.path.join(
        get_package_share_directory('rm_force_publisher'),
        'config',
        'force_calibration.yaml'
    )

    return LaunchDescription([
        Node(
            package='rm_force_publisher',
            executable='force_publisher',
            name='force_publisher',
            output='screen',
            parameters=[{'calibration_file': calib_file}],
        )
    ])

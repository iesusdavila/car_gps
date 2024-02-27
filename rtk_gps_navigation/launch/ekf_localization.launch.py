from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization_with_gps',
            parameters=[get_package_share_directory('rtk_gps_navigation') + '/config/gps_localization_config.yaml']
        )
    ])

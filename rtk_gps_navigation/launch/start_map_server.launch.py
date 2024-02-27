from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=get_package_share_directory('rtk_gps_navigation') + '/maps/mymap_empty.yaml',
            description='Path to the map file'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            arguments=[{'use_sim_time': 'true'}, {'yaml_filename': 'map_file'}]
        )
    ])
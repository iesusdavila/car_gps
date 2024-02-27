from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_config = "ps3"
    joy_dev = "/dev/input/js0"
    config_filepath = get_package_share_directory('rtk_gps_teleoperation') + '/config/' + joy_config + '.config.yaml'

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': joy_dev, 'deadzone': 0.3, 'autorepeat_rate': 20.0, 'config_filepath': config_filepath}]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[{'config_filepath': config_filepath}],
            output='screen'
        )
    ])

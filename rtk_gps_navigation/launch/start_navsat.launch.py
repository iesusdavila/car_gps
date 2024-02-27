from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    navsat_transform_params_file = get_package_share_directory('rtk_gps_navigation') + '/config/navsat_transform_params.yaml'

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                {'magnetic_declination_radians': 0.0},
                {'yaw_offset': 0.0},
                {'zero_altitude': True},
                {'use_odometry_yaw': False},
                {'wait_for_datum': True},
            ],
            remappings=[
                ('/imu/data', '/handsfree/imu'),
                ('/gps/fix', '/cell/fix'),
                ('/odometry/filtered', '/odom'),
            ],
            arguments=[navsat_transform_params_file]
        )
    ])


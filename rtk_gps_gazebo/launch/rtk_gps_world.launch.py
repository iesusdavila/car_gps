import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, EnvironmentVariable, LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    navigation = LaunchConfiguration('navigation', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    x_pose = LaunchConfiguration('x_pose', default='-1.0')
    y_pose = LaunchConfiguration('y_pose', default=' 0.5')

    # Directories
    rtk_gps_gazebo_package_dir = get_package_share_directory('rtk_gps_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    rtk_gps_teleoperation_pkg_dir = get_package_share_directory('rtk_gps_teleoperation')
    rtk_gps_navigation_pkg_dir = get_package_share_directory('rtk_gps_navigation')

    world = os.path.join(
        rtk_gps_gazebo_package_dir,
        'worlds',
        'world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot Description
    robot_description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtk_gps_gazebo_package_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # URDF Spawner
    urdf_spawner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtk_gps_gazebo_package_dir, 'launch', 'spawn_car.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Rviz View
    rviz_view_node = Node(
        condition=UnlessCondition(navigation),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(rtk_gps_gazebo_package_dir, 'rviz', 'rtk_gps_rviz.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Add the launch files
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([rtk_gps_teleoperation_pkg_dir, 'launch', 'rtk_gps_teleop.launch.py']))
    )
    ekf_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([rtk_gps_navigation_pkg_dir, 'launch', 'ekf_localization.launch.py']))
    )
    start_navsat2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([rtk_gps_navigation_pkg_dir, 'launch', 'start_navsat.launch.py']))
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([rtk_gps_navigation_pkg_dir, 'launch', 'navigation.launch.py'])),
        condition=IfCondition(navigation),
        launch_arguments={
            'map': os.path.join(rtk_gps_navigation_pkg_dir, 'maps', 'map.yaml'),
            'params_file': os.path.join(rtk_gps_navigation_pkg_dir, 'param', 'rtk.yaml'),
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_description_node,
        urdf_spawner_node,
        rviz_view_node,
        teleop_launch,
        ekf_localization_launch,
        start_navsat2_launch,
        navigation_launch,
    ])

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, EnvironmentVariable, LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Directories
    rtk_gps_gazebo_package_dir = get_package_share_directory('rtk_gps_gazebo')
    rtk_gps_description_pkg_dir = get_package_share_directory('rtk_gps_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # adl200_teleop_package_dir = get_package_share_directory('adl200_teleop')
    # gps_navigation_package_dir = get_package_share_directory('gps_navigation')
    # ricardo_move_package_dir = get_package_share_directory('ricardo_move')

    urdf_path = os.path.join(
        rtk_gps_description_pkg_dir,
        'urdf',
        'adl200_description.urdf')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    rviz_file = os.path.join(rtk_gps_gazebo_package_dir,'rviz','rtk_gps_rviz.rviz')

    # Arguments
    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')

    world = os.path.join(
        rtk_gps_gazebo_package_dir,
        'worlds',
        'empty_world.world'
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
    robot_description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc
        }],
    )

    # URDF Spawner
    urdf_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=['-entity', 'robot_model', '-topic', 'robot_description']
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_file]
    )

    # Archivos de lanzamiento incluidos
    # teleop_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([adl200_teleop_package_dir, 'launch', 'teleop.launch.py']))
    # )
    # ekf_localization_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([gps_navigation_package_dir, 'launch', 'ekf_localization3.launch.py']))
    # )
    # start_map_server_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([gps_navigation_package_dir, 'launch', 'start_map_server.launch.py']))
    # )
    # start_navsat2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([gps_navigation_package_dir, 'launch', 'start_navsat2.launch.py']))
    # )
    # ricardo_move_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([ricardo_move_package_dir, 'launch', 'ricardo_move2.launch.py']))
    # )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_description_node,
        urdf_spawner_node,
        joint_state_publisher_node,
        rviz_node,
        # teleop_launch,
        # ekf_localization_launch,
        # start_map_server_launch,
        # start_navsat2_launch,
        # ricardo_move_launch
    ])

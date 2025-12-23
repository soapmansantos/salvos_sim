# salvos_gz/launch/gazebo.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('salvos_gz')

    world = LaunchConfiguration('world', default='empty.sdf')
    mesh_base_abs = 'file://' + os.path.join(pkg_share, 'models', 'hermes', 'meshes')

    # Robot State Publisher (forces sim time + absolute mesh paths for Gazebo)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'mesh_base': mesh_base_abs
        }.items()
    )

    # Gazebo sim launcher
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v4 ', world],
                          'on_exit_shutdown': 'true'}.items()
    )

    # Spawn Hermes into Gazebo
    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', 'robot_description',
                           '-name', 'hermes',
                           '-z', '0.5'],
                output='screen'
            )
        ]
    )

    # Joint State Publisher GUI
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # ROS–Gazebo Bridge (loads config file)
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='World to load in Gazebo'),
        rsp,
        gazebo,
        spawn,
        jsp_gui,
        rviz,
        ros_gz_bridge   # <--- add bridge node here
    ])

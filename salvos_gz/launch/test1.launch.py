import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('salvos_gz')
    xacro_file = os.path.join(pkg_share, 'models', 'urdf', 'hermes.urdf.xacro')

    # Absolute file:// path to meshes for Gazebo
    mesh_base_abs = 'file://' + os.path.join(pkg_share, 'models', 'hermes', 'meshes')

    # Generate URDF from Xacro with overridden mesh_base
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file,
        ' ',
        'mesh_base:=' + mesh_base_abs
    ])

    # Robot State Publisher (publishes /robot_description and TFs)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': True}]
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
        name='rviz2',
        output='screen',
    )

    # Start Gazebo
    gz = ExecuteProcess(
        cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', 'gz_args:=empty.sdf'],
        output='screen'
    )

    # Spawn Hermes robot in Gazebo after 2 sec
    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'hermes',
                    '-z', '0.5'
                ]
            )
        ]
    )

    return LaunchDescription([gz, rsp, jsp_gui, rviz, spawn])

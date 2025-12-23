import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('salvos_gz')
    urdf_path = os.path.join(pkg_share, 'models', 'urdf', 'hermes.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'empty_world.sdf')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Convert package:// URIs to absolute file:// URIs for Gazebo
    mesh_path = os.path.join(pkg_share, 'models', 'hermes', 'meshes')
    robot_desc = robot_desc.replace(
        'package://salvos_gz/models/hermes/meshes/',
        f'file://{mesh_path}/'
    )

    # Define rotor joints to bridge
    rotor_joints = [
        'nose_rotor_joint',
        'fl_rotor_joint',
        'fr_rotor_joint',
        'rl_rotor_joint',
        'rr_rotor_joint'
    ]

    # Create bridge nodes for all rotor joints
    bridge_nodes = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{joint}',
            output='screen',
            arguments=[
                f'/model/hermes/joint/{joint}/cmd_vel'
                + '@std_msgs/msg/Float64[gz.msgs.Double'
            ]
        )
        for joint in rotor_joints
    ]

    return LaunchDescription([
        # Export GAZEBO_MODEL_PATH so Gazebo can resolve model:// URIs
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.path.join(pkg_share, 'models')
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc},
                        {'use_sim_time': True}]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),

        *bridge_nodes,

        # Launch Gazebo with the world
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),

        # Spawn ground plane
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_ground_plane',
                    output='screen',
                    arguments=[
                        '-name', 'ground_plane',
                        '-x', '0', '-y', '0', '-z', '0',
                        '-file', os.path.join(pkg_share, 'models', 'ground_plane', 'model.sdf')
                    ]
                )
            ]
        ),

        # Spawn Hermes
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_hermes',
                    output='screen',
                    arguments=[
                        '-topic', 'robot_description',
                        '-name', 'hermes',
                        '-x', '0', '-y', '0', '-z', '0.5'
                    ]
                )
            ]
        ),
    ])

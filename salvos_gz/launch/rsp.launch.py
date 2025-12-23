# salvos_gz/launch/rsp.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('salvos_gz')
    xacro_file = os.path.join(pkg_share, 'models', 'urdf', 'hermes.urdf.xacro')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mesh_base = LaunchConfiguration('mesh_base', default='package://salvos_gz/models/hermes/meshes')

    # Process Xacro with mesh_base
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file,
        ' ',
        'mesh_base:=', mesh_base
    ])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('mesh_base',
                              default_value='package://salvos_gz/models/hermes/meshes',
                              description='Base path for mesh files'),
        rsp
    ])

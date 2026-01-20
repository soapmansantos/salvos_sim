from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():
    world_path = '/home/santos/dev_ws/src/salvos_gz/worlds/hermes.world.sdf'
    assets_dir = '/home/santos/dev_ws/src/salvos_gz/models/hermes'  #meshes/
    bridge_yaml = '/home/santos/dev_ws/src/salvos_gz/config/gz_bridge.yaml'

    set_res_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=assets_dir
    )

    gz_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_path],
        output='screen'
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}']
    )


    gz_bridge_control = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_control',
        output='screen',
        arguments=[
            '/world/empty/control@ros_gz_interfaces/srv/ControlWorld@gz.msgs.WorldControl@gz.msgs.Boolean'
        ]
    )

    pose_bridge = Node(
        package='salvos_gz',
        executable='pose_bridge',
        name='pose_bridge',
        output='screen'
    )


    controller_node = Node(
        package='salvos_control',
        executable='main',
        name='main',
        output='screen',
        # parameters=[...],
        # remappings=[...],
    )

    controller_delayed = TimerAction(
        period=10.0,
        actions=[controller_node]
    )

    return LaunchDescription([
        set_res_path,
        gz_cmd,
        gz_bridge,
        gz_bridge_control,
        controller_delayed
        #pose_bridge,
    ])

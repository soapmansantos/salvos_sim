from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import EnvironmentVariable, TextSubstitution


def generate_launch_description():
    world_name = LaunchConfiguration("world_name")
    world_file = LaunchConfiguration("world_file")
    wait_topic = LaunchConfiguration("wait_topic")

    declare_world_name = DeclareLaunchArgument(
        "world_name",
        default_value=TextSubstitution(text="empty"),
        description="Gazebo world name used in /world/<world_name>/... topics/services",
    )

    declare_world_file = DeclareLaunchArgument(
        "world_file",
        default_value=TextSubstitution(text="hermes.world.sdf"),
        description="World SDF filename inside salvos_gz/worlds/",
    )

    declare_wait_topic = DeclareLaunchArgument(
        "wait_topic",
        default_value=TextSubstitution(text="/clock"),
        description="ROS topic to wait for before starting controller (e.g. /clock)",
    )

    salvos_gz_share = FindPackageShare("salvos_gz")

    world_path = PathJoinSubstitution([salvos_gz_share, "worlds", world_file])
    bridge_yaml = PathJoinSubstitution([salvos_gz_share, "config", "gz_bridge.yaml"])

    models_dir = PathJoinSubstitution([salvos_gz_share, "models"])


    set_res_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            models_dir,
            TextSubstitution(text=":"),
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")
        ],
    )

    # --- Start Gazebo ---
    gz_cmd = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", world_path],
        output="screen",
    )

    # --- Bridges ---
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        arguments=["--ros-args", "-p", ["config_file:=", bridge_yaml]],
    )

    gz_bridge_control = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge_control",
        output="screen",
        arguments=[
            ["/world/", world_name, "/control@ros_gz_interfaces/srv/ControlWorld@gz.msgs.WorldControl@gz.msgs.Boolean"]
        ],
    )

    # --- Controller node (salvos_control) ---
    controller_node = Node(
        package="salvos_control",
        executable="main", 
        name="main",
        output="screen",
    )

    wait_for_ready = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            [
                "set -e; "
                "echo '[wait_for_ready] Waiting for topic: '", wait_topic, "; "
                "until ros2 topic list 2>/dev/null | grep -qx '", wait_topic, "'; do "
                "  sleep 0.5; "
                "done; "
                "echo '[wait_for_ready] Topic available. Starting controller...';"
            ],
        ],
        output="screen",
    )

    start_controller_when_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_ready,
            on_exit=[controller_node],
        )
    )

    return LaunchDescription(
        [
            declare_world_name,
            declare_world_file,
            declare_wait_topic,
            set_res_path,
            gz_cmd,
            gz_bridge,
            gz_bridge_control,
            wait_for_ready,
            start_controller_when_ready,
        ]
    )
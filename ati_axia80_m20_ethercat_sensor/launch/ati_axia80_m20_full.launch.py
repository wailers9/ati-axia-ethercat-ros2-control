from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("ati_axia80_m20_ethercat_sensor")
    demo_launch = PathJoinSubstitution(
        [package_share, "launch", "ati_axia80_m20_demo.launch.py"]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "master_index",
            default_value="0",
            description="IgH/EtherLab EtherCAT master index.",
        ),
        DeclareLaunchArgument(
            "slave_position",
            default_value="0",
            description="EtherCAT slave position of the ATI Axia80-M20 sensor.",
        ),
        DeclareLaunchArgument(
            "slave_alias",
            default_value="0",
            description="EtherCAT slave alias. Use 0 when aliases are not configured.",
        ),
        DeclareLaunchArgument(
            "monitor_host",
            default_value="0.0.0.0",
            description="HTTP/WebSocket bind address for the monitor dashboard.",
        ),
        DeclareLaunchArgument(
            "monitor_port",
            default_value="8765",
            description="HTTP/WebSocket port for the monitor dashboard.",
        ),
        DeclareLaunchArgument(
            "monitor_check_period_sec",
            default_value="10.0",
            description="Low-frequency monitor check period. 10.0 sec is 0.1 Hz.",
        ),
    ]

    sensor_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch),
        launch_arguments={
            "master_index": LaunchConfiguration("master_index"),
            "slave_position": LaunchConfiguration("slave_position"),
            "slave_alias": LaunchConfiguration("slave_alias"),
        }.items(),
    )

    monitor = Node(
        package="ati_axia80_m20_ethercat_sensor",
        executable="axia80_monitor_node.py",
        name="axia80_monitor_node",
        output="screen",
        parameters=[
            {
                "host": LaunchConfiguration("monitor_host"),
                "port": LaunchConfiguration("monitor_port"),
                "check_period_sec": LaunchConfiguration("monitor_check_period_sec"),
            }
        ],
    )

    return LaunchDescription(declared_arguments + [sensor_stack, monitor])

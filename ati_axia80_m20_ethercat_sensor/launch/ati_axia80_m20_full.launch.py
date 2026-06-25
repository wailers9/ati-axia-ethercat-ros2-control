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
            "sample_rate_code",
            default_value="1",
            description="Axia sample rate: 0=487, 1=975, 2=1990, 3=3900 Hz.",
        ),
        DeclareLaunchArgument(
            "expected_sensor_rate_hz",
            default_value="975",
            description="Sensor rate used by sample-counter diagnostics.",
        ),
        DeclareLaunchArgument(
            "read_rate_hz",
            default_value="975",
            description="controller_manager update rate and expected ROS read rate.",
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
        DeclareLaunchArgument(
            "monitor_telemetry_period_sec",
            default_value="1.0",
            description="Full dashboard telemetry refresh period. Defaults to 1 Hz.",
        ),
        DeclareLaunchArgument(
            "monitor_wrench_push_rate_hz",
            default_value="25.0",
            description="WebSocket wrench push rate for chart data. Use 20-50 Hz.",
        ),
        DeclareLaunchArgument(
            "monitor_chart_refresh_rate_hz",
            default_value="20.0",
            description="Browser chart redraw rate. Use 10-30 Hz.",
        ),
    ]

    sensor_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch),
        launch_arguments={
            "master_index": LaunchConfiguration("master_index"),
            "slave_position": LaunchConfiguration("slave_position"),
            "slave_alias": LaunchConfiguration("slave_alias"),
            "sample_rate_code": LaunchConfiguration("sample_rate_code"),
            "expected_sensor_rate_hz": LaunchConfiguration("expected_sensor_rate_hz"),
            "read_rate_hz": LaunchConfiguration("read_rate_hz"),
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
                "telemetry_period_sec": LaunchConfiguration("monitor_telemetry_period_sec"),
                "wrench_push_rate_hz": LaunchConfiguration("monitor_wrench_push_rate_hz"),
                "chart_refresh_rate_hz": LaunchConfiguration("monitor_chart_refresh_rate_hz"),
            }
        ],
    )

    return LaunchDescription(declared_arguments + [sensor_stack, monitor])

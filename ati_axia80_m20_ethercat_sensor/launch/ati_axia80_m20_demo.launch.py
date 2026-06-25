from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            default_value="0",
            description="Axia sample rate: 0=487, 1=975, 2=1990, 3=3900 Hz.",
        ),
        DeclareLaunchArgument(
            "expected_sensor_rate_hz",
            default_value="487",
            description="Sensor rate used by sample-counter diagnostics.",
        ),
        DeclareLaunchArgument(
            "read_rate_hz",
            default_value="487",
            description="controller_manager update rate and expected ROS read rate.",
        ),
    ]

    package_share = FindPackageShare("ati_axia80_m20_ethercat_sensor")
    robot_description_xacro = PathJoinSubstitution(
        [package_share, "examples", "ati_axia80_m20_demo.urdf.xacro"]
    )
    controllers_yaml = PathJoinSubstitution(
        [package_share, "config", "ati_axia80_m20_controllers.yaml"]
    )

    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                robot_description_xacro,
                " master_index:=",
                LaunchConfiguration("master_index"),
                " slave_position:=",
                LaunchConfiguration("slave_position"),
                " slave_alias:=",
                LaunchConfiguration("slave_alias"),
                " sample_rate_code:=",
                LaunchConfiguration("sample_rate_code"),
                " expected_sensor_rate_hz:=",
                LaunchConfiguration("expected_sensor_rate_hz"),
                " expected_read_rate_hz:=",
                LaunchConfiguration("read_rate_hz"),
            ]
        )
    }

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_yaml,
            {"update_rate": LaunchConfiguration("read_rate_hz")},
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ati_axia80_m20_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments + [robot_state_publisher, ros2_control_node, broadcaster_spawner]
    )

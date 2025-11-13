import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    pkg_holoocean_bridge = get_package_share_directory("holoocean_bridge")
    params_file = os.path.join(pkg_holoocean_bridge, "config", "bridge_params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (HoloOcean) clock if true",
            ),
            Node(
                package="holoocean_bridge",
                executable="depth_converter",
                name="depth_converter_node",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="holoocean_bridge",
                executable="gps_converter",
                name="gps_converter_node",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="holoocean_bridge",
                executable="cmd_vel_converter",
                name="cmd_vel_converter_node",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="holoocean_bridge",
                executable="dvl_converter",
                name="dvl_converter_node",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="holoocean_bridge",
                executable="heading_converter",
                name="heading_converter_node",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="holoocean_bridge",
                executable="fin_state_publisher",
                name="fin_state_publisher_node",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="holoocean_bridge",
                executable="location_converter",
                name="location_converter_node",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="holoocean_bridge",
                executable="hsd_commander",
                name="hsd_commander_node",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="holoocean_bridge",
                executable="imu_converter",
                name="imu_converter_node",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )

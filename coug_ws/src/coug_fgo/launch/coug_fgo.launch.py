import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    pkg_share = get_package_share_directory("coug_fgo")
    fgo_params_file = os.path.join(pkg_share, "config", "fgo_params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (HoloOcean) clock if true",
            ),
            Node(
                package="coug_fgo",
                executable="factor_graph",
                name="factor_graph_node",
                output="screen",
                parameters=[fgo_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="coug_fgo",
                executable="navsat_preprocessor",
                name="navsat_preprocessor_node",
                output="screen",
                parameters=[fgo_params_file, {"use_sim_time": use_sim_time}],
            ),
            # Disable this if using a local EKF to publish odom->base_link TF
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_pub",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
    )

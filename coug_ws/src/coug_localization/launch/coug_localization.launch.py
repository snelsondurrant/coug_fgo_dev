import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    coug_loc_dir = get_package_share_directory("coug_localization")
    coug_loc_params_file = os.path.join(
        coug_loc_dir, "config", "localization_params.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (HoloOcean) clock if true",
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[coug_loc_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "odometry/local")],
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[coug_loc_params_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("odometry/filtered", "odometry/global_ekf")
                ],  # Different topic to avoid conflict with GTSAM
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            Node(
                package="robot_localization",
                executable="ukf_node",
                name="ukf_filter_node_map",
                output="screen",
                parameters=[coug_loc_params_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("odometry/filtered", "odometry/global_ukf")
                ],  # Different topic to avoid conflict with GTSAM
            ),
        ]
    )

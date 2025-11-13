import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    pkg_share = get_package_share_directory("coug_gui")
    mapviz_params_file = os.path.join(pkg_share, "config", "mapviz_params.mvc")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (HoloOcean) clock if true",
            ),
            Node(
                package="mapviz",
                executable="mapviz",
                name="mapviz",
                parameters=[{"config": mapviz_params_file}, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="swri_transform_util",
                executable="initialize_origin.py",
                name="initialize_origin",
                remappings=[
                    ("fix", "origin"),  # set by 'navsat_preprocessor_node'
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="swri_transform",
                arguments=["0", "0", "0", "0", "0", "0", "map", "origin"],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )

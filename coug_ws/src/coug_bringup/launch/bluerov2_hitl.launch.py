import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    urdf_file = LaunchConfiguration("urdf_file", default="urdf/bluerov2_holoocean.urdf.xacro")

    coug_des_dir = get_package_share_directory("coug_description")
    coug_des_launch_dir = os.path.join(coug_des_dir, "launch")
    coug_fgo_dir = get_package_share_directory("coug_fgo")
    coug_fgo_launch_dir = os.path.join(coug_fgo_dir, "launch")
    holo_bridge_dir = get_package_share_directory("holoocean_bridge")
    holo_bridge_launch_dir = os.path.join(holo_bridge_dir, "launch")

    coug_des_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_des_launch_dir, "coug_description.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "urdf_file": urdf_file,
        }.items(),
    )

    coug_fgo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fgo_launch_dir, "coug_fgo.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    holo_bridge_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(holo_bridge_launch_dir, "holoocean_bridge.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (HoloOcean) clock if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "urdf_file",
            default_value="urdf/couguv_holoocean.urdf.xacro",
            description="URDF file to load",
        )
    )
    ld.add_action(coug_des_cmd)
    ld.add_action(coug_fgo_cmd)
    ld.add_action(holo_bridge_cmd)

    return ld

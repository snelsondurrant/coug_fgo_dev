import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    coug_gui_dir = get_package_share_directory("coug_gui")
    coug_gui_launch_dir = os.path.join(coug_gui_dir, "launch")

    coug_mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_gui_launch_dir, "mapviz.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    coug_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_gui_launch_dir, "rviz.launch.py")
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
    ld.add_action(coug_mapviz_cmd)
    ld.add_action(coug_rviz_cmd)

    return ld

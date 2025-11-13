from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition


def generate_launch_description():

    pkg_share = FindPackageShare(package="coug_description").find("coug_description")
    default_urdf_path = "urdf/couguv_holoocean.urdf.xacro"
    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf_file = LaunchConfiguration("urdf_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (HoloOcean) clock if true",
            ),
            DeclareLaunchArgument(
                "urdf_file",
                default_value=default_urdf_path,
                description="Relative path to the robot's URDF or Xacro file within the package",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        # Use the xacro command to process the URDF file
                        "robot_description": ParameterValue(
                            Command(
                                ["xacro ", PathJoinSubstitution([pkg_share, urdf_file])]
                            ),
                            value_type=str,
                        ),
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=UnlessCondition(
                    use_sim_time
                ),  # HoloOcean publishes joint states
            ),
        ]
    )

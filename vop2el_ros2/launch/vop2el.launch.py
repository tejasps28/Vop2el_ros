from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    publish_debug = LaunchConfiguration("publish_debug")
    ini_file = LaunchConfiguration("ini_file")
    use_opencl = LaunchConfiguration("use_opencl")
    opencv_num_threads = LaunchConfiguration("opencv_num_threads")

    default_config = PathJoinSubstitution(
        [FindPackageShare("vop2el_ros2"), "config", "vop2el.yaml"]
    )
    default_ini = PathJoinSubstitution(
        [FindPackageShare("vop2el_ros2"), "config", "Vop2elParameters.txt"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("publish_debug", default_value="false"),
            DeclareLaunchArgument("ini_file", default_value=default_ini),
            DeclareLaunchArgument("use_opencl", default_value="false"),
            DeclareLaunchArgument("opencv_num_threads", default_value="0"),
            Node(
                package="vop2el_ros2",
                executable="vop2el_node",
                name="vop2el",
                output="screen",
                parameters=[
                    config,
                    {
                        "publish_debug": publish_debug,
                        "ini_file": ini_file,
                        "use_opencl": use_opencl,
                        "opencv_num_threads": opencv_num_threads,
                    },
                ],
            ),
        ]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    bag_file = LaunchConfiguration("bag_file")
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
            DeclareLaunchArgument("bag_file", default_value="vop2el_debug_bag"),
            DeclareLaunchArgument("ini_file", default_value=default_ini),
            DeclareLaunchArgument("use_opencl", default_value="false"),
            DeclareLaunchArgument("opencv_num_threads", default_value="0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("vop2el_ros2"), "launch", "vop2el.launch.py"]
                    )
                ),
                launch_arguments={
                    "config": config,
                    "publish_debug": "true",
                    "ini_file": ini_file,
                    "use_opencl": use_opencl,
                    "opencv_num_threads": opencv_num_threads,
                }.items(),
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    bag_file,
                    "/vo/debug",
                    "/vo/odom",
                    "/vo/path",
                    "/vo/features",
                ],
                output="screen",
            ),
        ]
    )

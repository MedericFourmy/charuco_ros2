from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("charuco_ros2")

    params_file = os.path.join(
        pkg_share,
        "config",
        "params.yaml"
    )

    camera_namespace_arg = DeclareLaunchArgument(
        "camera_namespace",
        default_value="camera",
        description="Namespace for the camera"
    )
    
    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="camera",
        description="Name of the camera"
    )

    return LaunchDescription([
        camera_namespace_arg,
        camera_name_arg,
        Node(
            package="charuco_ros2",
            executable="charuco_ros2_node",
            name="charuco_ros2_node",
            output="screen",
            parameters=[params_file],
            namespace=LaunchConfiguration("camera_namespace"),
            remappings=[
                ("image_raw", [LaunchConfiguration("camera_name"), "/color/image_raw"]),
                ("camera_info", [LaunchConfiguration("camera_name"), "/color/camera_info"]),
            ],
        )
    ])


import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("charuco_ros2")

    params_file = os.path.join(
        pkg_share,
        "config",
        "params.yaml"
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("charuco_ros2"),
            "rviz",
            "debug.rviz",
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_namespace",
            default_value="camera",
            description="Namespace for the camera"
        ),
        DeclareLaunchArgument(
            "camera_name",
            default_value="camera",
            description="Name of the camera"
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch rviz2 with debug configuration"
        ),
        DeclareLaunchArgument(
            "rviz_config_path",
            default_value=rviz_config_file,
            description="Path to a file containing RViz view configuration.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz with default view.",
        ),
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
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        )
    ])

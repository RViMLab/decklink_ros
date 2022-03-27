from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_args = []
    launch_args.append(
        DeclareLaunchArgument(
            name="cname",
            default_value="decklink",
            description="Camera name."
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="url",
            default_value="''",
            description="Camera calibration YAML file."
        )
    )

    decklink_camera = Node(
        package="decklink_ros",
        executable="decklink_ros_node",
        namespace=LaunchConfiguration("cname"),
        parameters=[{
            "cname": LaunchConfiguration("cname"),
            "url": LaunchConfiguration("url")
        }]
    )

    return LaunchDescription(
        launch_args + [
            decklink_camera
    ])

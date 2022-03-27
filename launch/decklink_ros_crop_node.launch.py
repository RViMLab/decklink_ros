from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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

    launch_args.append(
        DeclareLaunchArgument(
            name="width",
            default_value="0",
            description="Cropping width"
        )
    )


    launch_args.append(
        DeclareLaunchArgument(
            name="height",
            default_value="0",
            description="Cropping height"
        )
    )


    launch_args.append(
        DeclareLaunchArgument(
            name="offset_x",
            default_value="0",
            description="Cropping offset_x"
        )
    )


    launch_args.append(
        DeclareLaunchArgument(
            name="offset_y",
            default_value="0",
            description="Cropping offset_y"
        )
    )

    decklink_camera = Node(
        package="decklink_ros",
        executable="decklink_ros_node",
        namespace=LaunchConfiguration("cname"),
        parameters=[{
            "cname": LaunchConfiguration("cname"),
            "url": LaunchConfiguration("url")
        } ]
    )

    crop_node = ComposableNodeContainer(
        name="image_proc_container",
        namespace=LaunchConfiguration("cname"),
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::CropDecimateNode",
                name="crop_decimate_node",
                namespace=LaunchConfiguration("cname"),
                remappings=[
                    ("in/image_raw", "image_raw"),
                    ("in/camera_info", "camera_info"),
                    ("out/image_raw", "image_raw/crop"),
                    ("out/camera_info", "camera_info/crop")
                ],
                parameters=[{
                    "queue_size": 1,
                    "width": LaunchConfiguration("width"),
                    "height": LaunchConfiguration("height"),
                    "offset_x": LaunchConfiguration("offset_x"),
                    "offset_y": LaunchConfiguration("offset_y")
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output="both"
    )

    return LaunchDescription(
        launch_args + [
            decklink_camera,
            crop_node
    ])

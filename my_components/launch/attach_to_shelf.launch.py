from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    obstacle = LaunchConfiguration("obstacle")
    degrees = LaunchConfiguration("degrees")

    container = ComposableNodeContainer(
        name="my_container",
        namespace="",
        package="rclcpp_components",
        # Version multi-thread : le service /approach_shelf est bloquant.
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="my_components",
                plugin="my_components::PreApproach",
                name="pre_approach",
                parameters=[
                    {
                        "obstacle": ParameterValue(obstacle, value_type=float),
                        "degrees": ParameterValue(degrees, value_type=int),
                    }
                ],
            ),
            ComposableNode(
                package="my_components",
                plugin="my_components::AttachServer",
                name="attach_server",
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("obstacle", default_value="0.3"),
            DeclareLaunchArgument("degrees", default_value="-90"),
            container,
        ]
    )

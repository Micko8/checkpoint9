from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("my_components"),
        "rviz_config",
        "config.rviz",
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_node",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": True}],
    )

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
                parameters=[{"use_sim_time": True}],
            ),
            ComposableNode(
                package="my_components",
                plugin="my_components::AttachServer",
                name="attach_server",
                parameters=[{"use_sim_time": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_node,
            container,
        ]
    )

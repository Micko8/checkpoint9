from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    obstacle = LaunchConfiguration('obstacle')
    degrees = LaunchConfiguration('degrees')
    final_approach = LaunchConfiguration('final_approach')

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('attach_shelf'),
                'launch',
                'start_rviz.launch.py',
            ])
        )
    )

    approach_server = Node(
        package='attach_shelf',
        executable='approach_service_server_exe',
        name='approach_service_server',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],
    )

    pre_approach = Node(
        package='attach_shelf',
        executable='pre_approach_v2_exe',
        name='pre_approach_v2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'obstacle': ParameterValue(obstacle, value_type=float),
            'degrees': ParameterValue(degrees, value_type=int),
            'final_approach': ParameterValue(
                final_approach, value_type=bool
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('obstacle', default_value='0.0'),
        DeclareLaunchArgument('degrees', default_value='0'),
        DeclareLaunchArgument('final_approach', default_value='false'),
        rviz_launch,
        approach_server,
        pre_approach,
    ])

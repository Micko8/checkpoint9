import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Usage examples:
# ros2 launch launch_tests_pkg start_rviz_with_arguments.launch.py
# ros2 launch launch_tests_pkg start_rviz_with_arguments.launch.py rviz_config_file_name:=launch_part.rviz


def generate_launch_description():
    package_name = 'attach_shelf'

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'rviz_config',
        'config.rviz',
    ])

    log_rviz_path = LogInfo(msg=['RViz config: ', rviz_config_path])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        log_rviz_path,
        rviz_node,
    ])
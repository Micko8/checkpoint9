from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'my_components'

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
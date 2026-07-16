import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_name = "map_server"

    map_file = LaunchConfiguration("map_file")

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="warehouse_map_sim.yaml",
        description="Name of the map file name to read by the map_server"
    )

    map_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        "config",
        map_file
    ])

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "yaml_filename": map_path,
            },
        ],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapper",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "map_server",
                ],
            },
        ],
    )

    rviz_config_file_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'rviz',
        'map_display.rviz'
    ])

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_file_path],
        parameters=[{"use_sim_time" : True}]
    )

    return LaunchDescription([
        map_file_arg,
        map_server_node,
        rviz2_node,
        lifecycle_manager_node,
    ])

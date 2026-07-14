import os
from launch import LaunchDescription

from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():

    package_name = "map_server"
    
    map_file_name = "warehouse_map_sim.yaml"

    map_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        map_file_name
    )

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

    rviz_config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'map_display.rviz'
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_file_path],
        parameters=[{"use_sim_time" : True}]
    )

    return LaunchDescription([
        map_server_node,
        rviz2_node,
        lifecycle_manager_node,
    ])

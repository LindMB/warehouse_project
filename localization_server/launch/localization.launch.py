from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() :

    map_file = LaunchConfiguration("map_file")

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="warehouse_map_sim.yaml",
        description="Name of the map file name to read by the map_server"
    )

    map_path = PathJoinSubstitution([
        FindPackageShare("map_server"),
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

    amcl_config_yaml_filepath = PathJoinSubstitution([
        FindPackageShare("localization_server"),
        "config",
        "amcl_config_sim.yaml"
    ])

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config_yaml_filepath,
            {"use_sim_time": True},
        ]
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
                    "amcl"
                ],
            },
        ],
    )

    rviz_config_file_path = PathJoinSubstitution([
        FindPackageShare("map_server"),
        'rviz',
        'map_display.rviz'
    ])

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file_path],
        parameters=[{"use_sim_time" : True}]
    )

    return LaunchDescription([
        map_file_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz2_node
    ])
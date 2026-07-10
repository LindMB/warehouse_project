import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description() :

    cartographer_config_dir = os.path.join(
        get_package_share_directory('cartographer_slam'), 
        'config'
    )

    configuration_basename = 'cartographer_sim.lua'

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename]
    )

    occupancy_grid_node =  Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "-resolution", "0.05",
            "-publish_period_sec", "1.0"
        ],
    )

    package_name = "cartographer_slam"

    rviz_config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'mapping.rviz'
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d ', rviz_config_file_path],
        parameters=[{"use_sim_time" : True}]
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
        rviz2_node
    ])
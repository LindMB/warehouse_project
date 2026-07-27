from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use the simulation clock when true.",
    )

    package_share = FindPackageShare("path_planner_server")

    filters_config = PathJoinSubstitution([
        package_share,
        "config",
        "filters.yaml",
    ])

    planner_config_file = PythonExpression([
        "'planner_sim.yaml' if ",
        use_sim_time,
        " else 'planner_real.yaml'",
    ])

    planner_config = PathJoinSubstitution([
        package_share,
        "config",
        planner_config_file,
    ])

    controller_config_file = PythonExpression([
        "'controller_sim.yaml' if ",
        use_sim_time,
        " else 'controller_real.yaml'",
    ])

    controller_config = PathJoinSubstitution([
        package_share,
        "config",
        controller_config_file,
    ])


    recoveries_config_file = PythonExpression([
        "'recoveries_sim.yaml' if ",
        use_sim_time,
        " else 'recoveries_real.yaml'",
    ])

    recoveries_config = PathJoinSubstitution([
        package_share,
        "config",
        recoveries_config_file,
    ])


    bt_navigator_config_file = PythonExpression([
        "'bt_navigator_sim.yaml' if ",
        use_sim_time,
        " else 'bt_navigator_real.yaml'",
    ])

    bt_navigator_config = PathJoinSubstitution([
        package_share,
        "config",
        bt_navigator_config_file,
    ])

    rviz_config = PathJoinSubstitution([
        package_share,
        "rviz",
        "pathplanning.rviz",
    ])

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            planner_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    robot_cmd_vel_topic = PythonExpression([
        "'/diffbot_base_controller/cmd_vel_unstamped' if ",
        use_sim_time,
        " else '/cmd_vel'",
    ])

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            controller_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cmd_vel", robot_cmd_vel_topic),
        ],
    )

    recoveries_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            recoveries_config,
            {"use_sim_time": use_sim_time},
        ],

        remappings=[
            ("cmd_vel", robot_cmd_vel_topic),
        ],
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            bt_navigator_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    filter_mask_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[filters_config]
    )

    costmap_filter_info_server_node = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[filters_config]    
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_pathplanner",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,

                "node_names": [
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                    "filter_mask_server",
                    "costmap_filter_info_server"
                ],
            }
        ],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            planner_server_node,
            controller_server_node,
            recoveries_server_node,
            bt_navigator_node,
            filter_mask_server_node,
            costmap_filter_info_server_node,
            lifecycle_manager_node,
            rviz2_node,
        ]
    )
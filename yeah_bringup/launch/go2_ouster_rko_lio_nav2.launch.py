from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    rko_rviz = LaunchConfiguration("rko_rviz")
    rko_mode = LaunchConfiguration("rko_mode")
    rko_config_file = LaunchConfiguration("rko_config_file")
    rko_global_map_path = LaunchConfiguration("rko_global_map_path")
    rko_start_delay_sec = LaunchConfiguration("rko_start_delay_sec")

    map_yaml = LaunchConfiguration("map_yaml")

    input_cloud_topic = LaunchConfiguration("input_cloud_topic")
    output_scan_topic = LaunchConfiguration("output_scan_topic")
    scan_target_frame = LaunchConfiguration("scan_target_frame")
    scan_transform_tolerance = LaunchConfiguration("scan_transform_tolerance")
    scan_min_height = LaunchConfiguration("scan_min_height")
    scan_max_height = LaunchConfiguration("scan_max_height")
    scan_range_min = LaunchConfiguration("scan_range_min")
    scan_range_max = LaunchConfiguration("scan_range_max")

    nav2_launch_dir = LaunchConfiguration("nav2_launch_dir")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav2_use_composition = LaunchConfiguration("nav2_use_composition")
    nav2_use_respawn = LaunchConfiguration("nav2_use_respawn")
    nav2_container_name = LaunchConfiguration("nav2_container_name")
    nav2_log_level = LaunchConfiguration("nav2_log_level")
    nav2_start_delay_sec = LaunchConfiguration("nav2_start_delay_sec")

    use_nav2_rviz = LaunchConfiguration("use_nav2_rviz")
    nav2_rviz_config = LaunchConfiguration("nav2_rviz_config")
    use_pointcloud_to_laserscan = LaunchConfiguration("use_pointcloud_to_laserscan")

    # nav2_bringup's launch files internally use PythonExpression("not <arg>"),
    # which requires capitalized booleans ('True'/'False').
    nav2_use_composition_normalized = PythonExpression(
        [
            "'True' if '",
            nav2_use_composition,
            "'.lower() in ['true', '1', 'yes'] else 'False'",
        ]
    )
    nav2_use_respawn_normalized = PythonExpression(
        [
            "'True' if '",
            nav2_use_respawn,
            "'.lower() in ['true', '1', 'yes'] else 'False'",
        ]
    )

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("yeah_bringup"), "launch", "go2_ouster_rko_lio.launch.py"]
            )
        ),
        launch_arguments={
            "rko_rviz": rko_rviz,
            "mode": rko_mode,
            "config_file": rko_config_file,
            "global_map_path": rko_global_map_path,
            "rko_start_delay_sec": rko_start_delay_sec,
            "go2_use_rviz": "false",
        }.items(),
    )

    cloud_to_scan = Node(
        condition=IfCondition(use_pointcloud_to_laserscan),
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_nav2",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "target_frame": scan_target_frame,
                "transform_tolerance": ParameterValue(
                    scan_transform_tolerance, value_type=float
                ),
                "min_height": ParameterValue(scan_min_height, value_type=float),
                "max_height": ParameterValue(scan_max_height, value_type=float),
                "angle_min": -3.141592653589793,
                "angle_max": 3.141592653589793,
                "angle_increment": 0.003490658503988659,
                "scan_time": 0.1,
                "range_min": ParameterValue(scan_range_min, value_type=float),
                "range_max": ParameterValue(scan_range_max, value_type=float),
                "use_inf": True,
                "inf_epsilon": 1.0,
                "queue_size": 1,
            }
        ],
        remappings=[
            ("cloud_in", input_cloud_topic),
            ("scan", output_scan_topic),
        ],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "yaml_filename": map_yaml,
            }
        ],
        arguments=["--ros-args", "--log-level", nav2_log_level],
    )

    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"autostart": ParameterValue(autostart, value_type=bool)},
            {"node_names": ["map_server"]},
        ],
        arguments=["--ros-args", "--log-level", nav2_log_level],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_launch_dir, "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": nav2_params_file,
            "use_composition": nav2_use_composition_normalized,
            "use_respawn": nav2_use_respawn_normalized,
            "container_name": nav2_container_name,
            "log_level": nav2_log_level,
        }.items(),
    )

    nav2_rviz = Node(
        condition=IfCondition(use_nav2_rviz),
        package="rviz2",
        executable="rviz2",
        name="nav2_rviz",
        arguments=["-d", nav2_rviz_config],
        output="screen",
    )

    goal_pose_bridge = Node(
        condition=IfCondition(LaunchConfiguration("use_goal_pose_bridge")),
        package="yeah_bringup",
        executable="goal_pose_to_nav2_action.py",
        name="goal_pose_to_nav2_action_bridge",
        output="screen",
        parameters=[
            {
                "input_topic": "/goal_pose",
                "action_name": "/navigate_to_pose",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("rko_rviz", default_value="false"),
            DeclareLaunchArgument("rko_mode", default_value="online"),
            DeclareLaunchArgument(
                "rko_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("yeah_bringup"), "config", "rko_lio_go2.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "rko_global_map_path",
                default_value="/home/taehyun/go2_ws/glim_map/b1_hallway_20260323.ply",
            ),
            DeclareLaunchArgument("rko_start_delay_sec", default_value="0.0"),
            DeclareLaunchArgument(
                "map_yaml",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("yeah_bringup"), "map", "b1_hallway", "map.yaml"]
                ),
            ),
            DeclareLaunchArgument("input_cloud_topic", default_value="/ouster/points"),
            DeclareLaunchArgument("output_scan_topic", default_value="/scan_nav2"),
            DeclareLaunchArgument("scan_target_frame", default_value="base_link"),
            DeclareLaunchArgument("scan_transform_tolerance", default_value="0.1"),
            DeclareLaunchArgument("scan_min_height", default_value="-0.02"),
            DeclareLaunchArgument("scan_max_height", default_value="0.50"),
            DeclareLaunchArgument("scan_range_min", default_value="0.35"),
            DeclareLaunchArgument("scan_range_max", default_value="20.0"),
            DeclareLaunchArgument("use_pointcloud_to_laserscan", default_value="true"),
            DeclareLaunchArgument(
                "nav2_launch_dir",
                default_value="/opt/ros/humble/share/nav2_bringup/launch",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("yeah_bringup"), "config", "nav2_go2.yaml"]
                ),
            ),
            DeclareLaunchArgument("nav2_use_composition", default_value="False"),
            DeclareLaunchArgument("nav2_use_respawn", default_value="False"),
            DeclareLaunchArgument("nav2_container_name", default_value="nav2_container"),
            DeclareLaunchArgument("nav2_log_level", default_value="info"),
            # RKO-LIO online localization needs an /initialpose before publishing a stable odom tree.
            # Keep a conservative default delay so Nav2 does not activate too early.
            DeclareLaunchArgument("nav2_start_delay_sec", default_value="30.0"),
            DeclareLaunchArgument("use_nav2_rviz", default_value="false"),
            DeclareLaunchArgument("use_goal_pose_bridge", default_value="true"),
            DeclareLaunchArgument(
                "nav2_rviz_config",
                default_value="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz",
            ),
            base_stack,
            cloud_to_scan,
            map_server,
            map_lifecycle_manager,
            TimerAction(period=nav2_start_delay_sec, actions=[navigation]),
            nav2_rviz,
            goal_pose_bridge,
        ]
    )

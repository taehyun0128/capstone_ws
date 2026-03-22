from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    play_bag = LaunchConfiguration("play_bag")
    bag_loop = LaunchConfiguration("bag_loop")
    bag_rate = LaunchConfiguration("bag_rate")
    bag_start_offset = LaunchConfiguration("bag_start_offset")
    bag_delay_sec = LaunchConfiguration("bag_delay_sec")

    use_sim_time = LaunchConfiguration("use_sim_time")
    points_topic = LaunchConfiguration("points_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    globalmap_pcd = LaunchConfiguration("globalmap_pcd")
    odom_child_frame_id = LaunchConfiguration("odom_child_frame_id")
    robot_odom_frame_id = LaunchConfiguration("robot_odom_frame_id")

    use_imu = LaunchConfiguration("use_imu")
    invert_imu_acc = LaunchConfiguration("invert_imu_acc")
    invert_imu_gyro = LaunchConfiguration("invert_imu_gyro")
    use_global_localization = LaunchConfiguration("use_global_localization")
    enable_robot_odometry_prediction = LaunchConfiguration(
        "enable_robot_odometry_prediction"
    )
    reg_method = LaunchConfiguration("reg_method")
    ndt_neighbor_search_method = LaunchConfiguration("ndt_neighbor_search_method")
    ndt_neighbor_search_radius = LaunchConfiguration("ndt_neighbor_search_radius")
    ndt_resolution = LaunchConfiguration("ndt_resolution")
    downsample_resolution = LaunchConfiguration("downsample_resolution")

    publish_lidar_tf = LaunchConfiguration("publish_lidar_tf")
    lidar_tf_parent_frame = LaunchConfiguration("lidar_tf_parent_frame")
    lidar_tf_child_frame = LaunchConfiguration("lidar_tf_child_frame")
    lidar_tf_x = LaunchConfiguration("lidar_tf_x")
    lidar_tf_y = LaunchConfiguration("lidar_tf_y")
    lidar_tf_z = LaunchConfiguration("lidar_tf_z")
    lidar_tf_roll = LaunchConfiguration("lidar_tf_roll")
    lidar_tf_pitch = LaunchConfiguration("lidar_tf_pitch")
    lidar_tf_yaw = LaunchConfiguration("lidar_tf_yaw")

    use_imu_bridge = LaunchConfiguration("use_imu_bridge")
    imu_bridge_in_topic = LaunchConfiguration("imu_bridge_in_topic")
    imu_bridge_out_topic = LaunchConfiguration("imu_bridge_out_topic")
    imu_bridge_frame_id = LaunchConfiguration("imu_bridge_frame_id")

    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    play_bag_if_path_given = IfCondition(
        PythonExpression(["'", play_bag, "' == 'true' and '", bag_path, "' != ''"])
    )
    play_bag_once = IfCondition(
        PythonExpression(
            [
                "'",
                play_bag,
                "' == 'true' and '",
                bag_loop,
                "' == 'false' and '",
                bag_path,
                "' != ''",
            ]
        )
    )
    play_bag_looping = IfCondition(
        PythonExpression(
            [
                "'",
                play_bag,
                "' == 'true' and '",
                bag_loop,
                "' == 'true' and '",
                bag_path,
                "' != ''",
            ]
        )
    )

    declare_args = [
        DeclareLaunchArgument(
            "bag_path",
            default_value="",
            description="Absolute path to rosbag2 directory. Set this when play_bag:=true.",
        ),
        DeclareLaunchArgument("play_bag", default_value="true"),
        DeclareLaunchArgument("bag_loop", default_value="false"),
        DeclareLaunchArgument("bag_rate", default_value="1.0"),
        DeclareLaunchArgument("bag_start_offset", default_value="0.0"),
        DeclareLaunchArgument("bag_delay_sec", default_value="2.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("points_topic", default_value="/ouster/points"),
        DeclareLaunchArgument("imu_topic", default_value="/imu_for_glim"),
        DeclareLaunchArgument(
            "globalmap_pcd",
            default_value=PathJoinSubstitution(
                [FindPackageShare("yeah_bringup"), "map", "glim_map.pcd"]
            ),
            description="Absolute path to global map PCD file.",
        ),
        DeclareLaunchArgument("odom_child_frame_id", default_value="base_link"),
        DeclareLaunchArgument("robot_odom_frame_id", default_value="odom"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("invert_imu_acc", default_value="false"),
        DeclareLaunchArgument("invert_imu_gyro", default_value="false"),
        DeclareLaunchArgument("use_global_localization", default_value="false"),
        DeclareLaunchArgument("enable_robot_odometry_prediction", default_value="false"),
        DeclareLaunchArgument("reg_method", default_value="NDT_OMP"),
        DeclareLaunchArgument("ndt_neighbor_search_method", default_value="DIRECT7"),
        DeclareLaunchArgument("ndt_neighbor_search_radius", default_value="1.0"),
        DeclareLaunchArgument("ndt_resolution", default_value="0.5"),
        DeclareLaunchArgument("downsample_resolution", default_value="0.1"),
        DeclareLaunchArgument("publish_lidar_tf", default_value="false"),
        DeclareLaunchArgument("lidar_tf_parent_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_tf_child_frame", default_value="os_sensor"),
        DeclareLaunchArgument("lidar_tf_x", default_value="0.20"),
        DeclareLaunchArgument("lidar_tf_y", default_value="0.00"),
        DeclareLaunchArgument("lidar_tf_z", default_value="0.08"),
        DeclareLaunchArgument("lidar_tf_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_imu_bridge", default_value="false"),
        DeclareLaunchArgument("imu_bridge_in_topic", default_value="/imu"),
        DeclareLaunchArgument("imu_bridge_out_topic", default_value="/imu_for_glim"),
        DeclareLaunchArgument("imu_bridge_frame_id", default_value="imu"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("hdl_localization"),
                    "rviz",
                    "hdl_localization_ros2.rviz",
                ]
            ),
        ),
    ]

    container = ComposableNodeContainer(
        name="hdl_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="hdl_localization",
                plugin="hdl_localization::GlobalmapServerNodelet",
                name="GlobalmapServerNodelet",
                parameters=[
                    {"globalmap_pcd": globalmap_pcd},
                    {"convert_utm_to_local": True},
                    {"downsample_resolution": downsample_resolution},
                ],
            ),
            ComposableNode(
                package="hdl_localization",
                plugin="hdl_localization::HdlLocalizationNodelet",
                name="HdlLocalizationNodelet",
                remappings=[
                    ("/ouster/points", points_topic),
                    ("/velodyne_points", points_topic),
                    ("/gpsimu_driver/imu_data", imu_topic),
                ],
                parameters=[
                    {"odom_child_frame_id": odom_child_frame_id},
                    {"robot_odom_frame_id": robot_odom_frame_id},
                    {"use_imu": use_imu},
                    {"invert_acc": invert_imu_acc},
                    {"invert_gyro": invert_imu_gyro},
                    {"cool_time_duration": 2.0},
                    {"enable_robot_odometry_prediction": enable_robot_odometry_prediction},
                    {"reg_method": reg_method},
                    {"ndt_neighbor_search_method": ndt_neighbor_search_method},
                    {"ndt_neighbor_search_radius": ndt_neighbor_search_radius},
                    {"ndt_resolution": ndt_resolution},
                    {"downsample_resolution": downsample_resolution},
                    {"specify_init_pose": True},
                    {"init_pos_x": 0.0},
                    {"init_pos_y": 0.0},
                    {"init_pos_z": 0.0},
                    {"init_ori_w": 1.0},
                    {"init_ori_x": 0.0},
                    {"init_ori_y": 0.0},
                    {"init_ori_z": 0.0},
                    {"use_global_localization": use_global_localization},
                ],
            ),
        ],
        output="screen",
    )

    optional_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="hdl_lidar_tf",
        condition=IfCondition(publish_lidar_tf),
        arguments=[
            lidar_tf_x,
            lidar_tf_y,
            lidar_tf_z,
            lidar_tf_roll,
            lidar_tf_pitch,
            lidar_tf_yaw,
            lidar_tf_parent_frame,
            lidar_tf_child_frame,
        ],
    )

    optional_imu_bridge = Node(
        package="yeah_bringup",
        executable="imu_to_go2imu.py",
        name="imu_to_glim_bridge",
        condition=IfCondition(use_imu_bridge),
        output="screen",
        parameters=[
            {
                "in_topic": imu_bridge_in_topic,
                "out_topic": imu_bridge_out_topic,
                "frame_id": imu_bridge_frame_id,
            }
        ],
    )

    optional_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(rviz),
        arguments=["-d", rviz_config],
        output="screen",
    )

    bag_path_missing_log = LogInfo(
        condition=IfCondition(
            PythonExpression(["'", play_bag, "' == 'true' and '", bag_path, "' == ''"])
        ),
        msg="play_bag:=true but bag_path is empty. Set bag_path:=/absolute/path/to/rosbag2_dir",
    )

    bag_play_once = ExecuteProcess(
        condition=play_bag_once,
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--clock",
            "--rate",
            bag_rate,
            "--start-offset",
            bag_start_offset,
        ],
        output="screen",
    )

    bag_play_loop = ExecuteProcess(
        condition=play_bag_looping,
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--clock",
            "--loop",
            "--rate",
            bag_rate,
            "--start-offset",
            bag_start_offset,
        ],
        output="screen",
    )

    delayed_bag_player = TimerAction(
        period=bag_delay_sec,
        condition=play_bag_if_path_given,
        actions=[bag_play_once, bag_play_loop],
    )

    return LaunchDescription(
        declare_args
        + [
            SetParameter(name="use_sim_time", value=use_sim_time),
            bag_path_missing_log,
            optional_lidar_tf,
            optional_imu_bridge,
            container,
            optional_rviz,
            delayed_bag_player,
        ]
    )

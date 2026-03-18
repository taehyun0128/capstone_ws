from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    points_topic = LaunchConfiguration("points_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    globalmap_pcd = LaunchConfiguration("globalmap_pcd")
    odom_child_frame_id = LaunchConfiguration("odom_child_frame_id")
    robot_odom_frame_id = LaunchConfiguration("robot_odom_frame_id")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_imu = LaunchConfiguration("use_imu")
    invert_imu_acc = LaunchConfiguration("invert_imu_acc")
    invert_imu_gyro = LaunchConfiguration("invert_imu_gyro")
    use_global_localization = LaunchConfiguration("use_global_localization")
    enable_robot_odometry_prediction = LaunchConfiguration("enable_robot_odometry_prediction")

    publish_lidar_tf = LaunchConfiguration("publish_lidar_tf")
    lidar_tf_parent_frame = LaunchConfiguration("lidar_tf_parent_frame")
    lidar_tf_child_frame = LaunchConfiguration("lidar_tf_child_frame")
    lidar_tf_x = LaunchConfiguration("lidar_tf_x")
    lidar_tf_y = LaunchConfiguration("lidar_tf_y")
    lidar_tf_z = LaunchConfiguration("lidar_tf_z")
    lidar_tf_roll = LaunchConfiguration("lidar_tf_roll")
    lidar_tf_pitch = LaunchConfiguration("lidar_tf_pitch")
    lidar_tf_yaw = LaunchConfiguration("lidar_tf_yaw")

    declare_args = [
        DeclareLaunchArgument("globalmap_pcd", default_value="", description="Absolute path to GLIM map PCD"),
        DeclareLaunchArgument("points_topic", default_value="/ouster/points"),
        DeclareLaunchArgument("imu_topic", default_value="/imu_for_glim"),
        DeclareLaunchArgument("odom_child_frame_id", default_value="base_link"),
        DeclareLaunchArgument("robot_odom_frame_id", default_value="odom"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("invert_imu_acc", default_value="false"),
        DeclareLaunchArgument("invert_imu_gyro", default_value="false"),
        DeclareLaunchArgument("use_global_localization", default_value="false"),
        DeclareLaunchArgument("enable_robot_odometry_prediction", default_value="false"),
        DeclareLaunchArgument("publish_lidar_tf", default_value="false"),
        DeclareLaunchArgument("lidar_tf_parent_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_tf_child_frame", default_value="os_sensor"),
        DeclareLaunchArgument("lidar_tf_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_z", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_yaw", default_value="0.0"),
    ]

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
                    {"downsample_resolution": 0.1},
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
                    {"reg_method": "NDT_OMP"},
                    {"ndt_neighbor_search_method": "DIRECT7"},
                    {"ndt_neighbor_search_radius": 1.0},
                    {"ndt_resolution": 0.5},
                    {"downsample_resolution": 0.1},
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

    return LaunchDescription(
        declare_args
        + [
            launch_ros.actions.SetParameter(name="use_sim_time", value=use_sim_time),
            optional_lidar_tf,
            container,
        ]
    )

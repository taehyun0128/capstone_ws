from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # ---- launch args (you can override at runtime) ----
    ouster_ns = LaunchConfiguration("ouster_ns")
    sensor_hostname = LaunchConfiguration("sensor_hostname")
    udp_dest = LaunchConfiguration("udp_dest")
    lidar_port = LaunchConfiguration("lidar_port")
    imu_port = LaunchConfiguration("imu_port")
    timestamp_mode = LaunchConfiguration("timestamp_mode")
    sensor_frame = LaunchConfiguration("sensor_frame")
    lidar_frame = LaunchConfiguration("lidar_frame")
    imu_frame = LaunchConfiguration("imu_frame")
    point_cloud_frame = LaunchConfiguration("point_cloud_frame")
    pub_static_tf = LaunchConfiguration("pub_static_tf")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_imu_bridge = LaunchConfiguration("use_imu_bridge")
    imu_bridge_in_topic = LaunchConfiguration("imu_bridge_in_topic")
    imu_bridge_out_topic = LaunchConfiguration("imu_bridge_out_topic")
    imu_bridge_frame_id = LaunchConfiguration("imu_bridge_frame_id")

    # static tf (edit these after you measure the bracket)
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    pitch = LaunchConfiguration("pitch")
    roll = LaunchConfiguration("roll")

    # ---- include go2.launch.py ----
    go2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("go2_bringup").find("go2_bringup"),
                "launch",
                "go2.launch.py",
            )
        ),
        launch_arguments={
            "rviz": "False",
        }.items(),
    )

    # ---- include ouster sensor launch ----
    ouster_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("ouster_ros").find("ouster_ros"),
                "launch",
                "sensor.independent.launch.xml",
            )
        ),
        launch_arguments={
            "ouster_ns": ouster_ns,
            "sensor_hostname": sensor_hostname,
            "udp_dest": udp_dest,
            "lidar_port": lidar_port,
            "imu_port": imu_port,
            "timestamp_mode": timestamp_mode,
            "sensor_frame": sensor_frame,
            "lidar_frame": lidar_frame,
            "imu_frame": imu_frame,
            "point_cloud_frame": point_cloud_frame,
            "pub_static_tf": pub_static_tf,
            "viz": "False",  # avoid launching ouster's own rviz
        }.items(),
    )

    # ---- static TF: base_link -> ouster frame (choose os_sensor or os_lidar) ----
    # NOTE: in sensor.independent.launch.xml default frames are os_sensor/os_lidar/os_imu
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_ouster_tf",
        arguments=[x, y, z, yaw, pitch, roll, "base_link", "os_sensor"],
        output="screen",
    )

    # ---- RViz (optional): use go2_rviz config if you want ----
    rviz_node = Node(
        condition=IfCondition(
            PythonExpression(["'", use_rviz, "'.lower() == 'true'"])
        ),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    imu_bridge_node = Node(
        condition=IfCondition(use_imu_bridge),
        package="yeah_bringup",
        executable="imu_to_go2imu.py",
        name="imu_to_glim_bridge",
        output="screen",
        parameters=[
            {
                "in_topic": imu_bridge_in_topic,
                "out_topic": imu_bridge_out_topic,
                "frame_id": imu_bridge_frame_id,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ouster_ns", default_value="ouster"),
            DeclareLaunchArgument("sensor_hostname", default_value="192.168.123.32"),
            DeclareLaunchArgument("udp_dest", default_value="192.168.123.101"),
            DeclareLaunchArgument("lidar_port", default_value="7502"),
            DeclareLaunchArgument("imu_port", default_value="7503"),
            DeclareLaunchArgument(
                "timestamp_mode", default_value="TIME_FROM_ROS_TIME"
            ),
            DeclareLaunchArgument("sensor_frame", default_value="os_sensor"),
            DeclareLaunchArgument("lidar_frame", default_value="os_lidar"),
            DeclareLaunchArgument("imu_frame", default_value="os_imu"),
            DeclareLaunchArgument("point_cloud_frame", default_value="os_sensor"),
            DeclareLaunchArgument("pub_static_tf", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_imu_bridge", default_value="true"),
            DeclareLaunchArgument("imu_bridge_in_topic", default_value="/imu"),
            DeclareLaunchArgument(
                "imu_bridge_out_topic", default_value="/go2/imu"
            ),
            DeclareLaunchArgument("imu_bridge_frame_id", default_value="imu"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    FindPackageShare("yeah_bringup").find("yeah_bringup"),
                    "config",
                    "go2_ouster_odom.rviz",
                ),
            ),
            # static tf defaults (TEMP). Fix after measurement.
            DeclareLaunchArgument("x", default_value="0.20"),
            DeclareLaunchArgument("y", default_value="0.00"),
            DeclareLaunchArgument("z", default_value="0.08"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("pitch", default_value="0.0"),
            DeclareLaunchArgument("roll", default_value="0.0"),
            go2_launch,
            ouster_launch,
            static_tf,
            imu_bridge_node,
            rviz_node,
        ]
    )

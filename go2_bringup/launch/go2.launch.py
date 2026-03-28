import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    realsense = LaunchConfiguration('realsense')
    rviz = LaunchConfiguration('rviz')
    # ADDED (2026-03-27): pass-through args for go2_driver TF isolation.
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    publish_odom_msg = LaunchConfiguration('publish_odom_msg')
    tf_topic = LaunchConfiguration('tf_topic')

    declare_realsense_cmd = DeclareLaunchArgument(
        'realsense',
        default_value='False',
        description='Launch realsense driver'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Launch rviz'
    )

    declare_publish_odom_tf_cmd = DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='True',
        description='Publish odom->base_link TF from go2_driver'
    )

    declare_publish_odom_msg_cmd = DeclareLaunchArgument(
        'publish_odom_msg',
        default_value='True',
        description='Publish /odom from go2_driver'
    )

    declare_tf_topic_cmd = DeclareLaunchArgument(
        'tf_topic',
        default_value='/tf',
        description='Output TF topic for go2_driver'
    )

    # 실기체면 use_sim_time False 추천
    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_description'),
            'launch/'), 'robot.launch.py']),
        launch_arguments={'use_sim_time': 'False'}.items()
    )

    driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_driver'),
            'launch/'), 'go2_driver.launch.py']),
        # KEPT AS COMMENT (2026-03-27): original launch did not pass driver args.
        # launch_arguments={}.items()
        launch_arguments={
            'publish_odom_tf': publish_odom_tf,
            'publish_odom_msg': publish_odom_msg,
            'tf_topic': tf_topic,
        }.items()
    )

    realsense_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch/'), 'rs_launch.py']),
        condition=IfCondition(PythonExpression([realsense]))
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_rviz'),
            'launch/'), 'rviz.launch.py']),
        condition=IfCondition(PythonExpression([rviz]))
    )

    ld = LaunchDescription()
    ld.add_action(declare_realsense_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_publish_odom_tf_cmd)
    ld.add_action(declare_publish_odom_msg_cmd)
    ld.add_action(declare_tf_topic_cmd)
    ld.add_action(robot_description_cmd)
    ld.add_action(realsense_cmd)
    ld.add_action(driver_cmd)
    ld.add_action(rviz_cmd)

    return ld

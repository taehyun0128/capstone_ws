# Copyright (c) 2024 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ADDED (2026-03-27): launch args to control go2_driver TF/odom outputs.
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    publish_odom_msg = LaunchConfiguration('publish_odom_msg')
    tf_topic = LaunchConfiguration('tf_topic')
    composable_nodes = []

    composable_node = ComposableNode(
        package='go2_driver',
        plugin='go2_driver::Go2Driver',
        name='go2_driver',
        namespace='',
        # KEPT AS COMMENT (2026-03-27): original launch had no TF remap.
        # remappings=[],
        remappings=[
            ('/tf', tf_topic),
        ],
        # KEPT AS COMMENT (2026-03-27): original launch had no parameters here.
        # parameters=[],
        parameters=[{
            'publish_odom_tf': ParameterValue(publish_odom_tf, value_type=bool),
            'publish_odom_msg': ParameterValue(publish_odom_msg, value_type=bool),
        }],
    )
    composable_nodes.append(composable_node)

    container = ComposableNodeContainer(
        name='go2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    pointclod_to_laserscan_cmd = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        namespace='',
        output='screen',
        remappings=[('/cloud_in', '/pointcloud')],
        parameters=[{
                'target_frame': 'radar',
                'transform_tolerance': 0.01,
            }],
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            'tf_topic',
            default_value='/tf',
            description='Output TF topic for go2_driver',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='true',
            description='Publish odom -> base_link TF from go2_driver',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'publish_odom_msg',
            default_value='true',
            description='Publish /odom from go2_driver',
        )
    )
    ld.add_action(container)
    ld.add_action(pointclod_to_laserscan_cmd)

    return ld

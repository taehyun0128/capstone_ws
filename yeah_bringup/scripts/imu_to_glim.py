#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from unitree_go.msg import IMUState


class ImuToGlimBridge(Node):
    def __init__(self) -> None:
        super().__init__("imu_to_glim_bridge")

        self.declare_parameter("in_topic", "/imu")
        self.declare_parameter("out_topic", "/imu_for_glim")
        self.declare_parameter("frame_id", "imu")

        in_topic = str(self.get_parameter("in_topic").value)
        out_topic = str(self.get_parameter("out_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.sub = self.create_subscription(
            IMUState, in_topic, self.callback, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(Imu, out_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f"Bridging {in_topic} (unitree_go/IMUState) -> {out_topic} (sensor_msgs/Imu), frame_id={self.frame_id}"
        )

    def callback(self, m: IMUState) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Unitree IMUState quaternion order is [qw, qx, qy, qz].
        qw = float(m.quaternion[0])
        qx = float(m.quaternion[1])
        qy = float(m.quaternion[2])
        qz = float(m.quaternion[3])

        norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if norm > 1.0e-9:
            qw /= norm
            qx /= norm
            qy /= norm
            qz /= norm
        else:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz

        msg.angular_velocity.x = float(m.gyroscope[0])
        msg.angular_velocity.y = float(m.gyroscope[1])
        msg.angular_velocity.z = float(m.gyroscope[2])

        msg.linear_acceleration.x = float(m.accelerometer[0])
        msg.linear_acceleration.y = float(m.accelerometer[1])
        msg.linear_acceleration.z = float(m.accelerometer[2])

        msg.orientation_covariance = [
            1.0e-3,
            0.0,
            0.0,
            0.0,
            1.0e-3,
            0.0,
            0.0,
            0.0,
            1.0e-3,
        ]
        msg.angular_velocity_covariance = [
            4.0e-4,
            0.0,
            0.0,
            0.0,
            4.0e-4,
            0.0,
            0.0,
            0.0,
            4.0e-4,
        ]
        msg.linear_acceleration_covariance = [
            2.5e-3,
            0.0,
            0.0,
            0.0,
            2.5e-3,
            0.0,
            0.0,
            0.0,
            2.5e-3,
        ]

        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ImuToGlimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

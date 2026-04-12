#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalPoseToNav2ActionBridge(Node):
    def __init__(self) -> None:
        super().__init__("goal_pose_to_nav2_action_bridge")

        self.declare_parameter("input_topic", "/goal_pose")
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("frame_id_override", "map")

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._action_name = str(self.get_parameter("action_name").value)
        self._frame_id_override = str(self.get_parameter("frame_id_override").value)
        self._pending_goal: PoseStamped | None = None

        self._action_client = ActionClient(self, NavigateToPose, self._action_name)

        self._sub = self.create_subscription(
            PoseStamped, self._input_topic, self._goal_callback, 10
        )
        self._timer = self.create_timer(0.5, self._try_send_pending_goal)

        self.get_logger().info(
            f"Bridging {self._input_topic} (PoseStamped) -> {self._action_name} (NavigateToPose)"
        )

    def _goal_callback(self, msg: PoseStamped) -> None:
        if self._frame_id_override:
            msg.header.frame_id = self._frame_id_override

        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()

        self._pending_goal = msg
        self._try_send_pending_goal()

        self.get_logger().info(
            f"Received RViz goal: "
            f"x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}"
        )

    def _try_send_pending_goal(self) -> None:
        if self._pending_goal is None:
            return

        if not self._action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warning(
                f"Action server {self._action_name} not available yet. Waiting to forward goal.",
                throttle_duration_sec=2.0,
            )
            return

        # Use TF latest time (0) to avoid stale-timestamp extrapolation errors
        # during long-running replans.
        self._pending_goal.header.stamp.sec = 0
        self._pending_goal.header.stamp.nanosec = 0

        goal = NavigateToPose.Goal()
        goal.pose = self._pending_goal
        self._pending_goal = None

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

        self.get_logger().info(f"Forwarded goal to {self._action_name}.")

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send goal: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected by Nav2 action server.")
            return

        self.get_logger().info("Goal accepted by Nav2 action server.")


def main() -> None:
    rclpy.init()
    node = GoalPoseToNav2ActionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

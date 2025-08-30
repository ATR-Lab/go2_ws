#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from builtin_interfaces.msg import Time
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose, NavigateThroughPoses

def now_stamp(node: Node) -> Time:
    return node.get_clock().now().to_msg()

class NavigateToPoseRelay(Node):
    def __init__(self):
        super().__init__('restamp_nav_to_pose_relay')
        # Upstream server (what RViz talks to)
        self.server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb)
        # Downstream client (real BT Navigator)
        self.client = ActionClient(self, NavigateToPose, '/bt_navigator/navigate_to_pose')
        self._downstream_goal_handle = None

    def goal_cb(self, goal_request: NavigateToPose.Goal):
        # Always accept; we’ll restamp inside execute_cb
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        # Cancel downstream if active
        if self._downstream_goal_handle is not None:
            self._downstream_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        # Copy & restamp goal
        g = NavigateToPose.Goal()
        g.pose = goal_handle.request.pose
        # g.pose.header.stamp = now_stamp(self)
        g.pose.header.stamp = Time()
        # Ensure the frame is sensible (keep existing frame_id)
        # Connect to downstream
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('BT Navigator navigate_to_pose not available')
            goal_handle.abort()
            return NavigateToPose.Result()

        # Send to downstream and stream feedback back to the upstream client
        def fb_cb(fb):
            goal_handle.publish_feedback(fb.feedback)

        self._downstream_goal_handle = await self.client.send_goal_async(g, feedback_callback=fb_cb)
        if not self._downstream_goal_handle.accepted:
            goal_handle.abort()
            return NavigateToPose.Result()

        result = await self._downstream_goal_handle.get_result_async()
        self._downstream_goal_handle = None

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
        elif status == GoalStatus.STATUS_ABORTED:
            goal_handle.abort()
        else:
            goal_handle.abort()

        return result.result


class NavigateThroughPosesRelay(Node):
    def __init__(self):
        super().__init__('restamp_nav_through_poses_relay')
        self.server = ActionServer(
            self, NavigateThroughPoses, 'navigate_through_poses',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb)
        self.client = ActionClient(self, NavigateThroughPoses, '/bt_navigator/navigate_through_poses')
        self._downstream_goal_handle = None

    def goal_cb(self, goal_request: NavigateThroughPoses.Goal):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        if self._downstream_goal_handle is not None:
            self._downstream_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        g = NavigateThroughPoses.Goal()
        g.poses = []
        # Restamp every pose in the sequence
        for p in goal_handle.request.poses:
            # p.header.stamp = now_stamp(self)
            p.header.stamp = Time()
            g.poses.append(p)

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('BT Navigator navigate_through_poses not available')
            goal_handle.abort()
            return NavigateThroughPoses.Result()

        def fb_cb(fb):
            goal_handle.publish_feedback(fb.feedback)

        self._downstream_goal_handle = await self.client.send_goal_async(g, feedback_callback=fb_cb)
        if not self._downstream_goal_handle.accepted:
            goal_handle.abort()
            return NavigateThroughPoses.Result()

        result = await self._downstream_goal_handle.get_result_async()
        self._downstream_goal_handle = None

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
        elif status == GoalStatus.STATUS_ABORTED:
            goal_handle.abort()
        else:
            goal_handle.abort()

        return result.result


def main():
    rclpy.init()
    # Use a MultiThreadedExecutor so feedback/result callbacks can overlap
    executor = rclpy.executors.MultiThreadedExecutor()
    n1 = NavigateToPoseRelay()
    n2 = NavigateThroughPosesRelay()
    executor.add_node(n1)
    executor.add_node(n2)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n1.destroy_node()
        n2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3


import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


def closest_point_in_bbox(px: float, py: float, bbox: list) -> tuple:

    x1, y1, x2, y2 = bbox
    xmin, xmax = min(x1, x2), max(x1, x2)
    ymin, ymax = min(y1, y2), max(y1, y2)
    cx = min(max(px, xmin), xmax)
    cy = min(max(py, ymin), ymax)
    return cx, cy


class MissionBridgeNode(Node):

    def __init__(self):
        super().__init__('mission_bridge_node')

        self.declare_parameter('robot_namespace', 'r0')
        self.robot_ns = self.get_parameter('robot_namespace').get_parameter_value().string_value

        self._current_pose = None

        amcl_qos = QoSProfile(depth=1)
        amcl_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        amcl_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{self.robot_ns}/amcl_pose',
            self._pose_callback,
            amcl_qos,
        )

        self.plan_sub = self.create_subscription(
            String,
            '/semantic_task_plan',
            self._plan_callback,
            10,
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, f'/{self.robot_ns}/navigate_to_pose'
        )

        self.get_logger().info(
            f'mission_bridge_node ready, target robot: {self.robot_ns}, '
            f'listening on /semantic_task_plan'
        )

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        self._current_pose = msg.pose.pose

    def _plan_callback(self, msg: String):
        try:
            plan = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('received task_plan is not valid JSON, ignoring')
            return

        if not plan.get('executable', False):
            self.get_logger().warn(
                f"plan rejected at '{plan.get('failed_check')}': "
                f"{plan.get('reason')} -- goal not sent"
            )
            return

        action = plan.get('action')

        if action == 'navigate_to_area':
            if self._current_pose is None:
                self.get_logger().error(
                    'area plan received but robot pose is not yet available '
                    '(amcl_pose has not published yet) -- discarding plan'
                )
                return
            x, y = closest_point_in_bbox(
                self._current_pose.position.x,
                self._current_pose.position.y,
                plan['region_bbox'],
            )
            self.get_logger().info(
                f"area plan: region={plan['region_id']} -- "
                f"closest point to robot within the area: ({x:.2f}, {y:.2f})"
            )
        elif action == 'navigate_to':
            target = plan['target_position']
            x, y = target['x'], target['y']
            self.get_logger().info(
                f"object plan: object={plan['object_id']} region={plan['region_id']} "
                f"target=({x}, {y})"
            )
        else:
            self.get_logger().error(f"unknown action in plan: '{action}'")
            return

        self._send_nav_goal(x, y)

    def _send_nav_goal(self, x: float, y: float):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'action server /{self.robot_ns}/navigate_to_pose not available'
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0  # yaw=0, no specific target orientation

        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal')
            return
        self.get_logger().info('Nav2 accepted the goal, navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'distance remaining: {remaining:.2f} m', throttle_duration_sec=2.0)

    def _result_callback(self, future):
        self.get_logger().info('navigation finished')


def main(args=None):
    rclpy.init(args=args)
    node = MissionBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

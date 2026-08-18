#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import math
from enum import Enum, auto
from typing import Optional
#NAV2
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class RobotState(Enum):
    UNKNOWN = auto()
    IDLE = auto()
    BUSY = auto()
    UNRESPONSIVE = auto()


class RobotHandle:
    def __init__(self, namespace, action_client):
        self.namespace = namespace
        self.action_client = action_client
        self.state = RobotState.UNKNOWN
        self.current_pose = None
        self.assigned_waypoint_idx = None


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


##ROS2 NODE Definition
class MissionPlanner(Node):
    def __init__(self):
        super().__init__("mission_planner_node")
        self.get_logger().info("Mission planner started.")

        amcl_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # - Parameters
        self.declare_parameter('robot_namespaces', ['r0', 'r1'])
        self.declare_parameter('waypoints_x', [0.0])
        self.declare_parameter('waypoints_y', [0.0])

        # - Action server timeout
        self.declare_parameter('action_server_wait_timeout_sec', 3.0)
        self.action_server_wait_timeout_sec = self.get_parameter(
            'action_server_wait_timeout_sec').value

        self.declare_parameter('planning_tick_period_sec', 1.0)
        self.planning_tick_period_sec = self.get_parameter(
            'planning_tick_period_sec').value

        robot_namespaces = self.get_parameter('robot_namespaces').value
        wx = self.get_parameter('waypoints_x').value
        wy = self.get_parameter('waypoints_y').value

        if len(wx) != len(wy):
            raise ValueError(
                f"waypoints_x ({len(wx)}) and waypoints_y ({len(wy)}) "
                f"must be the same length."
            )
        self.waypoints = list(zip(wx, wy))  # Add them as a list of points

        # ----Robot handles ----
        self.robots = {}

        # Assign robots NS, get current action and subscribe to position
        for ns in robot_namespaces:
            action_name = f"/{ns}/navigate_to_pose"
            client = ActionClient(self, NavigateToPose, action_name)
            self.robots[ns] = RobotHandle(ns, client)

            self.create_subscription(
                PoseWithCovarianceStamped,
                f"/{ns}/amcl_pose",
                lambda msg, ns=ns: self._pose_callback(ns, msg),
                amcl_qos,
            )

        self.remaining_waypoints = list(range(len(self.waypoints)))
        self.mission_complete = False

        self.get_logger().info(
            f"{len(self.waypoints)} waypoints, robots: {robot_namespaces}"
        )

        # Timer that drives the assignment loop
        self.create_timer(self.planning_tick_period_sec, self._planning_tick)

    def _pose_callback(self, ns, msg):
        p = msg.pose.pose.position
        self.robots[ns].current_pose = (p.x, p.y)

    def _handshake(self, robot):
        return robot.action_client.wait_for_server(
            timeout_sec=self.action_server_wait_timeout_sec
        )

    def _planning_tick(self):
        if self.mission_complete:
            return

        if not self.remaining_waypoints:
            self.mission_complete = True
            self.get_logger().info("All waypoints visited. Mission complete.")
            return

        idle_robots = []
        for ns, robot in self.robots.items():
            if robot.state == RobotState.UNKNOWN:
                if self._handshake(robot):
                    robot.state = RobotState.IDLE
                else:
                    robot.state = RobotState.UNRESPONSIVE
            if robot.state == RobotState.IDLE:
                idle_robots.append(robot)

        self.get_logger().info(
            f"[debug] states: { {ns: r.state.name for ns, r in self.robots.items()} }, "
            f"poses: { {ns: r.current_pose for ns, r in self.robots.items()} }, "
            f"remaining_waypoints: {self.remaining_waypoints}"
        )

        candidate_waypoints = list(self.remaining_waypoints)

        while idle_robots and candidate_waypoints:
            best_distance = None
            best_robot = None
            best_wp_idx = None

            for robot in idle_robots:
                if robot.current_pose is None:
                    continue
                for wp_idx in candidate_waypoints:
                    d = distance(robot.current_pose, self.waypoints[wp_idx])
                    if best_distance is None or d < best_distance:
                        best_distance = d
                        best_robot = robot
                        best_wp_idx = wp_idx

            if best_robot is None:
                break

            idle_robots.remove(best_robot)
            candidate_waypoints.remove(best_wp_idx)
            self.remaining_waypoints.remove(best_wp_idx)

            best_robot.state = RobotState.BUSY
            best_robot.assigned_waypoint_idx = best_wp_idx
            self._dispatch(best_robot, best_wp_idx)

    def _dispatch(self, robot, wp_idx):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.waypoints[wp_idx][0]
        goal.pose.pose.position.y = self.waypoints[wp_idx][1]
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"[{robot.namespace}] -> waypoint #{wp_idx} {self.waypoints[wp_idx]}"
        )

        send_future = robot.action_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda fut, robot=robot, wp_idx=wp_idx: self._on_goal_response(fut, robot, wp_idx)
        )

    def _on_goal_response(self, future, robot, wp_idx):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn(
                f"[{robot.namespace}] goal for waypoint #{wp_idx} rejected."
            )
            robot.state = RobotState.IDLE
            robot.assigned_waypoint_idx = None
            self.remaining_waypoints.append(wp_idx)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, robot=robot, wp_idx=wp_idx: self._on_goal_result(fut, robot, wp_idx)
        )

    def _on_goal_result(self, future, robot, wp_idx):
        result = future.result()
        status = result.status

        SUCCEEDED = 4
        if status == SUCCEEDED:
            self.get_logger().info(f"[{robot.namespace}] reached waypoint #{wp_idx}.")
        else:
            self.get_logger().warn(
                f"[{robot.namespace}] failed waypoint #{wp_idx} (status={status}); "
                f"returning it to the pool."
            )
            self.remaining_waypoints.append(wp_idx)

        robot.state = RobotState.IDLE
        robot.assigned_waypoint_idx = None


def main():
    rclpy.init()
    planner = MissionPlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

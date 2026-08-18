#!/usr/bin/env python3
"""
Centralized mission planner for the multi-robot fleet (task 2.1, last bullet).

Algorithm (as designed with the user):
  1. Nearest-waypoint-first greedy assignment: at every decision point, look
     at all currently IDLE robots and all currently UNASSIGNED waypoints,
     and assign the single closest (robot, waypoint) pair in the whole
     fleet. Repeat until no idle robot / unassigned waypoint pair remains.
     This naturally guarantees no two robots are ever assigned the same
     waypoint, because a waypoint is removed from the pool the instant it
     is assigned to anyone.
  2. Liveness handshake: before a robot is trusted with a new assignment
     (including its very first one), the planner checks that its
     NavigateToPose action server is actually up (short wait_for_server
     timeout) rather than assuming it's alive just because it was alive
     last time. A robot that fails this check is marked UNRESPONSIVE and
     its pending waypoint (if any) is returned to the pool for
     reassignment to a healthy robot.
  3. Reactivation reconciliation: UNRESPONSIVE robots are periodically
     re-polled. If a robot's action server comes back, the planner does
     NOT blindly add it back to the idle pool -- it first re-verifies the
     robot isn't already tracked as ACTIVE/BUSY under some other state
     (defensive check against double-bookkeeping), then transitions it to
     IDLE so it can receive new assignments on the next planning pass.
     Any waypoint that was already reassigned to someone else while this
     robot was down stays with the new owner -- a returning robot never
     reclaims a waypoint it lost.

Requires: nav2_simple_commander (ros-humble-nav2-simple-commander)
"""

import math
import time
import threading
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


# --------------------------------------------------------------------------
# Configuration -- edit for your fleet / mission
# --------------------------------------------------------------------------

ROBOT_NAMESPACES = ["r0", "r1", "r2"]

# 10 waypoints, (x, y) in the 'map' frame. Replace with real coordinates
# once you've confirmed them against the saved warehouse map.
WAYPOINTS = [
    (1.5, 0.0), (3.5, 0.0), (5.5, 0.0), (1.5, 2.0), (3.5, 2.0),
    (5.5, 2.0), (1.5, -2.0), (3.5, -2.0), (5.5, -2.0), (0.0, 3.0),
]

ACTION_SERVER_WAIT_TIMEOUT_SEC = 3.0     # handshake: is the server even up?
LIVENESS_POLL_PERIOD_SEC = 5.0           # how often to re-check downed robots
PLANNING_TICK_PERIOD_SEC = 1.0           # how often to try new assignments
GOAL_RESULT_TIMEOUT_SEC = 120.0          # safety cap per waypoint


class RobotState(Enum):
    UNKNOWN = auto()        # never successfully handshaken
    IDLE = auto()            # healthy, no goal in flight
    BUSY = auto()             # healthy, goal in flight
    UNRESPONSIVE = auto()   # failed handshake / goal, awaiting recovery


@dataclass
class RobotHandle:
    namespace: str
    node: Node
    action_client: ActionClient
    state: RobotState = RobotState.UNKNOWN
    current_pose: Optional[tuple] = None       # (x, y), updated externally
    current_goal_handle: object = None
    assigned_waypoint_idx: Optional[int] = None
    lock: threading.Lock = field(default_factory=threading.Lock)


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# --------------------------------------------------------------------------
# Planner
# --------------------------------------------------------------------------

class MissionPlanner(Node):

    def __init__(self):
        super().__init__("mission_planner")
        self.cb_group = ReentrantCallbackGroup()

        self.robots: dict[str, RobotHandle] = {}
        for ns in ROBOT_NAMESPACES:
            action_name = f"/{ns}/navigate_to_pose"
            client = ActionClient(
                self, NavigateToPose, action_name, callback_group=self.cb_group
            )
            handle = RobotHandle(namespace=ns, node=self, action_client=client)
            self.robots[ns] = handle

            # Subscribe to each robot's AMCL pose so distance-based
            # assignment uses real current position, not just spawn pose.
            self.create_subscription(
                PoseStamped,
                f"/{ns}/amcl_pose",  # NOTE: amcl_pose is actually
                                     # PoseWithCovarianceStamped -- see
                                     # _pose_callback for the real type used.
                lambda msg, ns=ns: None,  # placeholder, real sub added below
                10,
            )

        # amcl_pose is PoseWithCovarianceStamped, not PoseStamped -- fix the
        # subscription type properly here (kept separate from the loop above
        # for clarity on the exact message type mismatch to avoid silently
        # mis-typing the callback, a mistake worth avoiding after all the
        # tf/namespace gotchas already hit earlier in this project).
        from geometry_msgs.msg import PoseWithCovarianceStamped
        for ns in ROBOT_NAMESPACES:
            self.create_subscription(
                PoseWithCovarianceStamped,
                f"/{ns}/amcl_pose",
                lambda msg, ns=ns: self._pose_callback(ns, msg),
                10,
            )

        self.remaining_waypoints = list(range(len(WAYPOINTS)))  # indices
        self.waypoints_lock = threading.Lock()

        self.mission_complete = False

        # Timers
        self.create_timer(
            PLANNING_TICK_PERIOD_SEC, self._planning_tick, callback_group=self.cb_group
        )
        self.create_timer(
            LIVENESS_POLL_PERIOD_SEC, self._liveness_poll, callback_group=self.cb_group
        )

        self.get_logger().info(
            f"Mission planner started. {len(WAYPOINTS)} waypoints, "
            f"{len(ROBOT_NAMESPACES)} robots: {ROBOT_NAMESPACES}"
        )

    # ------------------------------------------------------------------
    # Pose tracking
    # ------------------------------------------------------------------

    def _pose_callback(self, ns, msg):
        p = msg.pose.pose.position
        self.robots[ns].current_pose = (p.x, p.y)

    # ------------------------------------------------------------------
    # Handshake / liveness
    # ------------------------------------------------------------------

    def _handshake(self, robot: RobotHandle) -> bool:
        """Check whether a robot's NavigateToPose action server is up.
        Used both before first assignment and before trusting a robot
        that was previously marked UNRESPONSIVE.
        """
        alive = robot.action_client.wait_for_server(
            timeout_sec=ACTION_SERVER_WAIT_TIMEOUT_SEC
        )
        return alive

    def _liveness_poll(self):
        """Periodically re-check UNRESPONSIVE robots. If one comes back,
        reconcile its state carefully before returning it to the idle pool.
        """
        for ns, robot in self.robots.items():
            with robot.lock:
                if robot.state != RobotState.UNRESPONSIVE:
                    continue

            alive = self._handshake(robot)
            if not alive:
                continue  # still down, nothing to do

            with robot.lock:
                # Defensive re-check: don't blindly flip to IDLE if
                # something already marked this robot BUSY/ACTIVE via a
                # different code path in the meantime (shouldn't normally
                # happen with the lock, but this guards against any future
                # refactor that adds another writer to robot.state).
                if robot.state != RobotState.UNRESPONSIVE:
                    self.get_logger().warn(
                        f"[{ns}] liveness poll found server up, but robot "
                        f"state changed to {robot.state} before reconciling "
                        f"-- skipping to avoid double-booking."
                    )
                    continue

                self.get_logger().info(
                    f"[{ns}] came back online. Any waypoint it previously "
                    f"held has already been reassigned if it timed out; "
                    f"returning it to the idle pool."
                )
                robot.assigned_waypoint_idx = None
                robot.state = RobotState.IDLE

    def _mark_unresponsive(self, robot: RobotHandle, reason: str):
        with robot.lock:
            was_busy_with = robot.assigned_waypoint_idx
            robot.state = RobotState.UNRESPONSIVE
            robot.assigned_waypoint_idx = None

        self.get_logger().error(f"[{robot.namespace}] marked UNRESPONSIVE: {reason}")

        # Return its in-flight waypoint (if any) to the pool so a healthy
        # robot can pick it up on the next planning tick.
        if was_busy_with is not None:
            with self.waypoints_lock:
                if was_busy_with not in self.remaining_waypoints:
                    self.remaining_waypoints.append(was_busy_with)
            self.get_logger().warn(
                f"[{robot.namespace}] waypoint #{was_busy_with} returned to "
                f"pool for reassignment."
            )

    # ------------------------------------------------------------------
    # Assignment: nearest (robot, waypoint) pair, globally, each tick
    # ------------------------------------------------------------------

    def _planning_tick(self):
        if self.mission_complete:
            return

        with self.waypoints_lock:
            if not self.remaining_waypoints:
                # Check if any robot is still BUSY; if not, mission is done.
                any_busy = any(
                    r.state == RobotState.BUSY for r in self.robots.values()
                )
                if not any_busy:
                    self.mission_complete = True
                    self.get_logger().info("All waypoints visited. Mission complete.")
                return

            candidate_waypoints = list(self.remaining_waypoints)

        # Gather idle robots and handshake any UNKNOWN ones before trusting
        # them with an assignment.
        idle_robots = []
        for ns, robot in self.robots.items():
            with robot.lock:
                state = robot.state
            if state == RobotState.IDLE:
                idle_robots.append(robot)
            elif state == RobotState.UNKNOWN:
                if self._handshake(robot):
                    with robot.lock:
                        robot.state = RobotState.IDLE
                    idle_robots.append(robot)
                else:
                    self._mark_unresponsive(robot, "failed initial handshake")

        if not idle_robots:
            return  # nothing to assign right now

        # Greedy nearest pair, repeated until no more idle robots or
        # waypoints remain. This is O(n*m) per tick which is fine for
        # 3 robots x 10 waypoints.
        while idle_robots and candidate_waypoints:
            best = None  # (dist, robot, wp_idx)
            for robot in idle_robots:
                if robot.current_pose is None:
                    continue  # no AMCL fix yet, skip this robot for now
                for wp_idx in candidate_waypoints:
                    d = distance(robot.current_pose, WAYPOINTS[wp_idx])
                    if best is None or d < best[0]:
                        best = (d, robot, wp_idx)

            if best is None:
                break  # no robot has a pose fix yet

            _, robot, wp_idx = best
            idle_robots.remove(robot)
            candidate_waypoints.remove(wp_idx)

            with self.waypoints_lock:
                if wp_idx in self.remaining_waypoints:
                    self.remaining_waypoints.remove(wp_idx)

            self._dispatch(robot, wp_idx)

    def _dispatch(self, robot: RobotHandle, wp_idx: int):
        # Final handshake immediately before sending -- a robot can go
        # from IDLE to actually-dead between the top of the tick and now.
        if not self._handshake(robot):
            self._mark_unresponsive(robot, "handshake failed at dispatch time")
            with self.waypoints_lock:
                self.remaining_waypoints.append(wp_idx)
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = WAYPOINTS[wp_idx][0]
        goal.pose.pose.position.y = WAYPOINTS[wp_idx][1]
        goal.pose.pose.orientation.w = 1.0

        with robot.lock:
            robot.state = RobotState.BUSY
            robot.assigned_waypoint_idx = wp_idx

        self.get_logger().info(
            f"[{robot.namespace}] -> waypoint #{wp_idx} {WAYPOINTS[wp_idx]}"
        )

        send_future = robot.action_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda fut, robot=robot, wp_idx=wp_idx: self._on_goal_response(
                fut, robot, wp_idx
            )
        )

    def _on_goal_response(self, future, robot: RobotHandle, wp_idx: int):
        try:
            goal_handle = future.result()
        except Exception as e:
            self._mark_unresponsive(robot, f"send_goal_async raised: {e}")
            with self.waypoints_lock:
                self.remaining_waypoints.append(wp_idx)
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                f"[{robot.namespace}] goal for waypoint #{wp_idx} rejected."
            )
            with robot.lock:
                robot.state = RobotState.IDLE
                robot.assigned_waypoint_idx = None
            with self.waypoints_lock:
                self.remaining_waypoints.append(wp_idx)
            return

        with robot.lock:
            robot.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, robot=robot, wp_idx=wp_idx: self._on_goal_result(
                fut, robot, wp_idx
            )
        )

        # Safety timer: if the result never comes back within
        # GOAL_RESULT_TIMEOUT_SEC, treat the robot as unresponsive rather
        # than waiting forever.
        def _timeout_check():
            time.sleep(GOAL_RESULT_TIMEOUT_SEC)
            with robot.lock:
                still_this_goal = (
                    robot.assigned_waypoint_idx == wp_idx
                    and robot.state == RobotState.BUSY
                )
            if still_this_goal:
                self._mark_unresponsive(
                    robot,
                    f"no result for waypoint #{wp_idx} within "
                    f"{GOAL_RESULT_TIMEOUT_SEC}s (stuck / lost robot)",
                )

        threading.Thread(target=_timeout_check, daemon=True).start()

    def _on_goal_result(self, future, robot: RobotHandle, wp_idx: int):
        with robot.lock:
            # If this robot was already reassigned away from this waypoint
            # (e.g. it timed out and got marked UNRESPONSIVE, then someone
            # else took wp_idx), don't let a late result flip its state
            # back incorrectly.
            if robot.assigned_waypoint_idx != wp_idx:
                self.get_logger().info(
                    f"[{robot.namespace}] late result for waypoint #{wp_idx} "
                    f"ignored (robot has moved on)."
                )
                return

        try:
            result = future.result()
            status = result.status  # GoalStatus
        except Exception as e:
            self._mark_unresponsive(robot, f"get_result_async raised: {e}")
            with self.waypoints_lock:
                self.remaining_waypoints.append(wp_idx)
            return

        # status values from action_msgs/msg/GoalStatus
        SUCCEEDED = 4
        if status == SUCCEEDED:
            self.get_logger().info(
                f"[{robot.namespace}] reached waypoint #{wp_idx}."
            )
            with robot.lock:
                robot.state = RobotState.IDLE
                robot.assigned_waypoint_idx = None
        else:
            self.get_logger().warn(
                f"[{robot.namespace}] failed waypoint #{wp_idx} "
                f"(status={status}); returning it to the pool."
            )
            with robot.lock:
                robot.state = RobotState.IDLE
                robot.assigned_waypoint_idx = None
            with self.waypoints_lock:
                self.remaining_waypoints.append(wp_idx)


def main():
    rclpy.init()
    planner = MissionPlanner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(planner)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

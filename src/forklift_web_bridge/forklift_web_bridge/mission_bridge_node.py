#!/usr/bin/env python3
"""
mission_bridge_node.py

Thin translation layer exposing forklift_mission_manager's ExecuteMission
action to the web dashboard as JSON commands over a topic, relaying its
feedback/result back the same way. No mission-sequencing logic here --
that all lives in forklift_mission_manager.

Command topic (sub): web_bridge/mission_command  (std_msgs/String, JSON)
    {"command": "execute", "mission_id": "m1", "mission_type": "PALLET_TRANSPORT",
     "pick_dock_id": "pallet_pick_dock_3", "drop_dock_id": "pallet_drop_dock_1"}
    {"command": "execute", "mission_type": "PATROL",
     "waypoints": [[1.0, 0.0, 0.0], [1.0, 1.0, 1.5708]]}
    {"command": "execute", "mission_type": "CHARGING", "dock_id": "charging_dock_1"}
    {"command": "execute", "mission_type": "STATION_OPERATION",
     "dock_id": "station_dock_1", "station_wait_duration": 8.0}
    {"command": "cancel"}
    {"command": "query"}

Status topic (pub): web_bridge/mission_status  (std_msgs/String, JSON)
    {"state": ..., "mission_id": ..., "current_task": ..., "current_task_index": ...,
     "total_tasks": ..., "progress": ..., "message": ..., "task": {...}}

Task topic (pub): web_bridge/task_status  (std_msgs/String, JSON)
    The same "task" object on its own, one message per step transition,
    for dashboards that want task progress without the mission wrapper
    (and for REST's GET /task/status). Published from here rather than
    from a node of its own because this node already holds the
    task_status subscription the object is built from.

The "task" sub-object comes from forklift_task_manager's task_status
topic (forklift_interfaces/TaskStepStatus). ExecuteMission feedback only
names the running task ("STATION_DOCK"), so a DOCK step could sit there
for 30s and fail without the dashboard ever learning why. task_status
carries the step detail -- which dock operation, whether the dock was
detected, fork moves, undock, success -- published by the node that
actually runs those steps. Subscribing is all that happens here; no task
or docking logic lives in this node either.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from forklift_interfaces.action import ExecuteMission
from forklift_interfaces.msg import TaskStepStatus

# Steps that are real entries in the task's step list -- finishing one of
# these advances the task. Everything else task_manager reports (STAGING,
# DETECT, APPROACH, CONTACT, WAIT_FOR_CHARGE, CHARGING, UNDOCK_STAGING)
# is a sub-step *inside* the running DOCK/UNDOCK step, so it must not
# count as a completed step or the progress bar jumps back and forth.
TASK_STEP_TYPES = frozenset({
    "NAVIGATE", "DOCK", "LIFT", "LOWER", "UNDOCK", "MONITOR_CHARGING", "STATION_OPERATION",
})


def _pose_from_xyz_yaw(xyz_yaw, frame_id):
    pose = PoseStamped()
    if not xyz_yaw:
        return pose  # empty frame_id = "not set", matches mission_manager's own convention
    x, y, yaw = xyz_yaw
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return pose


class MissionBridgeNode(Node):
    def __init__(self):
        super().__init__("mission_bridge_node")

        self.declare_parameter("mission_command_topic", "web_bridge/mission_command")
        self.declare_parameter("mission_status_topic", "web_bridge/mission_status")
        self.declare_parameter("execute_mission_action_name", "execute_mission")
        self.declare_parameter("default_frame_id", "map")
        # Step detail published by forklift_task_manager.
        self.declare_parameter("task_status_topic", "task_status")
        # JSON mirror of the above for web clients (WebSocket + REST).
        self.declare_parameter("task_status_bridge_topic", "web_bridge/task_status")
        # task_status only fires on transitions, but a retrying DOCK step
        # can burst; republish mission_status at most this often for them.
        self.declare_parameter("task_status_min_period_sec", 0.2)
        # Recent step transitions kept for the dashboard's detail view.
        self.declare_parameter("task_event_history", 10)

        cmd_topic = self.get_parameter("mission_command_topic").value
        status_topic = self.get_parameter("mission_status_topic").value
        action_name = self.get_parameter("execute_mission_action_name").value
        self.default_frame_id = self.get_parameter("default_frame_id").value

        self.status_pub = self.create_publisher(String, status_topic, 10)
        # Transient local: a dashboard connecting mid-task gets the current
        # step immediately, like the task_status topic it mirrors.
        self.task_status_pub = self.create_publisher(
            String, self.get_parameter("task_status_bridge_topic").value,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.cmd_sub = self.create_subscription(String, cmd_topic, self._on_command, 10)

        self.mission_client = ActionClient(self, ExecuteMission, action_name)
        self._goal_handle = None
        self._mission_status = {
            "state": "IDLE", "mission_id": "", "message": "No mission executed yet"}

        self._task = self._empty_task()
        self._task_dirty = False
        self._task_last_publish = 0.0
        self._event_history = int(self.get_parameter("task_event_history").value)

        # Transient local, matching task_manager's publisher: a dashboard
        # connecting mid-task gets the current step right away instead of
        # waiting for the next transition.
        task_status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            TaskStepStatus, self.get_parameter("task_status_topic").value,
            self._on_task_status, task_status_qos)

        self.create_timer(
            float(self.get_parameter("task_status_min_period_sec").value),
            self._publish_task_if_dirty)

        self._publish_status(self._mission_status)
        self.get_logger().info(f"mission_bridge_node ready, listening on '{cmd_topic}'")

    # ---------------- status publishing ----------------

    def _publish_status(self, status_dict):
        """Publish mission status with the current task-step detail merged in."""
        self._mission_status = status_dict
        payload = dict(status_dict)
        payload["task"] = dict(self._task)
        self._task_dirty = False
        self._task_last_publish = self._now()
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_task_if_dirty(self):
        """Timer: emit step changes between ExecuteMission feedback msgs."""
        if not self._task_dirty:
            return
        min_period = float(self.get_parameter("task_status_min_period_sec").value)
        if self._now() - self._task_last_publish < min_period:
            return
        self._publish_status(self._mission_status)

    # ---------------- task step detail ----------------

    @staticmethod
    def _empty_task():
        return {
            "task_id": "",
            "task_type": None,    # PICK_PALLET_DOCK | DROP_PALLET_DOCK |
                                  # CHARGING_DOCK | STATION_DOCK | ...
            "step": None,         # NAVIGATE | DOCK | STAGING | DETECT | APPROACH |
                                  # CONTACT | WAIT_FOR_CHARGE | CHARGING | LIFT |
                                  # LOWER | UNDOCK | UNDOCK_STAGING | TASK
            "state": None,        # RUNNING | SUCCESS | FAILED | RETRY | CANCELED
            "step_index": 0,
            "total_steps": 0,
            "progress": 0.0,      # 0-100 over the task's steps
            "step_progress": 0.0,  # as task_manager reported it
            "label": "",          # human-readable progress line
            "attempt": 0,
            "max_attempts": 0,
            "dock_id": "",
            "lift_height": 0.0,
            "detected": False,    # dock detected during this task
            "charging": False,    # robot reported charging on the dock
            "success": False,     # task finished successfully
            "error_code": "",
            "message": "",
            "events": [],         # recent {stamp, step, state, message}
        }

    def _task_progress(self, msg):
        """Percent of the task's steps done, 0-100.

        TaskStepStatus.progress counts steps *started*; a finished step
        should read as done, and a finished task as 100, so the dashboard
        never shows "3/4" sitting at 75% after the last step succeeded.
        Only real steps advance it (see TASK_STEP_TYPES), and the value
        never decreases within a task -- a dock sub-step reporting from
        inside step 2 must not drag the bar back to where step 2 started.
        """
        if msg.step == "TASK":
            if msg.state == "SUCCESS":
                return 100.0
            progress = round(msg.progress, 1)
        elif msg.total_steps <= 0:
            progress = round(msg.progress, 1)
        else:
            done = msg.step_index
            if msg.state == "SUCCESS" and msg.step in TASK_STEP_TYPES:
                done += 1
            progress = round(100.0 * min(done, msg.total_steps) / msg.total_steps, 1)
        return max(progress, float(self._task.get("progress") or 0.0))

    @staticmethod
    def _task_label(msg):
        """One human-readable line for the dashboard's progress row."""
        labels = {
            "TASK": "Task",
            "NAVIGATE": "Navigating",
            "STAGING": "Driving to staging pose",
            "DOCK": "Docking",
            "DETECT": "Detecting dock",
            "APPROACH": "Approaching dock",
            "CONTACT": "Docked",
            "WAIT_FOR_CHARGE": "Waiting for charge",
            "CHARGING": "Charging",
            "LIFT": "Lifting fork",
            "LOWER": "Lowering fork",
            "UNDOCK": "Undocking",
            "UNDOCK_STAGING": "Back at staging pose",
            "MONITOR_CHARGING": "Waiting for charge",
            "STATION_OPERATION": "Station operation",
        }
        label = labels.get(msg.step, msg.step)
        if msg.step in ("LIFT", "LOWER") and msg.lift_height:
            label = f"{label} to {msg.lift_height:.2f} m"
        if msg.state == "RETRY" and msg.max_attempts > 1:
            return f"{label} - retry {msg.attempt}/{msg.max_attempts}"
        if msg.state == "FAILED":
            return f"{label} failed"
        if msg.step == "TASK":
            return {"RUNNING": "Task started", "SUCCESS": "Task completed",
                    "CANCELED": "Task canceled"}.get(msg.state, "Task failed")
        return label

    def _on_task_status(self, msg: TaskStepStatus):
        # A new task supersedes the previous one's detail, including a
        # stale "FAILED" left over from the task before it.
        if msg.task_id != self._task.get("task_id"):
            self._task = self._empty_task()

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        event = {
            "stamp": stamp, "step": msg.step, "state": msg.state, "message": msg.message,
        }
        events = self._task["events"] + [event]

        self._task.update({
            "task_id": msg.task_id,
            "task_type": msg.task_type,
            "step": msg.step,
            "state": msg.state,
            "step_index": msg.step_index,
            "total_steps": msg.total_steps,
            "step_progress": round(msg.progress, 1),
            "attempt": msg.attempt,
            "max_attempts": msg.max_attempts,
            "dock_id": msg.dock_id,
            "lift_height": round(msg.lift_height, 3),
            "detected": msg.detected,
            "charging": msg.charging,
            "success": msg.success,
            "error_code": msg.error_code,
            "message": msg.message,
            "events": events[-self._event_history:],
            "progress": self._task_progress(msg),
            "label": self._task_label(msg),
        })
        self._task_dirty = True

        # Mirror every transition straight out, unthrottled: this topic is
        # the dashboard's progress feed, and transitions are already rare
        # (repeats are collapsed by task_manager).
        task_msg = String()
        task_msg.data = json.dumps(self._task)
        self.task_status_pub.publish(task_msg)

    # ---------------- mission command/status ----------------

    def _on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON on mission_command: {msg.data}")
            return

        action = cmd.get("command")
        if action == "execute":
            self._send_mission_goal(cmd)
        elif action == "cancel":
            self._cancel_goal()
        elif action == "query":
            self._publish_status(self._mission_status)
        else:
            self.get_logger().warn(f"Unknown mission command: {action}")

    def _send_mission_goal(self, cmd):
        if not self.mission_client.wait_for_server(timeout_sec=2.0):
            self._publish_status(
                {"state": "FAILED", "message": "ExecuteMission action server not available"})
            return

        goal = ExecuteMission.Goal()
        goal.mission_id = cmd.get("mission_id", "")
        goal.mission_type = cmd.get("mission_type", "")
        goal.pick_dock_id = cmd.get("pick_dock_id", "")
        goal.drop_dock_id = cmd.get("drop_dock_id", "")
        goal.dock_id = cmd.get("dock_id", "")
        goal.station_wait_duration = float(cmd.get("station_wait_duration", 0.0))

        if "pick_area_pose" in cmd:
            goal.pick_area_pose = _pose_from_xyz_yaw(cmd["pick_area_pose"], self.default_frame_id)
        if "drop_area_pose" in cmd:
            goal.drop_area_pose = _pose_from_xyz_yaw(cmd["drop_area_pose"], self.default_frame_id)
        if "waypoints" in cmd:
            goal.waypoints = [
                _pose_from_xyz_yaw(wp, self.default_frame_id) for wp in cmd["waypoints"]]

        # Fresh mission -> drop the previous mission's step detail, so the
        # dashboard never shows a stale "FAILED" next to a new goal.
        self._task = self._empty_task()

        self._publish_status({
            "state": "SENDING", "mission_id": goal.mission_id,
            "message": f"Sending mission {goal.mission_type}",
        })

        send_future = self.mission_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._publish_status({"state": "FAILED", "message": "Mission goal rejected"})
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self._publish_status({
            "state": fb.state,
            "mission_id": fb.mission_id,
            "current_task": fb.current_task,
            "current_task_index": fb.current_task_index,
            "total_tasks": fb.total_tasks,
            "progress": fb.progress,
            "message": fb.message,
        })

    def _on_result(self, future):
        result = future.result()
        self._goal_handle = None
        code = result.status
        state = {4: "COMPLETED", 5: "CANCELED", 6: "FAILED"}.get(code, "FAILED")
        self._publish_status({
            "state": state,
            "message": result.result.message,
            "failed_task": result.result.failed_task,
            "success": result.result.success,
        })

    def _cancel_goal(self):
        if self._goal_handle is None:
            self._publish_status({"state": "IDLE", "message": "No active mission to cancel"})
            return
        self._goal_handle.cancel_goal_async()
        self._publish_status({"state": "CANCELING", "message": "Cancel requested"})


def main():
    rclpy.init()
    node = MissionBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

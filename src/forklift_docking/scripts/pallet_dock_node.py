"""
pallet_dock_workflow_node.py

Orchestrates the full pick/drop sequence in response to a single trigger
topic:

    /dock_pallet (std_msgs/String, data = dock_id, e.g. "pallet_pick_dock_1")
        |
        v
    1. Send DockRobot action goal (use_dock_id=true, dock_id=<received>)
    2. On success: wait `post_dock_delay` seconds
    3. Call /set_height service (forklift_interfaces/srv/SetLiftHeight)
       with `lift_target_height` / `lift_speed` parameters
    4. Send UndockRobot action goal, dock_type derived from dock_id by
       stripping the trailing "_<N>" instance suffix, e.g.
       "pallet_pick_dock_1" -> "pallet_pick_dock" -- matching the naming
       convention used in dock_database.yaml.

This node deliberately does ONE thing (dock -> lift -> undock) and does not
itself decide *when* to trigger a pick/drop -- that is forklift_mission_manager's
job; it only needs to publish a dock_id string on /dock_pallet.

Parameters:
    dock_pallet_topic     - trigger topic to subscribe to (default /dock_pallet)
    post_dock_delay       - seconds to wait after DockRobot succeeds, before
                             calling /set_height (default 2.0)
    lift_target_height    - target fork height in meters sent to /set_height
                             (default 0.15)
    lift_speed            - lift speed in m/s sent to /set_height, 0.0 =
                             use lift_controller default (default 0.0)
    dock_action_name      - DockRobot action name (default /dock_robot)
    undock_action_name    - UndockRobot action name (default /undock_robot)
    set_height_service_name - SetLiftHeight service name (default /set_height)
    undock_max_time       - seconds, max_undocking_time sent to UndockRobot
                             (default 15.0)
"""

import re
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from opennav_docking_msgs.action import DockRobot, UndockRobot
from forklift_interfaces.srv import SetLiftHeight
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


def dock_type_from_dock_id(dock_id: str) -> str:
    """Strips a trailing '_<N>' instance suffix, e.g.
    'pallet_pick_dock_1' -> 'pallet_pick_dock'. Falls back to the
    unmodified dock_id if no numeric suffix is found -- matches the
    naming convention used across this project's dock_database.yaml."""
    return re.sub(r'_\d+$', '', dock_id)


class PalletDockWorkflowNode(Node):
    def __init__(self):
        super().__init__('pallet_dock_workflow_node')

        self.declare_parameter(
            'dock_pallet_topic', '/dock_pallet',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            'post_dock_delay', 2.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Seconds to wait after DockRobot succeeds, before calling /set_height.'))
        self.declare_parameter(
            'lift_target_height', 0.1,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Target fork height (m) sent to /set_height after a successful dock.'))
        self.declare_parameter(
            'lift_speed', 0.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Lift speed (m/s) sent to /set_height. 0.0 = use lift_controller default.'))
        self.declare_parameter(
            'dock_action_name', '/dock_robot',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            'undock_action_name', '/undock_robot',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            'set_height_service_name', '/set_height',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            'undock_max_time', 15.0,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))

        self.dock_pallet_topic = self.get_parameter('dock_pallet_topic').value
        self.post_dock_delay = self.get_parameter('post_dock_delay').value
        self.lift_target_height = self.get_parameter('lift_target_height').value
        self.lift_speed = self.get_parameter('lift_speed').value
        self.undock_max_time = self.get_parameter('undock_max_time').value

        cb_group = ReentrantCallbackGroup()

        self.dock_client = ActionClient(
            self, DockRobot, self.get_parameter('dock_action_name').value,
            callback_group=cb_group)
        self.undock_client = ActionClient(
            self, UndockRobot, self.get_parameter('undock_action_name').value,
            callback_group=cb_group)
        self.set_height_client = self.create_client(
            SetLiftHeight, self.get_parameter('set_height_service_name').value,
            callback_group=cb_group)

        self.dock_pallet_sub = self.create_subscription(
            String, self.dock_pallet_topic, self._on_dock_pallet, 10,
            callback_group=cb_group)

        # Prevents two overlapping workflows if a second trigger arrives
        # while one is already running.
        self._busy_lock = threading.Lock()

        self.get_logger().info(
            f"pallet_dock_workflow_node ready, listening on '{self.dock_pallet_topic}'")

    def _on_dock_pallet(self, msg: String):
        dock_id = msg.data.strip()
        if not dock_id:
            self.get_logger().warn("Received empty dock_id on /dock_pallet, ignoring.")
            return

        if not self._busy_lock.acquire(blocking=False):
            self.get_logger().warn(
                f"Workflow already in progress, ignoring new request for '{dock_id}'.")
            return

        # Run the (blocking, multi-step) workflow on its own thread so this
        # subscription callback returns immediately, and so the executor
        # keeps servicing other callbacks (action feedback, etc.) while
        # this workflow's _wait_for_future() polls for results below.
        threading.Thread(
            target=self._run_workflow, args=(dock_id,), daemon=True
        ).start()

    def _run_workflow(self, dock_id: str):
        try:
            self.get_logger().info(f"=== Starting pallet workflow for '{dock_id}' ===")

            # ---- Step 1: DockRobot ----
            if not self._send_dock_goal(dock_id):
                self.get_logger().error(f"DockRobot failed for '{dock_id}', aborting workflow.")
                return

            # ---- Step 2: delay ----
            self.get_logger().info(
                f"Dock succeeded. Waiting {self.post_dock_delay:.1f}s before lifting...")
            time.sleep(self.post_dock_delay)

            # ---- Step 3: /set_height ----
            if not self._call_set_height():
                self.get_logger().error("SetLiftHeight call failed, aborting before undock.")
                return

            # ---- Step 4: UndockRobot ----
            dock_type = dock_type_from_dock_id(dock_id)
            self._send_undock_goal(dock_type)

            self.get_logger().info(f"=== Workflow for '{dock_id}' complete ===")
        finally:
            self._busy_lock.release()

    # ------------------------------------------------------------------
    # Step 1
    # ------------------------------------------------------------------
    def _send_dock_goal(self, dock_id: str) -> bool:
        if not self.dock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("DockRobot action server not available.")
            return False

        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = dock_id
        goal.navigate_to_staging_pose = True

        send_future = self.dock_client.send_goal_async(
            goal, feedback_callback=self._dock_feedback_cb)
        goal_handle = self._wait_for_future(send_future, timeout_sec=10.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("DockRobot goal was rejected or timed out.")
            return False

        result_future = goal_handle.get_result_async()
        result_wrapper = self._wait_for_future(result_future, timeout_sec=60.0)
        if result_wrapper is None:
            self.get_logger().error("DockRobot result timed out.")
            return False

        result = result_wrapper.result
        if not result.success:
            self.get_logger().error(
                f"DockRobot reported failure: {getattr(result, 'error_code', '')}")
            return False

        return True

    def _dock_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        state = getattr(fb, 'state', '')
        if state:
            self.get_logger().debug(f"DockRobot feedback: state={state}")

    # ------------------------------------------------------------------
    # Step 3
    # ------------------------------------------------------------------
    def _call_set_height(self) -> bool:
        if not self.set_height_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/set_height service not available.")
            return False

        request = SetLiftHeight.Request()
        request.target_height = float(self.lift_target_height)
        request.speed = float(self.lift_speed)

        future = self.set_height_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=15.0)
        if response is None:
            self.get_logger().error("/set_height call timed out.")
            return False

        if not response.success:
            self.get_logger().error(f"/set_height reported failure: {response.message}")
            return False

        self.get_logger().info(
            f"Lift height set OK: current_height={response.current_height:.3f}m "
            f"({response.message})")
        return True

    # ------------------------------------------------------------------
    # Step 4
    # ------------------------------------------------------------------
    def _send_undock_goal(self, dock_type: str):
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("UndockRobot action server not available.")
            return

        goal = UndockRobot.Goal()
        goal.dock_type = dock_type
        goal.max_undocking_time = float(self.undock_max_time)

        send_future = self.undock_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(send_future, timeout_sec=10.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("UndockRobot goal was rejected or timed out.")
            return

        result_future = goal_handle.get_result_async()
        result_wrapper = self._wait_for_future(result_future, timeout_sec=30.0)
        if result_wrapper is None:
            self.get_logger().error("UndockRobot result timed out.")
            return

        result = result_wrapper.result
        if result.success:
            self.get_logger().info(f"Undock succeeded (dock_type='{dock_type}').")
        else:
            self.get_logger().error(
                f"Undock reported failure: {getattr(result, 'error_code', '')}")

    # ------------------------------------------------------------------
    # Generic future-wait helper.
    #
    # NOTE: this node is spun with a MultiThreadedExecutor (see main()).
    # Each /dock_pallet message runs its workflow on a dedicated Python
    # thread (_run_workflow), NOT on the executor's own callback thread --
    # so polling here does not stall the executor, which keeps servicing
    # action/service callbacks concurrently. This mirrors the same lesson
    # from the C++ docking_server: never call something like
    # spin_until_future_complete on a node that's already being spun
    # elsewhere -- poll the future instead.
    # ------------------------------------------------------------------
    def _wait_for_future(self, future, timeout_sec: float):
        start = time.time()
        while rclpy.ok():
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f"Future raised an exception: {e}")
                    return None
            if time.time() - start > timeout_sec:
                return None
            time.sleep(0.05)
        return None


def main():
    rclpy.init()
    node = PalletDockWorkflowNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
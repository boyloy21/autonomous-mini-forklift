#!/usr/bin/env python3
"""
fastapi_bridge_node.py

Thin REST facade over the exact same web_bridge command/status topics
used by the WebSocket (rosbridge/roslibjs) path -- GET endpoints serve
the latest cached status from web_bridge/robot_state,
web_bridge/navigation_status, web_bridge/mission_status,
web_bridge/diagnostics_summary, web_bridge/status; POST endpoints
publish onto web_bridge/nav_command and web_bridge/mission_command. No
action-calling or business logic lives here a second time -- REST and
WebSocket clients both ultimately drive navigation_bridge_node/
mission_bridge_node, never bypass them.

rclpy spins on a background thread; uvicorn runs FastAPI on the main
thread -- the two event loops are entirely independent. Communication
between them is plain Python object reads/writes on a cached-state dict,
protected by a lock since ROS2 subscription callbacks run on a different
thread than FastAPI's request handlers.

The one exception to "topics only" is the static route graph
(GET/POST /routes/graph): it is a file on disk, not a topic. GET reads
the GeoJSON file route_server is actually configured with (its
graph_filepath parameter, unless route_graph_file overrides it); POST
writes that file and asks route_server to reload it through its
route_server/set_route_graph service (nav2_msgs/srv/SetRouteGraph), so
no route_server restart is needed. Live route state (/route_graph
MarkerArray, /plan, /local_plan) stays on rosbridge.

The dock database (GET/POST /docks) is the same kind of exception:
dock_database.yaml is read from docking_server's dock_database
parameter (falling back to forklift_docking's installed copy). POST
appends one entry as text -- so hand-written comments/formatting are
never reformatted -- then calls every docking_server's reload_database
service, since docking_server caches the file at activation while
forklift_task_manager and pallet_dock_workflow_node re-read it per task.

Mission templates (GET/POST /missions/templates) work the same way on
forklift_mission_manager's missions.yaml. No reload call is needed there:
mission_server loads that file only to offer named lookups that nothing
currently calls -- every mission is executed from a fully-specified
ExecuteMission goal -- so a new template is usable as soon as it is
written.
"""

import json
import math
import os
import re
import tempfile
import threading
from typing import List, Literal, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from nav2_msgs.srv import SetRouteGraph
from opennav_docking_msgs.srv import ReloadDatabase
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from fastapi import Body, FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
import yaml

DOCK_TYPES = ("charging_dock", "station_dock", "pallet_pick_dock", "pallet_drop_dock")
ENTRY_NAME_RE = re.compile(r"^[A-Za-z0-9_]+$")
MISSION_TYPES = ("PALLET_TRANSPORT", "CHARGING", "PATROL", "STATION_OPERATION")
# Per mission type: fields the caller must send, and the extra ones
# accepted. missions.yaml spells the type "type"; REST spells it
# "mission_type" (as in ExecuteMission.action), translated both ways.
MISSION_REQUIRED_FIELDS = {
    "PALLET_TRANSPORT": ("pick_dock_id", "drop_dock_id"),
    "CHARGING": ("dock_id",),
    "PATROL": ("waypoints",),
    "STATION_OPERATION": ("dock_id",),
}
MISSION_OPTIONAL_FIELDS = {
    "PALLET_TRANSPORT": ("pick_area_pose", "drop_area_pose"),
    "CHARGING": (),
    "PATROL": (),
    "STATION_OPERATION": ("station_wait_duration",),
}
MISSION_DOCK_FIELDS = ("pick_dock_id", "drop_dock_id", "dock_id")
FRAME_RE = re.compile(r"^[A-Za-z0-9_/]+$")
RELOAD_DB_SERVICE_SUFFIX = "/docking_server/reload_database"
RELOAD_DB_SERVICE_TYPE = "opennav_docking_msgs/srv/ReloadDatabase"


class NavigateRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"


class MissionRequest(BaseModel):
    mission_id: str = ""
    mission_type: str
    pick_dock_id: str = ""
    drop_dock_id: str = ""
    dock_id: str = ""
    station_wait_duration: float = 0.0
    waypoints: List[List[float]] = []
    pick_area_pose: Optional[List[float]] = None
    drop_area_pose: Optional[List[float]] = None


class DockCreateRequest(BaseModel):
    name: Optional[str] = None
    type: Literal[DOCK_TYPES]
    frame: str = "map"
    pose: List[float]
    # ArUco marker id; the existing file also gives charging/station
    # docks one, so it's accepted for every type, not only pallet docks.
    id: Optional[int] = None
    lift: Optional[float] = None


class MissionTemplateRequest(BaseModel):
    name: str
    mission_type: Literal[MISSION_TYPES]
    pick_dock_id: Optional[str] = None
    drop_dock_id: Optional[str] = None
    dock_id: Optional[str] = None
    station_wait_duration: Optional[float] = None
    waypoints: Optional[List[List[float]]] = None
    pick_area_pose: Optional[List[float]] = None
    drop_area_pose: Optional[List[float]] = None


class FastApiBridgeNode(Node):
    def __init__(self):
        super().__init__("fastapi_bridge_node")

        self.declare_parameter("rest_port", 8000)
        self.declare_parameter("rest_address", "0.0.0.0")
        self.declare_parameter("robot_state_topic", "web_bridge/robot_state")
        self.declare_parameter("navigation_status_topic", "web_bridge/navigation_status")
        self.declare_parameter("mission_status_topic", "web_bridge/mission_status")
        self.declare_parameter("task_status_topic", "web_bridge/task_status")
        self.declare_parameter("diagnostics_summary_topic", "web_bridge/diagnostics_summary")
        self.declare_parameter("bridge_status_topic", "web_bridge/status")
        self.declare_parameter("nav_command_topic", "web_bridge/nav_command")
        self.declare_parameter("mission_command_topic", "web_bridge/mission_command")
        # Route graph: relative names, so under namespace:=forklift1 these
        # resolve to /forklift1/route_server/... like every other topic here.
        self.declare_parameter("route_server_node", "route_server")
        # Empty = ask route_server for its graph_filepath parameter, so the
        # dashboard always sees the graph route_server actually loaded.
        self.declare_parameter("route_graph_file", "")
        self.declare_parameter("route_service_timeout_sec", 5.0)
        # Dock database: empty = ask docking_server for its dock_database
        # parameter, else forklift_docking's installed config.
        self.declare_parameter("docking_server_node", "docking_server")
        self.declare_parameter("dock_database_file", "")
        # Mission templates: empty = ask mission_server for its
        # mission_database parameter, else forklift_mission_manager's
        # installed config.
        self.declare_parameter("mission_server_node", "mission_server")
        self.declare_parameter("mission_database_file", "")

        self._lock = threading.Lock()
        # Serializes POST /routes/graph so two dashboards can't interleave
        # a write + reload of the same file.
        self._graph_write_lock = threading.Lock()
        self._route_graph_file = None
        # Same for POST /docks (read-check-append must be one step, or two
        # clients could both claim charging_dock_2).
        self._dock_write_lock = threading.Lock()
        self._dock_database_file = None
        self._reload_db_clients = {}
        self._mission_write_lock = threading.Lock()
        self._mission_database_file = None
        self._cache = {
            "robot_state": None,
            "navigation_status": None,
            "mission_status": None,
            "task_status": None,
            "diagnostics_summary": None,
            "bridge_status": None,
        }

        self.create_subscription(
            String, self.get_parameter("robot_state_topic").value,
            self._make_cache_callback("robot_state"), 10)
        self.create_subscription(
            String, self.get_parameter("navigation_status_topic").value,
            self._make_cache_callback("navigation_status"), 10)
        self.create_subscription(
            String, self.get_parameter("mission_status_topic").value,
            self._make_cache_callback("mission_status"), 10)
        self.create_subscription(
            String, self.get_parameter("diagnostics_summary_topic").value,
            self._make_cache_callback("diagnostics_summary"), 10)
        # Transient local, matching mission_bridge_node's publisher, so a
        # REST client that starts mid-task still sees the current step.
        self.create_subscription(
            String, self.get_parameter("task_status_topic").value,
            self._make_cache_callback("task_status"),
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            String, self.get_parameter("bridge_status_topic").value,
            self._make_cache_callback("bridge_status"), 10)

        self.nav_cmd_pub = self.create_publisher(
            String, self.get_parameter("nav_command_topic").value, 10)
        self.mission_cmd_pub = self.create_publisher(
            String, self.get_parameter("mission_command_topic").value, 10)

        route_server = self.get_parameter("route_server_node").value
        self.route_params_client = self.create_client(
            GetParameters, f"{route_server}/get_parameters")
        self.set_route_graph_client = self.create_client(
            SetRouteGraph, f"{route_server}/set_route_graph")
        docking_server = self.get_parameter("docking_server_node").value
        self.docking_params_client = self.create_client(
            GetParameters, f"{docking_server}/get_parameters")
        mission_server = self.get_parameter("mission_server_node").value
        self.mission_params_client = self.create_client(
            GetParameters, f"{mission_server}/get_parameters")

        self.get_logger().info("fastapi_bridge_node ROS2 side ready")

    def _make_cache_callback(self, key):
        def callback(msg: String):
            with self._lock:
                try:
                    self._cache[key] = json.loads(msg.data)
                except json.JSONDecodeError:
                    self._cache[key] = {"raw": msg.data}
        return callback

    def get_cached(self, key):
        with self._lock:
            return self._cache.get(key)

    def publish_nav_command(self, command: dict):
        msg = String()
        msg.data = json.dumps(command)
        self.nav_cmd_pub.publish(msg)

    def publish_mission_command(self, command: dict):
        msg = String()
        msg.data = json.dumps(command)
        self.mission_cmd_pub.publish(msg)

    def _call_service(self, client, request):
        """Blocking service call from a FastAPI worker thread.

        Safe only because rclpy spins on its own thread: that thread
        delivers the response and completes the future while this one
        waits on the Event. Returns None on timeout / unavailable.
        """
        timeout = self.get_parameter("route_service_timeout_sec").value
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            future.cancel()
            return None
        return future.result()

    def get_route_graph_file(self):
        """Path of the GeoJSON route_server is configured with, or None."""
        override = self.get_parameter("route_graph_file").value
        if override:
            return override
        with self._lock:
            if self._route_graph_file:
                return self._route_graph_file
        request = GetParameters.Request()
        request.names = ["graph_filepath"]
        response = self._call_service(self.route_params_client, request)
        if response is None or not response.values:
            return None
        path = parameter_value_to_python(response.values[0])
        if not path:
            return None
        with self._lock:
            self._route_graph_file = path
        return path

    def _config_file_from_node(self, cache_key, override_param, client, node_param,
                               fallback_package, fallback_relpath):
        """Path a ROS node is configured with, with a packaged fallback.

        Asks the owning node for its own parameter so the REST view can
        never drift from the file that node actually loaded. If the node
        is down, falls back to the installed config (so GET still works)
        without caching it -- the node is asked again next time.
        """
        override = self.get_parameter(override_param).value
        if override:
            return override
        with self._lock:
            cached = getattr(self, cache_key)
        if cached:
            return cached
        request = GetParameters.Request()
        request.names = [node_param]
        response = self._call_service(client, request)
        path = None
        if response is not None and response.values:
            path = parameter_value_to_python(response.values[0]) or None
        if path is None:
            try:
                return os.path.join(
                    get_package_share_directory(fallback_package), *fallback_relpath)
            except PackageNotFoundError:
                return None
        with self._lock:
            setattr(self, cache_key, path)
        return path

    def get_dock_database_file(self):
        """Path of dock_database.yaml: override > docking_server > package."""
        return self._config_file_from_node(
            "_dock_database_file", "dock_database_file", self.docking_params_client,
            "dock_database", "forklift_docking", ("config", "dock_database.yaml"))

    def get_mission_database_file(self):
        """Path of missions.yaml: override > mission_server > package."""
        return self._config_file_from_node(
            "_mission_database_file", "mission_database_file", self.mission_params_client,
            "mission_database", "forklift_mission_manager", ("config", "missions.yaml"))

    def reload_docking_servers(self, path):
        """Call reload_database on every docking_server on the graph.

        Docks are fixed map features shared by the whole fleet (one
        dock_database.yaml for all robots), so every robot's
        docking_server is reloaded, not only this namespace's. Returns
        {service_name: True/False/None}; False = rejected (e.g. docking
        in progress), None = no answer.
        """
        results = {}
        for name, types in self.get_service_names_and_types():
            if not name.endswith(RELOAD_DB_SERVICE_SUFFIX) or RELOAD_DB_SERVICE_TYPE not in types:
                continue
            client = self._reload_db_clients.get(name)
            if client is None:
                client = self.create_client(ReloadDatabase, name)
                self._reload_db_clients[name] = client
            request = ReloadDatabase.Request()
            request.filepath = path
            response = self._call_service(client, request)
            results[name] = None if response is None else bool(response.success)
        return results

    def reload_route_graph(self, path):
        """Ask route_server to load `path`. True / False / None (no answer)."""
        request = SetRouteGraph.Request()
        request.graph_filepath = path
        response = self._call_service(self.set_route_graph_client, request)
        return None if response is None else bool(response.success)


def validate_route_graph(graph):
    """Check a route graph matches what GeoJsonGraphFileLoader expects.

    Returns a list of human-readable problems (empty = valid). Catching
    these here gives the dashboard a clear 400 instead of route_server
    rejecting the file after it has already been written.
    """
    if not isinstance(graph, dict) or graph.get("type") != "FeatureCollection":
        return ['top level must be a GeoJSON object with "type": "FeatureCollection"']
    features = graph.get("features")
    if not isinstance(features, list):
        return ['"features" must be a list']

    errors = []
    node_ids = set()
    edges = []
    for i, feature in enumerate(features):
        geometry = feature.get("geometry") if isinstance(feature, dict) else None
        props = feature.get("properties") if isinstance(feature, dict) else None
        if not isinstance(geometry, dict) or not isinstance(props, dict):
            errors.append(f"feature {i}: needs \"geometry\" and \"properties\" objects")
            continue
        kind = geometry.get("type")
        if kind == "Point":
            node_id = props.get("id")
            coords = geometry.get("coordinates")
            if not isinstance(node_id, int) or isinstance(node_id, bool):
                errors.append(f"feature {i}: node id must be an integer")
            elif node_id in node_ids:
                errors.append(f"feature {i}: duplicate node id {node_id}")
            else:
                node_ids.add(node_id)
            if (not isinstance(coords, list) or len(coords) < 2 or
                    not all(isinstance(c, (int, float)) and not isinstance(c, bool)
                            for c in coords[:2])):
                errors.append(f"feature {i}: node coordinates must be [x, y]")
        elif kind == "MultiLineString":
            edges.append((i, props))
        else:
            errors.append(f"feature {i}: unsupported geometry type {kind!r} "
                          "(expected Point or MultiLineString)")

    edge_ids = set()
    for i, props in edges:
        edge_id = props.get("id")
        if not isinstance(edge_id, int) or isinstance(edge_id, bool):
            errors.append(f"feature {i}: edge id must be an integer")
        elif edge_id in edge_ids:
            errors.append(f"feature {i}: duplicate edge id {edge_id}")
        else:
            edge_ids.add(edge_id)
        for key in ("startid", "endid"):
            if props.get(key) not in node_ids:
                errors.append(f"feature {i}: edge {edge_id} {key}={props.get(key)!r} "
                              "is not an existing node id")

    if not node_ids:
        errors.append("graph has no nodes")
    return errors


def dock_to_json(name, fields):
    """One dock_database.yaml entry -> the REST shape the dashboard uses.

    The file stores id as a quoted string ("4") and drop docks omit it;
    both readers (opennav_docking, forklift DockDatabase) accept that, but
    the dashboard sends and expects an int or null.
    """
    fields = fields if isinstance(fields, dict) else {}
    marker_id = fields.get("id")
    try:
        marker_id = int(marker_id) if marker_id is not None else None
    except (TypeError, ValueError):
        marker_id = None
    lift = fields.get("lift")
    try:
        lift = float(lift) if lift is not None else None
    except (TypeError, ValueError):
        lift = None
    return {
        "name": name,
        "type": fields.get("type"),
        "frame": fields.get("frame", "map"),
        "pose": fields.get("pose"),
        "id": marker_id,
        "lift": lift,
    }


def next_dock_name(dock_type, existing_names):
    """charging_dock_1, charging_dock_3 -> charging_dock_4 (max + 1)."""
    pattern = re.compile(rf"^{re.escape(dock_type)}_(\d+)$")
    numbers = [int(m.group(1)) for m in map(pattern.match, existing_names) if m]
    return f"{dock_type}_{max(numbers, default=0) + 1}"


def format_yaml_float(value):
    """3.14 -> '3.14', 0 -> '0.0', 1e-7 -> '0.0' (never YAML-1.1-string '1e-05')."""
    text = f"{round(float(value), 6):.6f}".rstrip("0")
    text = text + "0" if text.endswith(".") else text
    return "0.0" if text == "-0.0" else text


def append_yaml_entry(text, top_key, name, render_fields):
    """Return `text` with one entry appended under the trailing `<top_key>:`.

    Appending text (instead of load + dump) keeps every hand-written
    comment and alignment in the file untouched. Only valid while
    `top_key` is the last top-level key; the caller re-parses the result
    and refuses to write if it doesn't come out as exactly old + new.
    `render_fields(indent)` returns the entry's field lines.
    """
    lines = text.splitlines()
    top_level = [i for i, line in enumerate(lines)
                 if line and not line[0].isspace() and not line.startswith("#")]
    key_line = next(
        (i for i in top_level if re.match(rf"^{re.escape(top_key)}\s*:", lines[i])), None)
    if key_line is None or key_line != top_level[-1]:
        raise ValueError(f"'{top_key}:' must be the last top-level key to append safely")
    docks_line = key_line
    if lines[docks_line].split(":", 1)[1].split("#", 1)[0].strip():
        # e.g. `docks: {}` -- nothing to append under; start a real block.
        lines[docks_line] = f"{top_key}:"

    # Match the file's own indentation (first dock key and its fields).
    key_indent, field_indent = "  ", "    "
    for i in range(docks_line + 1, len(lines)):
        stripped = lines[i].strip()
        if stripped and not stripped.startswith("#"):
            key_indent = lines[i][:len(lines[i]) - len(lines[i].lstrip())]
            for j in range(i + 1, len(lines)):
                if lines[j].strip() and not lines[j].strip().startswith("#"):
                    indent = lines[j][:len(lines[j]) - len(lines[j].lstrip())]
                    if len(indent) > len(key_indent):
                        field_indent = indent
                    break
            break
    if len(field_indent) <= len(key_indent):
        field_indent = key_indent + "  "

    entry = ["", f"{key_indent}{name}:"] + render_fields(field_indent)

    while lines and not lines[-1].strip():
        lines.pop()
    return "\n".join(lines + entry) + "\n"


def dock_entry_lines(fields):
    """Renderer for append_yaml_entry: one dock_database.yaml entry."""
    def render(indent):
        pose = ", ".join(format_yaml_float(v) for v in fields["pose"])
        lines = [f"{indent}type: {fields['type']}",
                 f"{indent}frame: {fields['frame']}",
                 f"{indent}pose: [{pose}]"]
        if fields.get("id") is not None:
            lines.append(f'{indent}id: "{fields["id"]}"')
        lines.append(f"{indent}lift: {format_yaml_float(fields['lift'])}")
        return lines
    return render


def mission_entry_lines(fields):
    """Renderer for append_yaml_entry: one missions.yaml template.

    Field order follows the hand-written entries: type first, then the
    ids/poses for that type, waypoints last as a block list.
    """
    def render(indent):
        lines = [f"{indent}type: {fields['mission_type']}"]
        for key in ("pick_dock_id", "drop_dock_id", "dock_id"):
            if fields.get(key) is not None:
                lines.append(f"{indent}{key}: {fields[key]}")
        for key in ("pick_area_pose", "drop_area_pose"):
            if fields.get(key) is not None:
                pose = ", ".join(format_yaml_float(v) for v in fields[key])
                lines.append(f"{indent}{key}: [{pose}]")
        if fields.get("station_wait_duration") is not None:
            lines.append(
                f"{indent}station_wait_duration: "
                f"{format_yaml_float(fields['station_wait_duration'])}")
        if fields.get("waypoints") is not None:
            lines.append(f"{indent}waypoints:")
            for waypoint in fields["waypoints"]:
                pose = ", ".join(format_yaml_float(v) for v in waypoint)
                lines.append(f"{indent}  - [{pose}]")
        return lines
    return render


def mission_to_json(name, fields):
    """One missions.yaml entry -> the REST shape ('type' -> 'mission_type')."""
    fields = fields if isinstance(fields, dict) else {}
    entry = {"name": name, "mission_type": fields.get("type")}
    for key in ("pick_dock_id", "drop_dock_id", "dock_id", "pick_area_pose",
                "drop_area_pose", "waypoints"):
        if fields.get(key) is not None:
            entry[key] = fields[key]
    if fields.get("station_wait_duration") is not None:
        try:
            entry["station_wait_duration"] = float(fields["station_wait_duration"])
        except (TypeError, ValueError):
            pass
    return entry


def write_file_atomic(path, text):
    """Write via temp file + rename so readers never see a half file.

    Follows symlinks first: with colcon --symlink-install the configured
    path is install/.../share/... -> src/..., and renaming onto the link
    itself would replace it with a plain file, cutting it off from src/.
    """
    path = os.path.realpath(path)
    directory = os.path.dirname(path)
    fd, tmp_path = tempfile.mkstemp(dir=directory, prefix=".route_graph_", suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        # mkstemp creates 0600; keep the original file's permissions.
        mode = os.stat(path).st_mode & 0o777 if os.path.exists(path) else 0o644
        os.chmod(tmp_path, mode)
        os.replace(tmp_path, path)
    except BaseException:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)
        raise


def build_app(ros_node: FastApiBridgeNode) -> FastAPI:
    app = FastAPI(title="forklift_web_bridge REST API")

    # The dashboard runs on another origin (different PC/port), so browsers
    # send a CORS preflight (OPTIONS) before POSTs with a JSON body; without
    # this middleware FastAPI answers those with 405 Method Not Allowed.
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @app.get("/health")
    def health():
        return {"status": "ok"}

    @app.get("/robot/state")
    def robot_state():
        return ros_node.get_cached("robot_state") or {}

    @app.get("/navigation/status")
    def navigation_status():
        return ros_node.get_cached("navigation_status") or {}

    @app.post("/navigation/goal")
    def navigation_goal(req: NavigateRequest):
        ros_node.publish_nav_command({
            "command": "navigate", "x": req.x, "y": req.y,
            "yaw": req.yaw, "frame_id": req.frame_id,
        })
        return {"accepted": True}

    @app.post("/navigation/cancel")
    def navigation_cancel():
        ros_node.publish_nav_command({"command": "cancel"})
        return {"accepted": True}

    @app.get("/mission/status")
    def mission_status():
        return ros_node.get_cached("mission_status") or {}

    @app.get("/task/status")
    def task_status():
        """Progress of the task forklift_task_manager is running now."""
        return ros_node.get_cached("task_status") or {}

    @app.post("/mission/execute")
    def mission_execute(req: MissionRequest):
        command = {"command": "execute", **req.dict(exclude_none=True)}
        ros_node.publish_mission_command(command)
        return {"accepted": True}

    @app.post("/mission/cancel")
    def mission_cancel():
        ros_node.publish_mission_command({"command": "cancel"})
        return {"accepted": True}

    @app.get("/diagnostics/summary")
    def diagnostics_summary():
        return ros_node.get_cached("diagnostics_summary") or {}

    @app.get("/bridge/status")
    def bridge_status():
        return ros_node.get_cached("bridge_status") or {}

    def _route_graph_path():
        path = ros_node.get_route_graph_file()
        if not path:
            raise HTTPException(
                status_code=503,
                detail="route graph file unknown: route_server not reachable "
                       "(set route_graph_file to override)")
        return path

    @app.get("/routes/graph")
    def get_route_graph():
        path = _route_graph_path()
        try:
            with open(path) as f:
                return json.load(f)
        except FileNotFoundError:
            raise HTTPException(status_code=404, detail=f"route graph file not found: {path}")
        except json.JSONDecodeError as e:
            raise HTTPException(status_code=500, detail=f"route graph file is not valid JSON: {e}")

    @app.post("/routes/graph")
    def post_route_graph(graph: dict = Body(...), force: bool = False):
        # route_server clears its graph before loading the new one, so
        # swapping it under an active route would drop the route mid-drive.
        nav = ros_node.get_cached("navigation_status") or {}
        if nav.get("goal_active") and not force:
            raise HTTPException(
                status_code=409,
                detail="navigation goal active; cancel it first or pass ?force=true")

        errors = validate_route_graph(graph)
        if errors:
            raise HTTPException(status_code=400, detail={"errors": errors})

        path = _route_graph_path()
        with ros_node._graph_write_lock:
            try:
                with open(path) as f:
                    previous = f.read()
            except FileNotFoundError:
                previous = None

            try:
                write_file_atomic(path, json.dumps(graph, indent=2) + "\n")
            except OSError as e:
                raise HTTPException(status_code=500, detail=f"could not write {path}: {e}")

            reloaded = ros_node.reload_route_graph(path)
            if reloaded:
                if previous is not None:
                    # One-step undo, next to the live file.
                    write_file_atomic(os.path.realpath(path) + ".bak", previous)
                return {"success": True, "graph_filepath": path}

            # Rejected or no answer: put the old file back so disk and
            # route_server agree, and have route_server load it again (it
            # already dropped its graph when the bad load started).
            restored = False
            if previous is not None:
                write_file_atomic(path, previous)
                restored = bool(ros_node.reload_route_graph(path))
            reason = ("route_server rejected the graph" if reloaded is False
                      else "route_server set_route_graph service did not answer")
            raise HTTPException(
                status_code=502,
                detail={"error": reason, "previous_graph_restored": restored})

    def _read_yaml_section(path, top_key, what):
        """-> (raw text, entries dict in file order) under `top_key`."""
        if not path:
            raise HTTPException(status_code=503, detail=f"{what} file unknown")
        try:
            with open(path) as f:
                text = f.read()
        except FileNotFoundError:
            raise HTTPException(status_code=404, detail=f"{what} not found: {path}")
        try:
            data = yaml.safe_load(text) or {}
        except yaml.YAMLError as e:
            raise HTTPException(status_code=500, detail=f"{what} is not valid YAML: {e}")
        entries = data.get(top_key) if isinstance(data, dict) else None
        if entries is None:
            entries = {}
        if not isinstance(entries, dict):
            raise HTTPException(
                status_code=500, detail=f"{what} '{top_key}' is not a mapping")
        return text, entries

    def _append_entry(path, text, existing, top_key, name, render):
        """Append one entry, verify nothing else changed, write. -> new entry."""
        try:
            new_text = append_yaml_entry(text, top_key, name, render)
            parsed = (yaml.safe_load(new_text) or {}).get(top_key) or {}
        except (ValueError, yaml.YAMLError) as e:
            raise HTTPException(status_code=500, detail=f"cannot append to {path} safely: {e}")
        # Must be exactly the old entries, same order, plus the new one.
        if (list(parsed) != list(existing) + [name] or
                any(parsed[k] != existing[k] for k in existing)):
            raise HTTPException(
                status_code=500,
                detail=f"appending to {path} would change existing entries; "
                       "file left untouched")
        try:
            write_file_atomic(path, new_text)
        except OSError as e:
            raise HTTPException(status_code=500, detail=f"could not write {path}: {e}")
        return parsed[name]

    def _read_dock_database():
        """-> (path, raw text, docks dict in file order)."""
        path = ros_node.get_dock_database_file()
        text, docks = _read_yaml_section(path, "docks", "dock database")
        return path, text, docks

    @app.get("/docks")
    def list_docks():
        _, _, docks = _read_dock_database()
        return [dock_to_json(name, fields) for name, fields in docks.items()]

    @app.post("/docks", status_code=201)
    def create_dock(req: DockCreateRequest):
        if len(req.pose) != 3 or not all(math.isfinite(v) for v in req.pose):
            raise HTTPException(status_code=400, detail="pose must be [x, y, yaw] finite numbers")
        if not FRAME_RE.match(req.frame):
            raise HTTPException(status_code=400, detail=f"invalid frame {req.frame!r}")
        if req.lift is not None and not math.isfinite(req.lift):
            raise HTTPException(status_code=400, detail="lift must be a finite number")
        if req.id is not None and req.id < 0:
            raise HTTPException(status_code=400, detail="id (ArUco marker) must be >= 0")
        if req.name is not None and not ENTRY_NAME_RE.match(req.name):
            raise HTTPException(
                status_code=400, detail="name may only contain letters, digits and '_'")

        with ros_node._dock_write_lock:
            path, text, docks = _read_dock_database()
            name = req.name or next_dock_name(req.type, docks.keys())
            if name in docks:
                raise HTTPException(status_code=400, detail=f"dock '{name}' already exists")

            fields = {
                "type": req.type,
                "frame": req.frame,
                "pose": [float(v) for v in req.pose],
                "id": req.id,
                # Every existing entry carries lift; 0.0 = no lift move.
                "lift": req.lift if req.lift is not None else 0.0,
            }
            saved_fields = _append_entry(
                path, text, docks, "docks", name, dock_entry_lines(fields))
            reload = ros_node.reload_docking_servers(path)

        saved = dock_to_json(name, saved_fields)
        saved["docking_server_reload"] = reload
        return saved

    def _read_mission_database():
        """-> (path, raw text, templates dict in file order)."""
        path = ros_node.get_mission_database_file()
        text, missions = _read_yaml_section(path, "missions", "mission database")
        return path, text, missions

    def _check_poses(poses, what):
        for pose in poses:
            if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
                raise HTTPException(
                    status_code=400, detail=f"{what} entries must be [x, y, yaw] finite numbers")

    @app.get("/missions/templates")
    def list_mission_templates():
        _, _, missions = _read_mission_database()
        return {"missions": [mission_to_json(name, fields)
                             for name, fields in missions.items()]}

    @app.post("/missions/templates", status_code=201)
    def create_mission_template(req: MissionTemplateRequest):
        name = (req.name or "").strip()
        if not name:
            raise HTTPException(status_code=400, detail="name is required")
        if not ENTRY_NAME_RE.match(name):
            raise HTTPException(
                status_code=400, detail="name may only contain letters, digits and '_'")

        # Keep only the fields that belong to this mission type, so a
        # dashboard that sends stale form state can't write a CHARGING
        # template carrying waypoints.
        allowed = (MISSION_REQUIRED_FIELDS[req.mission_type] +
                   MISSION_OPTIONAL_FIELDS[req.mission_type])
        fields = {"mission_type": req.mission_type}
        for key in allowed:
            value = getattr(req, key)
            if value is not None:
                fields[key] = value
        missing = [key for key in MISSION_REQUIRED_FIELDS[req.mission_type]
                   if fields.get(key) in (None, "", [])]
        if missing:
            raise HTTPException(
                status_code=400,
                detail=f"{req.mission_type} needs {', '.join(missing)}")

        if "waypoints" in fields:
            _check_poses(fields["waypoints"], "waypoints")
        for key in ("pick_area_pose", "drop_area_pose"):
            if key in fields:
                _check_poses([fields[key]], key)
        if fields.get("station_wait_duration") is not None:
            duration = fields["station_wait_duration"]
            if not math.isfinite(duration) or duration < 0.0:
                raise HTTPException(
                    status_code=400, detail="station_wait_duration must be >= 0")

        # Referenced docks must exist -- a template pointing at a missing
        # dock only fails much later, mid-mission. Skipped (not failed) if
        # the dock database can't be read at all.
        referenced = [fields[key] for key in MISSION_DOCK_FIELDS if key in fields]
        if referenced:
            try:
                _, _, docks = _read_dock_database()
            except HTTPException:
                docks = None
            if docks is not None:
                unknown = [dock_id for dock_id in referenced if dock_id not in docks]
                if unknown:
                    raise HTTPException(
                        status_code=400,
                        detail=f"unknown dock id(s) in dock database: {', '.join(unknown)}")

        with ros_node._mission_write_lock:
            path, text, missions = _read_mission_database()
            if name in missions:
                raise HTTPException(
                    status_code=400, detail=f"mission template '{name}' already exists")
            saved_fields = _append_entry(
                path, text, missions, "missions", name, mission_entry_lines(fields))

        # mission_server reads missions.yaml only at startup, but nothing
        # looks templates up at execution time (every ExecuteMission goal
        # carries its own parameters), so no reload call is needed here.
        return mission_to_json(name, saved_fields)

    return app


def main():
    rclpy.init()
    node = FastApiBridgeNode()

    # rclpy spins on its own thread -- uvicorn owns the main thread's
    # asyncio event loop, the two never share one.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    port = node.get_parameter("rest_port").value
    address = node.get_parameter("rest_address").value
    app = build_app(node)

    try:
        uvicorn.run(app, host=address, port=port, log_level="info")
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()

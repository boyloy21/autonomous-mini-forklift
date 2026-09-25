"""Bridges ROS <-> MQTT using paho-mqtt.
MQTT subscribe: factory/<robot_id>/command, factory/<robot_id>/mission
MQTT publish : factory/<robot_id>/status, /navigation, /task, /battery, /diagnostics, /alarm

paho-mqtt runs its own network thread (loop_start()) independent of rclpy's spin thread; callbacks from
paho land on that thread and just call self.<ros_publisher>.publish(...), which it thread-safe in rclpy.
Auto-reconnect is handled by paho's built-in reconnect_delay_set() plus an on_disconnect callback that re-triggers connect().
"""

import json

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from forklift_interfaces.msg import (
    NavigationCommand, MissionCommand, MissionStep, RobotState, TaskStatus,
    RobotHealth, SafetyStatus,
)

class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')
        
        self.enabled = self.declare_parameter('enable_mqtt', True).value
        self.robot_id = self.declare_parameter('robot_id', 'forklift01').value
        self.broker_host = self.declare_parameter('broker_host', 'localhost').value
        self.broker_port = self.declare_parameter('broker_port', 1883).value
        self.username = self.declare_parameter('username', '').value
        self.password = self.declare_parameter('password', '').value
        self.keepalive = self.declare_parameter('keepalive_sec', 30).value
        self.qos = self.declare_parameter('qos', 1).value
        self.reconnect_delay = self.declare_parameter('reconnect_delay_sec', 5.0).value
        heartbeat_rate = self.declare_parameter('heartbeat_rate_hz', 0.2).value
        self.prefix = self.declare_parameter('topic_prefix', 'factory').value
        
        self.base = f'{self.prefix}/{self.robot_id}'
        self.last_safety_leve = None
        
        self.nav_command_pub = self.create_publisher(
            NavigationCommand, 'web_bridge/navigation_command', 10)
        self.mission_command_pub = self.create_publisher(
            MissionCommand, 'web_bridge/mission_command', 10
        )
        self.estop_pub = self.create_publisher(
            Empty, 'web/emergency_stop', 10
        )
        
        # ROS side: subscriptions to relay outbound onto MQTT
        self.create_subscription(RobotState, 'robot_state', self._on_robot_state, 10)
        self.create_subscription(TaskStatus, 'task_feedback', self._on_task_status, 10)
        self.create_subscription(RobotHealth, 'diagnostics/summary', self._on_diagnostics, 10)
        self.create_subscription(SafetyStatus, 'safety/status', self._on_safety_status, 10)
        
        if not self.enabled:
            self.get_logger().warn('MQTT disabled via enable_mqtt=false')
            return
        
        self.client = mqtt.Client(client_id=f'forklift_web_bridge_{self.robot_id}')
        if self.username:
            self.client.username_pw_set(self.username, self.password)
        self.client.will_set(
            f'{self.base}/status', payload=json.dumps({'online': False}), qos=self.qos, retain=True
        )
        self.client.reconnect_delay_set(min_delay=1, max_delay=int(self.reconnect_delay))
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        
        try:
            self.client.connect(self.broker_host, self.broker_port, keepalive=self.keepalive)
            self.client.loop_start()
        except Exception as exc:
            self.get_logger().error(f'Initial MQTT connect failed: {exc} (will keep retrying)')
            self.client.loop_start()
            
        self.create_timer(1.0 / heartbeat_rate, self._heartbeat)
        self.get_logger().info(f"mqtt_bridge_node ready (broker={self.broker_host}:{self.broker_port})")
        
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('MQTT connected')
            client.subscribe(f'{self.base}/command', qos=self.qos)
            client.subscribe(f'{self.base}/mission', qos=self.qos)
            client.publish(f'{self.base}/status', json.dumps({'online': True}), qos=self.qos, retain=True)
        else:
            self.get_logger().error(f'MQTT connect failed, rc={rc}')

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warn(f'MQTT disconnected unexpectedly (rc={rc}), auto-reconnecting')
    def _handle_command(self, payload):
        kind = payload.get('type')
        if kind == 'navigation':
            cmd = NavigationCommand()
            action = payload.get('aciton', 'goto')
            cmd.request_id = payload.get('request_id', '')
            if action == 'goto':
                cmd.command = NavigationCommand.CMD_GOTO
                pose = payload.get('pose', {})
                cmd.target_pose.pose.position.x = float(pose.get('x', 0.0))
                cmd.target_pose.pose.position.y = float(pose.get('y', 0.0))
            elif action == 'cancel':
                cmd.command = NavigationCommand.CMD_CANCEL
            elif action == 'pause':
                cmd.command = NavigationCommand.CMD_PAUSE
            elif action == 'resume':
                cmd.command = NavigationCommand.CMD_RESUME   
            else:
                self.get_logger().warn(f'Unknown navigation action: {action}')
                return
            self.nav_command_pub.publish(cmd)
        elif kind == 'emergency_stop':
            self.estop_pub.publish(Empty())
            self.get_logger().warn('Emergency stop triggered via MQTT')
        else:
            self.get_logger().warn(f'Unknown command type: {kind}')
            
    def _handle_mission(self, payload):
        cmd = MissionCommand()
        action = payload.get('action', 'start')
        if action == 'start':
            cmd.command = MissionCommand.CMD_START
            mission = cmd.mission
            mission.mission_id = payload.get('mission_id', '')
            mission.priority = int(payload.get('priority', 0))
            for step_data in payload.get('steps', []):
                step = MissionStep()
                step.step_type = int(step_data.get('step_type', 0))
                step.pallet_id = step_data.get('pallet_id', '')
                step.station_id = step_data.get('station', '')
                step.lift_height = float(step_data.get('lift_height', 0.0))
                step.wait_duration = float(step_data.get('wait_duration', 0.0))
                mission.steps.append(step)
        elif action == 'cancel':
            cmd.command = MissionCommand.CMD_CANCEL
            cmd.mission_id = payload.get('mission_id', '')
        elif action == 'pause':
            cmd.command = MissionCommand.CMD_PAUSE
        elif action == 'resume':
            cmd.command = MissionCommand.CMD_RESUME
        else:
            self.get_logger().warn(f'Unknown mission action: {action}')
            return
        self.mission_command_pub.publish(cmd)
        
    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            self.get_logger().warn(f'Bad MQTT payload on {msg.topic}: {exec}')
            
        if msg.topic == f'{self.base}/command':
            self.__handle_command(payload)
        elif msg.topic == f'{self.base}/mission':
            self.__handle_mission(payload)
            
    def _on_robot_state(self, msg: RobotState):
        if not self.enabled:
            return
        payload = {
            'robot_id': msg.robot_id,
            'task_state': msg.task_state,
            'mission_state': msg.mission_state,
            'safety_allow_motion': msg.safety_allow_motion
        }
        self.client.publish(f'{self.base}/status', json.dumps(payload), qos=self.qos)
        self.client.publish(f'{self.base}/battery',  json.dumps({'percent': msg.battery_percent, 'voltage': msg.battery_voltage}),qos=self.qos)
        
    def _on_task_status(self, msg: TaskStatus):
        if not self.enabled:
            return
        payload = {'task_id': msg.task_id, 'state': msg.state, 'progress': msg.progress}
        self.client.publish(f'{self.base}/task', json.dumps(payload), qos=self.qos)
        
    def _on_diagnostics(self, msg: RobotHealth):
        if not self.enabled:
            return
        payload = {
            'overall_status': msg.overall_status,
            'cpu_usage': msg.cpu_usage,
            'gpu_usage': msg.gpu_usage,
            'ram_usage': msg.ram_usage,
            'battery_percentage': msg.battery_percent,
        }
        self.client.publish(f'{self.base}/diagnostics', json.dumps(payload), qos=self.qos)
        
    def _on_safety_status(self, msg: SafetyStatus):
        # Edge-triggered alarm - only publish when the state actually changes.
        if not self.enabled or msg.state == self.last_safety_leve:
            return
        self.last_safety_leve = msg.state
        if msg.state >= SafetyStatus.STATE_STOP:
            self.client.publish(f'{self.base}/alarm', json.dumps({'state': msg.state, 'message': msg.message}), qos=self.qos)
            
    def  _heartbeat(self):
        if not self.enabled:
            return
        self.client.publish(
            f'{self.base}/status',
            json.dumps({'online': True, 'heartbeat': self.get_clock.now().nanoseconds}),
            qos=self.qos, retain=True)
        
def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        if getattr(node, 'enabled', False):
            node.client.loop_stop()
            node.client.disconnect()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
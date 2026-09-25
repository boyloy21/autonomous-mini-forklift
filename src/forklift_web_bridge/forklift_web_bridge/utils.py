import math
import time
import uuid

def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (radians) from a quaternion, ignoring roll/pitch"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def quaternion_from_yaw(yaw: float):
    """Returns (x, y, z, w)"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def now_iso() -> str:
    """UTC ISO-8601 timestamp for JSON payload sent to the dashboard."""
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

def clamp(value: float, lo: float, hi: float) -> float: 
    return max(lo, min(hi, value))

def new_request_id(prefix='web'):
    return f'{prefix}_{uuid.uuid4().hex[:12]}'

def call_service_sync(client, request, timeout_sec=5.0):
    """Call a ROS2 service from a non-spinning thread (e.g. a FastAPI request handler) and block for the result
    Assume the client's node is being spun by an executor running in a different thread - call_async() queues the request there, and this just polls
    the returned future until it resolves or times out.
    """
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    while not future.done():
        if time.monotonic() > deadline:
            raise TimeoutError(f'Service call to {client.srv_name} timed out')
        time.sleep(0.01)
        
    if future.exception() is not None:
        raise future.exception()
    return future.result()
    
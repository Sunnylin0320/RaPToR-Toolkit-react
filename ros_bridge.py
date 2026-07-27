import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist

from sensor_websocket import (
    start_websocket_server,
    update_sensor_state,
    websocket_clients,
)

# --- Mapping from key name to linear/angular velocity ---
KEY_TO_TWIST = {
    "w": (0.3, 0.0),    # forward
    "s": (-0.3, 0.0),   # backward
    "a": (0.0, 0.5),    # turn left (spin in place)
    "d": (0.0, -0.5),   # turn right
    "stop": (0.0, 0.0), # explicit stop
}


class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('react_bridge_node')

        # --- Subscriber: battery state -> push to WebSocket clients ---
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazard_callback, 10)
        self.create_subscription(DockStatus, '/dock_status', self.dock_callback, 10)

        # --- Publisher: cmd_vel, driven by keyboard commands from React ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def battery_callback(self, msg: BatteryState):
        update_sensor_state(
            "/battery_state",
            enabled=True,
            value=f"{round(msg.percentage * 100, 1)}%"
        )

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        update_sensor_state(
            "/odom",
            enabled=True,
            value=f"x={x:.2f}, y={y:.2f}"
        )

    def hazard_callback(self, msg: HazardDetectionVector):
        if len(msg.detections) == 0:
            value = "None"
        else:
            types = [d.type for d in msg.detections]
            value = f"{len(msg.detections)} hazard(s): {types}"
        update_sensor_state("/hazard_detection", enabled=True, value=value)

    def dock_callback(self, msg: DockStatus):
        value = "Docked" if msg.is_docked else "Free"
        update_sensor_state("/dock_status", enabled=True, value=value)

    def publish_cmd_vel(self, key: str):
        linear, angular = KEY_TO_TWIST.get(key, (0.0, 0.0))
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Published cmd_vel for key '{key}': linear={linear}, angular={angular}")


# --- WebSocket receive handling: needs to hook into sensor_websocket's connections ---
# sensor_websocket.py's handler only sends data out; we patch in a receive loop here.
ros_node = None


async def handle_incoming_messages(websocket):
    """
    Listen for incoming messages (key presses) from a connected React client
    and forward them to the ROS 2 node to publish cmd_vel.
    """
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                key = data.get("key")
                if key and ros_node is not None:
                    ros_node.publish_cmd_vel(key)
            except json.JSONDecodeError:
                print(f"Received non-JSON message: {message}")
    except Exception as e:
        print(f"Incoming message handler stopped: {e}")


def start_ros_spin_thread(node):
    """Run rclpy.spin in a background thread so it doesn't block asyncio."""
    def spin():
        rclpy.spin(node)
    threading.Thread(target=spin, daemon=True).start()


def main():
    global ros_node

    rclpy.init()
    ros_node = RosBridgeNode()
    start_ros_spin_thread(ros_node)

    # Start the existing sensor WebSocket server (outgoing data: sensor values)
    start_websocket_server(host="0.0.0.0", port=6789)

    # NOTE: sensor_websocket.py's handler currently only sends data.
    # To receive incoming key press messages, we start a second, separate
    # WebSocket server on a different port dedicated to control commands.
    async def control_handler(websocket):
        print(f"Control client connected: {websocket.remote_address}")
        await handle_incoming_messages(websocket)

    async def start_control_server():
        import websockets
        async with websockets.serve(control_handler, "0.0.0.0", 6790):
            print("Control WebSocket server started on ws://0.0.0.0:6790")
            await asyncio.Future()

    def run_control_server():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(start_control_server())

    threading.Thread(target=run_control_server, daemon=True).start()

    print("Bridge running. Sensor data on :6789, control commands on :6790")
    print("Press Ctrl+C to stop")

    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\nShutting down")
        rclpy.shutdown()


if __name__ == "__main__":
    main()

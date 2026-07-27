# for webot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import websockets
import json
import threading

connected_clients = set()

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.run_ws_server, daemon=True).start()

    def run_ws_server(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.start_server())

    async def start_server(self):
        async with websockets.serve(self.ws_handler, "0.0.0.0", 6790):
            print("cmd_vel WebSocket bridge running on ws://0.0.0.0:6790")
            await asyncio.Future()

    async def ws_handler(self, websocket):
        connected_clients.add(websocket)
        print(f"Webots connected: {websocket.remote_address}")
        try:
            await websocket.wait_closed()
        finally:
            connected_clients.remove(websocket)

    def cmd_vel_callback(self, msg):
        data = {
            "linear_x": msg.linear.x,
            "angular_z": msg.angular.z
        }
        message = json.dumps(data)
        asyncio.run_coroutine_threadsafe(
            self.broadcast(message), self.loop
        )

    async def broadcast(self, message):
        if connected_clients:
            await asyncio.gather(
                *[client.send(message) for client in connected_clients]
            )

def main():
    rclpy.init()
    node = CmdVelBridge()
    print("Listening to /cmd_vel ...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
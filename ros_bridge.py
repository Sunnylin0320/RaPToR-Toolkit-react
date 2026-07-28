import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import BatteryState, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from rcl_interfaces.msg import ParameterEvent, Log
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from lifecycle_msgs.msg import TransitionEvent
from control_msgs.msg import DynamicJointState
from ros_gz_interfaces.msg import Contacts
from irobot_create_msgs.msg import (
    HazardDetectionVector,
    DockStatus,
    InterfaceButtons,
    IrIntensityVector,
    IrOpcode,
    KidnapStatus,
    Mouse,
    SlipStatus,
    StopStatus,
    WheelStatus,
    WheelTicks,
    WheelVels,
)

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

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # --- Create 3 robot sensors / status (14) ---
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazard_callback, 10)
        self.create_subscription(DockStatus, '/dock_status', self.dock_callback, sensor_qos)
        self.create_subscription(InterfaceButtons, '/interface_buttons', self.interface_buttons_callback, sensor_qos)
        self.create_subscription(IrIntensityVector, '/ir_intensity', self.ir_intensity_callback, 10)
        self.create_subscription(IrOpcode, '/ir_opcode', self.ir_opcode_callback, sensor_qos)
        self.create_subscription(KidnapStatus, '/kidnap_status', self.kidnap_callback, 10)
        self.create_subscription(Mouse, '/mouse', self.mouse_callback, sensor_qos)
        self.create_subscription(SlipStatus, '/slip_status', self.slip_callback, sensor_qos)
        self.create_subscription(StopStatus, '/stop_status', self.stop_callback, 10)
        self.create_subscription(WheelStatus, '/wheel_status', self.wheel_status_callback, 10)
        self.create_subscription(WheelTicks, '/wheel_ticks', self.wheel_ticks_callback, 10)
        self.create_subscription(WheelVels, '/wheel_vels', self.wheel_vels_callback, 10)

        # --- Gazebo simulation-only topics (5) ---
        self.create_subscription(Contacts, '/bumper_contact', self.bumper_callback, 10)
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.create_subscription(Odometry, '/sim_ground_truth_pose', self.sim_gt_pose_callback, sensor_qos)
        self.create_subscription(Odometry, '/sim_ground_truth_dock_pose', self.sim_gt_dock_pose_callback, sensor_qos)
        self.create_subscription(Twist, '/diffdrive_controller/cmd_vel_unstamped', self.diffdrive_cmd_vel_callback, 10)

        # --- RViz user interaction inputs (3) ---
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)

        # --- Robot model description (4) ---
        self.create_subscription(String, '/robot_description', self.robot_description_callback, 10)
        self.create_subscription(String, '/standard_dock_description', self.dock_description_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(DynamicJointState, '/dynamic_joint_states', self.dynamic_joint_states_callback, 10)

        # --- ROS 2 system-level topics (6) ---
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.create_subscription(TFMessage, '/tf_static', self.tf_static_callback, 10)
        self.create_subscription(ParameterEvent, '/parameter_events', self.parameter_events_callback, 10)
        self.create_subscription(Log, '/rosout', self.rosout_callback, 10)
        self.create_subscription(TransitionEvent, '/joint_state_broadcaster/transition_event', self.jsb_transition_callback, 10)
        self.create_subscription(TransitionEvent, '/diffdrive_controller/transition_event', self.diffdrive_transition_callback, 10)

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

    def interface_buttons_callback(self, msg: InterfaceButtons):
        # NOTE: verify field names with `ros2 interface show irobot_create_msgs/msg/InterfaceButtons`
        pressed = []
        if msg.button_1.is_pressed:
            pressed.append("button_1")
        if msg.button_power.is_pressed:
            pressed.append("button_power")
        if msg.button_2.is_pressed:
            pressed.append("button_2")
        value = ", ".join(pressed) if pressed else "None pressed"
        update_sensor_state("/interface_buttons", enabled=True, value=value)

    def ir_intensity_callback(self, msg: IrIntensityVector):
        # NOTE: verify field names with `ros2 interface show irobot_create_msgs/msg/IrIntensityVector`
        values = [r.value for r in msg.readings]
        update_sensor_state("/ir_intensity", enabled=True, value=str(values))

    def ir_opcode_callback(self, msg: IrOpcode):
        # NOTE: verify field names with `ros2 interface show irobot_create_msgs/msg/IrOpcode`
        update_sensor_state("/ir_opcode", enabled=True, value=f"opcode={msg.opcode}, sensor={msg.sensor}")

    def kidnap_callback(self, msg: KidnapStatus):
        value = "Kidnapped" if msg.is_kidnapped else "Normal"
        update_sensor_state("/kidnap_status", enabled=True, value=value)

    def mouse_callback(self, msg: Mouse):
        # NOTE: verify field names with `ros2 interface show irobot_create_msgs/msg/Mouse`
        update_sensor_state(
            "/mouse", enabled=True,
            value=f"x={msg.integrated_x:.2f}, y={msg.integrated_y:.2f}"
        )

    def slip_callback(self, msg: SlipStatus):
        value = "Slipping" if msg.is_slipping else "Normal"
        update_sensor_state("/slip_status", enabled=True, value=value)

    def stop_callback(self, msg: StopStatus):
        value = "Stopped" if msg.is_stopped else "Moving"
        update_sensor_state("/stop_status", enabled=True, value=value)

    def wheel_status_callback(self, msg: WheelStatus):
        # NOTE: verify field names with `ros2 interface show irobot_create_msgs/msg/WheelStatus`
        value = f"enabled={msg.wheels_enabled}, current_L={msg.current_ma_left}, current_R={msg.current_ma_right}"
        update_sensor_state("/wheel_status", enabled=True, value=value)

    def wheel_ticks_callback(self, msg: WheelTicks):
        update_sensor_state("/wheel_ticks", enabled=True, value=f"L={msg.ticks_left}, R={msg.ticks_right}")

    def wheel_vels_callback(self, msg: WheelVels):
        update_sensor_state(
            "/wheel_vels", enabled=True,
            value=f"L={msg.velocity_left:.2f}, R={msg.velocity_right:.2f}"
        )

    # --- Gazebo simulation-only topics ---

    def bumper_callback(self, msg: Contacts):
        update_sensor_state("/bumper_contact", enabled=True, value=f"{len(msg.contacts)} contact(s)")

    def clock_callback(self, msg: Clock):
        update_sensor_state("/clock", enabled=True, value=f"sec={msg.clock.sec}")

    def sim_gt_pose_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        update_sensor_state("/sim_ground_truth_pose", enabled=True, value=f"x={x:.2f}, y={y:.2f}")

    def sim_gt_dock_pose_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        update_sensor_state("/sim_ground_truth_dock_pose", enabled=True, value=f"x={x:.2f}, y={y:.2f}")

    def diffdrive_cmd_vel_callback(self, msg: Twist):
        update_sensor_state(
            "/diffdrive_controller/cmd_vel_unstamped", enabled=True,
            value=f"linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}"
        )

    # --- RViz user interaction inputs ---

    def clicked_point_callback(self, msg: PointStamped):
        update_sensor_state(
            "/clicked_point", enabled=True,
            value=f"x={msg.point.x:.2f}, y={msg.point.y:.2f}"
        )

    def goal_pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        update_sensor_state("/goal_pose", enabled=True, value=f"x={x:.2f}, y={y:.2f}")

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        update_sensor_state("/initialpose", enabled=True, value=f"x={x:.2f}, y={y:.2f}")

    # --- Robot model description ---

    def robot_description_callback(self, msg: String):
        update_sensor_state("/robot_description", enabled=True, value=f"{len(msg.data)} chars")

    def dock_description_callback(self, msg: String):
        update_sensor_state("/standard_dock_description", enabled=True, value=f"{len(msg.data)} chars")

    def joint_states_callback(self, msg: JointState):
        update_sensor_state("/joint_states", enabled=True, value=f"{len(msg.name)} joint(s): {list(msg.name)}")

    def dynamic_joint_states_callback(self, msg: DynamicJointState):
        # NOTE: verify field names with `ros2 interface show control_msgs/msg/DynamicJointState`
        update_sensor_state("/dynamic_joint_states", enabled=True, value=f"{len(msg.joint_names)} joint(s)")

    # --- ROS 2 system-level topics ---

    def tf_callback(self, msg: TFMessage):
        update_sensor_state("/tf", enabled=True, value=f"{len(msg.transforms)} transform(s)")

    def tf_static_callback(self, msg: TFMessage):
        update_sensor_state("/tf_static", enabled=True, value=f"{len(msg.transforms)} transform(s)")

    def parameter_events_callback(self, msg: ParameterEvent):
        update_sensor_state("/parameter_events", enabled=True, value=f"node: {msg.node}")

    def rosout_callback(self, msg: Log):
        update_sensor_state("/rosout", enabled=True, value=f"[{msg.name}] {msg.msg}")

    def jsb_transition_callback(self, msg: TransitionEvent):
        # NOTE: verify field names with `ros2 interface show lifecycle_msgs/msg/TransitionEvent`
        update_sensor_state("/joint_state_broadcaster/transition_event", enabled=True, value=str(msg.transition.label))

    def diffdrive_transition_callback(self, msg: TransitionEvent):
        update_sensor_state("/diffdrive_controller/transition_event", enabled=True, value=str(msg.transition.label))

     # --- Movement control ---

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

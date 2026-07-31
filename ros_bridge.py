"""
Bridge script: connects ROS 2 (Gazebo simulation) <-> WebSocket <-> React frontend
"""
import asyncio
import json
import threading
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from rosidl_runtime_py.set_message import set_message_fields
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
from irobot_create_msgs.action import (
    Dock,
    Undock,
    RotateAngle,
    DriveDistance,
    DriveArc,
    NavigateToPosition,
    AudioNoteSequence,
    LedAnimation,
)

from sensor_websocket import (
    start_websocket_server,
    update_sensor_state,
    websocket_clients,
)

KEY_TO_TWIST = {
    "w": (0.3, 0.0),
    "x": (-0.3, 0.0),
    "a": (0.0, 0.5),
    "d": (0.0, -0.5),
    "q": (0.3, 0.5),
    "e": (0.3, -0.5),
    "z": (-0.3, 0.5),
    "c": (-0.3, -0.5),
    "s": (0.0, 0.0),
    "stop": (0.0, 0.0),
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

        # --- Action clients ---
        self.dock_client = ActionClient(self, Dock, '/dock')
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.rotate_angle_client = ActionClient(self, RotateAngle, '/rotate_angle')
        self.drive_distance_client = ActionClient(self, DriveDistance, '/drive_distance')
        self.drive_arc_client = ActionClient(self, DriveArc, '/drive_arc')
        self.navigate_client = ActionClient(self, NavigateToPosition, '/navigate_to_position')
        self.audio_client = ActionClient(self, AudioNoteSequence, '/audio_note_sequence')
        self.led_client = ActionClient(self, LedAnimation, '/led_animation')

    # --- Create 3 robot sensors / status ---

    def battery_callback(self, msg: BatteryState):
        update_sensor_state("/battery_state", enabled=True, value=f"{round(msg.percentage * 100, 1)}%")

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        update_sensor_state("/odom", enabled=True, value=f"x={x:.2f}, y={y:.2f}")

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
        values = [r.value for r in msg.readings]
        update_sensor_state("/ir_intensity", enabled=True, value=str(values))

    def ir_opcode_callback(self, msg: IrOpcode):
        update_sensor_state("/ir_opcode", enabled=True, value=f"opcode={msg.opcode}, sensor={msg.sensor}")

    def kidnap_callback(self, msg: KidnapStatus):
        value = "Kidnapped" if msg.is_kidnapped else "Normal"
        update_sensor_state("/kidnap_status", enabled=True, value=value)

    def mouse_callback(self, msg: Mouse):
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

    # --- Action execution ---

    def send_action(self, action_name: str, params: dict):
        self.get_logger().info(">>> USING set_message_fields VERSION <<<")
        """
        Send a goal to the specified action server.
        params is a dict already parsed from the JSON sent by React.
        Uses rosidl_runtime_py to correctly populate nested message fields
        (e.g. navigate_to_position's goal_pose, which must be a PoseStamped).
        """
        action_map = {
            "dock": (self.dock_client, Dock.Goal),
            "undock": (self.undock_client, Undock.Goal),
            "rotate_angle": (self.rotate_angle_client, RotateAngle.Goal),
            "drive_distance": (self.drive_distance_client, DriveDistance.Goal),
            "drive_arc": (self.drive_arc_client, DriveArc.Goal),
            "navigate_to_position": (self.navigate_client, NavigateToPosition.Goal),
            "audio_note_sequence": (self.audio_client, AudioNoteSequence.Goal),
            "led_animation": (self.led_client, LedAnimation.Goal),
        }

        if action_name not in action_map:
            self.get_logger().warn(f"Unknown action: {action_name}")
            return

        client, goal_type = action_map[action_name]
        goal_msg = goal_type()

        try:
            set_message_fields(goal_msg, params)
        except Exception as e:
            self.get_logger().error(f"Failed to set fields for action '{action_name}': {e}")
            return

        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"Action server for '{action_name}' not available!")
            return

        self.get_logger().info(f"Sending action '{action_name}' with params: {params}")
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f: self._goal_response_callback(f, action_name)
        )

    def _goal_response_callback(self, future, action_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Action '{action_name}' was REJECTED by the action server")
            return

        self.get_logger().info(f"Action '{action_name}' was ACCEPTED, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._goal_result_callback(f, action_name)
        )

    def _goal_result_callback(self, future, action_name):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Action '{action_name}' finished with status={status}, result={result}")


ros_node = None


async def handle_incoming_messages(websocket):
    """
    Listen for incoming messages from a connected React client.
    Two message shapes are supported:
      - {"key": "w"}                          -> movement command
      - {"action": "dock", "params": {...}}    -> action command
      - {"command": "ros2 topic list"}         -> execute a shell command
    """
    try:
        async for message in websocket:
            try:
                data = json.loads(message)

                if "key" in data:
                    key = data.get("key")
                    if key and ros_node is not None:
                        ros_node.publish_cmd_vel(key)

                elif "action" in data:
                    action_name = data.get("action")
                    params = data.get("params", {})
                    if action_name and ros_node is not None:
                        ros_node.send_action(action_name, params)

                elif "command" in data:
                    command = data.get("command")
                    if command:
                        asyncio.create_task(execute_terminal_command(websocket, command))

            except json.JSONDecodeError:
                print(f"Received non-JSON message: {message}")
    except Exception as e:
        print(f"Incoming message handler stopped: {e}")

async def execute_terminal_command(websocket, command: str):
    """
    Execute a shell command, matching the original Tkinter terminal.py
    behavior: runs the raw command via the shell, streams stdout and
    stderr back to the client as they arrive, tagging stderr lines as
    errors so the frontend can style them differently.

    Mirrors terminal.py's use of subprocess.Popen(shell=True, ...).
    """
    if command.strip().lower() == "clear":
        await websocket.send(json.dumps({"type": "terminal_clear"}))
        return

    process = subprocess.Popen(
        command,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    loop = asyncio.get_event_loop()

    def read_stream(stream, is_error):
        for line in iter(stream.readline, ""):
            asyncio.run_coroutine_threadsafe(
                websocket.send(json.dumps({
                    "type": "terminal_output",
                    "line": line,
                    "is_error": is_error,
                })),
                loop
            )
        stream.close()

    # Read stdout and stderr in separate threads, same reasoning as
    # terminal.py: don't block the main event loop while the command runs.
    stdout_thread = threading.Thread(target=read_stream, args=(process.stdout, False))
    stderr_thread = threading.Thread(target=read_stream, args=(process.stderr, True))
    stdout_thread.start()
    stderr_thread.start()

    await loop.run_in_executor(None, process.wait)
    stdout_thread.join()
    stderr_thread.join()


def start_ros_spin_thread(node):
    def spin():
        rclpy.spin(node)
    threading.Thread(target=spin, daemon=True).start()


def main():
    global ros_node

    rclpy.init()
    ros_node = RosBridgeNode()
    start_ros_spin_thread(ros_node)

    start_websocket_server(host="0.0.0.0", port=6789)

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
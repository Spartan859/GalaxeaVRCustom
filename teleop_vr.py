import json
import struct
from queue import Queue
import threading
import socket
from typing import Any

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.teleoperators.utils import TeleopEvents
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError


class RemoteVRTeleop(Teleoperator):
    """
    VR Teleoperator that receives control data from Unity VR Controller (Quest 3).
    Uses the same protocol as ControllerScript.cs: 4-byte length prefix (big-endian) + UTF-8 JSON.
    
    Handles two types of commands:
    - send_action: Contains bimanual EE poses, grippers, chassis, torso speeds, and optional reset flag
    - ping: Heartbeat check
    
    Reset is now integrated into send_action via the "reset" field.
    """

    name = "vr_bimanual"

    def __init__(self, config=None, host="0.0.0.0", port=50051):
        super().__init__(config)

        self.config = config
        self.robot_type = getattr(config, "type", "bimanual")

        # networking
        self.host = host
        self.port = port
        self.sock = None
        self.conn = None
        self.listener_thread = None
        self._stop_flag = False
        self._connected = False

        # latest action data from VR
        self.action_lock = threading.Lock()
        self.latest_action = None
        self.last_command_time = 0

        # teleop events
        self.is_intervention_flag = False

    # ---------------------------------------------------------
    # REQUIRED BY LEROBOT
    # ---------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def feedback_features(self):
        return {}

    @property
    def is_calibrated(self) -> bool:
        return True
    
    def calibrate(self) -> None:
        pass

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    # ---------------------------------------------------------
    # Networking
    # ---------------------------------------------------------
    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError("RemoteVRTeleop already connected.")

        print(f"[VR Server] Starting TCP server at {self.host}:{self.port} ...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)

        print("[VR Server] Waiting for VR client to connect...")
        self.conn, addr = self.sock.accept()
        print(f"[VR Server] VR client connected from {addr}")

        self._connected = True
        self._stop_flag = False

        # start receiver thread
        self.listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listener_thread.start()

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError("RemoteVRTeleop is not connected")

        print("[VR Server] Disconnecting VR client...")
        self._connected = False
        self._stop_flag = True

        try:
            if self.conn:
                self.conn.close()
        except:
            pass

        try:
            if self.sock:
                self.sock.close()
        except:
            pass

        print("[VR Server] Disconnect finished.")

    def _listen_loop(self):
        """Main receive loop using length-prefix protocol (4-byte big-endian + JSON)"""
        try:
            while not self._stop_flag:
                # Read 4-byte length prefix
                length_data = self._recv_exactly(4)
                if not length_data:
                    break
                
                message_length = struct.unpack('>I', length_data)[0]  # big-endian uint32
                
                # Read message body
                message_data = self._recv_exactly(message_length)
                if not message_data:
                    break
                
                message_str = message_data.decode('utf-8')
                message = json.loads(message_str)
                
                # Process message
                self._handle_message(message)
                
        except Exception as e:
            print(f"[VR Server] Listen loop error: {e}")
            self._connected = False

    def _recv_exactly(self, n):
        """Receive exactly n bytes from socket"""
        data = b''
        while len(data) < n:
            chunk = self.conn.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _send_response(self, response_dict):
        """Send response using length-prefix protocol"""
        try:
            response_str = json.dumps(response_dict)
            response_bytes = response_str.encode('utf-8')
            length_prefix = struct.pack('>I', len(response_bytes))  # big-endian uint32
            self.conn.sendall(length_prefix + response_bytes)
        except Exception as e:
            print(f"[VR Server] Error sending response: {e}")

    # ---------------------------------------------------------
    # MESSAGE HANDLING
    # ---------------------------------------------------------
    def _handle_message(self, message):
        """Handle incoming messages from VR client"""
        cmd = message.get("cmd")
        
        if cmd == "send_action":
            self._handle_action(message.get("action", {}))
            # Send action response
            self._send_response({"cmd": "action", "ok": True})
            
        elif cmd == "ping":
            # Send pong response
            self._send_response({"cmd": "pong", "ok": True})

    def _handle_action(self, action_data):
        """Process action data from VR"""
        with self.action_lock:
            self.latest_action = action_data
            self.is_intervention_flag = True  # Mark intervention when receiving data

    # ---------------------------------------------------------
    # CONFIGURE
    # ---------------------------------------------------------
    def configure(self):
        pass

    # ---------------------------------------------------------
    # ACTION FEATURES
    # ---------------------------------------------------------
    @property
    def action_features(self) -> dict[str, Any]:
        """Define action space matching Unity's send_action structure"""
        return {
            "left_ee_pose": (6,),
            "right_ee_pose": (6,),
            "left_gripper": float,
            "right_gripper": float,
            "chassis_speed": (3,),
            "torso_speed": (4,),
            "reset": bool,
        }

    def get_action(self) -> dict[str, Any]:
        """Get latest action from VR"""
        if not self.is_connected:
            raise DeviceNotConnectedError("RemoteVRTeleop is not connected.")

        with self.action_lock:
            if self.latest_action is None:
                # Return zero action if no data received yet
                return self._get_zero_action()
            
            action_data = self.latest_action.copy()
            
            # Clear delta values after reading to prevent applying the same delta multiple times
            # Keep speed values (chassis_speed, torso_speed) as they represent continuous velocities
            self.latest_action = {
                "left_ee_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # delta - clear
                "right_ee_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # delta - clear
                "left_gripper": 0.0,  # delta - clear
                "right_gripper": 0.0,  # delta - clear
                "chassis_speed": action_data.get("chassis_speed", [0.0, 0.0, 0.0]),  # velocity - keep
                "torso_speed": action_data.get("torso_speed", [0.0, 0.0, 0.0, 0.0]),  # velocity - keep
                "reset": False,  # flag - clear
            }

        # Return action in the same format as Unity sends it
        # Expected format from Unity:
        # {
        #   "left_ee_pose": [dx, dy, dz, droll, dpitch, dyaw],
        #   "right_ee_pose": [dx, dy, dz, droll, dpitch, dyaw],
        #   "left_gripper": delta_value,
        #   "right_gripper": delta_value,
        #   "chassis_speed": [vx, vy, w],
        #   "torso_speed": [vx, vz, w_pitch, w_yaw],
        #   "reset": true/false
        # }
        
        action = {
            "left_ee_pose": action_data.get("left_ee_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "right_ee_pose": action_data.get("right_ee_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "left_gripper": action_data.get("left_gripper", 0.0),
            "right_gripper": action_data.get("right_gripper", 0.0),
            "chassis_speed": action_data.get("chassis_speed", [0.0, 0.0, 0.0]),
            "torso_speed": action_data.get("torso_speed", [0.0, 0.0, 0.0, 0.0]),
            "reset": action_data.get("reset", False),
        }

        return action

    def _get_zero_action(self):
        """Return zero action (no movement)"""
        return {
            "left_ee_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "right_ee_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "left_gripper": 0.0,
            "right_gripper": 0.0,
            "chassis_speed": [0.0, 0.0, 0.0],
            "torso_speed": [0.0, 0.0, 0.0, 0.0],
            "reset": False,
        }

    # ---------------------------------------------------------
    # TELEOP EVENTS
    # ---------------------------------------------------------
    def get_teleop_events(self) -> dict[str, Any]:
        """Get teleoperation events"""
        if not self.is_connected:
            return {
                TeleopEvents.IS_INTERVENTION: False,
                TeleopEvents.TERMINATE_EPISODE: False,
                TeleopEvents.SUCCESS: False,
                TeleopEvents.RERECORD_EPISODE: False,
            }

        with self.action_lock:
            # Intervention is active if we're receiving data
            is_intervention = self.is_intervention_flag
            
            # Reset intervention flag (will be set again if new data arrives)
            self.is_intervention_flag = False

        return {
            TeleopEvents.IS_INTERVENTION: is_intervention,
            TeleopEvents.TERMINATE_EPISODE: False,  # Controlled externally, not by VR
            TeleopEvents.SUCCESS: False,  # VR doesn't have explicit success signal
            TeleopEvents.RERECORD_EPISODE: False,  # Controlled externally, not by VR
        }

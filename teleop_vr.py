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
    - send_action: Contains bimanual EE poses, grippers, chassis, torso speeds, intervention flag, and teleop event
    - ping: Heartbeat check
    """

    name = "remote_vr"

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
        self.delta_left_ee = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.delta_right_ee = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.delta_left_gripper = 0.0
        self.delta_right_gripper = 0.0
        self.latest_chassis_speed = [0.0, 0.0, 0.0]
        self.latest_torso_speed = [0.0, 0.0, 0.0, 0.0]
        self.last_command_time = 0

        # teleop events
        self.is_intervention_flag = False
        self.event_success = False
        self.event_rerecord = False
        self.event_terminate = False

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
        self._stop_flag = False

        # start accept/receiver thread
        self.listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listener_thread.start()

        # Non-blocking: accept happens in listener thread; get_action will return zero until connected

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError("RemoteVRTeleop is not connected")

        print("[VR Server] Disconnecting VR client...")
        self._connected = False
        self._stop_flag = True

        try:
            if self.conn:
                self.conn.close()
        except Exception:
            pass

        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass

        print("[VR Server] Disconnect finished.")

    def _listen_loop(self):
        """Main receive loop using length-prefix protocol (4-byte big-endian + JSON)"""
        try:
            while not self._stop_flag:
                if self.conn is None:
                    try:
                        print(f"[VR Server] Waiting for VR client to connect at {self.host}:{self.port} ...")
                        self.conn, addr = self.sock.accept()
                        self._connected = True
                        print(f"[VR Server] VR client connected from {addr}")
                    except Exception as e:
                        if not self._stop_flag:
                            print(f"[VR Server] Accept error: {e}")
                        continue

                # Read 4-byte length prefix
                length_data = self._recv_exactly(4)
                if not length_data:
                    self._reset_connection()
                    continue
                
                message_length = struct.unpack('>I', length_data)[0]  # big-endian uint32
                
                # Read message body
                message_data = self._recv_exactly(message_length)
                if not message_data:
                    self._reset_connection()
                    continue
                
                message_str = message_data.decode('utf-8')
                message = json.loads(message_str)

                # Process message
                self._handle_message(message)
                
        except Exception as e:
            print(f"[VR Server] Listen loop error: {e}")
            self._reset_connection()

    def _recv_exactly(self, n):
        """Receive exactly n bytes from socket"""
        data = b''
        while len(data) < n and self.conn is not None:
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

    def _reset_connection(self):
        """Cleanup current client connection and allow new connections."""
        try:
            if self.conn:
                self.conn.close()
        except Exception:
            pass
        self.conn = None
        self._connected = False
        with self.action_lock:
            self._clear_accumulators()

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
            left = action_data.get("left_ee_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            right = action_data.get("right_ee_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            # Accumulate delta-style commands so high-frequency producer won't lose increments
            for i in range(6):
                self.delta_left_ee[i] += float(left[i]) if i < len(left) else 0.0
                self.delta_right_ee[i] += float(right[i]) if i < len(right) else 0.0

            self.delta_left_gripper += float(action_data.get("left_gripper", 0.0))
            self.delta_right_gripper += float(action_data.get("right_gripper", 0.0))

            # Speeds overwrite (latest wins)
            chassis = action_data.get("chassis_speed", self.latest_chassis_speed)
            torso = action_data.get("torso_speed", self.latest_torso_speed)
            self.latest_chassis_speed = [float(v) for v in chassis[:3]] + [0.0] * (3 - len(chassis))
            self.latest_torso_speed = [float(v) for v in torso[:4]] + [0.0] * (4 - len(torso))

            # Intervention and teleop events
            self.is_intervention_flag = bool(action_data.get("isIntervention", False))
            event_str = str(action_data.get("event", "none")).lower()
            if event_str == "success":
                self.event_success = True
            elif event_str == "rerecord":
                self.event_rerecord = True
            elif event_str == "terminate":
                self.event_terminate = True

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
        """Define action spec in flat 14D format for logging/datasets."""
        return {
            "dtype": "float32",
            # Layout (flat):
            # [ left_ee_pose(6), right_ee_pose(6), left_gripper(1), right_gripper(1),
            #   chassis_speed(3), torso_speed(4) ]
            "shape": (21,),
            "names": {
                "left_ee_pose": 0,   # 0..5
                "right_ee_pose": 6,  # 6..11
                "left_gripper": 12,  # 12
                "right_gripper": 13, # 13
                "chassis_speed": 14, # 14..16
                "torso_speed": 17,   # 17..20
            },
        }

    def get_action(self) -> dict[str, Any]:
        """Get latest action from VR"""
        if not self.is_connected:
            # If not connected yet, return zero action instead of raising
            return self._get_zero_action()

        with self.action_lock:
            action = {
                "left_ee_pose": list(self.delta_left_ee),
                "right_ee_pose": list(self.delta_right_ee),
                "left_gripper": self.delta_left_gripper,
                "right_gripper": self.delta_right_gripper,
                "chassis_speed": list(self.latest_chassis_speed),
                "torso_speed": list(self.latest_torso_speed),
            }

            # Clear accumulated deltas after consumption; keep speeds unless explicitly cleared
            self._clear_accumulators(clear_speeds=False)

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
        }

    def _clear_accumulators(self, clear_speeds: bool = True):
        """Reset accumulated deltas; optionally reset speeds."""
        self.delta_left_ee = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.delta_right_ee = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.delta_left_gripper = 0.0
        self.delta_right_gripper = 0.0
        if clear_speeds:
            self.latest_chassis_speed = [0.0, 0.0, 0.0]
            self.latest_torso_speed = [0.0, 0.0, 0.0, 0.0]

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
            is_intervention = self.is_intervention_flag
            terminate = self.event_terminate or self.event_rerecord
            success = self.event_success
            rerecord = self.event_rerecord

            # Reset flags after consumption
            self.is_intervention_flag = False
            self.event_terminate = False
            self.event_success = False
            self.event_rerecord = False

        return {
            TeleopEvents.IS_INTERVENTION: is_intervention,
            TeleopEvents.TERMINATE_EPISODE: terminate,
            TeleopEvents.SUCCESS: success,
            TeleopEvents.RERECORD_EPISODE: rerecord,
        }

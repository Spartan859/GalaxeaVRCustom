import json
from queue import Queue
import threading
import socket
from typing import Any

from lerobot.teleoperators.keyboard.configuration_keyboard import KeyboardEndEffectorTeleopConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.teleoperators.utils import TeleopEvents
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError


class RemoteKeyboardTeleop(Teleoperator):
    """
    Remote version of KeyboardTeleop.
    Works the same way but receives events from TCP instead of local pynput.
    """

    config_class = KeyboardEndEffectorTeleopConfig
    name = "keyboard_ee"

    def __init__(self, config=None, host="0.0.0.0", port=5005):
        super().__init__(config)

        self.config = config
        self.robot_type = getattr(config, "type", None)

        # networking
        self.host = host
        self.port = port
        self.sock = None
        self.conn = None
        self.listener_thread = None
        self._stop_flag = False
        self._connected = False

        # keyboard behavior (same as KeyboardTeleop)
        self.event_queue = Queue()
        self.misc_keys_queue = Queue()
        self.current_pressed = {}
        self.logs = {}

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
            raise DeviceAlreadyConnectedError("RemoteKeyboardTeleop already connected.")

        print(f"[Server] Starting TCP server at {self.host}:{self.port} ...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)

        print("[Server] Waiting for client to connect...")
        self.conn, addr = self.sock.accept()
        print(f"[Server] Keyboard client connected from {addr}")

        self._connected = True
        self._stop_flag = False

        # start receiver thread
        self.listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listener_thread.start()

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError("RemoteKeyboardTeleop is not connected")

        print("[Server] Disconnecting remote keyboard...")
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

        print("[Server] Disconnect finished.")

    def _listen_loop(self):
        buffer = ""
        while not self._stop_flag:
            try:
                data = self.conn.recv(1024).decode()
                if not data:
                    continue

                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if line.strip():
                        event = json.loads(line)
                        self._on_event(event)

            except Exception as e:
                print("[Server] Error:", e)
                self._connected = False
                break

    # ---------------------------------------------------------
    # EVENT HANDLING (same semantics as KeyboardTeleop)
    # ---------------------------------------------------------
    def _on_event(self, event):
        key = event["key"]
        state = event["event"] == "press"  # "press" → True, "release" → False
        self.event_queue.put((key, state))

    def _drain_pressed_keys(self):
        while not self.event_queue.empty():
            key_char, is_pressed = self.event_queue.get_nowait()
            self.current_pressed[key_char] = is_pressed

    def configure(self):
        pass

    # ---------------------------------------------------------
    # END-EFFECTOR LOGIC (same as KeyboardEndEffectorTeleop)
    # ---------------------------------------------------------
    @property
    def action_features(self) -> dict:
        return {
            "dtype": "float32",
            "shape": (7,),
            "names": {"d_x": 0, "d_y": 1, "d_z": 2, "d_roll": 3, 
                      "d_pitch": 4, "d_yaw": 5, "d_gripper": 6},
        }

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError("RemoteKeyboardTeleop is not connected.")

        self._drain_pressed_keys()

        d_x, d_y, d_z = 0.0, 0.0, 0.0
        d_roll, d_pitch, d_yaw = 0.0, 0.0, 0.0
        d_gripper = 0.0

        # exactly match KeyboardEndEffectorTeleop key behavior
        for key, val in self.current_pressed.items():
            if key == "up":
                d_y = int(val)
            elif key == "down":
                d_y = -int(val)
            elif key == "left":
                d_x = -int(val)
            elif key == "right":
                d_x = int(val)
            elif key == "shift":
                d_z = -int(val)
            elif key == "shift_r":
                d_z = int(val)
            elif key == "i":
                d_y = int(val)
            elif key == "k":
                d_y = -int(val)
            elif key == "j":
                d_x = -int(val)
            elif key == "l":
                d_x = int(val)
            elif key == "u":
                d_z = -int(val)
            elif key == "o":
                d_z = int(val)
            elif key == "<":
                d_gripper = -int(val)
            elif key == ">":
                d_gripper = int(val)
            elif val:
                # other keys: push to misc queue
                self.misc_keys_queue.put(key)

        self.current_pressed.clear()

        action = {
            "delta_x": delta_x,
            "delta_y": delta_y,
            "delta_z": delta_z,
        }

        if getattr(self.config, "use_gripper", False):
            action["gripper"] = gripper_action

        return action

    # ---------------------------------------------------------
    # Teleop events (same as KeyboardEndEffectorTeleop)
    # ---------------------------------------------------------
    def get_teleop_events(self) -> dict[str, Any]:
        if not self.is_connected:
            return {
                TeleopEvents.IS_INTERVENTION: False,
                TeleopEvents.TERMINATE_EPISODE: False,
                TeleopEvents.SUCCESS: False,
                TeleopEvents.RERECORD_EPISODE: False,
            }

        movement_keys = [
            "up", "down", "left", "right",
            "shift", "shift_r", "ctrl_r", "ctrl_l"
        ]
        is_intervention = any(self.current_pressed.get(k, False) for k in movement_keys)

        terminate_episode = False
        success = False
        rerecord_episode = False

        while not self.misc_keys_queue.empty():
            key = self.misc_keys_queue.get_nowait()
            if key == "s":
                success = True
            elif key == "r":
                terminate_episode = True
                rerecord_episode = True
            elif key == "q":
                terminate_episode = True

        return {
            TeleopEvents.IS_INTERVENTION: is_intervention,
            TeleopEvents.TERMINATE_EPISODE: terminate_episode,
            TeleopEvents.SUCCESS: success,
            TeleopEvents.RERECORD_EPISODE: rerecord_episode,
        }
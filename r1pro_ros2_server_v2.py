#!/usr/bin/env python3
"""
Jetson 上的 R1 PRO ROS2 + TCP server

- ROS2 部分：
    R1ProConfig / _R1Node / R1ProRobot
    EEF pose: 7D [x, y, z, qx, qy, qz, qw]
    gripper: 单独标量 (mm)，topic 来自 /hdas/feedback_gripper_*

- TCP 部分：
    协议为简单的 length-prefixed JSON：
        [4 bytes length][JSON bytes]

    支持的命令：
        {"cmd": "ping"}
        {"cmd": "wait_ready", "timeout": float}
        {"cmd": "get_obs", "include_images": bool}
        {"cmd": "send_action", "action": {...}}
        {"cmd": "set_brake", "enabled": bool}

    其中 obs / action 结构：

        obs:
            "left_ee_pose":  [x,y,z,qx,qy,qz,qw]       (np.float32)
            "right_ee_pose": [x,y,z,qx,qy,qz,qw]
            "left_gripper":  [g]                       (np.float32, shape (1,))
            "right_gripper": [g]
            (如果 enable_cameras=True 时还有 *_rgb)

        action:
            "left_ee_pose":  [x,y,z,qx,qy,qz,qw]
            "right_ee_pose": [x,y,z,qx,qy,qz,qw]
            "left_gripper":  float 或 [float]
            "right_gripper": float 或 [float]
"""

from __future__ import annotations

import base64
import json
import os
import socketserver
import struct
import threading
import traceback
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import TransformException


# =========================
# ROS2 Adapter
# =========================

@dataclass
class R1ProConfig:
    ros_domain_id: Optional[int] = 72
    ros_localhost_only: bool = True
    enable_cameras: bool = False        # ⭐ 建议先关掉相机，调通控制之后再开
    use_sdk_topics: bool = False
    action_is_delta: bool = True
    quaternion_normalize: bool = True
    warmup_timeout_sec: float = 8.0
    # control mode
    use_joints: bool = False            # ⭐ 我们用 EEF pose 模式

    # feedback topics
    left_js_topic: str = "/hdas/feedback_arm_left"
    right_js_topic: str = "/hdas/feedback_arm_right"
    left_gripper_topic: str = "/hdas/feedback_gripper_left"
    right_gripper_topic: str = "/hdas/feedback_gripper_right"

    camera_topics: Dict[str, str] = field(default_factory=lambda: {
        "head_rgb": "/hdas/camera_head/left_raw/image_raw_color/compressed",
        "left_wrist_rgb": "/hdas/camera_wrist_left/color/image_raw/compressed",
        "right_wrist_rgb": "/hdas/camera_wrist_right/color/image_raw/compressed",
    })

    # control topics base
    left_pose_topic: str = "/motion_target/target_pose_arm_left"
    right_pose_topic: str = "/motion_target/target_pose_arm_right"
    left_gripper_cmd_topic: str = "/motion_target/target_position_gripper_left"
    right_gripper_cmd_topic: str = "/motion_target/target_position_gripper_right"
    left_arm_joint_cmd_topic: str = "/motion_target/target_joint_state_arm_left"
    right_arm_joint_cmd_topic: str = "/motion_target/target_joint_state_arm_right"
    brake_topic: str = "/motion_target/brake_mode"

    pose_frame_id: str = "torso_link4"
    left_eef_frame: str = "left_gripper_link"
    right_eef_frame: str = "right_gripper_link"

    gripper_min: float = 0.0
    gripper_max: float = 100.0

    # observation keys
    obs_left_ee: str = "left_ee_pose"
    obs_right_ee: str = "right_ee_pose"
    obs_left_gripper: str = "left_gripper"
    obs_right_gripper: str = "right_gripper"
    obs_head_img: str = "head_rgb"
    obs_left_img: str = "left_wrist_rgb"
    obs_right_img: str = "right_wrist_rgb"

    # action keys
    act_left_ee: str = "left_ee_pose"
    act_right_ee: str = "right_ee_pose"
    act_left_gripper: str = "left_gripper"
    act_right_gripper: str = "right_gripper"
    act_left_joints: str = "left_arm_joints"
    act_right_joints: str = "right_arm_joints"


class _R1Node(Node):
    def __init__(self, cfg: R1ProConfig):
        super().__init__("r1pro_dp_minimal")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._cfg = cfg
        self._lock = threading.Lock()
        self._left_js = None
        self._right_js = None
        self._left_grip = None
        self._right_grip = None
        self._left_ee_pose: Optional[np.ndarray] = None
        self._right_ee_pose: Optional[np.ndarray] = None
        self._images: Dict[str, Optional[np.ndarray]] = {}

        # subscriptions
        self.create_subscription(JointState, cfg.left_js_topic, self._on_left_js, qos)
        self.create_subscription(JointState, cfg.right_js_topic, self._on_right_js, qos)
        self.create_subscription(JointState, cfg.left_gripper_topic, self._on_left_gripper, qos)
        self.create_subscription(JointState, cfg.right_gripper_topic, self._on_right_gripper, qos)
        # subscribe to EE poses from motion_control
        self.create_subscription(
            PoseStamped,
            "/motion_control/pose_ee_arm_left",
            self._on_left_ee_pose,
            qos,
        )
        self.create_subscription(
            PoseStamped,
            "/motion_control/pose_ee_arm_right",
            self._on_right_ee_pose,
            qos,
        )
        if cfg.enable_cameras:
            for k, topic in cfg.camera_topics.items():
                self._images[k] = None
                self.create_subscription(
                    CompressedImage,
                    topic,
                    lambda msg, key=k: self._on_image(key, msg),
                    qos,
                )

        suffix = "_sdk" if cfg.use_sdk_topics else ""
        self._pub_left_pose = self.create_publisher(PoseStamped, cfg.left_pose_topic + suffix, qos)
        self._pub_right_pose = self.create_publisher(PoseStamped, cfg.right_pose_topic + suffix, qos)
        self._pub_left_gripper = self.create_publisher(JointState, cfg.left_gripper_cmd_topic + suffix, qos)
        self._pub_right_gripper = self.create_publisher(JointState, cfg.right_gripper_cmd_topic + suffix, qos)
        self._pub_left_arm_joints = self.create_publisher(JointState, cfg.left_arm_joint_cmd_topic + suffix, qos)
        self._pub_right_arm_joints = self.create_publisher(JointState, cfg.right_arm_joint_cmd_topic + suffix, qos)
        self._pub_brake = self.create_publisher(Bool, cfg.brake_topic, qos)

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)

    def _on_left_js(self, msg: JointState):
        with self._lock:
            self._left_js = np.array(msg.position[:7], dtype=np.float32) if msg.position else None

    def _on_right_js(self, msg: JointState):
        with self._lock:
            self._right_js = np.array(msg.position[:7], dtype=np.float32) if msg.position else None

    def _on_left_gripper(self, msg: JointState):
        with self._lock:
            self._left_grip = float(msg.position[0]) if msg.position else None

    def _on_right_gripper(self, msg: JointState):
        with self._lock:
            self._right_grip = float(msg.position[0]) if msg.position else None

    def _on_left_ee_pose(self, msg: PoseStamped):
        with self._lock:
            p = msg.pose.position
            q = msg.pose.orientation
            self._left_ee_pose = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=np.float32)

    def _on_right_ee_pose(self, msg: PoseStamped):
        with self._lock:
            p = msg.pose.position
            q = msg.pose.orientation
            self._right_ee_pose = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=np.float32)

    def _on_image(self, key: str, msg: CompressedImage):
        arr = np.frombuffer(msg.data, np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        with self._lock:
            self._images[key] = rgb

    def lookup_pose(self, frame: str) -> Optional[np.ndarray]:
        # Deprecated for EE pose: now using direct topics from motion_control
        try:
            tf = self._tf_buffer.lookup_transform(
                self._cfg.pose_frame_id,
                frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return np.array([t.x, t.y, t.z, r.x, r.y, r.z, r.w], dtype=np.float32)

    def get_state(self):
        with self._lock:
            return (
                self._left_js,
                self._right_js,
                self._left_grip,
                self._right_grip,
                dict(self._images),
                self._left_ee_pose,
                self._right_ee_pose,
            )

    def publish(
        self,
        left_pose7: Optional[np.ndarray],
        right_pose7: Optional[np.ndarray],
        left_mm: Optional[float],
        right_mm: Optional[float],
    ):
        stamp = self.get_clock().now().to_msg()
        if left_pose7 is not None:
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self._cfg.pose_frame_id
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, left_pose7[0:3])
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(
                float, left_pose7[3:7]
            )
            self._pub_left_pose.publish(msg)
        if right_pose7 is not None:
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self._cfg.pose_frame_id
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, right_pose7[0:3])
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(
                float, right_pose7[3:7]
            )
            self._pub_right_pose.publish(msg)
        if left_mm is not None:
            js = JointState()
            js.name = ["left_gripper"]
            js.position = [float(np.clip(left_mm, self._cfg.gripper_min, self._cfg.gripper_max))]
            self._pub_left_gripper.publish(js)
        if right_mm is not None:
            js = JointState()
            js.name = ["right_gripper"]
            js.position = [float(np.clip(right_mm, self._cfg.gripper_min, self._cfg.gripper_max))]
            self._pub_right_gripper.publish(js)

    def publish_joints(
        self,
        left7: Optional[np.ndarray],
        right7: Optional[np.ndarray],
        left_mm: Optional[float],
        right_mm: Optional[float],
    ):
        stamp = self.get_clock().now().to_msg()
        if left7 is not None:
            js = JointState()
            js.header.stamp = stamp
            js.position = [float(x) for x in left7.reshape(-1)[:7]]
            self._pub_left_arm_joints.publish(js)
        if right7 is not None:
            js = JointState()
            js.header.stamp = stamp
            js.position = [float(x) for x in right7.reshape(-1)[:7]]
            self._pub_right_arm_joints.publish(js)
        if left_mm is not None:
            js = JointState()
            js.position = [float(np.clip(left_mm, self._cfg.gripper_min, self._cfg.gripper_max))]
            self._pub_left_gripper.publish(js)
        if right_mm is not None:
            js = JointState()
            js.position = [float(np.clip(right_mm, self._cfg.gripper_min, self._cfg.gripper_max))]
            self._pub_right_gripper.publish(js)

    def set_brake(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
        self._pub_brake.publish(msg)


class R1ProRobot:
    """
    将 104 发送来的 action 视为 *delta*，在这里累积成绝对位姿，然后通过 ROS2 publish 绝对位姿。

    约定：
      - action["left_ee_pose"], action["right_ee_pose"]:
            长度 6 的数组：
                [dx, dy, dz, droll, dpitch, dyaw]   （全部是增量，弧度）

        其中：
            - 前 3 维是 xyz 的增量（单位同 TF 中的米）
            - 后 3 维是 roll/pitch/yaw 的增量（弧度）

      - gripper:
            action["left_gripper"], action["right_gripper"]:
                视为绝对开的大小（0~100），单位 mm。
                如果该键不存在，则保持之前 target 的值。

      - 内部维护的目标：
            self._target_left_pose  = [x,y,z,qx,qy,qz,qw]  (绝对)
            self._target_left_rpy   = [roll,pitch,yaw]     (绝对)
            右臂同理
    """

    def __init__(self, cfg: R1ProConfig):
        self.cfg = cfg
        self._node: Optional[_R1Node] = None
        self._is_connected = False

        # 期望的“目标绝对位姿”
        self._target_left_pose: Optional[np.ndarray] = None   # shape (7,)
        self._target_right_pose: Optional[np.ndarray] = None  # shape (7,)
        self._target_left_grip: Optional[float] = None
        self._target_right_grip: Optional[float] = None

        # 目标姿态（欧拉角）
        self._target_left_rpy: Optional[np.ndarray] = None    # shape (3,)
        self._target_right_rpy: Optional[np.ndarray] = None   # shape (3,)

    # ---------- 基本连接 ----------

    @property
    def is_connected(self):
        return self._is_connected

    def connect(self):
        if self.cfg.ros_localhost_only:
            os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
        if self.cfg.ros_domain_id is not None:
            os.environ["ROS_DOMAIN_ID"] = str(self.cfg.ros_domain_id)
        rclpy.init(args=None)
        self._node = _R1Node(self.cfg)
        self._is_connected = True

    def disconnect(self):
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()
            self._is_connected = False

    def spin_once(self, timeout_sec: float = 0.0):
        if self._node:
            rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    # ---------- 工具：四元数 <-> 欧拉角 ----------

    @staticmethod
    def _quat_to_euler(qx, qy, qz, qw):
        """
        四元数 -> (roll, pitch, yaw)，单位弧度
        使用 ROS 常用 XYZ (roll-pitch-yaw) 约定。
        """
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.pi / 2 * np.sign(sinp)
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw], dtype=np.float32)

    @staticmethod
    def _euler_to_quat(roll, pitch, yaw):
        """
        (roll, pitch, yaw) -> 单位四元数 (qx, qy, qz, qw)
        """
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return np.array([qx, qy, qz, qw], dtype=np.float32)

    # ---------- 观测 ----------

    def get_observation(self) -> Optional[Dict[str, Any]]:
        """
        返回“真实观测”（来自 TF + feedback topic），不动 target。
        """
        if not self._node:
            return None
        l_js, r_js, l_g, r_g, images, l_pose, r_pose = self._node.get_state()

        if self.cfg.use_joints:
            if None in (l_js, r_js, l_g, r_g):
                return None
        else:
            if any(x is None for x in (l_js, r_js, l_g, r_g, l_pose, r_pose)):
                return None

        if self.cfg.enable_cameras:
            if (
                images.get("head_rgb") is None
                or images.get("left_wrist_rgb") is None
                or images.get("right_wrist_rgb") is None
            ):
                return None

        if self.cfg.use_joints:
            obs = {
                self.cfg.act_left_joints: l_js,
                self.cfg.act_right_joints: r_js,
                self.cfg.obs_left_gripper: np.array([l_g], dtype=np.float32),
                self.cfg.obs_right_gripper: np.array([r_g], dtype=np.float32),
            }
        else:
            obs = {
                self.cfg.obs_left_ee: l_pose,  # [x,y,z,qx,qy,qz,qw]
                self.cfg.obs_right_ee: r_pose,
                self.cfg.obs_left_gripper: np.array([l_g], dtype=np.float32),
                self.cfg.obs_right_gripper: np.array([r_g], dtype=np.float32),
            }
        if self.cfg.enable_cameras:
            obs[self.cfg.obs_head_img] = images["head_rgb"]
            obs[self.cfg.obs_left_img] = images["left_wrist_rgb"]
            obs[self.cfg.obs_right_img] = images["right_wrist_rgb"]

        return obs

    # ---------- 初始化 target ----------

    def _ensure_target_initialized(self):
        """
        确保 target_* 已经有初始值：
          - 第一次收到 delta action 或 wait_ready 成功后，用当前观测来初始化
        """
        if (
            self._target_left_pose is not None
            and self._target_right_pose is not None
            and self._target_left_grip is not None
            and self._target_right_grip is not None
            and self._target_left_rpy is not None
            and self._target_right_rpy is not None
        ):
            return

        obs = self.get_observation()
        if obs is None:
            raise RuntimeError("Cannot initialize target pose: observation not ready.")

        # 绝对位姿 + gripper
        self._target_left_pose = np.array(obs[self.cfg.obs_left_ee], dtype=np.float32).reshape(7)
        self._target_right_pose = np.array(obs[self.cfg.obs_right_ee], dtype=np.float32).reshape(7)
        self._target_left_grip = float(obs[self.cfg.obs_left_gripper][0])
        self._target_right_grip = float(obs[self.cfg.obs_right_gripper][0])

        # 从当前四元数计算绝对 rpy
        lx, ly, lz, lqx, lqy, lqz, lqw = self._target_left_pose
        rx, ry, rz, rqx, rqy, rqz, rqw = self._target_right_pose

        self._target_left_rpy = self._quat_to_euler(lqx, lqy, lqz, lqw)
        self._target_right_rpy = self._quat_to_euler(rqx, rqy, rqz, rqw)

        # 再把四元数规范化一下
        for pose in (self._target_left_pose, self._target_right_pose):
            q = pose[3:7]
            nq = np.linalg.norm(q)
            if nq > 1e-6:
                pose[3:7] = q / nq
            else:
                pose[3:7] = np.array([0, 0, 0, 1], dtype=np.float32)

    # ---------- 接收 delta action 并更新 target ----------

    def send_action(self, action: Dict[str, Any]):
        """
        从 104 收到的 action 视为 delta（增量），在 target 上更新后，
        通过 ROS2 publish 绝对位姿。

        action 字典可以包含：
            - "left_ee_pose":  [dx,dy,dz,droll,dpitch,dyaw]
            - "right_ee_pose": [dx,dy,dz,droll,dpitch,dyaw]
            - "left_gripper":  float 或 [float] （绝对张开度）
            - "right_gripper": float 或 [float]
        """
        self._ensure_target_initialized()
        # import pdb;pdb.set_trace()
        # ---- EEF delta 处理 ----
        l_delta = action.get(self.cfg.act_left_ee)
        r_delta = action.get(self.cfg.act_right_ee)

        if l_delta is not None:
            l_delta = np.array(l_delta, dtype=np.float32).reshape(-1)
            if l_delta.shape[0] < 6:
                raise ValueError(f"left_ee_pose delta must be length 6, got {l_delta.shape}")
            dx, dy, dz, droll, dpitch, dyaw = l_delta

            # 位置增量
            self._target_left_pose[0:3] += np.array([dx, dy, dz], dtype=np.float32)

            # 姿态增量：累加到绝对 rpy
            self._target_left_rpy += np.array([droll, dpitch, dyaw], dtype=np.float32)

            # rpy -> quat
            q_left = self._euler_to_quat(*self._target_left_rpy)
            self._target_left_pose[3:7] = q_left

        if r_delta is not None:
            r_delta = np.array(r_delta, dtype=np.float32).reshape(-1)
            if r_delta.shape[0] < 6:
                raise ValueError(f"right_ee_pose delta must be length 6, got {r_delta.shape}")
            dx, dy, dz, droll, dpitch, dyaw = r_delta

            self._target_right_pose[0:3] += np.array([dx, dy, dz], dtype=np.float32)
            self._target_right_rpy += np.array([droll, dpitch, dyaw], dtype=np.float32)

            q_right = self._euler_to_quat(*self._target_right_rpy)
            self._target_right_pose[3:7] = q_right

        # ---- gripper 处理（绝对量）----
        lg = action.get(self.cfg.act_left_gripper)
        rg = action.get(self.cfg.act_right_gripper)

        if lg is not None:
            self._target_left_grip = (
                float(lg[0]) if isinstance(lg, (list, np.ndarray)) else float(lg)
            )
        if rg is not None:
            self._target_right_grip = (
                float(rg[0]) if isinstance(rg, (list, np.ndarray)) else float(rg)
            )

        # 再次归一化四元数，避免数值漂移
        for pose in (self._target_left_pose, self._target_right_pose):
            q = pose[3:7]
            nq = np.linalg.norm(q)
            if nq > 1e-6:
                pose[3:7] = q / nq
            else:
                pose[3:7] = np.array([0, 0, 0, 1], dtype=np.float32)

        # 发布绝对位姿 + gripper
        self._node.publish(
            self._target_left_pose,
            self._target_right_pose,
            self._target_left_grip,
            self._target_right_grip,
        )

    # ---------- 其他 ----------

    def wait_ready(self) -> bool:
        import time

        deadline = time.time() + self.cfg.warmup_timeout_sec
        while time.time() < deadline and self.is_connected:
            self.spin_once(0.05)
            obs = self.get_observation()
            if obs is not None:
                # 初始化一次 target
                try:
                    self._ensure_target_initialized()
                except Exception:
                    pass
                return True
        return False

    def enable_brake(self, enabled: bool = True):
        if self._node:
            self._node.set_brake(enabled)



# =========================
# TCP Server Layer
# =========================

def _encode_obs(obs: Dict[str, Any]) -> Dict[str, Any]:
    """
    JSON 友好化:
    - np 向量 -> list
    - 图像 -> base64 JPEG
    """
    out: Dict[str, Any] = {}
    for k, v in obs.items():
        if isinstance(v, np.ndarray):
            if v.ndim == 3 and v.shape[2] == 3 and v.dtype == np.uint8:
                # RGB -> JPEG
                bgr = cv2.cvtColor(v, cv2.COLOR_RGB2BGR)
                ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    continue
                out[k] = {
                    "__type__": "jpeg",
                    "data": base64.b64encode(buf.tobytes()).decode("ascii"),
                }
            else:
                out[k] = v.tolist()
        elif isinstance(v, (np.floating, np.integer)):
            out[k] = v.item()
        else:
            out[k] = v
    return out


class R1ProTCPHandler(socketserver.BaseRequestHandler):
    robot: R1ProRobot = None  # 在 server 启动时注入

    # --- 基本收发 ---

    def _recv_exact(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = self.request.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("client disconnected")
            buf += chunk
        return buf

    def _recv_message(self) -> Dict[str, Any]:
        header = self._recv_exact(4)
        (length,) = struct.unpack("!I", header)
        data = self._recv_exact(length)
        try:
            return json.loads(data.decode("utf-8"))
        except UnicodeDecodeError as exc:
            # Reject invalid UTF-8 payloads to avoid crashing the handler
            raise ValueError(f"invalid UTF-8 payload: {exc}") from exc

    def _send_message(self, obj: Dict[str, Any]):
        data = json.dumps(obj).encode("utf-8")
        header = struct.pack("!I", len(data))
        self.request.sendall(header + data)

    # --- 主要逻辑 ---

    def handle(self):
        print(f"[R1Server] New connection from {self.client_address}")
        cfg = self.robot.cfg

        while True:
            try:
                msg = self._recv_message()
            except ConnectionError:
                print(f"[R1Server] Client {self.client_address} disconnected.")
                break
            except Exception as e:
                print(f"[R1Server] Error receiving message: {e}")
                traceback.print_exc()
                continue
                # break
            
            try:
                cmd = msg.get("cmd")

                if cmd == "ping":
                    self._send_message({"ok": True, "result": "pong"})
                    continue

                if cmd == "wait_ready":
                    timeout = float(msg.get("timeout", self.robot.cfg.warmup_timeout_sec))
                    import time

                    deadline = time.time() + timeout
                    ready = False
                    while time.time() < deadline and self.robot.is_connected:
                        self.robot.spin_once(0.05)
                        raw_obs = self.robot.get_observation()
                        if raw_obs is not None:
                            ready = True
                            break
                    self._send_message({"ok": True, "result": {"ready": ready}})
                    continue

                if cmd == "get_obs":
                    include_images = bool(msg.get("include_images", True))
                    raw_obs = self.robot.get_observation()
                    if raw_obs is None:
                        self._send_message({"ok": False, "error": "observation not ready"})
                        continue

                    # 原样透传 R1ProRobot 的观测，只是可选去掉图片
                    obs = dict(raw_obs)
                    if not include_images and cfg.enable_cameras:
                        for k in list(obs.keys()):
                            if k.endswith("_rgb"):
                                del obs[k]

                    encoded = _encode_obs(obs)
                    self._send_message({"ok": True, "result": {"observation": encoded}})
                    continue

                if cmd == "send_action":
                    
                    # 期望 action 中：
                    #   left_ee_pose:  [dx,dy,dz,droll,dpitch,dyaw]
                    #   right_ee_pose: 同上
                    #   left_gripper:  float 或 [float] （绝对量）
                    #   right_gripper: float 或 [float]
                    msg_act = msg.get("action") or {}
                    self.robot.send_action(msg_act)
                    self._send_message({"ok": True, "result": {"action": msg_act}})
                    print("euler_left: ", self.robot._target_left_rpy)
                    continue


                if cmd == "set_brake":
                    enabled = bool(msg.get("enabled", True))
                    self.robot.enable_brake(enabled)
                    self._send_message({"ok": True, "result": {"enabled": enabled}})
                    continue

                # 未知命令
                self._send_message({"ok": False, "error": f"unknown cmd: {cmd}"})

            except Exception as e:
                traceback.print_exc()
                self._send_message({"ok": False, "error": str(e)})


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True


def main():
    cfg = R1ProConfig(
        enable_cameras=False,  # 先只调通控制；需要图像时改为 True
        use_joints=False,
    )
    robot = R1ProRobot(cfg)
    print("[R1Server] Connecting to R1 PRO via ROS2...")
    robot.connect()
    ready = robot.wait_ready()
    print(f"[R1Server] Robot ready: {ready}")

    host = "0.0.0.0"
    port = 50051
    server = ThreadedTCPServer((host, port), R1ProTCPHandler)
    R1ProTCPHandler.robot = robot

    print(f"[R1Server] Listening on {host}:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("[R1Server] Shutting down...")
    finally:
        server.shutdown()
        robot.disconnect()


if __name__ == "__main__":
    main()

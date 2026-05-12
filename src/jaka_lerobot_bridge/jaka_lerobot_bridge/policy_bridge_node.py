#!/usr/bin/env python3
"""
policy_bridge_node
------------------

ROS 2 node (Python 3.10 / rclpy) that bridges to a LeRobot ACT policy
running in a separate Python 3.12 venv.

  observation  ──┐
   /joint_states │
   /gripper/state│      pack (state + images) via msgpack
   /camera_*/    ├──►  ZMQ REQ ─► policy_server (Py3.12 + LeRobot)
                 │
                 │     ◄── ZMQ REP (action vec: 6 arm + 1 gripper)
                 │
   /jaka_arm_servo_controller/commands  (std_msgs/Float64MultiArray)
   /gripper/ctrl  (dh_gripper_driver/GripperCtrl)

The actual policy.select_action() lives in offline/policy_server.py, which
must already be running (default endpoint tcp://127.0.0.1:5555).
"""

import threading
import time

import cv2
import msgpack
import numpy as np
import rclpy
import zmq
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray

try:
    from dh_gripper_driver.msg import GripperCtrl
    HAS_GRIPPER = True
except ImportError:
    HAS_GRIPPER = False

from .common import (
    ARM_CMD_TOPIC, ARM_JOINTS,
    CAMERA_TOPICS, GRIPPER_CLOSED_ANGLE_RAD,
    GRIPPER_CTRL_TOPIC, GRIPPER_FORCE_PCT, GRIPPER_JOINT,
    GRIPPER_MAX_MM, GRIPPER_SPEED_PCT, IMG_H, IMG_W,
    JOINT_STATES_TOPIC, POLICY_ZMQ_ENDPOINT, TARGET_FPS,
)


def _decode_compressed(msg: CompressedImage) -> np.ndarray:
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)        # BGR
    if img is None:
        return None
    if img.shape[:2] != (IMG_H, IMG_W):
        img = cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)      # RGB uint8 (H,W,3)


class PolicyBridge(Node):
    def __init__(self):
        super().__init__("policy_bridge")

        self.declare_parameter("endpoint", POLICY_ZMQ_ENDPOINT)
        self.declare_parameter("rate_hz", float(TARGET_FPS))
        self.declare_parameter("publish_gripper", True)
        self.declare_parameter("zmq_timeout_ms", 500)
        self.declare_parameter("warmup_messages", 5)

        self.endpoint        = self.get_parameter("endpoint").value
        self.rate_hz         = float(self.get_parameter("rate_hz").value)
        self.publish_gripper = bool(self.get_parameter("publish_gripper").value)
        self.zmq_timeout_ms  = int(self.get_parameter("zmq_timeout_ms").value)
        self.warmup_left     = int(self.get_parameter("warmup_messages").value)

        # ---- inputs ----
        self.last_js: JointState | None = None
        self.last_imgs: dict[str, np.ndarray | None] = {k: None for k in CAMERA_TOPICS}
        self._lock = threading.Lock()
        self._warn_last_t: dict[str, float] = {}

        self.create_subscription(JointState, JOINT_STATES_TOPIC,
                                 self._cb_js, 50)
        for cam, topic in CAMERA_TOPICS.items():
            self.create_subscription(
                CompressedImage, topic,
                lambda m, _c=cam: self._cb_img(_c, m),
                qos_profile_sensor_data,
            )

        # ---- outputs ----
        self.arm_pub = self.create_publisher(
            Float64MultiArray, ARM_CMD_TOPIC, 10)
        if self.publish_gripper:
            if not HAS_GRIPPER:
                self.get_logger().error(
                    "dh_gripper_driver msgs not found, disabling gripper publishing")
                self.publish_gripper = False
            else:
                self.grip_pub = self.create_publisher(
                    GripperCtrl, GRIPPER_CTRL_TOPIC, 10)

        # ---- ZMQ ----
        self.zctx = zmq.Context()
        self.zsock = self._connect_zmq()

        # Reset the policy queue when bridge starts.
        try:
            self._zmq_call({"cmd": "reset"})
            self.get_logger().info("Policy reset OK")
        except Exception as e:
            self.get_logger().warn(f"Policy reset failed (server up?): {e}")

        # ---- timer ----
        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self.step)

        self.get_logger().info(
            f"PolicyBridge ready  endpoint={self.endpoint}  "
            f"rate={self.rate_hz}Hz  publish_gripper={self.publish_gripper}")

    # ----- ZMQ helpers -----
    def _warn(self, key: str, msg: str, period: float = 1.0):
        """Throttled warn — emit at most once per `period` seconds per key."""
        now = time.time()
        if now - self._warn_last_t.get(key, 0.0) >= period:
            self.get_logger().warn(msg)
            self._warn_last_t[key] = now

    def _connect_zmq(self):
        sock = self.zctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVTIMEO, self.zmq_timeout_ms)
        sock.setsockopt(zmq.SNDTIMEO, self.zmq_timeout_ms)
        sock.connect(self.endpoint)
        return sock

    def _reset_zmq(self):
        """REQ sockets get stuck after a missed reply; rebuild."""
        try:
            self.zsock.close(0)
        except Exception:
            pass
        self.zsock = self._connect_zmq()

    def _zmq_call(self, pkt: dict) -> dict:
        self.zsock.send(msgpack.packb(pkt, use_bin_type=True))
        rep = self.zsock.recv()
        return msgpack.unpackb(rep, raw=False)

    # ----- ROS callbacks -----
    def _cb_js(self, msg: JointState):
        with self._lock:
            self.last_js = msg

    def _cb_img(self, cam: str, msg: CompressedImage):
        img = _decode_compressed(msg)
        if img is None:
            return
        with self._lock:
            self.last_imgs[cam] = img

    # ----- main step -----
    def _build_state(self, js: JointState) -> list[float] | None:
        n2p = dict(zip(js.name, js.position))
        try:
            arm = [float(n2p[j]) for j in ARM_JOINTS]
        except KeyError as e:
            self._warn("missing_joint", f"missing joint {e}", 2.0)
            return None
        fin = float(n2p.get(GRIPPER_JOINT, 0.0))
        # finger angle = closed_angle * (1 - open_norm)  =>  open_norm = 1 - fin/closed
        open_norm = 1.0 - float(np.clip(fin / GRIPPER_CLOSED_ANGLE_RAD, 0.0, 1.0))
        return arm + [open_norm]

    def step(self):
        with self._lock:
            js = self.last_js
            imgs = {k: (v.copy() if v is not None else None)
                    for k, v in self.last_imgs.items()}

        if js is None:
            return
        if any(v is None for v in imgs.values()):
            return

        state = self._build_state(js)
        if state is None:
            return

        pkt = {
            "state": state,
            "img_shape": {k: list(v.shape[:2]) for k, v in imgs.items()},
            "images":   {k: v.tobytes() for k, v in imgs.items()},
        }
        try:
            rep = self._zmq_call(pkt)
        except zmq.Again:
            self._warn("zmq_timeout", "policy server timeout; reconnecting", 1.0)
            self._reset_zmq()
            return
        except Exception as e:
            self.get_logger().error(f"policy call failed: {e}")
            self._reset_zmq()
            return

        if "action" not in rep:
            self.get_logger().error(f"bad reply: {rep}")
            return

        action = np.asarray(rep["action"], dtype=np.float32)
        if action.shape[0] < 7:
            self.get_logger().error(f"action dim={action.shape[0]} (need >=7)")
            return

        # Skip first few outputs so the policy stabilises before driving the arm.
        if self.warmup_left > 0:
            self.warmup_left -= 1
            return

        # Arm
        m = Float64MultiArray()
        m.data = [float(x) for x in action[:6]]
        self.arm_pub.publish(m)

        # Gripper
        if self.publish_gripper:
            g = GripperCtrl()
            g.initialize = False
            g.position = float(np.clip(action[6], 0.0, 1.0)) * GRIPPER_MAX_MM
            g.force = float(GRIPPER_FORCE_PCT)
            g.speed = float(GRIPPER_SPEED_PCT)
            self.grip_pub.publish(g)


def main():
    rclpy.init()
    node = PolicyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
bag_recorder_node
-----------------

Listens to /joy and drives `ros2 bag record` as a subprocess to record
one mcap bag per episode.

Buttons (Xbox layout, same as jaka_joy_to_servo.cpp):
    START -> begin / end current episode
    X     -> discard current episode (delete bag dir)
    BACK  -> shutdown the recorder node

Each episode is written to:
    <out_dir>/episode_<NNNN>/      (a ros2 bag mcap directory)

The numbering continues across runs: on startup we scan <out_dir> for
existing episode_XXXX/ dirs and continue from there.
"""

import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .common import (
    JOY_TOPIC,
    JOINT_STATES_TOPIC, ARM_CMD_TOPIC,
    GRIPPER_STATE_TOPIC, GRIPPER_CTRL_TOPIC,
    CAMERA_TOPICS,
    BTN_START, BTN_X, BTN_BACK,
)


def _record_topics():
    return [
        JOINT_STATES_TOPIC,
        ARM_CMD_TOPIC,
        GRIPPER_STATE_TOPIC,
        GRIPPER_CTRL_TOPIC,
        *CAMERA_TOPICS.values(),
    ]


def _next_episode_idx(out_dir: Path) -> int:
    if not out_dir.exists():
        return 0
    existing = [d.name for d in out_dir.iterdir() if d.is_dir()
                and d.name.startswith("episode_")]
    idxs = []
    for name in existing:
        try:
            idxs.append(int(name.split("_", 1)[1]))
        except (IndexError, ValueError):
            continue
    return (max(idxs) + 1) if idxs else 0


class BagRecorder(Node):
    def __init__(self):
        super().__init__("bag_recorder")

        self.declare_parameter("out_dir", os.path.expanduser("~/jaka_data/raw"))
        self.declare_parameter("storage", "mcap")
        self.declare_parameter("require_pressed_count", 1)

        self.out_dir = Path(self.get_parameter("out_dir").value).expanduser()
        self.storage = self.get_parameter("storage").value
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.proc = None
        self.cur_path: Path | None = None
        self.cur_idx = _next_episode_idx(self.out_dir)
        self.t_start = 0.0

        # Edge detection so a single press fires once.
        self.prev_start = 0
        self.prev_x = 0
        self.prev_back = 0

        self.create_subscription(Joy, JOY_TOPIC, self.cb, 10)

        self.get_logger().info(
            f"BagRecorder ready. out_dir={self.out_dir} "
            f"next_episode={self.cur_idx} storage={self.storage}")
        self.get_logger().info(
            "Buttons:  START=start/stop  X=discard  BACK=shutdown")

    # --------------- lifecycle ---------------
    def start_episode(self):
        if self.proc is not None:
            self.get_logger().warn("Already recording; ignoring start.")
            return
        self.cur_path = self.out_dir / f"episode_{self.cur_idx:04d}"
        if self.cur_path.exists():
            shutil.rmtree(self.cur_path, ignore_errors=True)
        cmd = [
            "ros2", "bag", "record",
            "--storage", self.storage,
            "-o", str(self.cur_path),
            *_record_topics(),
        ]
        # New process group so we can SIGINT the whole tree cleanly.
        self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.t_start = time.time()
        self.get_logger().info(
            f"[REC] >>> start episode {self.cur_idx}  ({self.cur_path.name})")

    def stop_episode(self, keep: bool):
        if self.proc is None:
            self.get_logger().warn("Not recording; nothing to stop.")
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            try:
                self.proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self.get_logger().error(
                    "ros2 bag record did not exit on SIGINT, sending SIGKILL")
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                self.proc.wait()
        finally:
            self.proc = None

        dur = time.time() - self.t_start
        if keep:
            self.get_logger().info(
                f"[REC] <<< saved episode {self.cur_idx} "
                f"({dur:.1f}s) -> {self.cur_path}")
            self.cur_idx += 1
        else:
            shutil.rmtree(self.cur_path, ignore_errors=True)
            self.get_logger().info(
                f"[REC] xxx discarded episode {self.cur_idx} ({dur:.1f}s)")
        self.cur_path = None

    # --------------- joy callback ---------------
    def _btn(self, msg: Joy, idx: int) -> int:
        return int(msg.buttons[idx]) if len(msg.buttons) > idx else 0

    def cb(self, msg: Joy):
        start = self._btn(msg, BTN_START)
        x     = self._btn(msg, BTN_X)
        back  = self._btn(msg, BTN_BACK)

        # START: toggle recording
        if start and not self.prev_start:
            if self.proc is None:
                self.start_episode()
            else:
                self.stop_episode(keep=True)

        # X: discard
        if x and not self.prev_x:
            if self.proc is not None:
                self.stop_episode(keep=False)
            else:
                self.get_logger().info("X ignored (not recording)")

        # BACK: shutdown
        if back and not self.prev_back:
            self.get_logger().info("BACK pressed -> shutting down recorder")
            if self.proc is not None:
                self.stop_episode(keep=True)
            rclpy.shutdown()

        self.prev_start, self.prev_x, self.prev_back = start, x, back


def main():
    rclpy.init()
    node = BagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # On Ctrl-C: save whatever we have, then shut down cleanly.
        if node.proc is not None:
            node.stop_episode(keep=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
bag_to_lerobot.py
-----------------

Convert ROS 2 mcap bags (one per episode) into a LeRobotDataset v3.0.

This script is meant to run in the LeRobot Python 3.12 venv. It does NOT
import rclpy or any ROS Python lib; mcap files are parsed via the standalone
`mcap` + `mcap-ros2-support` packages.

Install (LeRobot venv, Py 3.12):
    pip install lerobot mcap mcap-ros2-support opencv-python numpy

Layout expected:
    <bags_dir>/
        episode_0000/                # a single ros2 bag mcap dir
            metadata.yaml
            episode_0000_0.mcap
        episode_0001/
        ...

Recorded topics (must match jaka_lerobot_bridge.common):
    /joint_states                                       sensor_msgs/JointState
    /jaka_arm_servo_controller/commands                 std_msgs/Float64MultiArray
    /gripper/state                                      dh_gripper_driver/GripperState
    /gripper/ctrl                                       dh_gripper_driver/GripperCtrl
    /camera_*/color/image_raw/compressed                sensor_msgs/CompressedImage

Usage:
    python offline/bag_to_lerobot.py \
        --bags ~/jaka_data/raw \
        --repo-id local/jaka_pick_place \
        --out    ./data/jaka_pick_place \
        --task   "pick and place the cubes onto the matching-color zones"
"""

import argparse
import bisect
from pathlib import Path

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from lerobot.datasets.lerobot_dataset import LeRobotDataset

# =====================================================================
# Constants — keep in sync with jaka_lerobot_bridge/common.py
# =====================================================================
ARM_JOINTS = [
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
]
GRIPPER_JOINT = "gripper_finger1_joint"
GRIPPER_MAX_MM = 100.0
GRIPPER_CLOSED_ANGLE_RAD = -0.94

CAMERA_TOPICS = {
    "scene_cam": "/camera_scene/color/image_raw/compressed",
    "wrist_cam": "/camera_wrist/color/image_raw/compressed",
}
ARM_CMD_TOPIC       = "/jaka_arm_servo_controller/commands"
GRIPPER_CTRL_TOPIC  = "/gripper/ctrl"
JOINT_STATES_TOPIC  = "/joint_states"
GRIPPER_STATE_TOPIC = "/gripper/state"

TARGET_FPS = 30
IMG_H, IMG_W = 240, 320

STATE_NAMES  = list(ARM_JOINTS) + ["gripper"]
ACTION_NAMES = list(STATE_NAMES)


# =====================================================================
# mcap helpers
# =====================================================================
def _ros2_msgs():
    """Return the set of topic strings we care about."""
    return {JOINT_STATES_TOPIC, ARM_CMD_TOPIC,
            GRIPPER_STATE_TOPIC, GRIPPER_CTRL_TOPIC,
            *CAMERA_TOPICS.values()}


def read_bag_dir(bag_dir: Path) -> dict:
    """Read every .mcap file in `bag_dir` and merge into a dict
    {topic -> [(t_ns, msg_obj), ...]} sorted by time."""
    wanted = _ros2_msgs()
    buckets: dict[str, list] = {t: [] for t in wanted}

    mcap_files = sorted(bag_dir.glob("*.mcap"))
    if not mcap_files:
        raise FileNotFoundError(f"no .mcap in {bag_dir}")

    for mcap_path in mcap_files:
        with open(mcap_path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _schema, channel, msg, ros_msg in reader.iter_decoded_messages():
                if channel.topic in buckets:
                    buckets[channel.topic].append((msg.log_time, ros_msg))

    for k in buckets:
        buckets[k].sort(key=lambda x: x[0])
    return buckets


def latest_before(seq: list, t_ns: int):
    """Return msg with the largest timestamp <= t_ns, or None."""
    if not seq:
        return None
    # bisect on a derived list — small overhead, fine for <100k msgs/topic.
    i = bisect.bisect_right([s[0] for s in seq], t_ns) - 1
    return seq[i][1] if i >= 0 else None


# =====================================================================
# Per-message decoders
# =====================================================================
def joint_state_to_vec(js) -> np.ndarray | None:
    """JointState -> [6 arm pos, gripper_norm(0=closed,1=open)] float32."""
    n2p = dict(zip(js.name, js.position))
    try:
        arm = np.array([n2p[j] for j in ARM_JOINTS], dtype=np.float32)
    except KeyError:
        return None
    fin = float(n2p.get(GRIPPER_JOINT, 0.0))
    # finger angle = closed_angle * (1 - open_norm)  =>  open_norm = 1 - fin/closed
    open_norm = 1.0 - float(np.clip(fin / GRIPPER_CLOSED_ANGLE_RAD, 0.0, 1.0))
    return np.append(arm, np.float32(open_norm))


def arm_cmd_to_vec(cmd_msg, last_grip_ctrl_norm: float) -> np.ndarray:
    """Float64MultiArray.data[:6] + last gripper /ctrl normalized to [0,1]."""
    arr = np.asarray(cmd_msg.data, dtype=np.float32)[:6]
    return np.append(arr, np.float32(last_grip_ctrl_norm))


def decode_compressed_img(msg) -> np.ndarray | None:
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return None
    if img.shape[:2] != (IMG_H, IMG_W):
        img = cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


# =====================================================================
# Dataset
# =====================================================================
def build_features(use_videos: bool = True):
    dtype = "video" if use_videos else "image"
    feat = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(STATE_NAMES),),
            "names": STATE_NAMES,
        },
        "action": {
            "dtype": "float32",
            "shape": (len(ACTION_NAMES),),
            "names": ACTION_NAMES,
        },
    }
    for cam in CAMERA_TOPICS:
        feat[f"observation.images.{cam}"] = {
            "dtype": dtype,
            "shape": (IMG_H, IMG_W, 3),
            "names": ["height", "width", "channels"],
        }
    return feat


def load_or_create_dataset(repo_id: str, out_root: Path, fps: int,
                           use_videos: bool = True) -> LeRobotDataset:
    info = out_root / "meta" / "info.json"
    if info.exists():
        print(f"[Dataset] resume {out_root}")
        resume_fn = getattr(LeRobotDataset, "resume", None)
        if callable(resume_fn):
            return LeRobotDataset.resume(
                repo_id=repo_id, root=out_root,
                image_writer_processes=1, image_writer_threads=2,
            )
        return LeRobotDataset(repo_id, root=out_root)

    print(f"[Dataset] create {out_root}  (videos={use_videos})")
    return LeRobotDataset.create(
        repo_id=repo_id, fps=fps, root=out_root,
        features=build_features(use_videos=use_videos),
        robot_type="jaka_zu5_ag95", use_videos=use_videos,
        image_writer_processes=1, image_writer_threads=2,
    )


# =====================================================================
# Episode conversion
# =====================================================================
def convert_one_episode(ep_dir: Path, dataset: LeRobotDataset, task: str,
                        fps: int) -> int:
    buckets = read_bag_dir(ep_dir)

    ref_topic = CAMERA_TOPICS["scene_cam"]
    ref_seq = buckets[ref_topic]
    if not ref_seq:
        print(f"  [skip] no scene_cam frames in {ep_dir.name}")
        return 0

    # Resample grid at TARGET_FPS, starting from the first scene_cam ts.
    t0 = ref_seq[0][0]
    t_end = ref_seq[-1][0]
    period_ns = int(1e9 / fps)
    n_grid = max(0, (t_end - t0) // period_ns)

    last_grip_ctrl_norm = 1.0  # default = open
    added = 0
    for i in range(int(n_grid)):
        t_ns = t0 + i * period_ns

        js  = latest_before(buckets[JOINT_STATES_TOPIC], t_ns)
        cmd = latest_before(buckets[ARM_CMD_TOPIC],      t_ns)
        if js is None or cmd is None:
            continue

        gc = latest_before(buckets[GRIPPER_CTRL_TOPIC], t_ns)
        if gc is not None:
            # GripperCtrl.position is mm (0..GRIPPER_MAX_MM)
            last_grip_ctrl_norm = float(
                np.clip(gc.position / GRIPPER_MAX_MM, 0.0, 1.0))

        state = joint_state_to_vec(js)
        if state is None:
            continue
        action = arm_cmd_to_vec(cmd, last_grip_ctrl_norm)

        frame = {"observation.state": state, "action": action}
        ok = True
        for cam, topic in CAMERA_TOPICS.items():
            img_msg = latest_before(buckets[topic], t_ns)
            if img_msg is None:
                ok = False
                break
            img = decode_compressed_img(img_msg)
            if img is None:
                ok = False
                break
            frame[f"observation.images.{cam}"] = img
        if not ok:
            continue

        try:
            dataset.add_frame(frame, task=task)
        except TypeError:
            # very old lerobot API
            frame["task"] = task
            dataset.add_frame(frame)
        added += 1

    return added


def _clear_buffer(dataset):
    for attr in ("clear_episode_buffer", "clear_buffer", "reset_episode_buffer"):
        fn = getattr(dataset, attr, None)
        if callable(fn):
            try:
                fn()
                return
            except Exception:
                pass


# =====================================================================
# main
# =====================================================================
def main():
    ap = argparse.ArgumentParser(__doc__)
    ap.add_argument("--bags", required=True, type=Path,
                    help="Directory containing episode_NNNN bag dirs")
    ap.add_argument("--repo-id", default="local/jaka_pick_place")
    ap.add_argument("--out",     default="./data/jaka_pick_place", type=Path)
    ap.add_argument("--task",    default="pick and place")
    ap.add_argument("--fps",     type=int, default=TARGET_FPS)
    ap.add_argument("--image-mode", action="store_true",
                    help="Store as PNG images instead of MP4 videos.")
    args = ap.parse_args()

    dataset = load_or_create_dataset(
        args.repo_id, args.out, args.fps,
        use_videos=not args.image_mode,
    )

    ep_dirs = sorted(
        [p for p in args.bags.iterdir() if p.is_dir()
         and p.name.startswith("episode_")]
    )
    if not ep_dirs:
        print(f"[ERROR] no episode_* dirs under {args.bags}")
        return

    print(f"[CONV] {len(ep_dirs)} episode(s) -> {args.out}")
    total = 0
    for ep_dir in ep_dirs:
        try:
            n = convert_one_episode(ep_dir, dataset, args.task, args.fps)
        except Exception as e:
            print(f"  [error] {ep_dir.name}: {e}")
            _clear_buffer(dataset)
            continue
        if n > 0:
            dataset.save_episode()
            total += n
            print(f"  [ok] {ep_dir.name}  +{n} frames")
        else:
            _clear_buffer(dataset)
            print(f"  [skip] {ep_dir.name}  0 frames")

    fn = getattr(dataset, "finalize", None)
    if callable(fn):
        fn()
        print("[Dataset] finalize() done")

    print(f"\n[DONE] {dataset.num_episodes} episodes, {total} frames -> {args.out.resolve()}")


if __name__ == "__main__":
    main()

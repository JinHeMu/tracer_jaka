# jaka_lerobot_bridge

End-to-end pipeline that connects a **ROS 2 (Python 3.10)** Jaka Zu5 + DH
AG95 stack to **LeRobot (Python 3.12)** for imitation learning.

The two environments never need to share a Python interpreter:

```
┌──── ROS 2 / Py3.10 ─────────┐                ┌──── LeRobot / Py3.12 ─────┐
│ bag_recorder_node           │                │ bag_to_lerobot.py         │
│  → ros2 bag record (mcap)   │  *.mcap files  │  → LeRobotDataset v3.0    │
│                             │  ────────────► │  → lerobot-train          │
│                             │                │  → policy_server.py (REP) │
│ policy_bridge_node          │      ZMQ       │                           │
│  → /jaka_arm_servo_ctrl/... │  ◄─────────►   │ ACTPolicy.select_action() │
└─────────────────────────────┘                └───────────────────────────┘
```

## 1. Install

### ROS 2 side (Py3.10)

Put this package into your colcon workspace and build:

```bash
cd ~/ros2_ws/src
cp -r /path/to/jaka_lerobot_bridge .
cd ~/ros2_ws

# Runtime python deps for the bridge node:
pip install pyzmq msgpack opencv-python numpy

# rosbag2 mcap storage (Humble/Iron+):
sudo apt install ros-${ROS_DISTRO}-rosbag2-storage-mcap

colcon build --packages-select jaka_lerobot_bridge
source install/setup.bash
```

### LeRobot side (Py3.12), on workstation or GPU server

```bash
python3.12 -m venv ~/venvs/lerobot
source ~/venvs/lerobot/bin/activate
pip install lerobot torch torchvision \
            mcap mcap-ros2-support \
            pyzmq msgpack opencv-python numpy
```

No ROS install is needed in this venv. The mcap reader works standalone.

## 2. Record data on the real robot

Plug in the gamepad, power the arm and gripper, then:

```bash
ros2 launch jaka_lerobot_bridge record.launch.py \
    out_dir:=$HOME/jaka_data/raw
```

Buttons (matches `jaka_joy_to_servo.cpp`):

| Button   | Action                                    |
|----------|-------------------------------------------|
| START    | Begin / end current episode               |
| X        | Discard current recording                 |
| B / Y    | Gripper close / open (your existing map)  |
| BACK     | Shut down the recorder node               |

Each accepted episode is saved as
`~/jaka_data/raw/episode_NNNN/episode_NNNN_0.mcap`. Numbering continues
across re-launches.

> **Important:** `record.launch.py` keeps your existing joy → servo bridge
> running, so you teleop as before. The action recorded in the bag is
> `/jaka_arm_servo_controller/commands`, the IK target written by MoveIt
> Servo — **not** `/joint_states`. This matters: tracking lag and joint
> compliance otherwise get baked into the trained policy.

## 3. Convert bags to a LeRobotDataset (Py3.12 venv)

```bash
source ~/venvs/lerobot/bin/activate
cd ~/jaka_lerobot_bridge

python offline/bag_to_lerobot.py \
    --bags    $HOME/jaka_data/raw \
    --repo-id local/jaka_pick_place \
    --out     ./data/jaka_pick_place \
    --task    "pick and place the cubes onto the matching-color zones"
```

What it does:

* Reads every `episode_*` directory under `--bags`.
* For each one, resamples to **30 Hz** using the `scene_cam` timestamps
  as the reference grid; other topics are pulled in with zero-order hold
  (latest message at or before the grid time).
* `observation.state` is `[joint_1..6, gripper_open_norm]` decoded from
  `/joint_states` (gripper finger angle → normalised opening).
* `action` is `[arm_cmd_1..6, gripper_ctrl_norm]` from
  `/jaka_arm_servo_controller/commands` and `/gripper/ctrl`.
* Calls `dataset.finalize()` at the end — **required for v3.0**.

If video encoding crashes, re-run with `--image-mode` to fall back to PNGs
(\~30× more disk).

## 4. Train

Standard LeRobot training:

```bash
lerobot-train \
    --dataset.repo_id=local/jaka_pick_place \
    --dataset.root=./data/jaka_pick_place \
    --policy.type=act \
    --output_dir=outputs/train/jaka_act \
    --job_name=jaka_act \
    --policy.device=cuda \
    --batch_size=8 \
    --policy.push_to_hub=false
```

The checkpoint will land in
`outputs/train/jaka_act/checkpoints/last/pretrained_model/`.

## 5. Run policy inference

### 5a. Start the policy server (Py3.12 venv, anywhere)

```bash
source ~/venvs/lerobot/bin/activate
python offline/policy_server.py \
    --ckpt    outputs/train/jaka_act/checkpoints/last/pretrained_model \
    --device  cuda \
    --endpoint tcp://0.0.0.0:5555
```

ACT uses an action chunk internally — `select_action()` returns one step
at a time and refills the queue on its own. You don't need to buffer on
the client side.

### 5b. Start the ROS 2 side (Py3.10)

```bash
ros2 launch jaka_lerobot_bridge inference.launch.py \
    endpoint:=tcp://<policy-host>:5555
```

This **does not** start your joy → servo bridge — the policy is now the
sole writer to `/jaka_arm_servo_controller/commands`. If your existing
`servo_launch.py` always starts the joy bridge, gate it behind a launch
argument or comment it out for inference.

### 5c. Safety

* Bring the arm to a safe home pose first.
* The bridge has a `warmup_messages` parameter (default 5) — it discards
  the first few ACT outputs to let temporal ensembling settle before
  driving the arm. Increase if you see a jerk at start.
* Hit the e-stop and `Ctrl-C` the bridge if anything looks off.

## 6. Customising

* **Different cameras**: edit `config/camera_realsense.launch.py` (or
  ignore it and `cameras:=false`). Only the *topic names* must stay the
  same: `/camera_scene/color/image_raw/compressed` and
  `/camera_wrist/color/image_raw/compressed`. To change those, edit
  `CAMERA_TOPICS` in **both** `jaka_lerobot_bridge/common.py` and
  `offline/bag_to_lerobot.py`.
* **Different state/action dims**: change `STATE_NAMES` / `ACTION_NAMES`
  in `common.py` *and* `offline/bag_to_lerobot.py`, and update
  `STATE_DIM` / `ACTION_DIM` in `offline/policy_server.py`. Retrain.
* **Recording rate other than 30Hz**: change `TARGET_FPS` in both files
  and pass `--fps` to `bag_to_lerobot.py`.

## File map

```
jaka_lerobot_bridge/
├── jaka_lerobot_bridge/        # Py3.10 (rclpy) nodes
│   ├── common.py
│   ├── bag_recorder_node.py
│   └── policy_bridge_node.py
├── offline/                    # Py3.12 (LeRobot) scripts
│   ├── bag_to_lerobot.py
│   └── policy_server.py
├── launch/
│   ├── record.launch.py
│   └── inference.launch.py
├── config/
│   ├── camera_realsense.launch.py
│   └── topics.yaml
├── package.xml
├── setup.py
└── setup.cfg
```

"""
Shared constants for jaka_lerobot_bridge.

WARNING: Changes here must be propagated to offline/bag_to_lerobot.py and
offline/policy_server.py, because those two scripts run in a separate
Python 3.12 venv and *cannot* `import jaka_lerobot_bridge` (no ROS in that
env). The constants are duplicated there on purpose; keep them in sync.
"""

# ------------------- topics -------------------
ARM_CMD_TOPIC      = "/jaka_arm_servo_controller/commands"
JOINT_STATES_TOPIC = "/joint_states"
GRIPPER_STATE_TOPIC = "/gripper/state"
GRIPPER_CTRL_TOPIC  = "/gripper/ctrl"
JOY_TOPIC          = "/joy"

# Camera topics: name -> compressed image topic.
# Keep this dict in sync with offline/bag_to_lerobot.py CAMERA_TOPICS.
CAMERA_TOPICS = {
    "scene_cam": "/camera_scene/color/image_raw/compressed",
    "wrist_cam": "/camera_wrist/color/image_raw/compressed",
}

# ------------------- robot layout -------------------
ARM_JOINTS = [
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
]
# The joint name published by dh_ag95_driver in /joint_states.
GRIPPER_JOINT = "gripper_finger1_joint"

# AG95 mechanical limits (mm), see dh_ag95_driver.cpp.
GRIPPER_MAX_MM = 100.0
# The mapping the driver uses to render the finger joint angle:
#   angle = CLOSED_ANGLE * (max_mm - open_mm) / max_mm
# i.e. open(open_mm = max_mm) -> 0 rad, closed(open_mm = 0) -> CLOSED_ANGLE rad.
GRIPPER_CLOSED_ANGLE_RAD = -0.94

# Default gripper command parameters (force%, speed%) published to /gripper/ctrl.
GRIPPER_FORCE_PCT = 50.0
GRIPPER_SPEED_PCT = 50.0

# ------------------- xbox mapping (matches jaka_joy_to_servo.cpp) -------------------
BTN_A, BTN_B, BTN_X, BTN_Y = 0, 1, 2, 3
BTN_LB, BTN_RB = 4, 5
BTN_BACK, BTN_START = 6, 7

# ------------------- recording / inference -------------------
TARGET_FPS = 30
IMG_H, IMG_W = 240, 320

# State / action layout (must match what bag_to_lerobot.py writes).
# state  = [6 arm joint positions] + [gripper_norm]    , gripper_norm in [0,1] (1 = open)
# action = [6 arm command positions] + [gripper_norm]
STATE_NAMES  = list(ARM_JOINTS) + ["gripper"]
ACTION_NAMES = list(STATE_NAMES)
STATE_DIM = len(STATE_NAMES)
ACTION_DIM = len(ACTION_NAMES)

# ZMQ endpoint for the policy server.
POLICY_ZMQ_ENDPOINT = "tcp://127.0.0.1:5555"

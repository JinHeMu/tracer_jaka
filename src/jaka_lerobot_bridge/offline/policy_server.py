#!/usr/bin/env python3
"""
policy_server.py
----------------

LeRobot ACT policy server. Runs in the Py3.12 LeRobot venv. Pure ZMQ REP loop;
no ROS imports.

Protocol (msgpack-encoded dicts):

  Request:  {"cmd": "reset"}                       -> {"ok": True}
  Request:  {"state":     [STATE_DIM] floats,
             "img_shape": {cam: [H, W], ...},
             "images":    {cam: bytes(H*W*3 uint8 RGB), ...}}
            -> {"action": [ACTION_DIM] floats}
            -> {"error":  str}                     # on failure

Run:
    python offline/policy_server.py \
        --ckpt outputs/train/jaka_act/checkpoints/last/pretrained_model \
        --device cuda \
        --endpoint tcp://0.0.0.0:5555

The default endpoint binds to all interfaces so the bridge can connect from
another machine; flip to 127.0.0.1 if the bridge runs locally.
"""

import argparse
import time
import traceback

import msgpack
import numpy as np
import torch
import zmq

from lerobot.policies.act.modeling_act import ACTPolicy

# Must match common.py / bag_to_lerobot.py.
STATE_DIM = 7
ACTION_DIM = 7
IMG_H, IMG_W = 240, 320
CAMERA_NAMES = ["scene_cam", "wrist_cam"]


def _decode_image(buf: bytes, shape: tuple[int, int]) -> np.ndarray:
    h, w = shape
    arr = np.frombuffer(buf, dtype=np.uint8)
    return arr.reshape(h, w, 3)


def build_obs(pkt: dict, device: torch.device) -> dict:
    state = np.asarray(pkt["state"], dtype=np.float32)
    if state.shape[0] != STATE_DIM:
        raise ValueError(f"state dim {state.shape[0]} != {STATE_DIM}")

    obs = {
        "observation.state": torch.from_numpy(state)[None].to(device),
    }
    img_shape = pkt.get("img_shape", {})
    for cam in CAMERA_NAMES:
        if cam not in pkt["images"]:
            raise KeyError(f"missing camera {cam}")
        h, w = img_shape.get(cam, (IMG_H, IMG_W))
        img = _decode_image(pkt["images"][cam], (h, w))  # H,W,3 uint8 RGB
        t = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
        obs[f"observation.images.{cam}"] = t[None].to(device)
    return obs


def main():
    ap = argparse.ArgumentParser(__doc__)
    ap.add_argument("--ckpt", required=True,
                    help="Path to pretrained model (HF format dir, "
                         "e.g. .../checkpoints/last/pretrained_model)")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--endpoint", default="tcp://0.0.0.0:5555")
    args = ap.parse_args()

    device = torch.device(args.device)
    print(f"[Policy] loading {args.ckpt}")
    policy = ACTPolicy.from_pretrained(args.ckpt).to(device).eval()
    policy.reset()
    print(f"[Policy] ready on {device}")

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind(args.endpoint)
    print(f"[Policy] listening at {args.endpoint}")

    n_calls = 0
    t_last_log = time.time()

    while True:
        try:
            raw = sock.recv()
        except KeyboardInterrupt:
            break

        try:
            pkt = msgpack.unpackb(raw, raw=False)
        except Exception as e:
            sock.send(msgpack.packb({"error": f"unpack: {e}"}))
            continue

        try:
            if pkt.get("cmd") == "reset":
                policy.reset()
                sock.send(msgpack.packb({"ok": True}))
                print("[Policy] reset")
                continue

            obs = build_obs(pkt, device)
            with torch.inference_mode():
                action = policy.select_action(obs)
            action = action.squeeze(0).detach().cpu().numpy().astype(np.float32)
            if action.shape[0] != ACTION_DIM:
                raise ValueError(
                    f"action dim {action.shape[0]} != {ACTION_DIM}; "
                    "retrain matching state/action layout")

            sock.send(msgpack.packb({"action": action.tolist()}))
            n_calls += 1
            if time.time() - t_last_log > 5.0:
                print(f"[Policy] {n_calls} calls / 5s window")
                n_calls = 0
                t_last_log = time.time()

        except Exception as e:
            traceback.print_exc()
            try:
                sock.send(msgpack.packb({"error": str(e)}))
            except Exception:
                pass

    sock.close(0)
    ctx.term()


if __name__ == "__main__":
    main()

"""
export_policy.py — Floyd
========================
Converts an RSL-RL .pt checkpoint into a standalone ONNX file that can
run on the Jetson with onnxruntime (no Isaac Lab required).

Run on your training machine (Windows, env_isaaclab conda env):

    python scripts/export_policy.py --checkpoint logs/rsl_rl/floyd_velocity_flat/<run>/model_XXXX.pt

Output: model_XXXX.onnx written next to the .pt file.
"""

import argparse
import os
import torch
import torch.nn as nn

# ── Network shape (must match rsl_rl_ppo_cfg.py) ──────────────────────────────
OBS_DIM        = 35
ACTION_DIM     = 8
HIDDEN_DIMS    = [512, 256, 128]
ACTIVATION     = nn.ELU

def build_actor(obs_dim, hidden_dims, action_dim, activation_cls):
    layers = []
    prev = obs_dim
    for h in hidden_dims:
        layers.append(nn.Linear(prev, h))
        layers.append(activation_cls())
        prev = h
    layers.append(nn.Linear(prev, action_dim))
    return nn.Sequential(*layers)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True, help="Path to model_XXXX.pt")
    args = parser.parse_args()

    ckpt_path = args.checkpoint
    out_path  = os.path.splitext(ckpt_path)[0] + ".onnx"

    print(f"Loading checkpoint: {ckpt_path}")
    checkpoint = torch.load(ckpt_path, map_location="cpu")

    # RSL-RL saves full ActorCritic state dict under 'model_state_dict'
    full_state = checkpoint["model_state_dict"]

    # Extract only the actor weights (keys start with "actor.")
    actor_state = {
        k[len("actor."):]: v
        for k, v in full_state.items()
        if k.startswith("actor.")
    }

    actor = build_actor(OBS_DIM, HIDDEN_DIMS, ACTION_DIM, ACTIVATION)
    actor.load_state_dict(actor_state)
    actor.eval()

    # Verify forward pass
    dummy = torch.zeros(1, OBS_DIM)
    with torch.no_grad():
        out = actor(dummy)
    print(f"Actor forward pass OK — output shape: {out.shape}")

    # Export to ONNX
    torch.onnx.export(
        actor,
        dummy,
        out_path,
        input_names=["obs"],
        output_names=["actions"],
        dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
        opset_version=11,
    )
    print(f"Exported: {out_path}")

if __name__ == "__main__":
    main()

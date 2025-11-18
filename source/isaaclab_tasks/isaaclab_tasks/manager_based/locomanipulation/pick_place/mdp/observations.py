# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
from __future__ import annotations

import torch

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg


def upper_body_last_action(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Extract the last action of the upper body."""
    asset = env.scene[asset_cfg.name]
    joint_pos_target = asset.data.joint_pos_target

    # Use joint_names from asset_cfg to find indices
    joint_names = asset_cfg.joint_names if hasattr(asset_cfg, "joint_names") else None
    if joint_names is None:
        raise ValueError("asset_cfg must have 'joint_names' attribute for upper_body_last_action.")

    # Find joint indices matching the provided joint_names (supports regex)
    joint_indices, _ = asset.find_joints(joint_names)
    joint_indices = torch.tensor(joint_indices, dtype=torch.long)

    # Get upper body joint positions for all environments
    upper_body_joint_pos_target = joint_pos_target[:, joint_indices]

    return upper_body_joint_pos_target

def object_obs(
    env: ManagerBasedRLEnv,
    left_eef_link_name: str,
    right_eef_link_name: str,
    object_name: str = "object",
) -> torch.Tensor:
    """
    Object observations (in world frame):
        object pos,
        object quat,
        left_eef to object,
        right_eef_to object,
    
    Args:
        env: The RL environment instance.
        left_eef_link_name: Name of the left end-effector link.
        right_eef_link_name: Name of the right end-effector link.
        object_name: Name of the object entity in the scene (default: "object").
    """

    body_pos_w = env.scene["robot"].data.body_pos_w
    left_eef_idx = env.scene["robot"].data.body_names.index(left_eef_link_name)
    right_eef_idx = env.scene["robot"].data.body_names.index(right_eef_link_name)
    left_eef_pos = body_pos_w[:, left_eef_idx] - env.scene.env_origins
    right_eef_pos = body_pos_w[:, right_eef_idx] - env.scene.env_origins

    target_object = env.scene[object_name]
    object_pos = target_object.data.root_pos_w - env.scene.env_origins
    object_quat = target_object.data.root_quat_w

    left_eef_to_object = object_pos - left_eef_pos
    right_eef_to_object = object_pos - right_eef_pos

    return torch.cat(
        (
            object_pos,
            object_quat,
            left_eef_to_object,
            right_eef_to_object,
        ),
        dim=1,
    )


def get_eef_pos(env: ManagerBasedRLEnv, link_name: str) -> torch.Tensor:
    body_pos_w = env.scene["robot"].data.body_pos_w
    left_eef_idx = env.scene["robot"].data.body_names.index(link_name)
    left_eef_pos = body_pos_w[:, left_eef_idx] - env.scene.env_origins

    return left_eef_pos


def get_eef_quat(env: ManagerBasedRLEnv, link_name: str) -> torch.Tensor:
    body_quat_w = env.scene["robot"].data.body_quat_w
    left_eef_idx = env.scene["robot"].data.body_names.index(link_name)
    left_eef_quat = body_quat_w[:, left_eef_idx]

    return left_eef_quat


def get_robot_joint_state(
    env: ManagerBasedRLEnv,
    joint_names: list[str],
) -> torch.Tensor:
    # hand_joint_names is a list of regex, use find_joints
    indexes, _ = env.scene["robot"].find_joints(joint_names)
    indexes = torch.tensor(indexes, dtype=torch.long)
    robot_joint_states = env.scene["robot"].data.joint_pos[:, indexes]

    return robot_joint_states


def get_all_robot_link_state(
    env: ManagerBasedRLEnv,
) -> torch.Tensor:
    body_pos_w = env.scene["robot"].data.body_link_state_w[:, :, :]
    all_robot_link_pos = body_pos_w

    return all_robot_link_pos

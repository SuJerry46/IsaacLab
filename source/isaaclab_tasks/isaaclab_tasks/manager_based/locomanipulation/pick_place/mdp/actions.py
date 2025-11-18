# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets.articulation import Articulation
from isaaclab.managers.action_manager import ActionTerm
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.io.torchscript import load_torchscript_model

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .configs.action_cfg import AgileBasedLowerBodyActionCfg


class AgileBasedLowerBodyAction(ActionTerm):
    """Action term that is based on Agile lower body RL policy."""

    cfg: AgileBasedLowerBodyActionCfg
    """The configuration of the action term."""

    _asset: Articulation
    """The articulation asset to which the action term is applied."""

    def __init__(self, cfg: AgileBasedLowerBodyActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)

        # Save the observation config from cfg
        self._observation_cfg = env.cfg.observations
        self._obs_group_name = cfg.obs_group_name

        # Load policy here if needed
        _temp_policy_path = retrieve_file_path(cfg.policy_path)
        self._policy = load_torchscript_model(_temp_policy_path, device=env.device)
        self._env = env

        # Find joint ids for the lower body joints
        self._joint_ids, self._joint_names = self._asset.find_joints(
            self.cfg.joint_names, preserve_order=self.cfg.preserve_order
        )

        # DEBUG: Print joint mapping - write to file for easy access
        debug_output = []
        debug_output.append("\n" + "=" * 80)
        debug_output.append("🔍 [DEBUG] AgileBasedLowerBodyAction Initialization")
        debug_output.append("=" * 80)
        debug_output.append(f"Joint patterns: {self.cfg.joint_names}")
        debug_output.append(f"Preserve order: {self.cfg.preserve_order}")
        debug_output.append(f"Total matched joints: {len(self._joint_ids)}")
        debug_output.append("\n📋 Joint mapping (Policy output -> USD joint):")
        for idx, (joint_id, joint_name) in enumerate(zip(self._joint_ids, self._joint_names)):
            default_pos = self._asset.data.default_joint_pos[0, joint_id].item()
            debug_output.append(f"  Policy[{idx:2d}] -> USD[{joint_id:2d}] -> {joint_name:40s} (default_pos={default_pos:6.3f})")
        debug_output.append("=" * 80 + "\n")
        
        # Print to console
        for line in debug_output:
            print(line)
        
        # Also write to file
        debug_file = "/tmp/joint_mapping_debug.txt"
        with open(debug_file, "w") as f:
            f.write("\n".join(debug_output))
        print(f"💾 Joint mapping also saved to: {debug_file}\n")
        # Also print actuator gains (stiffness/damping) for these joints
        try:
            stiffness = self._asset.data.default_joint_stiffness[0, self._joint_ids].cpu().numpy()
            damping = self._asset.data.default_joint_damping[0, self._joint_ids].cpu().numpy()
            print("Actuator gains for matched joints:")
            for i, j_id in enumerate(self._joint_ids):
                print(f"  USD[{j_id:2d}] {self._joint_names[i]:30s} -> stiffness={stiffness[i]:7.1f}, damping={damping[i]:7.1f}")
            # append to debug file as well
            with open(debug_file, "a") as f:
                f.write("\nActuator gains for matched joints:\n")
                for i, j_id in enumerate(self._joint_ids):
                    f.write(f"  USD[{j_id:2d}] {self._joint_names[i]:30s} -> stiffness={stiffness[i]:7.1f}, damping={damping[i]:7.1f}\n")
        except Exception:
            print("Could not read actuator gains from asset data.")

        # Get the scale and offset from the configuration
        self._policy_output_scale = torch.tensor(cfg.policy_output_scale, device=env.device)
        self._policy_output_offset = self._asset.data.default_joint_pos[:, self._joint_ids].clone()

        # Store command history length for policy input composition
        self._command_history_length = cfg.command_history_length

        # Create tensors to store raw and processed actions
        self._raw_actions = torch.zeros(self.num_envs, len(self._joint_ids), device=self.device)
        self._processed_actions = torch.zeros(self.num_envs, len(self._joint_ids), device=self.device)

    """
    Properties.
    """

    @property
    def action_dim(self) -> int:
        """Lower Body Action: [vx, vy, wz, hip_height]"""
        return 4

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def _compose_policy_input(self, base_command: torch.Tensor, obs_tensor: torch.Tensor) -> torch.Tensor:
        """Compose the policy input by concatenating repeated commands with observations.

        Args:
            base_command: The base command tensor [vx, vy, wz, hip_height].
            obs_tensor: The observation tensor from the environment.

        Returns:
            The composed policy input tensor with repeated commands concatenated to observations.
        """
      
        # Repeat commands based on history length and concatenate with observations
        repeated_commands = base_command.unsqueeze(1).repeat(1, self._command_history_length, 1).reshape(base_command.shape[0], -1)
        policy_input = torch.cat([repeated_commands, obs_tensor], dim=-1)

        return policy_input

    def process_actions(self, actions: torch.Tensor):
        """Process the input actions using the locomotion policy.

        Args:
            actions: The lower body commands.
        """

        # Extract base command from the action tensor
        # Assuming the base command [vx, vy, wz, hip_height]
        base_command = actions

        obs_tensor = self._env.obs_buf["lower_body_policy"]

        # Compose policy input using helper function
        policy_input = self._compose_policy_input(base_command, obs_tensor)

        joint_actions = self._policy.forward(policy_input)

        self._raw_actions[:] = joint_actions

        # Apply scaling and offset to the raw actions from the policy
        self._processed_actions = joint_actions * self._policy_output_scale + self._policy_output_offset

        # DEBUG: Print action details on first step
        if not hasattr(self, '_debug_printed'):
            print("\n" + "-" * 80)
            print("[DEBUG] First action processing")
            print("-" * 80)
            print(f"Input command shape: {base_command.shape}, values: {base_command[0].cpu().numpy()}")
            print(f"Observation tensor shape: {obs_tensor.shape}")
            print(f"Policy input shape: {policy_input.shape}")
            print(f"Policy output (raw) shape: {joint_actions.shape}")
            print(f"Policy output (raw) sample: {joint_actions[0, :3].cpu().numpy()}")
            print(f"Policy output scale: {self._policy_output_scale.item()}")
            print(f"Default joint positions: {self._policy_output_offset[0].cpu().numpy()}")
            print(f"Processed actions sample: {self._processed_actions[0, :3].cpu().numpy()}")
            print("-" * 80 + "\n")
            self._debug_printed = True

        # Clip actions if configured
        if self.cfg.clip is not None:
            self._processed_actions = torch.clamp(
                self._processed_actions, min=self._clip[:, :, 0], max=self._clip[:, :, 1]
            )

    def apply_actions(self):
        """Apply the actions to the environment."""
        # Store the raw actions
        # Debug: log the exact joint targets we're applying for the first several steps
        try:
            if not hasattr(self, '_apply_debug_count'):
                self._apply_debug_count = 0
            # limit amount of logging to avoid huge logs
            if self._apply_debug_count < 200:
                dbg_lines = []
                dbg_lines.append("\n" + "~" * 80)
                dbg_lines.append(f"[DEBUG] Applying joint position targets (call #{self._apply_debug_count + 1})")
                dbg_lines.append(f"Joint ids: {self._joint_ids}")
                # log targets for env 0 (if multiple envs, focus on first env)
                targets = self._processed_actions[0].cpu().numpy()
                dbg_lines.append(f"Processed action vector (env0): {targets.tolist()}")
                for i, j_id in enumerate(self._joint_ids):
                    dbg_lines.append(f"  Policy[{i:2d}] -> USD[{j_id:2d}] {self._joint_names[i]:30s} -> target={targets[i]: .4f}")
                dbg_lines.append("~" * 80 + "\n")
                for line in dbg_lines:
                    print(line)
                # append to debug file as well
                try:
                    with open("/tmp/joint_mapping_debug.txt", "a") as f:
                        f.write("\n".join(dbg_lines))
                except Exception:
                    pass
            self._apply_debug_count += 1
        except Exception:
            # best-effort logging; don't fail action application
            pass

        self._asset.set_joint_position_target(self._processed_actions, joint_ids=self._joint_ids)

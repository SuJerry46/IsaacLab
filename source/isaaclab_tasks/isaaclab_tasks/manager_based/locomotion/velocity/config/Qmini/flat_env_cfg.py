# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg

##
# Pre-defined configs
##
from isaaclab_assets.robots.Qmini import Qmini_CFG


@configclass
class QminiRewards(RewardsCfg):
    """Reward terms for the MDP."""

    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=2.50,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="ankle_pitch_.*"),
            "command_name": "base_velocity",
            "threshold": 0.3,
        },
    )
    # penalize ankle joint limits
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names="ankle_pitch_.*")},
    )
    # penalize deviation from default of the joints that are not essential for locomotion
    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["hip_yaw_.*", "hip_roll_.*"])},
    )
    # 添加站立時的姿態穩定獎勵
    stand_still_joint_deviation = RewTerm(
        func=mdp.stand_still_joint_deviation_l1,
        weight=-1.0,
        params={
            "command_name": "base_velocity",
            "command_threshold": 0.1,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )


@configclass
class QminiFlatEnvCfg(LocomotionVelocityRoughEnvCfg):
    """Qmini flat environment configuration."""

    rewards: QminiRewards = QminiRewards()

    def __post_init__(self):
        super().__post_init__()
        # scene
        self.scene.robot = Qmini_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/base_link"

        # actions - 適度增加動作範圍以支持更大步伐
        self.actions.joint_pos.scale = 0.6  # 適度增加到0.6，避免過度運動導致不穩定

        # events - 暫時關閉可能導致偏差的隨機化
        self.events.push_robot = None
        self.events.add_base_mass = None  # 關閉質量隨機化
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.base_external_force_torque.params["asset_cfg"].body_names = ["base_link"]
        self.events.base_com = None  # 關閉COM隨機化
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.2, 0.2), "y": (-0.2, 0.2), "yaw": (-0.5, 0.5)},  # 減小初始位置範圍
            "velocity_range": {
                "x": (-0.1, 0.1),  # 小幅度初始速度
                "y": (-0.1, 0.1),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (-0.1, 0.1),
            },
        }

        # terminations
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            "base_link",
            "hip_yaw_.*",
            "hip_roll_.*",
            # "hip_pitch_.*",  # 可選：包含髖關節pitch
        ]

        # rewards - 重新平衡以避免偏向，增加穩定性
        self.rewards.undesired_contacts = None
        self.rewards.dof_torques_l2.weight = -8.0e-6  # 增加力矩懲罰以促進平滑運動
        self.rewards.track_lin_vel_xy_exp.weight = 2.5  # 適度降低速度跟蹤權重
        self.rewards.track_ang_vel_z_exp.weight = 1.5   # 適度降低角速度跟蹤權重
        self.rewards.action_rate_l2.weight *= 2.0  # 增加動作變化率懲罰以提高穩定性
        self.rewards.dof_acc_l2.weight *= 2.0  # 增加關節加速度懲罰
        self.rewards.flat_orientation_l2.weight = -3.0  # 增加姿態穩定性權重
        self.rewards.feet_air_time.weight = 2.0  # 降低單腳時間獎勵避免偏向

        # commands - 適度增加步伐大小和轉向幅度，確保穩定性
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 1.2)  # 適度增加最大前進速度到1.2m/s
        self.commands.base_velocity.ranges.lin_vel_y = (-0.3, 0.3)  # 適度增加側向移動能力
        self.commands.base_velocity.ranges.ang_vel_z = (-1.5, 1.5)  # 適度增加轉向幅度到1.5rad/s
        self.commands.base_velocity.rel_standing_envs = 0.1  # 10%環境保持靜止

        # change terrain
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # no terrain curriculum
        self.curriculum.terrain_levels = None

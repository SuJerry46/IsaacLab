# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


from isaaclab_assets.robots.unitree import G1_INSPIRE_FTP_CFG

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
import isaaclab.controllers.utils as ControllerUtils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.devices.device_base import DevicesCfg
from isaaclab.devices.openxr import OpenXRDeviceCfg, XrCfg
from isaaclab.devices.openxr.retargeters.humanoid.unitree.inspire.g1_upper_body_retargeter import (
    UnitreeG1RetargeterCfg,
)
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

from isaaclab_tasks.manager_based.locomanipulation.pick_place import mdp as locomanip_mdp
from isaaclab_tasks.manager_based.manipulation.pick_place import mdp as manip_mdp

from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.pink_controller_cfg import (  # isort: skip
    G1_INSPIRE_UPPER_BODY_IK_ACTION_CFG,
)


##
# Scene definition
##
@configclass
class FixedBaseUpperBodyIKG1InspireSceneCfg(InteractiveSceneCfg):
    """Scene configuration for fixed base upper body IK environment with G1 robot.

    This configuration sets up the G1 humanoid robot with fixed pelvis and legs,
    allowing only arm manipulation while the base remains stationary. The robot is
    controlled using upper body IK.
    """

    # Table
    packing_table = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, 0.55, -0.3], rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/PackingTable/packing_table.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )

    object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Object",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[-0.35, 0.45, 0.6996], rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Mimic/pick_place_task/pick_place_assets/steering_wheel.usd",
            scale=(0.75, 0.75, 0.75),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        ),
    )

   
    robot: ArticulationCfg = G1_INSPIRE_FTP_CFG.replace(
        spawn=G1_INSPIRE_FTP_CFG.spawn.replace(
            usd_path="/workspace/isaaclab/source/isaaclab_assets/data/unitree_isaac/usd/g1_inspire_hand/g1_inspire_hand.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,  # 與 G1 29DOF 相同
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                fix_root_link=False,  # Allow the robot to move (locomotion)
                solver_position_iteration_count=8,  # 與 G1 29DOF 相同
                solver_velocity_iteration_count=4,  # 與 G1 29DOF 相同
            ),
        ),
        prim_path="/World/envs/env_.*/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.75),  # 與 G1_29DOF_CFG 相同
            rot=(0.7071, 0, 0, 0.7071),
            joint_pos={
                # right-arm
                "right_shoulder_pitch_joint": 0.0,
                "right_shoulder_roll_joint": -1.0,
                "right_shoulder_yaw_joint": -0.4,
                "right_elbow_joint": -0.3,
                "right_wrist_yaw_joint": 0.8,
                "right_wrist_roll_joint": 0.5,
                "right_wrist_pitch_joint": -0.6,
                # left-arm
                "left_shoulder_pitch_joint": 0.0,
                "left_shoulder_roll_joint": 1.0,
                "left_shoulder_yaw_joint": 0.4,
                "left_elbow_joint": -0.3,
                "left_wrist_yaw_joint": -0.8,
                "left_wrist_roll_joint": -0.5,
                "left_wrist_pitch_joint": -0.6,
                # --
                "waist_.*": 0.0,
                # -- left/right hand
                ".*_thumb_.*": 0.0,
                ".*_index_.*": 0.0,
                ".*_middle_.*": 0.0,
                ".*_ring_.*": 0.0,
                ".*_pinky_.*": 0.0,
                # --lower body (使用與 G1 29DOF 相同的站立姿態)
                ".*_hip_pitch_joint": -0.10,  # 改為 -0.10 (與 G1 29DOF 相同)
                ".*_hip_roll_joint": 0.0,
                ".*_hip_yaw_joint": 0.0,
                ".*_knee_joint": 0.30,  # 改為 0.30 (與 G1 29DOF 相同)
                ".*_ankle_pitch_joint": -0.20,  # 改為 -0.20 (與 G1 29DOF 相同)
                ".*_ankle_roll_joint": 0.0,
            },
            joint_vel={".*": 0.0},
        ),
    )

    # Ground plane
    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=GroundPlaneCfg(),
    )

    # Lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    def __post_init__(self):
        """Post initialization."""
        # Set the robot to fixed base
        self.robot.spawn.articulation_props.fix_root_link = True


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    upper_body_ik = G1_INSPIRE_UPPER_BODY_IK_ACTION_CFG


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP.
    This class is required by the environment configuration but not used in this implementation
    """

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group with state values."""

        actions = ObsTerm(func=manip_mdp.last_action)
        robot_joint_pos = ObsTerm(
            func=base_mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        robot_root_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("robot")})
        robot_root_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("robot")})
        object_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("object")})
        object_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("object")})
        robot_links_state = ObsTerm(func=manip_mdp.get_all_robot_link_state)

        left_eef_pos = ObsTerm(func=manip_mdp.get_eef_pos, params={"link_name": "left_wrist_yaw_link"})
        left_eef_quat = ObsTerm(func=manip_mdp.get_eef_quat, params={"link_name": "left_wrist_yaw_link"})
        right_eef_pos = ObsTerm(func=manip_mdp.get_eef_pos, params={"link_name": "right_wrist_yaw_link"})
        right_eef_quat = ObsTerm(func=manip_mdp.get_eef_quat, params={"link_name": "right_wrist_yaw_link"})

        hand_joint_state = ObsTerm(func=manip_mdp.get_robot_joint_state, params={"joint_names": [".*_thumb_.*", ".*_index_.*", ".*_middle_.*", ".*_ring_.*", ".*_pinky_.*"]})
        head_joint_state = ObsTerm(func=manip_mdp.get_robot_joint_state, params={"joint_names": []})

        object = ObsTerm(
            func=manip_mdp.object_obs,
            params={"left_eef_link_name": "left_wrist_yaw_link", "right_eef_link_name": "right_wrist_yaw_link"},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=locomanip_mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=base_mdp.root_height_below_minimum, params={"minimum_height": 0.5, "asset_cfg": SceneEntityCfg("object")}
    )

    success = DoneTerm(func=manip_mdp.task_done_pick_place, params={"task_link_name": "right_wrist_yaw_link"})


##
# MDP settings
##


@configclass
class FixedBaseUpperBodyIKG1InspireEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the G1 fixed base upper body IK environment.

    This environment is designed for manipulation tasks where the G1 humanoid robot
    has a fixed pelvis and legs, allowing only arm and hand movements for manipulation. The robot is
    controlled using upper body IK.
    """

    # Scene settings
    scene: FixedBaseUpperBodyIKG1InspireSceneCfg = FixedBaseUpperBodyIKG1InspireSceneCfg(
        num_envs=1, env_spacing=2.5, replicate_physics=True
    )
    # MDP settings
    terminations: TerminationsCfg = TerminationsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()

    # Unused managers
    commands = None
    rewards = None
    curriculum = None

    # Position of the XR anchor in the world frame
    xr: XrCfg = XrCfg(
        anchor_pos=(0.0, 0.0, -0.45),
        anchor_rot=(1.0, 0.0, 0.0, 0.0),
    )

    # Temporary directory for URDF conversion
    temp_urdf_dir: str = "/tmp/g1_inspire_urdf"

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 1 / 200  # 200Hz
        self.sim.render_interval = 2

        # Convert USD to URDF and change revolute joints to fixed
        temp_urdf_output_path, temp_urdf_meshes_output_path = ControllerUtils.convert_usd_to_urdf(
            self.scene.robot.spawn.usd_path, self.temp_urdf_dir, force_conversion=True
        )

        # Set the URDF and mesh paths for the IK controller
        self.actions.upper_body_ik.controller.urdf_path = temp_urdf_output_path
        self.actions.upper_body_ik.controller.mesh_path = temp_urdf_meshes_output_path

        self.teleop_devices = DevicesCfg(
            devices={
                "handtracking": OpenXRDeviceCfg(
                    retargeters=[
                        UnitreeG1RetargeterCfg(
                            enable_visualization=True,
                            # OpenXR hand tracking has 26 joints per hand
                            num_open_xr_hand_joints=2 * 26,
                            sim_device=self.sim.device,
                            hand_joint_names=self.actions.upper_body_ik.hand_joint_names,
                        ),
                    ],
                    sim_device=self.sim.device,
                    xr_cfg=self.xr,
                ),
            }
        )

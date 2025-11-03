# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
import tempfile
import torch

import carb
from pink.tasks import FrameTask

import isaaclab.controllers.utils as ControllerUtils
import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.controllers.pink_ik import NullSpacePostureTask, PinkIKControllerCfg
from isaaclab.devices.device_base import DevicesCfg
from isaaclab.devices.openxr import ManusViveCfg, OpenXRDeviceCfg, XrCfg
from isaaclab.devices.openxr.retargeters.humanoid.unitree.inspire.g1_upper_body_retargeter import UnitreeG1RetargeterCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.actions.pink_actions_cfg import PinkInverseKinematicsActionCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.schemas.schemas_cfg import MassPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

from . import mdp

# Primary object to manipulate in this task. Used by observations and success checks.
# NOTE: The scene defines several objects (box, blue_box, red_bottle, orange_can, green_soap, yellow_chip).
# For recording demos in this task variant, we treat 'box' as the default "object".
PRIMARY_OBJECT_NAME = "box"

from isaaclab_assets.robots.unitree import G1_INSPIRE_FTP_CFG  # isort: skip


##
# Scene definition
##
@configclass
class ObjectTableSceneCfg(InteractiveSceneCfg):

    # Table
    packing_table = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-0.5, -0.3, 0.0], rot=[0.0, 0.0, 0.0, 0.0]),
        spawn=UsdFileCfg(
            usd_path="/workspace/isaaclab/source/isaaclab_assets/data/unitree_isaac/usd/g1_vla_stage/Collected_EastRural_Table/eastrural_table_1.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )
    # Manipulation objects
    box = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Box",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[-0.05, 0.3, 0.8], rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(
            usd_path="/workspace/isaaclab/source/isaaclab_assets/data/unitree_isaac/usd/g1_vla_stage/Item/box.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=0.08),
        ),
    )

    blue_box = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/BlueBox",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[-0.15, 0.5, 0.8], rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(
            usd_path="/workspace/isaaclab/source/isaaclab_assets/data/unitree_isaac/usd/g1_vla_stage/Item/blue_box.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            # mass_props=MassPropertiesCfg(mass=0.08),  # Commented out - USD file has mass defined
        ),
    )

    red_bottle = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/RedBottle",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[-0.06, 0.35, 0.8], rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(
            usd_path="/workspace/isaaclab/source/isaaclab_assets/data/unitree_isaac/usd/g1_vla_stage/Item/red_bottle.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            # mass_props=MassPropertiesCfg(mass=0.08),  # Commented out - USD file has mass defined
        ),
    )

    orange_can = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/OrangeCan",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[0.05, 0.28, 0.8], rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(
            usd_path="/workspace/isaaclab/source/isaaclab_assets/data/unitree_isaac/usd/g1_vla_stage/Item/orange_can.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            # mass_props=MassPropertiesCfg(mass=0.06),  # Commented out - USD file has mass defined
        ),
    )

    green_soap = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/GreenSoap",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[0.05, 0.46, 0.8], rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(
            usd_path="/workspace/isaaclab/source/isaaclab_assets/data/unitree_isaac/usd/g1_vla_stage/Item/green_soap.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            # mass_props=MassPropertiesCfg(mass=0.07),  # Commented out - USD file has mass defined
        ),
    )

    yellow_chip = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/YellowChip",
        init_state=RigidObjectCfg.InitialStateCfg(pos=[0.04, 0.54, 0.8], rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(
            usd_path="/workspace/isaaclab/source/isaaclab_assets/data/unitree_isaac/usd/g1_vla_stage/Item/yellow_chip.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            # mass_props=MassPropertiesCfg(mass=0.04),  # Commented out - USD file has mass defined
        ),
    )

    # Note: The target container (basket) is part of the EastRural table model

    # Humanoid robot w/ arms higher
    robot: ArticulationCfg = G1_INSPIRE_FTP_CFG.replace(
        prim_path="/World/envs/env_.*/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0, 0.8),
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
                ".*_hip_.*": 0.0,
                ".*_knee_.*": 0.0,
                ".*_ankle_.*": 0.0,
                # -- left/right hand
                ".*_thumb_.*": 0.0,
                ".*_index_.*": 0.0,
                ".*_middle_.*": 0.0,
                ".*_ring_.*": 0.0,
                ".*_pinky_.*": 0.0,
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


##
# MDP settings
##
@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    pink_ik_cfg = PinkInverseKinematicsActionCfg(
        pink_controlled_joint_names=[
            ".*_shoulder_pitch_joint",
            ".*_shoulder_roll_joint",
            ".*_shoulder_yaw_joint",
            ".*_elbow_joint",
            ".*_wrist_yaw_joint",
            ".*_wrist_roll_joint",
            ".*_wrist_pitch_joint",
        ],
        hand_joint_names=[
            # All the drive and mimic joints, total 24 joints
            "L_index_proximal_joint",
            "L_middle_proximal_joint",
            "L_pinky_proximal_joint",
            "L_ring_proximal_joint",
            "L_thumb_proximal_yaw_joint",
            "R_index_proximal_joint",
            "R_middle_proximal_joint",
            "R_pinky_proximal_joint",
            "R_ring_proximal_joint",
            "R_thumb_proximal_yaw_joint",
            "L_index_intermediate_joint",
            "L_middle_intermediate_joint",
            "L_pinky_intermediate_joint",
            "L_ring_intermediate_joint",
            "L_thumb_proximal_pitch_joint",
            "R_index_intermediate_joint",
            "R_middle_intermediate_joint",
            "R_pinky_intermediate_joint",
            "R_ring_intermediate_joint",
            "R_thumb_proximal_pitch_joint",
            "L_thumb_intermediate_joint",
            "R_thumb_intermediate_joint",
            "L_thumb_distal_joint",
            "R_thumb_distal_joint",
        ],
        target_eef_link_names={
            "left_wrist": "left_wrist_yaw_link",
            "right_wrist": "right_wrist_yaw_link",
        },
        # the robot in the sim scene we are controlling
        asset_name="robot",
        controller=PinkIKControllerCfg(
            articulation_name="robot",
            base_link_name="pelvis",
            num_hand_joints=24,
            show_ik_warnings=False,
            fail_on_joint_limit_violation=False,
            variable_input_tasks=[
                FrameTask(
                    "g1_29dof_rev_1_0_left_wrist_yaw_link",
                    position_cost=8.0,  # [cost] / [m]
                    orientation_cost=2.0,  # [cost] / [rad]
                    lm_damping=10,  # dampening for solver for step jumps
                    gain=0.5,
                ),
                FrameTask(
                    "g1_29dof_rev_1_0_right_wrist_yaw_link",
                    position_cost=8.0,  # [cost] / [m]
                    orientation_cost=2.0,  # [cost] / [rad]
                    lm_damping=10,  # dampening for solver for step jumps
                    gain=0.5,
                ),
                NullSpacePostureTask(
                    cost=0.5,
                    lm_damping=1,
                    controlled_frames=[
                        "g1_29dof_rev_1_0_left_wrist_yaw_link",
                        "g1_29dof_rev_1_0_right_wrist_yaw_link",
                    ],
                    controlled_joints=[
                        "left_shoulder_pitch_joint",
                        "left_shoulder_roll_joint",
                        "left_shoulder_yaw_joint",
                        "right_shoulder_pitch_joint",
                        "right_shoulder_roll_joint",
                        "right_shoulder_yaw_joint",
                        "waist_yaw_joint",
                        "waist_pitch_joint",
                        "waist_roll_joint",
                    ],
                    gain=0.3,
                ),
            ],
            fixed_input_tasks=[],
            xr_enabled=bool(carb.settings.get_settings().get("/app/xr/enabled")),
        ),
        enable_gravity_compensation=False,
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group with state values."""

        actions = ObsTerm(func=base_mdp.last_action)
        robot_joint_pos = ObsTerm(
            func=base_mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        robot_root_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("robot")})
        robot_root_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("robot")})
        # Primary manipulation objects observations
        # object_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("object")})
        # object_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("object")})
        box_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("box")})
        box_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("box")})
        
        # Additional objects observations
        blue_box_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("blue_box")})
        blue_box_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("blue_box")})
        red_bottle_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("red_bottle")})
        red_bottle_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("red_bottle")})
        orange_can_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("orange_can")})
        orange_can_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("orange_can")})
        green_soap_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("green_soap")})
        green_soap_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("green_soap")})
        yellow_chip_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("yellow_chip")})
        yellow_chip_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("yellow_chip")})

        # Target container is static (kinematic), no need for observations

        robot_links_state = ObsTerm(func=mdp.get_all_robot_link_state)

        left_eef_pos = ObsTerm(func=mdp.get_eef_pos, params={"link_name": "left_wrist_yaw_link"})
        left_eef_quat = ObsTerm(func=mdp.get_eef_quat, params={"link_name": "left_wrist_yaw_link"})
        right_eef_pos = ObsTerm(func=mdp.get_eef_pos, params={"link_name": "right_wrist_yaw_link"})
        right_eef_quat = ObsTerm(func=mdp.get_eef_quat, params={"link_name": "right_wrist_yaw_link"})

        hand_joint_state = ObsTerm(func=mdp.get_robot_joint_state, params={"joint_names": ["R_.*", "L_.*"]})

        # Detailed object observations for manipulation targets
        # object = ObsTerm(
        #     func=mdp.object_obs,
        #     params={
        #         "left_eef_link_name": "left_wrist_yaw_link",
        #         "right_eef_link_name": "right_wrist_yaw_link",
        #         "object_name": "object"
        #     },
        # )
        object = ObsTerm(
            func=mdp.object_obs,
            params={
                "left_eef_link_name": "left_wrist_yaw_link",
                "right_eef_link_name": "right_wrist_yaw_link",
                "object_name": PRIMARY_OBJECT_NAME,
            },
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)
    
    box_dropping = DoneTerm(
        func=base_mdp.root_height_below_minimum, params={"minimum_height": 0.5, "asset_cfg": SceneEntityCfg("box")}
    )

    # Additional objects dropping - apply same logic to all objects
    blue_box_dropping = DoneTerm(
        func=base_mdp.root_height_below_minimum, params={"minimum_height": 0.5, "asset_cfg": SceneEntityCfg("blue_box")}
    )
    
    red_bottle_dropping = DoneTerm(
        func=base_mdp.root_height_below_minimum, params={"minimum_height": 0.5, "asset_cfg": SceneEntityCfg("red_bottle")}
    )
    
    orange_can_dropping = DoneTerm(
        func=base_mdp.root_height_below_minimum, params={"minimum_height": 0.5, "asset_cfg": SceneEntityCfg("orange_can")}
    )
    
    green_soap_dropping = DoneTerm(
        func=base_mdp.root_height_below_minimum, params={"minimum_height": 0.5, "asset_cfg": SceneEntityCfg("green_soap")}
    )
    
    yellow_chip_dropping = DoneTerm(
        func=base_mdp.root_height_below_minimum, params={"minimum_height": 0.5, "asset_cfg": SceneEntityCfg("yellow_chip")}
    )

    # Success condition: object placed in container (container center at 0.5, 0.5, 0.8)
    success = DoneTerm(
        func=mdp.task_done_pick_place,
        params={
            "task_link_name": "right_wrist_yaw_link",
            "object_cfg": SceneEntityCfg(PRIMARY_OBJECT_NAME),
            # Container center at (0.5, 0.5, 0.8), allow reasonable tolerance
            "min_x": 0.35,           # Container center (0.5) - 0.15m tolerance
            "max_x": 0.65,           # Container center (0.5) + 0.15m tolerance
            "min_y": 0.35,           # Container center (0.5) - 0.15m tolerance
            "max_y": 0.65,           # Container center (0.5) + 0.15m tolerance
            "max_height": 0.95,      # Container center height (0.8) + 0.15m tolerance
            "right_wrist_max_x": 0.30,  # Hand should retract back
            "min_vel": 0.15,         # Object should be nearly stationary
        },
    )
    
    # Add a manual reset trigger for testing (uncomment to test reset)
    # manual_reset = DoneTerm(func=base_mdp.time_out, params={"time_out": 10.0})  # Reset every 10 seconds for testing


@configclass
class EventCfg:
    """Configuration for events."""

    reset_all = EventTerm(func=base_mdp.reset_scene_to_default, mode="reset")

    # Reset primary manipulation objects
    # reset_object = EventTerm(
    #     func=base_mdp.reset_root_state_uniform,
    #     mode="reset",
    #     params={
    #         "pose_range": {
    #             "x": [-0.01, 0.01],
    #             "y": [-0.01, 0.01],
    #         },
    #         "velocity_range": {},
    #         "asset_cfg": SceneEntityCfg("object"),
    #     },
    # )
    
    reset_box = EventTerm(
        func=base_mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": [-0.075, 0.075],   # ±7.5cm around initial position (-0.05)
                "y": [-0.075, 0.075],   # ±7.5cm around initial position (0.3)
                "z": [-0.02, 0.02],     # Small vertical variation
            },
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("box"),
        },
    )
    
    # Reset all additional objects within 15cm x 15cm area on table
    reset_blue_box = EventTerm(
        func=base_mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": [-0.075, 0.075],   # ±7.5cm around initial position (-0.15)
                "y": [-0.075, 0.075],   # ±7.5cm around initial position (0.5)
                "z": [-0.02, 0.02],     # Small vertical variation
                "roll": [-0.3, 0.3],    # Random roll orientation
                "pitch": [-0.3, 0.3],   # Random pitch orientation
                "yaw": [-3.14, 3.14],   # Random yaw orientation (full rotation)
            },
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("blue_box"),
        },
    )
    
    reset_red_bottle = EventTerm(
        func=base_mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": [-0.075, 0.075],   # ±7.5cm around initial position (-0.06)
                "y": [-0.075, 0.075],   # ±7.5cm around initial position (0.35)
                "z": [-0.02, 0.02],     # Small vertical variation
                "roll": [-0.2, 0.2],    # Smaller roll for bottles (more stable)
                "pitch": [-0.2, 0.2],   # Smaller pitch for bottles
                "yaw": [-3.14, 3.14],   # Full yaw rotation
            },
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("red_bottle"),
        },
    )
    
    reset_orange_can = EventTerm(
        func=base_mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": [-0.075, 0.075],   # ±7.5cm around initial position (0.05)
                "y": [-0.075, 0.075],   # ±7.5cm around initial position (0.28)
                "z": [-0.02, 0.02],     # Small vertical variation
                "roll": [-0.2, 0.2],    # Smaller roll for cans (cylindrical objects)
                "pitch": [-0.2, 0.2],   # Smaller pitch for cans
                "yaw": [-3.14, 3.14],   # Full yaw rotation
            },
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("orange_can"),
        },
    )
    
    reset_green_soap = EventTerm(
        func=base_mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": [-0.075, 0.075],   # ±7.5cm around initial position (0.05)
                "y": [-0.075, 0.075],   # ±7.5cm around initial position (0.46)
                "z": [-0.02, 0.02],     # Small vertical variation
                "roll": [-0.3, 0.3],    # Random roll orientation
                "pitch": [-0.3, 0.3],   # Random pitch orientation
                "yaw": [-3.14, 3.14],   # Full yaw rotation
            },
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("green_soap"),
        },
    )
    
    reset_yellow_chip = EventTerm(
        func=base_mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": [-0.075, 0.075],   # ±7.5cm around initial position (0.04)
                "y": [-0.075, 0.075],   # ±7.5cm around initial position (0.54)
                "z": [-0.02, 0.02],     # Small vertical variation
                "roll": [-0.4, 0.4],    # Larger roll for flat objects like chips
                "pitch": [-0.4, 0.4],   # Larger pitch for flat objects
                "yaw": [-3.14, 3.14],   # Full yaw rotation
            },
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("yellow_chip"),
        },
    )


@configclass
class PickPlaceG1InspireFTPEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the G1 environment."""

    # Scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=1, env_spacing=2.5, replicate_physics=True)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    # MDP settings
    terminations: TerminationsCfg = TerminationsCfg()
    events = EventCfg()

    # Unused managers
    commands = None
    rewards = None
    curriculum = None

    # Position of the XR anchor in the world frame
    xr: XrCfg = XrCfg(
        anchor_pos=(0.0, 0.0, 0.0),
        anchor_rot=(1.0, 0.0, 0.0, 0.0),
    )

    # Temporary directory for URDF files
    temp_urdf_dir = tempfile.gettempdir()

    # Idle action to hold robot in default pose
    # Action format: [left arm pos (3), left arm quat (4), right arm pos (3), right arm quat (4),
    #                 left hand joint pos (12), right hand joint pos (12)]
    idle_action = torch.tensor([
        # 14 hand joints for EEF control
        -0.1487,
        0.2038,
        1.0952,
        0.707,
        0.0,
        0.0,
        0.707,
        0.1487,
        0.2038,
        1.0952,
        0.707,
        0.0,
        0.0,
        0.707,
        # left hand joints (7)
        0.0,
        1.0,
        0.4,
        -0.3,
        -0.8,
        -0.5,
        -0.6,
        # left hand joints (5)
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        #right hand joints (7)
        0.0,
        -1.0,
        -0.4,
        -0.3,
        0.8,
        0.5,
        -0.6,
        # right hand joints (7)
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ])

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 6
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 1 / 120  # 120Hz
        self.sim.render_interval = 2

        # Convert USD to URDF and change revolute joints to fixed
        temp_urdf_output_path, temp_urdf_meshes_output_path = ControllerUtils.convert_usd_to_urdf(
            self.scene.robot.spawn.usd_path, self.temp_urdf_dir, force_conversion=True
        )

        # Set the URDF and mesh paths for the IK controller
        self.actions.pink_ik_cfg.controller.urdf_path = temp_urdf_output_path
        self.actions.pink_ik_cfg.controller.mesh_path = temp_urdf_meshes_output_path

        self.teleop_devices = DevicesCfg(
            devices={
                "handtracking": OpenXRDeviceCfg(
                    retargeters=[
                        UnitreeG1RetargeterCfg(
                            enable_visualization=True,
                            # number of joints in both hands
                            num_open_xr_hand_joints=2 * 26,
                            sim_device=self.sim.device,
                            # Please confirm that self.actions.pink_ik_cfg.hand_joint_names is consistent with robot.joint_names[-24:]
                            # The order of the joints does matter as it will be used for converting pink_ik actions to final control actions in IsaacLab.
                            hand_joint_names=self.actions.pink_ik_cfg.hand_joint_names,
                        ),
                    ],
                    sim_device=self.sim.device,
                    xr_cfg=self.xr,
                ),
                "manusvive": ManusViveCfg(
                    retargeters=[
                        UnitreeG1RetargeterCfg(
                            enable_visualization=True,
                            num_open_xr_hand_joints=2 * 26,
                            sim_device=self.sim.device,
                            hand_joint_names=self.actions.pink_ik_cfg.hand_joint_names,
                        ),
                    ],
                    sim_device=self.sim.device,
                    xr_cfg=self.xr,
                ),
            },
        )

# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


from isaaclab_assets.robots.unitree import G1_INSPIRE_FTP_CFG

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
import isaaclab.controllers.utils as ControllerUtils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.devices.device_base import DevicesCfg
from isaaclab.devices.openxr import OpenXRDeviceCfg, XrCfg
from isaaclab.devices.openxr.retargeters.humanoid.unitree.g1_lower_body_standing import G1LowerBodyStandingRetargeterCfg
from isaaclab.devices.openxr.retargeters.humanoid.unitree.inspire.g1_upper_body_retargeter import (
    UnitreeG1RetargeterCfg,
)
from isaaclab.envs import ManagerBasedRLEnvCfg
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

from isaaclab_tasks.manager_based.locomanipulation.pick_place import mdp as locomanip_mdp
from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.action_cfg import AgileBasedLowerBodyActionCfg
from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.agile_locomotion_observation_cfg import (
    SoloAgileTeacherPolicyObservationsCfg,
)
# from isaaclab_tasks.manager_based.locomanipulation.pick_place import mdp as locomanipu_mdp
from . import mdp

from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.pink_controller_cfg import (  # isort: skip
    G1_INSPIRE_UPPER_BODY_IK_ACTION_CFG,
)

PRIMARY_OBJECT_NAME = "box"

##
# Scene definition
##
@configclass
class LocomanipulationG1InspireSceneCfg(InteractiveSceneCfg):
    """Scene configuration for locomanipulation environment with G1 robot.

    This configuration sets up the G1 humanoid robot for locomanipulation tasks,
    allowing both locomotion and manipulation capabilities. The robot can move its
    base and use its arms for manipulation tasks.
    """

    # Table
    packing_table = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-0.5, -0.3, 0.0], rot=[0.0, 0.0, 0.0, 0.0]),
        spawn=UsdFileCfg(
            usd_path="/workspace/isaaclab/source/isaaclab_assets/data/unitree_isaac/usd/g1_vla_stage/Collected_EastRural_Table/eastrural_table_1.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
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



    # Humanoid robot w/ inspire hands
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
        soft_joint_pos_limit_factor=0.9,
        actuators={
            "legs": DCMotorCfg(
                joint_names_expr=[
                    ".*_hip_yaw_joint",
                    ".*_hip_roll_joint",
                    ".*_hip_pitch_joint",
                    ".*_knee_joint",
                ],
                effort_limit={
                    ".*_hip_yaw_joint": 88.0,
                    ".*_hip_roll_joint": 88.0,
                    ".*_hip_pitch_joint": 88.0,
                    ".*_knee_joint": 139.0,
                },
                velocity_limit={
                    ".*_hip_yaw_joint": 32.0,
                    ".*_hip_roll_joint": 32.0,
                    ".*_hip_pitch_joint": 32.0,
                    ".*_knee_joint": 20.0,
                },
                stiffness={
                    ".*_hip_yaw_joint": 100.0,
                    ".*_hip_roll_joint": 100.0,
                    ".*_hip_pitch_joint": 100.0,
                    ".*_knee_joint": 200.0,
                },
                damping={
                    ".*_hip_yaw_joint": 2.5,
                    ".*_hip_roll_joint": 2.5,
                    ".*_hip_pitch_joint": 2.5,
                    ".*_knee_joint": 5.0,
                },
                armature={
                    ".*_hip_.*": 0.03,
                    ".*_knee_joint": 0.03,
                },
                saturation_effort=180.0,
            ),
            "feet": DCMotorCfg(
                joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
                stiffness={
                    ".*_ankle_pitch_joint": 20.0,  # 恢復標準值以改善平衡（之前過高的 100.0 導致不穩定）
                    ".*_ankle_roll_joint": 20.0,    # 恢復標準值以改善平衡（之前過高的 100.0 導致不穩定）
                },
                damping={
                    ".*_ankle_pitch_joint": 0.2,  # 恢復標準值以改善平衡（之前過高的 2.5 導致不穩定）
                    ".*_ankle_roll_joint": 0.1,    # 恢復標準值以改善平衡（之前過高的 2.5 導致不穩定）
                },
                effort_limit={
                    ".*_ankle_pitch_joint": 50.0,
                    ".*_ankle_roll_joint": 50.0,
                },
                velocity_limit={
                    ".*_ankle_pitch_joint": 37.0,
                    ".*_ankle_roll_joint": 37.0,
                },
                armature=0.03,
                saturation_effort=80.0,
            ),
            "waist": ImplicitActuatorCfg(
                joint_names_expr=[
                    "waist_.*_joint",
                ],
                effort_limit={
                    "waist_yaw_joint": 88.0,
                    "waist_roll_joint": 50.0,
                    "waist_pitch_joint": 50.0,
                },
                velocity_limit={
                    "waist_yaw_joint": 32.0,
                    "waist_roll_joint": 37.0,
                    "waist_pitch_joint": 37.0,
                },
                stiffness={
                    "waist_yaw_joint": 5000.0,
                    "waist_roll_joint": 5000.0,
                    "waist_pitch_joint": 5000.0,
                },
                damping={
                    "waist_yaw_joint": 5.0,
                    "waist_roll_joint": 5.0,
                    "waist_pitch_joint": 5.0,
                },
                armature=0.001,
            ),
            "arms": ImplicitActuatorCfg(
                joint_names_expr=[
                    ".*_shoulder_pitch_joint",
                    ".*_shoulder_roll_joint",
                    ".*_shoulder_yaw_joint",
                    ".*_elbow_joint",
                    ".*_wrist_.*_joint",
                ],
                effort_limit=300,
                velocity_limit=100,
                stiffness=3000.0,
                damping=100.0,
                armature={
                    ".*_shoulder_.*": 0.001,
                    ".*_elbow_.*": 0.001,
                    ".*_wrist_.*_joint": 0.001,
                },
            ),
            "hands": ImplicitActuatorCfg(
                joint_names_expr=[
                    ".*_index_.*",
                    ".*_middle_.*",
                    ".*_thumb_.*",
                    ".*_ring_.*",
                    ".*_pinky_.*",
                ],
                effort_limit_sim=300.0,
                velocity_limit_sim=100.0,
                stiffness=20.0,
                damping=2,
                armature=0.001,
            ),
        },
        #prim_path="/World/envs/env_.*/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.75),  # 與 G1_29DOF_CFG 相同
            rot=(0.7071, 0, 0, 0.7071),
            joint_pos={
                # right-arm
                # "right_shoulder_pitch_joint": 0.0,
                # "right_shoulder_roll_joint": -1.0,
                # "right_shoulder_yaw_joint": -0.4,
                # "right_elbow_joint": -0.3,
                # "right_wrist_yaw_joint": 0.8,
                # "right_wrist_roll_joint": 0.5,
                # "right_wrist_pitch_joint": -0.6,
                # # left-arm
                # "left_shoulder_pitch_joint": 0.0,
                # "left_shoulder_roll_joint": 1.0,
                # "left_shoulder_yaw_joint": 0.4,
                # "left_elbow_joint": -0.3,
                # "left_wrist_yaw_joint": -0.8,
                # "left_wrist_roll_joint": -0.5,
                # "left_wrist_pitch_joint": -0.6,
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


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    upper_body_ik = G1_INSPIRE_UPPER_BODY_IK_ACTION_CFG

    lower_body_joint_pos = AgileBasedLowerBodyActionCfg(
        asset_name="robot",
        joint_names=[
            # Explicit joint names for lower body (12 joints total)
            # Hip joints (6 joints: left/right x yaw/roll/pitch)
            "left_hip_pitch_joint",
            "right_hip_pitch_joint",
            "left_hip_roll_joint",
            "right_hip_roll_joint",
            "left_hip_yaw_joint",
            "right_hip_yaw_joint",
            # Knee joints (2 joints)
            "left_knee_joint",
            "right_knee_joint",
            # Ankle joints (4 joints: left/right x pitch/roll)
            "left_ankle_pitch_joint",
            "right_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_ankle_roll_joint",
        ],
        policy_output_scale=0.25,  
        preserve_order=True,  
        obs_group_name="lower_body_policy",  # need to be the same name as the on in ObservationCfg
        policy_path=f"{ISAACLAB_NUCLEUS_DIR}/Policies/Agile/agile_locomotion.pt",
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP.
    This class is required by the environment configuration but not used in this implementation
    """

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
        # object_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("object")})
        # object_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("object")})
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

        robot_links_state = ObsTerm(func=mdp.get_all_robot_link_state)

        left_eef_pos = ObsTerm(func=mdp.get_eef_pos, params={"link_name": "left_wrist_yaw_link"})
        left_eef_quat = ObsTerm(func=mdp.get_eef_quat, params={"link_name": "left_wrist_yaw_link"})
        right_eef_pos = ObsTerm(func=mdp.get_eef_pos, params={"link_name": "right_wrist_yaw_link"})
        right_eef_quat = ObsTerm(func=mdp.get_eef_quat, params={"link_name": "right_wrist_yaw_link"})

        hand_joint_state = ObsTerm(func=mdp.get_robot_joint_state, params={"joint_names": [".*_thumb_.*", ".*_index_.*", ".*_middle_.*", ".*_ring_.*", ".*_pinky_.*"]})

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
    lower_body_policy: SoloAgileTeacherPolicyObservationsCfg = SoloAgileTeacherPolicyObservationsCfg()


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
        func=mdp.task_done_pick_lift,
        params={
            "task_link_name": "right_wrist_yaw_link",
            "object_cfg": SceneEntityCfg(PRIMARY_OBJECT_NAME),
        },
    )




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

##
# MDP settings
##


@configclass
class LocomanipulationG1InspireEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the G1 locomanipulation environment.

    This environment is designed for locomanipulation tasks where the G1 humanoid robot
    can perform both locomotion and manipulation simultaneously. The robot can move its
    base and use its arms for manipulation tasks, enabling complex mobile manipulation
    behaviors.
    """

    # Scene settings
    scene: LocomanipulationG1InspireSceneCfg = LocomanipulationG1InspireSceneCfg(num_envs=1, env_spacing=2.5, replicate_physics=True)
    # MDP settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands = None
    terminations: TerminationsCfg = TerminationsCfg()

    # Unused managers
    rewards = None
    curriculum = None

    # Position of the XR anchor in the world frame
    xr: XrCfg = XrCfg(
        anchor_pos=(0.0, 0.0, -0.35),
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
                            # OpenXR hand tracking has 26 joints per hand (52 total)
                            num_open_xr_hand_joints=2 * 26,
                            sim_device=self.sim.device,
                            hand_joint_names=self.actions.upper_body_ik.hand_joint_names,
                        ),
                        G1LowerBodyStandingRetargeterCfg(
                            sim_device=self.sim.device,
                        ),
                    ],
                    sim_device=self.sim.device,
                    xr_cfg=self.xr,
                ),
            }
        )
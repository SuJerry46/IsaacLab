# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Disney Research robots.

The following configuration parameters are available:

* :obj:`BDX_CFG`: The BD-X robot with implicit Actuator model

Reference:

* https://github.com/rimim/AWD/tree/main/awd/data/assets/go_bdx

"""

from isaaclab_assets import ISAACLAB_ASSETS_DATA_DIR

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
# from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

Qmini_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Disney/BDX/BDX.usd",
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/q1/q1.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.45),
        joint_pos={
            
            # Left leg joints 
            "hip_yaw_l": 0.35,      #-0.17 → 0.61rad  -10  →  35 deg
            "hip_roll_l": 0.0,      #-0.26 → 0.52rad  -15  →  30 deg
            "hip_pitch_l": -1.75,   #-2.09 → 0.00rad  -120 →   0 deg
            "knee_pitch_l": 1.05,   # 0.00 → 2.09rad     0 → 120 deg
            "ankle_pitch_l": -1.05, #-2.44 → 0.00rad  -140 →   0 deg
            # Right leg joints 
            "hip_yaw_r": -0.35,     #-0.61 → 0.17rad  -35 →   10 deg
            "hip_roll_r": 0.0,      #-0.52 → 0.26rad  -30 →   15 deg
            "hip_pitch_r": 1.75,    # 0.00 → 2.09rad    0 →  120 deg
            "knee_pitch_r": -1.05,  #-2.09 → 0.00rad -120 →    0 deg
            "ankle_pitch_r": 1.05,  # 0.00 → 2.44rad    0 →  140 deg
        },
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=["hip_yaw_.*", "hip_roll_.*", "hip_pitch_.*", "knee_pitch_.*", "ankle_pitch_.*"],
            stiffness={
                "hip_yaw_.*": 100.0,
                "hip_roll_.*": 80.0,
                "hip_pitch_.*": 120.0,
                "knee_pitch_.*": 200.0,
                "ankle_pitch_.*": 200.0,
            },
            damping={
                "hip_yaw_.*": 3.0,
                "hip_roll_.*": 3.0,
                "hip_pitch_.*": 6.0,
                "knee_pitch_.*": 6.0,
                "ankle_pitch_.*": 6.0,
            },
        ),

    },
    soft_joint_pos_limit_factor=0.65,
)
"""Configuration for the Disney BD-X robot with implicit actuator model."""

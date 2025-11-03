# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import numpy as np
from isaacsim.core.api.controllers.articulation_controller import ArticulationController
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera

from unitree_extension_g1_python.modules.fastapi.websocket_server import (
    WebsocketServer,
)
from unitree_extension_g1_python.modules.omni_graph.camera import CameraGraph
from unitree_extension_g1_python.modules.ros2.jointstate_node import Ros2Interface
from pxr import Gf
import random
import math
import numpy as np
from .item_random import ResetItem
import time

def set_camera_intrinsics(
    camera: Camera,
    resolution,
    camera_intrinsics,
    clipping_range=(0.05, 1.0e5),
    pixel_size=3,
    f_stop=2.0,
    focus_distance=0.6,
):
    """
    Sets the camera intrinsics and related parameters for the camera object.
    Args:
        resolution (tuple): The (width, height) resolution of the camera in pixels.
        pixel_size (float, optional): The size of a single pixel in microns.
        f_stop (float, optional): The f-number (aperture) of the camera lens.
        foucus_distance (float, optional): The distance from the camera to the object plane in meters.
        camera_intrinsics (any): The camera intrinsic matrix or parameters, typically containing focal lengths and principal point.
            Used to extract fx, fy, cx, cy for further calculations.
    Notes:
        - The function calculates the focal length and aperture size based on the camera matrix and pixel size.
        - It sets the camera's focal length, focus distance, lens aperture, horizontal and vertical apertures, and clipping range.
    """
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()

    width, height = resolution

    # Calculate the focal length and aperture size from the camera matrix
    ((fx, _, cx), (_, fy, cy), (_, _, _)) = camera_intrinsics
    # convert pixel size to mm, then multiply by width of image in pixels
    horizontal_aperture = width * pixel_size * 1e-3
    # convert pixel size to mm, then multiply by height of image in pixels
    vertical_aperture = height * pixel_size * 1e-3
    # multiply pixel count by pixel size in mm
    focal_length_x = fx * pixel_size * 1e-3
    # multiply pixel count by pixel size in mm
    focal_length_y = fy * pixel_size * 1e-3

    focal_length = (focal_length_x + focal_length_y) / 2  # in mm

    camera.set_focal_length(focal_length)
    camera.set_focus_distance(focus_distance)
    camera.set_lens_aperture(f_stop * 1e3)
    camera.set_horizontal_aperture(horizontal_aperture)
    camera.set_vertical_aperture(vertical_aperture)
    camera.set_clipping_range(*clipping_range)


class UnitreeG1Script:
    def __init__(self):
        self._articulation = None
        self.robot_prim_path = "/World/unitree_g1"
        self.head_camera_prim = "/World/unitree_g1/torso_link/d435_link/Camera"
        self.right_hand_camera_prim = "/World/unitree_g1/right_wrist_yaw_link/right_rubber_hand/R_hand_base_link/right_camera_stand_link/r_hand_camera"
        self.left_hand_camera_prim = "/World/unitree_g1/left_wrist_yaw_link/left_rubber_hand/L_hand_base_link/left_camera_stand_link/l_hand_camera"
        self.api_interface = WebsocketServer()
        self.ros2_interface = Ros2Interface()
        self.reset_class = ResetItem()
        self.reset_item = True
        self.stage_mode_set = False
        self.reset_arm_done = False
        self.head_camera_resolution = (640, 480)
        self.head_camera_frequency = 30
        self.head_camera_martix = [
            [607.621, 0.0, 327.374],
            [0.0, 607.415, 239.772],
            [0.0, 0.0, 1.0],
        ]

        self.right_hand_camera_resolution = (640, 480)
        self.right_hand_camera_frequency = 30
        self.right_hand_camera_martix = [
            [431.638, 0.0, 316.702],
            [0.0, 431.098, 241.933],
            [0.0, 0.0, 1.0],
        ]

        self.left_hand_camera_resolution = (640, 480)
        self.left_hand_camera_frequency = 30
        self.left_hand_camera_martix = [
            [431.63, 0.0, 335.288],
            [0.0, 431.076, 242.212],
            [0.0, 0.0, 1.0],
        ]

        self.flashing = False

    def enable_apiserver(self):
        self.api_interface.enable_server()

    def disable_apiserver(self):
        if self.api_interface.is_running():
            self.api_interface.disable_server()

    def enable_ros_communication(self):
        print("[Unitree-G1 Extension] ROS2 communication enabled!")
        self.gen_action_graph()
        self.ros2_interface.start_ros2_spin_thread()

    def diable_ros_communication(self):
        if self.ros2_interface.is_running():
            print("[Unitree-G1 Extension] ROS2 communication disabled!")
            self.ros2_interface.stop_ros2_spin_thread()

    def enable_articulation_controller(self):
        print("[Unitree-G1 Extension] Articulation controller enabled!")

        self._articulation = Articulation(
            prim_paths_expr=self.robot_prim_path,
            name="unitree_g1_view",
            positions=np.array([[0, 0, 0.79184]]),
        )

        self.articulation_controller = ArticulationController()
        self.articulation_controller.initialize(self._articulation)

        self._head_camera = Camera(
            prim_path=self.head_camera_prim,
            frequency=self.head_camera_frequency,
            resolution=self.head_camera_resolution,
            # position=np.array([0.0, 0.0, 25.0]),
            # translation=np.array([0.0, 0.0, 0.0]),
            # orientation=rot_utils.euler_angles_to_quats(
            #     np.array([0, 90, 0]), degrees=True
            # ),
        )

        set_camera_intrinsics(
            camera=self._head_camera,
            resolution=self.head_camera_resolution,
            camera_intrinsics=self.head_camera_martix,
        )

        self._right_hand_camera = Camera(
            prim_path=self.right_hand_camera_prim,
            frequency=self.right_hand_camera_frequency,
            resolution=self.right_hand_camera_resolution,
        )
        set_camera_intrinsics(
            camera=self._right_hand_camera,
            resolution=self.right_hand_camera_resolution,
            camera_intrinsics=self.right_hand_camera_martix,
        )

        self._left_hand_camera = Camera(
            prim_path=self.left_hand_camera_prim,
            frequency=self.left_hand_camera_frequency,
            resolution=self.left_hand_camera_resolution,
        )
        set_camera_intrinsics(
            camera=self._left_hand_camera,
            resolution=self.left_hand_camera_resolution,
            camera_intrinsics=self.left_hand_camera_martix,
        )

    def diable_articulation_controller(self):
        if self._articulation is not None:
            print("[Unitree-G1 Extension] Articulation controller disabled!")
            self.articulation_controller = None
            self._articulation = None

    def stage_mode_enable(self):
        self.stage_mode_set = True
        self.reset_arm()
        self.reset_class.stage_reset_mode()

    def stage_mode_disable(self):
        self.stage_mode_set = False

    def item_random(self):
        self.reset_arm()
        self.reset_class.custom_reset()

    def reset_arm(self):
        self._articulation_control(
            joint_name=[
                "right_shoulder_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_wrist_pitch_joint",
                "right_wrist_roll_joint",
                "right_wrist_yaw_joint",
                "right_elbow_joint",
                "left_shoulder_pitch_joint",
                "left_shoulder_roll_joint",
                "left_shoulder_yaw_joint",
                "left_wrist_pitch_joint",
                "left_wrist_roll_joint",
                "left_wrist_yaw_joint",
                "left_elbow_joint",
                "L_pinky_proximal_joint",
                "L_ring_proximal_joint",
                "L_middle_proximal_joint",
                "L_index_proximal_joint",
                "L_thumb_proximal_pitch_joint",
                "L_thumb_proximal_yaw_joint",
                "R_pinky_proximal_joint",
                "R_ring_proximal_joint",
                "R_middle_proximal_joint",
                "R_index_proximal_joint",
                "R_thumb_proximal_pitch_joint",
                "R_thumb_proximal_yaw_joint",],
            joint_position=[0.0] * 26,
            joint_velocity=None,
            joint_effort=None,
        )
    """
    The following two functions demonstrate the mechanics of running code in a script-like way
    from a UI-based extension.  This takes advantage of Python's yield/generator framework.  

    The update() function is tied to a physics subscription, which means that it will be called
    one time on every physics step (usually 60 frames per second).  Each time it is called, it
    queries the script generator using next().  This makes the script generator execute until it hits
    a yield().  In this case, no value need be yielded.  This behavior can be nested into subroutines
    using the "yield from" keywords.
    """

    def update(self, step: float):
        if self._articulation is None:
            print("Please enable the articulation controller!")
            return

        if self.ros2_interface.is_running():
            self._ros2_control()

        if self.api_interface.is_running():
            self._api_control()

        self.end_time = time.time()
        if not self.stage_mode_set:
            if self.reset_item:
                self.faild_time = time.time()
            else:
                if self.end_time - self.start_time > 5 and not self.reset_arm_done:
                    self.reset_arm()
                    print("reset arm")
                    self.reset_arm_done = True

                if self.end_time - self.start_time > 6 and self.reset_arm_done:
                    print("item reset")
                    self.reset_class.custom_reset()
                    self.reset_arm_done = False
                    self.reset_item = True

                # print("time:", self.end_time - self.faild_time)
                if self.end_time - self.faild_time > 50:
                    self.reset_class.stop_infer()

    def _ros2_control(self):
        # Send the joint states to the ROS
        current_name, current_position, current_velocity, current_efforts = (
            self._get_joint_feedback()
        )
        self.ros2_interface.node.pub_current_joint(
            joint_name=current_name,
            joint_position=current_position,
            joint_velocity=current_velocity,
            joint_effort=current_efforts,
        )

        # Receive the joint states command from the ROS
        joint_name = self.ros2_interface.node.get_joint_name()  # shape (29,)
        joint_position = self.ros2_interface.node.get_joint_position()
        joint_velocity = self.ros2_interface.node.get_joint_velocity()
        joint_effort = self.ros2_interface.node.get_joint_effort()

        if joint_position is None or joint_name is None:
            print("No joint information received from the ROS")
            return

        self._articulation_control(
            joint_name=joint_name,
            joint_position=joint_position,
            joint_velocity=joint_velocity,
            joint_effort=joint_effort,
        )

    def _api_control(self):
        # Send the joint states to the API server
        current_name, current_position, current_velocity, current_effort = (
            self._get_joint_feedback()
        )
        self.api_interface.current_jointstates.set_name(current_name)
        self.api_interface.current_jointstates.set_position(current_position)
        self.api_interface.current_jointstates.set_velocity(current_velocity)
        self.api_interface.current_jointstates.set_effort(current_effort)
        self.api_interface.current_jointstates.new_data_event.set()

        # Send the image to the API server
        rgb_image = self._head_camera.get_rgb()
        depth_image = self._head_camera.get_depth()
        self.api_interface.image.set_rgb(rgb_image)
        self.api_interface.image.set_depth(depth_image)
        self.api_interface.image.new_data_event.set()

        right_hand_rgb_image = self._right_hand_camera.get_rgb()
        right_hand_depth_image = self._right_hand_camera.get_depth()
        self.api_interface.right_hand_image.set_rgb(right_hand_rgb_image)
        self.api_interface.right_hand_image.set_depth(right_hand_depth_image)
        self.api_interface.right_hand_image.new_data_event.set()

        left_hand_depth_image = self._left_hand_camera.get_depth()
        left_hand_rgb_image = self._left_hand_camera.get_rgb()
        self.api_interface.left_hand_image.set_rgb(left_hand_rgb_image)
        self.api_interface.left_hand_image.set_depth(left_hand_depth_image)
        self.api_interface.left_hand_image.new_data_event.set()

        # Receive the action(joint states) from the API server
        if self.api_interface.action_jointstates.new_data_event.is_set():
            self.api_interface.action_jointstates.new_data_event.clear()

            action_name = self.api_interface.action_jointstates.get_name()
            action_position = self.api_interface.action_jointstates.get_position()
            action_velocity = self.api_interface.action_jointstates.get_velocity()
            action_effort = self.api_interface.action_jointstates.get_effort()

            if action_name is None or action_position is None:
                print("No joint information received from the API server")
                return

            self._articulation_control(
                joint_name=action_name,
                joint_position=action_position,
                joint_velocity=action_velocity,
                joint_effort=action_effort,
            )

    def _articulation_control(
        self, joint_name, joint_position, joint_velocity=None, joint_effort=None
    ):
        if not (
            len(joint_name) == len(joint_position)
            and (joint_velocity is None or len(joint_name) == len(joint_velocity))
            and (joint_effort is None or len(joint_name) == len(joint_effort))
        ):
            raise ValueError(
                "Input lengths are inconsistent: "
                f"joint_name({len(joint_name)}), "
                f"joint_position({len(joint_position)}), "
                f"joint_velocity({len(joint_velocity) if joint_velocity else 'None'}), "
                f"joint_effort({len(joint_effort) if joint_effort else 'None'})"
            )

        joint_index = []
        for name in joint_name:
            cleaned_name = name.replace("robot0/", "")
            index = self._articulation.get_dof_index(cleaned_name)
            joint_index.append(index)

        action = ArticulationAction(
            joint_positions=joint_position,
            joint_velocities=joint_velocity,
            joint_efforts=joint_effort,
            joint_indices=joint_index,
        )
        self.articulation_controller.apply_action(action)

        self.start_time = time.time()
        self.reset_class.post = True
        self.reset_item = False

    def _get_joint_feedback(self):
        joint_name = self._articulation.dof_names
        joint_positions = self._articulation.get_joint_positions()
        joint_velocitys = self._articulation.get_joint_velocities()
        joint_efforts = self._articulation.get_measured_joint_efforts()

        return joint_name, joint_positions, joint_velocitys, joint_efforts

    def gen_action_graph(self):
        self.head_camera_graph = CameraGraph(
            graph_name="/HeadCamera",
            camera_prim=self.head_camera_prim,
            resolution=self.head_camera_resolution,
            rgb_topic_name="/isaac_cam/head_camera/rgb",
            depth_topic_name="/isaac_cam/head_camera/depth",
        )

        self.right_hand_camera_graph = CameraGraph(
            graph_name="/RightHandCamera",
            camera_prim=self.right_hand_camera_prim,
            resolution=self.right_hand_camera_resolution,
            rgb_topic_name="/isaac_cam/right_hand_camera/rgb",
            depth_topic_name="/isaac_cam/right_hand_camera/depth",
        )

        self.left_hand_camera_graph = CameraGraph(
            graph_name="/LeftHandCamera",
            camera_prim=self.left_hand_camera_prim,
            resolution=self.left_hand_camera_resolution,
            rgb_topic_name="/isaac_cam/left_hand_camera/rgb",
            depth_topic_name="/isaac_cam/left_hand_camera/depth",
        )

        self.head_camera_graph.gen_camera_graph()
        self.right_hand_camera_graph.gen_camera_graph()
        self.left_hand_camera_graph.gen_camera_graph()

    def delete_action_graph(self):
        self.head_camera_graph.delete_camera_graph()
        self.right_hand_camera_graph.delete_camera_graph()
        self.left_hand_camera_graph.delete_camera_graph()
        del (
            self.head_camera_graph,
            self.right_hand_camera_graph,
            self.left_hand_camera_graph,
        )

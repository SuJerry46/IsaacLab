import random
import isaacsim.core.utils.prims as prims_utils
from scipy.spatial.transform import Rotation as R
from pxr import Gf
import numpy as np
import requests
import yaml


class ResetItem:
    def __init__(self):
        self.post = True
        self.item_name = [
            "OREO",
            "PRINGLES53g",
            "DULOXETINE",
            "IBL_handwash",
            "a8congee",
            "Taivision_anti_blue_light"
        ]
        self.r_items = [
            "OREO",
            "a8congee",
            "Taivision_anti_blue_light",
            "PRINGLES53g",
        ]
        self.l_items = [
            "DULOXETINE",
            "OREO",
            "IBL_handwash",
            "a8congee",
            "PRINGLES53g",
            "Taivision_anti_blue_light",
        ]
        self._items_hight = {
            "OREO": 0.88,
            "PRINGLES53g": 0.9,
            "DULOXETINE": 0.85,
            "IBL_handwash": 0.88,
            "a8congee": 0.85,
            "Taivision_anti_blue_light": 0.88
        }
        self.waitarea = {
            "pose0":{
                "translate": (0.15, -0.7, 1.0),
                "rotation": (1.0, 0.0, 0.0, 0.0)
            },
            "pose1":{
                "translate": (0.15, -0.6, 1.0),
                "rotation": (1.0, 0.0, 0.0, 0.0)
            },
            "pose2":{
                "translate": (0.15, -0.5, 1.0),
                "rotation": (1.0, 0.0, 0.0, 0.0)
            },
            "pose3":{
                "translate": (0.15, 0.5, 1.0),
                "rotation": (1.0, 0.0, 0.0, 0.0)
            },
            "pose4":{
                "translate": (0.15, 0.6, 1.0),
                "rotation": (1.0, 0.0, 0.0, 0.0)
            },
            "pose5":{
                "translate": (0.15, 0.7, 1.0),
                "rotation": (1.0, 0.0, 0.0, 0.0)
            }
        }

    def set_item_pose(self, item_prim_path, position, orientation):
        """
        Set the position and orientation of an item in the scene.
        Args:
            item_prim_path (str): The path to the item prim.
            position (list or np.array): The position to set.
            orientation (list or np.array): The orientation to set.
        """
        prims_utils.set_prim_attribute_value(
            prim_path="/World/" + item_prim_path,
            attribute_name="xformOp:translate",
            value=Gf.Vec3d(*position),
        )
        a = random.uniform(0, 1)
        if a < 0.5:
            orientation = np.array([1.0, 0.0, 0.0, 0.0])
            if item_prim_path == "PRINGLES53g":
                orientation = np.array([0.0, 0.0, 1.0, 0.0])
        prims_utils.set_prim_attribute_value(
            prim_path="/World/" + item_prim_path,
            attribute_name="xformOp:orient",
            value=orientation,
        )

    def is_prim_active(self, prim_path):
        prim = prims_utils.get_prim_at_path("/World/" + prim_path)
        return prim and prim.IsActive()

    def _reset(self):
        # init_item = random.sample(self.item_name, 2)
        right_item = random.sample(self.r_items, 1)
        left_item = random.sample(self.l_items, 1)
        while right_item[0] == left_item[0] or (left_item[0] == "PRINGLES53g" and right_item[0] == "a8congee") or (left_item[0] == "a8congee" and right_item[0] == "PRINGLES53g"):
            left_item = random.sample(self.l_items, 1)
            

        x = random.uniform(0.35, 0.45)
        y = 0.05
        z = 0.9
        # print(f"left_item: {left_item}, right_item: {right_item}")
        # for i in init_item:
        if left_item[0] == "DULOXETINE":
            z = 0.85
        if right_item[0] == "PRINGLES53g":
            z = 0.95
        left_coordinate = [x, y, z]
        x = random.uniform(0.35, 0.45)
        right_coordinate = [x, -y, z]

        orient = []
        for _ in range(2):
            z = random.uniform(-45, 45)
            if left_item[0] == "PRINGLES53g" or right_item[0] == "PRINGLES53g":
                x = 180.0
            else:
                x = 0.0
            r = R.from_euler('xyz', [x, 0.0, z], degrees=True)
            quat = r.as_quat()
            quat_wxyz = [quat[3], quat[0], quat[1], quat[2]]
            # u1 = np.random.rand()
            # u2 = np.random.rand()
            # u3 = np.random.rand()

            # q1 = np.sqrt(1 - u1) * np.sin(2 * np.pi * u2)
            # q2 = np.sqrt(1 - u1) * np.cos(2 * np.pi * u2)
            # q3 = np.sqrt(u1) * np.sin(2 * np.pi * u3)
            # q4 = np.sqrt(u1) * np.cos(2 * np.pi * u3)
            # orient.append([q1, q2, q3, q4])
            orient.append(quat_wxyz)
        self.set_item_pose(left_item[0], left_coordinate, orient[0])
        self.set_item_pose(right_item[0], right_coordinate, orient[1])

    def clean_stage(self):
        for i, item in enumerate(self.item_name):
            # print(f"Resetting item: {item}")
            if self.is_prim_active(item):
                self.set_item_pose(
                    item,
                    self.waitarea[f"pose{i}"]["translate"],
                    self.waitarea[f"pose{i}"]["rotation"]
                )

    def custom_reset(self):
        self.clean_stage()
        self._reset()

    def stage_reset_mode(self):
        self.clean_stage()
        self.stage_reset()

    def stage_reset(self):
        x = random.uniform(0.35, 0.45)
        y = 0.05
        z = 0.9
        left_coordinate = [x, y, z]
        x = random.uniform(0.35, 0.45)
        right_coordinate = [x, -y, z]

        orient = []
        for i in range(2):
            z = random.uniform(-45, 45)
            if i == 0:
                x = 180.0
            else:
                x = 0.0
            r = R.from_euler('xyz', [x, 0.0, z], degrees=True)
            quat = r.as_quat()
            quat_wxyz = [quat[3], quat[0], quat[1], quat[2]]
            orient.append(quat_wxyz)
        self.set_item_pose("PRINGLES53g", left_coordinate, orient[0])
        self.set_item_pose("OREO", right_coordinate, orient[1])

    def stop_infer(self):

        if self.post:
            server_address = "192.168.50.19:9527"
            url = f"http://{server_address}/action/stop"
            headers = {
                "accept": "application/json",
            }
            data = {}
            try:
                response = requests.post(url, json=data, headers=headers)
                response.raise_for_status()
                self.post = False
                return response.json()
            except Exception as e:
                print(f"Error triggering FastAPI action: {e}")

                return f"Error triggering FastAPI action: {e}"

if __name__ == "__main__":
    reset_item = ResetItem()
    reset_item.clean_stage()  # Example usage

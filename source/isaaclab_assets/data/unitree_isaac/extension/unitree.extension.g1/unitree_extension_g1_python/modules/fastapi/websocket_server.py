import asyncio
import base64
import threading

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket


class RGBImage:
    def __init__(self):
        self.lock = threading.Lock()
        self.new_data_event = threading.Event()
        self.RGB = None
        self.depth = None

    def set_rgb(self, rgb):
        with self.lock:
            self.RGB = np.array(rgb).astype(np.uint8)

    def set_depth(self, depth):
        if depth is None:
            return

        with self.lock:
            self.depth = np.array(depth).astype(np.uint8)

    def to_json(self):
        with self.lock:
            if self.RGB is None or self.depth is None:
                print("RGB data is not set.")
                return {"rgb_image": None, "depth_image": None}

            _, rgb_buffer = cv2.imencode(".jpg", self.RGB)
            base64_rgb_image = base64.b64encode(rgb_buffer).decode("utf-8")

            depth_normalized = cv2.normalize(
                self.depth, None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)
            _, depth_buffer = cv2.imencode(".jpg", depth_normalized)
            base64_depth_image = base64.b64encode(depth_buffer).decode("utf-8")

            json = {"rgb_image": base64_rgb_image, "depth_image": base64_depth_image}
            return json


class JointData:
    def __init__(self):
        self.name = None
        self.position = None
        self.velocity = None
        self.effort = None
        self.lock = threading.Lock()
        self.new_data_event = threading.Event()

    def set_name(self, names):
        with self.lock:
            self.name = list(names)

    def set_position(self, position):
        with self.lock:
            self.position = self._check_format(position)

    def set_velocity(self, velocity):
        with self.lock:
            self.velocity = self._check_format(velocity)

    def set_effort(self, effort):
        with self.lock:
            self.effort = self._check_format(effort)

    def get_name(self):
        with self.lock:
            if self.name is None:
                print("Joint names have not been set.")
            return self.name

    def get_position(self):
        with self.lock:
            if self.position is None:
                print("Joint states have not been set.")
            return self.position

    def get_velocity(self):
        with self.lock:
            if self.velocity is None:
                print("Joint velocity have not been set.")
            return self.velocity

    def get_effort(self):
        with self.lock:
            if self.effort is None:
                print("Joint effort have not been set.")
            return self.effort

    def to_json(self):
        with self.lock:
            if self.name is None or self.position is None:
                print("Joint names or states are not set.")
                return {
                    "name": None,
                    "position": None,
                    "velocity": None,
                    "effort": None,
                }

            json = {
                "name": self.name,
                "position": self.position,
                "velocity": self.velocity,
                "effort": self.effort,
            }
            return json

    def _check_format(self, states):
        if states is None:
            return None

        if isinstance(states, np.ndarray):
            if states.ndim > 1:
                formatted_states = states.flatten().tolist()
            else:
                formatted_states = list(states)
        else:
            formatted_states = states

        formatted_states = [float(x) for x in formatted_states]
        return formatted_states


class WebsocketServer:
    def __init__(self):
        self.app = FastAPI()
        self.host = "0.0.0.0"
        self.port = 8787

        self.server_thread = None
        self.current_jointstates = JointData()
        self.action_jointstates = JointData()
        self.image = RGBImage()
        self.right_hand_image = RGBImage()
        self.left_hand_image = RGBImage()

        self.app.add_api_websocket_route("/isaac/ws/joints/action", self.receive_joint)
        self.app.add_api_websocket_route("/isaac/ws/joints/current", self.send_joints)
        # self.app.add_api_websocket_route("/ws/image", self.send_image)

        self.app.add_api_websocket_route(
            "/isaac/ws/images/head", self.make_send_image_handler(self.image)
        )
        self.app.add_api_websocket_route(
            "/isaac/ws/images/right-hand",
            self.make_send_image_handler(self.right_hand_image),
        )
        self.app.add_api_websocket_route(
            "/isaac/ws/images/left-hand",
            self.make_send_image_handler(self.left_hand_image),
        )

    async def receive_joint(self, websocket: WebSocket):
        await websocket.accept()
        print("WebSocket connection established for receiving.")
        try:
            while True:
                data = await websocket.receive_json()
                self.action_jointstates.set_name(data["name"])
                self.action_jointstates.set_position(data["position"])
                self.action_jointstates.set_velocity(data["velocity"])
                self.action_jointstates.set_effort(data["effort"])
                self.action_jointstates.new_data_event.set()
                # print("Received data:", data)
        except Exception as e:
            print("WebSocket connection closed (action joint):", e)
        # finally:
        #     await websocket.close()

    def make_send_image_handler(self, img_obj):
        async def send_image(websocket: WebSocket):
            await websocket.accept()
            print("WebSocket connection established for sending image.")
            try:
                while True:
                    await asyncio.get_event_loop().run_in_executor(
                        None, img_obj.new_data_event.wait
                    )
                    img_obj.new_data_event.clear()
                    data = img_obj.to_json()
                    await websocket.send_json(data)
            except Exception as e:
                print("WebSocket connection closed (image):", e)
            # finally:
            #     await websocket.close()

        return send_image

    async def send_joints(self, websocket: WebSocket):
        await websocket.accept()
        print("WebSocket connection established for sending current joints.")
        try:
            while True:
                await asyncio.get_event_loop().run_in_executor(
                    None, self.current_jointstates.new_data_event.wait
                )
                self.current_jointstates.new_data_event.clear()
                data = self.current_jointstates.to_json()
                await websocket.send_json(data)
        except Exception as e:
            print("WebSocket connection closed (current joints):", e)
        # finally:
        #     await websocket.close()

    def enable_server(self):
        def run_server():
            config = uvicorn.Config(
                app=self.app,
                host=self.host,
                port=self.port,
                loop="asyncio",
                lifespan="off",
            )
            self.server = uvicorn.Server(config=config)
            self.server.run()

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()

    def disable_server(self):
        if self.server_thread.is_alive():
            self.server.should_exit = True
            self.server.force_exit = True
            self.server_thread.join()

    def is_running(self):
        if self.server_thread is None:
            return False

        if self.server_thread.is_alive():
            return True
        else:
            return False

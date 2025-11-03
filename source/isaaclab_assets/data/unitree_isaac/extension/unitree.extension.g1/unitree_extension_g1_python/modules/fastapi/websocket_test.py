import asyncio
import base64
import json

import cv2
import numpy as np
import websockets


async def joint_send():
    uri = "ws://localhost:8787/isaac/ws/joints/action"
    loader_position = load_data("../test_data/test_position.npy")
    loader_name = load_data("../test_data/test_name.npy")
    async with websockets.connect(uri) as websocket:
        print("Connected to /isaac/ws/joints/action")
        while True:
            name = next(loader_name).tolist()
            position = next(loader_position).tolist()
            test_data = {
                "name": name,
                "position": position,
                "velocity": None,
                "effort": None,
            }
            await websocket.send(json.dumps(test_data))
            await asyncio.sleep(0.05)
            print(f"Sent to /ws/action_joint: {test_data}")


async def image_receive(uri, window_name="Received Image"):
    async with websockets.connect(uri, max_size=16 * 1024 * 1024) as websocket:
        # print("Connected to /isaac/ws/images/..")

        while True:
            response = await websocket.recv()
            data = json.loads(response)

            base64_rgb_image = data["rgb_image"]
            rgb_image_data = base64.b64decode(base64_rgb_image)
            # Decode to RGB (height, width, 3)
            rgb_image = cv2.imdecode(
                np.frombuffer(rgb_image_data, np.uint8), cv2.IMREAD_COLOR
            )

            base64_depth_image = data["depth_image"]
            depth_image_data = base64.b64decode(base64_depth_image)
            depth_image = cv2.imdecode(
                np.frombuffer(depth_image_data, np.uint8), cv2.IMREAD_GRAYSCALE
            )

            cv2.imshow(window_name + " RGB", rgb_image)
            cv2.imshow(window_name + " Depth", depth_image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break


async def joint_receive():
    uri = "ws://localhost:8787/isaac/ws/joints/current"
    async with websockets.connect(uri) as websocket:
        print("Connected to /isaac/ws/joints/current")
        while True:
            response = await websocket.recv()
            # print(f"Received from /ws/current_joint: {response}")


async def main():
    await asyncio.gather(
        joint_send(),
        joint_receive(),
        image_receive("ws://localhost:8787/isaac/ws/images/head", "Head view"),
        image_receive("ws://localhost:8787/isaac/ws/images/right-hand", "Right hand"),
        image_receive("ws://localhost:8787/isaac/ws/images/left-hand", "Left hand"),
    )


def load_data(file_path):
    data = np.load(file_path)
    for i in range(data.shape[0]):
        yield data[i]


if __name__ == "__main__":
    asyncio.run(main())

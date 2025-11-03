import omni.graph.core as og
from omni.usd import get_context


class CameraGraph:
    def __init__(
        self, graph_name, camera_prim, resolution, rgb_topic_name, depth_topic_name
    ):
        """
        Initializes the camera module with the specified parameters.
        Args:
            graph_name (str): The name of the graph to which this camera belongs.
            camera_prim (str): The path of the camera primitive in the simulation scene.
            resolution (tuple): The resolution of the camera output as a tuple (width, height).
            rgb_topic_name (str): The topic name for publishing RGB image data.
            depth_topic_name (str): The topic name for publishing depth image data.
        """

        self.graph_name = graph_name
        self.camera_prim = camera_prim
        self.resolution = resolution
        self.rgb_topic_name = rgb_topic_name
        self.depth_topic_name = depth_topic_name

    def delete_camera_graph(self):
        stage = get_context().get_stage()
        if stage.GetPrimAtPath(self.graph_name).IsValid():
            stage.RemovePrim(self.graph_name)

    def gen_camera_graph(self):
        self.delete_camera_graph()
        og.Controller.edit(
            {"graph_path": self.graph_name, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    (
                        "OnPlaybackTick",
                        "omni.graph.action.OnPlaybackTick",
                    ),
                    (
                        "RunOneFrame",
                        "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame",
                    ),
                    (
                        "RenderProduct",
                        "isaacsim.core.nodes.IsaacCreateRenderProduct",
                    ),
                    (
                        "ROS2Context",
                        "isaacsim.ros2.bridge.ROS2Context",
                    ),
                    (
                        "RgbCameraHelper",
                        "isaacsim.ros2.bridge.ROS2CameraHelper",
                    ),
                    (
                        "DepthCameraHelper",
                        "isaacsim.ros2.bridge.ROS2CameraHelper",
                    ),
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        "OnPlaybackTick.outputs:tick",
                        "RunOneFrame.inputs:execIn",
                    ),
                    (
                        "RunOneFrame.outputs:step",
                        "RenderProduct.inputs:execIn",
                    ),
                    (
                        "RenderProduct.outputs:execOut",
                        "RgbCameraHelper.inputs:execIn",
                    ),
                    (
                        "RenderProduct.outputs:renderProductPath",
                        "RgbCameraHelper.inputs:renderProductPath",
                    ),
                    (
                        "ROS2Context.outputs:context",
                        "RgbCameraHelper.inputs:context",
                    ),
                    (
                        "RenderProduct.outputs:execOut",
                        "DepthCameraHelper.inputs:execIn",
                    ),
                    (
                        "RenderProduct.outputs:renderProductPath",
                        "DepthCameraHelper.inputs:renderProductPath",
                    ),
                    (
                        "ROS2Context.outputs:context",
                        "DepthCameraHelper.inputs:context",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    (
                        "RenderProduct.inputs:cameraPrim",
                        self.camera_prim,
                    ),
                    (
                        "RgbCameraHelper.inputs:type",
                        "rgb",
                    ),
                    (
                        "RgbCameraHelper.inputs:topicName",
                        self.rgb_topic_name,
                    ),
                    (
                        "DepthCameraHelper.inputs:type",
                        "depth",
                    ),
                    (
                        "DepthCameraHelper.inputs:topicName",
                        self.depth_topic_name,
                    ),
                    ("RenderProduct.inputs:width", self.resolution[0]),
                    ("RenderProduct.inputs:height", self.resolution[1]),
                ],
            },
        )

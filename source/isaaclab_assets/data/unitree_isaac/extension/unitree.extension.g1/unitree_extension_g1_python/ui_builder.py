import omni.timeline
import omni.ui as ui
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.gui.components.element_wrappers import CollapsableFrame, StateButton
from isaacsim.gui.components.ui_utils import get_style
from omni.usd import StageEventType
from pxr import Sdf, UsdLux

from .scenario import UnitreeG1Script


class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        # Run initialization for the provided example
        self._on_init()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._reset_extension()
            self._reset_ui()

        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            pass

        if event.type == int(omni.timeline.TimelineEventType.PAUSE):
            pass

    def on_physics_step(self, step: float):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        # Physics steps can be set in physicsScence in the stage
        self._scenario.update(step)

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        if event.type == int(StageEventType.OPENED):
            # If the user opens a new stage, the extension should completely reset
            self._reset_extension()
            self._reset_ui()

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that should be called
        """
        self._reset_extension()
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """

        control_interface = CollapsableFrame("Control Interface", collapsed=False)

        with control_interface:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._joint_controller_btn = StateButton(
                    "Articulation Controller",
                    "START",
                    "STOP",
                    on_a_click_fn=self._on_run_controller_a_text,
                    on_b_click_fn=self._on_run_controller_b_text,
                    physics_callback_fn=None,
                )
                self._joint_controller_btn.enabled = True
                self.wrapped_ui_elements.append(self._joint_controller_btn)

                # self._joint_controller_btn.set_world_settings(
                #     physics_dt=1 / 60.0, rendering_dt=1 / 60.0
                # )

                self._ros_control_btn = StateButton(
                    "ROS2 Control",
                    "START",
                    "STOP",
                    on_a_click_fn=self._on_run_ros_control_a_text,
                    on_b_click_fn=self._on_run_ros_control_b_text,
                    physics_callback_fn=None,
                )
                self._ros_control_btn.enabled = True
                self.wrapped_ui_elements.append(self._ros_control_btn)

                self._fastapi_control_btn = StateButton(
                    "Fast API Control",
                    "START",
                    "STOP",
                    on_a_click_fn=self._on_run_fastapi_control_a_text,
                    on_b_click_fn=self._on_run_fastapi_control_b_text,
                    physics_callback_fn=None,
                )
                self._fastapi_control_btn.enabled = True
                self.wrapped_ui_elements.append(self._fastapi_control_btn)

                self._stage_mode_btn = StateButton(
                    "Stage_mode",
                    "START",
                    "STOP",
                    on_a_click_fn=self._on_run_stage_mode_control_a_text,
                    on_b_click_fn=self._on_run_stage_mode_control_b_text,
                    physics_callback_fn=None,
                )
                self._stage_mode_btn.enabled = True
                self.wrapped_ui_elements.append(self._stage_mode_btn)


    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################

    def _on_init(self):
        self._scenario = UnitreeG1Script()

    def _add_light_to_stage(self):
        """
        A new stage does not have a light by default.  This function creates a spherical light
        """
        sphereLight = UsdLux.SphereLight.Define(
            get_current_stage(), Sdf.Path("/World/SphereLight")
        )
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        SingleXFormPrim(str(sphereLight.GetPath())).set_world_pose([6.5, 0, 12])

    def _on_run_controller_a_text(self):
        self._scenario.enable_articulation_controller()

    def _on_run_controller_b_text(self):
        self._scenario.diable_articulation_controller()

    def _on_run_ros_control_a_text(self):
        self._fastapi_control_btn.enabled = False
        self._scenario.enable_ros_communication()

    def _on_run_ros_control_b_text(self):
        self._fastapi_control_btn.enabled = True
        self._scenario.diable_ros_communication()

    def _on_run_fastapi_control_a_text(self):
        self._ros_control_btn.enabled = False
        self._scenario.enable_apiserver()

    def _on_run_fastapi_control_b_text(self):
        self._ros_control_btn.enabled = True
        self._scenario.disable_apiserver()

    def _on_run_stage_mode_control_a_text(self):
        self._scenario.stage_mode_enable()

    def _on_run_stage_mode_control_b_text(self):
        self._scenario.stage_mode_disable()

    def _reset_extension(self):
        """This is called when the user opens a new stage from self.on_stage_event().
        All state should be reset.
        """
        self._scenario.diable_articulation_controller()
        self._scenario.diable_ros_communication()
        self._scenario.disable_apiserver()
        self._on_init()

    def _reset_ui(self):
        self._ros_control_btn.reset()
        self._ros_control_btn.enabled = True
        self._fastapi_control_btn.reset()
        self._fastapi_control_btn.enabled = True
        self._joint_controller_btn.reset()
        self._joint_controller_btn.enabled = True
        self._stage_mode_btn.reset()
        self._stage_mode_btn.enabled = True

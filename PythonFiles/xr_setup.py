import os
import sys

from py4godot.classes.Node3D import Node3D
from py4godot.core import Color
_CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_CURRENT_DIR)

for _path in (_CURRENT_DIR, _PROJECT_ROOT):
    if _path not in sys.path:
        sys.path.insert(0, _path)

try:
    from controller_input import ControllerInputReader
    from hand_visuals import HandVisualFactory
    from PythonFiles.input_client import InputClient
except ImportError:
    try:
        from PythonFiles.controller_input import ControllerInputReader
        from PythonFiles.hand_visuals import HandVisualFactory
        from PythonFiles.input_client import InputClient
    except ImportError:
        from .controller_input import ControllerInputReader
        from .hand_visuals import HandVisualFactory
        from .input_client import InputClient


class XRSetup(Node3D):

    def _ready(self):
        self._input_sender = InputClient()

    def read_controller_inputs(self, left_hand_node, right_hand_node):
        return {
            "left": ControllerInputReader.read_controller(left_hand_node),
            "right": ControllerInputReader.read_controller(right_hand_node),
        }

    def send_controller_inputs(self, left_hand_node, right_hand_node):
        inputs = self.read_controller_inputs(left_hand_node, right_hand_node)
        self._input_sender.publish(inputs)

    def initialize_spheres(self, left_hand_node, right_hand_node):
        print("Python: Core initialization hook received.")

        if left_hand_node and right_hand_node:
            self.attach_colored_sphere(left_hand_node, Color(0, 1, 0, 1))
            self.attach_colored_sphere(right_hand_node, Color(0, 0, 1, 1))
            print("Python: Tracking indicators attached successfully.")
        else:
            print("Python: Error - Received invalid tracking node pointers.")

    def attach_colored_sphere(self, parent_node, color_value):
        return HandVisualFactory.create_sphere(parent_node, color_value)

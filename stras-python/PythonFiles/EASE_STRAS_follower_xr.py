"""Compatibility shim for Pylance and runtime execution.

This module allows the UDP service to import the XR follower adapter regardless
of whether the project is launched from the stras-python root or the script's
own directory.
"""

from module_main.EASE_STRAS_follower_xr import *  # noqa: F401,F403

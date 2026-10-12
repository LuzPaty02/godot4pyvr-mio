I’m checking the actual imports and local dependencies used by the leader/follower adapters so I can tell you the minimum files you need to move with them.

Searched for regex `from model_utilities|import model_utilities|from useful|import useful|configFile|datafolders|camera_calibration|robot_config_|vtk_render|stras_model` (`**/exp-esr14/module_main/**/*.py`), 199 results

Read 

Read 

Read 

Read , lines 1 to 120

## What to bring with the XR adapter files

For the two XR adapters, the safe migration bundle is:

### Minimum set
- module_main/useful.py
  - this provides the mapping and ordering functions used by the XR logic
- module_main/model_utilities
  - especially:
    - module_main/model_utilities/camera_model.py
    - module_main/model_utilities/robot_model.py
    - module_main/model_utilities/robot_piecewise.py
    - module_main/model_utilities/stras_model.py
    - module_main/model_utilities/transformations.py
    - module_main/model_utilities/datafolders.txt
    - module_main/model_utilities/robot_config_left.txt
    - module_main/model_utilities/robot_config_right.txt
    - module_main/model_utilities/robot_config_endoscope.txt
- module_main/cam_calib
  - this is referenced by module_main/model_utilities/datafolders.txt, which points to the camera calibration file

### If you want visualization / preview to work
Also bring the render modules from module_main/model_utilities, such as:
- module_main/model_utilities/vtk_render.py
- and any rendering module the app actually imports

---

## What I would not bring
I would not migrate the whole robot-control stack unless you still need real hardware control. In particular, I would skip:

- module_main/EASE_STRAS_leader_real.py
- module_main/EASE_STRAS_follower_real.py
- the low-level socket/message protocol pieces
- the full control folder unless your XR app still depends on QP/trajectory control logic

---

## Why this is the right bundle

The XR adapters depend on:
- math and ordering helpers in module_main/useful.py
- kinematic models in module_main/model_utilities
- calibration data and robot config files from module_main/model_utilities/datafolders.txt

So the migration should be based on the import chain, not on all of the old robot-control files.

> In short: bring the model layer, the helper layer, and the calibration/config folder; avoid bringing the socket-heavy real-control code unless you are still driving the physical robot.

If you want, I can turn this into a clean “copy-to-new-repo” checklist or a minimal folder manifest.
# Godot VR / STRAS visualization

This project renders a simulation-only STRAS model driven by XR input. It does not control a physical robot; instead, it maps headset/controller motion into a virtual robot configuration, computes forward kinematics, and streams the resulting geometry back to Godot for visualization.

## Overview

The runtime is split into two parts:

- Godot handles XR capture and UDP transport.
- Python handles the STRAS model, forward kinematics, and geometry output.

The active pipeline is:

XR poses -> simulated q -> STRAS forward kinematics -> VirtualFollower state -> Godot geometry

This is intentionally a local simulation loop. No robot commands are sent to hardware.

## Project layout

- `main.tscn` and `main.gd`: Godot entry scene and runtime glue
- `mock_input.tscn` and `mock_input.gd`: headset-free UDP mock input for smoke testing
- `robot_control/`: Python simulation backend and STRAS logic
- `robot_control/tests/`: Python regression tests for input mapping, kinematics, and client/server behavior
- `module_main/`: model/calibration data and robot geometry utilities

## Requirements

- Godot 4 with OpenXR enabled
- WiVRn or another OpenXR runtime for VR tracking
- Conda environment named `pyvr` for the Python runner
- Python scientific dependencies installed in that environment, including `liegroups` for the model utilities
- Linux NVIDIA setup is assumed in the commands below; adjust as needed for your machine

Important:

- Godot does not need the Python environment for this project.
- The `pyvr` environment is for the Python runner only.
- Py4Godot is not used here; do not install scientific dependencies into the bundled Godot Python.

## Quick start

### 1. Start the Python runner

From the project root:

```bash
conda activate pyvr
python -m robot_control.stras_vr_simulator
```

This listens for XR snapshots on `127.0.0.1:9000` and streams geometry back to `127.0.0.1:9100`.

The runner sends geometry at up to 30 FPS by default and samples the newest available input. It keeps the internal `VirtualFollower` updated and does not send hardware commands.

### 2. Launch the Godot project

Start the project with OpenXR enabled and run the scene in the editor or standalone mode.

Godot sends local UDP snapshots to the Python runner; the Python process resolves the robot model and returns geometry for visualization.

### 3. Optional: enable statistics

For debugging or verification:

```bash
python -m robot_control.stras_vr_simulator --stats-interval 1
```

This prints cumulative input/FK counters periodically.

## Normalized input conventions

This project treats input values as normalized control signals before they are mapped into robot joint space.

- Trigger and grip values are normalized to the range `[0, 1]`.
- Primary and secondary stick axes are preserved as signed values in `[-1, 1]` and only clamped, not rescaled to `[0, 1]`.
- The Python runner also supports a normalized `joints` payload for scripted tests: every one of the ten joint labels must be present and each value must be finite and within `[0, 1]`.
- Those normalized values are converted back to the virtual robot range by `normalized_joints_to_q(...)` before forward kinematics runs.

This is important because the Godot side sends raw XR/controller data, while the Python FK side expects normalized control conventions before converting to physical `q` values.

## Teleop HUD and normalized arm state

The geometry stream includes a `hud` field that reports the arm state in normalized form for debugging and HUD display.

- The HUD values are computed for `BL`, `RL`, `TL`, `BR`, `RR`, and `TR`.
- Each joint is normalized against its virtual calibration range and then clamped to `[0, 1]`.
- This makes the HUD easy to visualize, while the actual robot model still keeps its raw joint values in physical units.
- The teleop UI is not the same as the headset’s real XR view; it is a secondary visualization layer for the simulated endoscope/arm state.

## Input mapping summary

The mapping is absolute and based on the first tracked neutral pose. The values are not integrated over time and do not use velocity estimates.

| Input | Mapped joint(s) |
| --- | --- |
| Right controller lateral + vertical offset | `BR` + `RR` / `TR` |
| Left controller lateral + vertical offset | `BL` + `RL` / `TL` |
| XR camera X / Y position | `EBH` / `EBV` |
| XR camera forward/back position | `ET` |
| XR camera yaw | `ER` |

The q order is:

`BR, RR, TR, BL, RL, TL, EBV, EBH, ER, ET`

Translations are in millimetres, rotations are in radians, and bend values are curvature values.

## Smoke test without VR hardware

You can validate the UDP flow without a headset by running the mock input server and the mock scene.

### Start the UDP listener

```bash
conda activate pyvr
python -m robot_control.stras.input.transport_and_validation.input_server
```

### Run the mock scene in Godot

- Open `mock_input.tscn`
- Use Run Current Scene (F6)

The mock scene sends neutral frames for one second, then holds the simulated grip and moves the right stick upward. Watch the terminal for increasing sequence numbers and `R.grip=1.0` output.

Do not run this test server at the same time as `stras_vr_simulator`; both bind to port `9000`.

## Linux + WiVRn launch notes

### Terminal 1: launch WiVRn

```bash
flatpak override --user --device=all --share=ipc io.github.wivrn.wivrn
```

```bash
flatpak run \
  --env=__NV_PRIME_RENDER_OFFLOAD=1 \
  --env=__GLX_VENDOR_LIBRARY_NAME=nvidia \
  io.github.wivrn.wivrn
```

### Terminal 2: launch Godot with the OpenXR runtime

```bash
cd ~/Documents/godot/godot-bin

export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export XR_RUNTIME_JSON=$HOME/Documents/godot/godot-bin/openxr_wivrn.json

# 3. Apply standard PRIME hardware parameters
export __NV_PRIME_RENDER_OFFLOAD=1
export __NV_PRIME_RENDER_OFFLOAD_PROVIDER=NVIDIA-G0
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# 4. Open the editor; press F5 to run the project
./Godot_v4.7-stable_linux.x86_64 -e --path ~/Documents/godot/virtual-stras-mio/godot4pyvr-mio/
```

Or run directly without the editor:

```bash
cd ~/Documents/godot/godot-bin

export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export __NV_PRIME_RENDER_OFFLOAD=1
export __NV_PRIME_RENDER_OFFLOAD_PROVIDER=NVIDIA-G0
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export CUDA_CACHE_DISABLE=0
export XR_RUNTIME_JSON=$HOME/Documents/godot/godot-bin/openxr_wivrn.json

./Godot_v4.7-stable_linux.x86_64 --path ~/Documents/godot/virtual-stras-mio/godot4pyvr-mio/ --xr-mode on --rendering-method forward_plus
```

## Notes for development

- The project renders the simulation-only model; it does not send commands to a physical STRAS robot.
- The first tracked poses establish the neutral frame.
- The robot-to-Godot basis can be adjusted visually as a rendering setting.
- The endoscope camera viewport is a preview overlay; it does not replace the user’s actual headset view.
- The runner also supports normalized `joints` payloads for scripted tests, provided all ten labels are present in the `[0, 1]` range.

## Troubleshooting

- If the Python runner cannot bind to the input port, another process may already be using `9000`.
- If you see no geometry, confirm that the Godot scene is running with OpenXR enabled and that the Python runner is alive.
- If imports fail with `ModuleNotFoundError: No module named 'liegroups'`, activate the `pyvr` environment and install the project’s scientific dependencies there.
- For simpler debugging, run the simulator with `--stats-interval 1`.
- If you are using the mock input path, remember to stop the full simulator before launching the mock listener because both use the same UDP port.

## Tests

Run the project tests from the repo root:

```bash
pytest -q
```

This validates the input protocol, mapping logic, kinematics, and client/server behavior.


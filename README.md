# Godot VR / STRAS visualization

The Godot project root is `~/Documents/godot/virtual-stras-mio/godot4pyvr-mio/` (the directory containing `project.godot`).

## VR-driven STRAS simulation

This is a local simulation loop only: XR poses → simulated `q` → STRAS forward
kinematics → `VirtualFollower` state → Godot geometry. It does not connect to a
physical STRAS robot or send robot commands. The VR headset and controllers are
the only physical hardware in scope; STRAS runs as a software model. The active
pipeline uses forward kinematics and Godot rendering only.

**Runtime split:** Godot uses GDScript for XR capture, hand markers, and UDP
transport. The Python FK backend runs separately in the `pyvr` conda environment.
Py4Godot is intentionally excluded from this Godot project; do not install the
model's scientific Python dependencies into Godot's bundled interpreter.

Start WiVRn, then run `python -m robot_control.fk_runner` from the project root
in the `pyvr` environment. Start the Godot project with OpenXR enabled. Godot
sends snapshots to `127.0.0.1:9000`; the Python runner sends geometry back to
`127.0.0.1:9100`. The runner limits geometry output to 30 FPS by default and
samples the latest input. First tracked poses establish the neutral frame; the
mapping is absolute and does not integrate velocities or depend on `dt`.
The runner updates only the in-process `VirtualFollower`; periodic UDP/FK
counter output is disabled by default (use `--stats-interval 1` to re-enable it).

The mapping is deliberately explicit:

| Input offset from neutral | q entry |
|---|---|
| Right controller X / Y / Z | `BR` / `RR` / `TR` |
| Left controller X / Y / Z | `BL` / `RL` / `TL` |
| XR camera X / Y position | `EBH` / `EBV` |
| XR camera forward/back position | `ET` (Godot forward is -Z) |
| XR camera yaw | `ER` |

For position inputs, 0.1 m reaches the corresponding signed virtual limit; for
yaw, 90 degrees reaches the signed `ER` limit. First tracked poses establish
neutral. Offsets are clamped to the virtual simulation limits. `q` uses order
`BR, RR, TR, BL, RL, TL, EBV, EBH, ER, ET`; translations are millimetres,
rotations radians, and bend values curvatures. The receiver scales sampled model
points from millimetres to metres. Its robot-to-Godot basis remains an adjustable
visual coordinate-frame setting. Godot displays a calibrated endoscope preview
in a small overlay while preserving the OpenXR headset camera. The preview uses
the endoscope pose and pinhole intrinsics; lens-distortion correction is not
applied to the generated 3D view. OpenCV is optional for loading intrinsics, but
is still required for `camera_model` image-rectification operations.

The runner also retains a normalized `joints` input format for scripted tests;
those values must include all ten labels in `[0, 1]`.

## Headset-free UDP smoke test

Start `python -m robot_control.input_server` from the project root in the
`pyvr` environment. In Godot, run `mock_input.tscn` with **Run Current Scene**
(F6). The native GDScript mock sends neutral frames for one second, then holds
the simulated right grip and moves the right stick upward. Look for increasing
sequence numbers and `R.grip=1.0` in the receiver output. Do not run this test
server at the same time as `fk_runner`; both listen on port 9000.


Godot does not need the `pyvr` conda environment or a Python `PYTHONPATH`.
The `pyvr` environment is only for the Python runner.


**Terminal sesion 1**

- Step 1: Grant Device Permissions to WiVRn:

```console
flatpak override --user --device=all --share=ipc io.github.wivrn.wivrn
```

- Step 2:  Launch WiVRn under the discrete NVIDIA graphic card, by the command:
```console
flatpak run \
  --env=__NV_PRIME_RENDER_OFFLOAD=1 \
  --env=__GLX_VENDOR_LIBRARY_NAME=nvidia \
  io.github.wivrn.wivrn
```

**Terminal sesion 2**

- Step 1: Go to the path of the binaries of the godot executable:
```console
cd ~/Documents/godot/godot-bin
```

- Step 2A: Launch godot while explicitly pointing out the path:
```console
cd ~/Documents/godot/godot-bin/

# 1. Clear out Intel Mesa targets from the loader scope completely
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

# 2. Select the WiVRn OpenXR runtime
export XR_RUNTIME_JSON=$HOME/Documents/godot/godot-bin/openxr_wivrn.json

# 3. Apply standard PRIME hardware parameters
export __NV_PRIME_RENDER_OFFLOAD=1
export __NV_PRIME_RENDER_OFFLOAD_PROVIDER=NVIDIA-G0
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# 4. Open the editor; press F5 to run the project
./Godot_v4.7-stable_linux.x86_64 -e --path ~/Documents/godot/virtual-stras-mio/godot4pyvr-mio/
```

- Step 2B: Launch godot while explicity pointing out the path and overwritting the video card without the explorer:
```console
cd ~/Documents/godot/godot-bin/

export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export __NV_PRIME_RENDER_OFFLOAD=1
export __NV_PRIME_RENDER_OFFLOAD_PROVIDER=NVIDIA-G0
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export CUDA_CACHE_DISABLE=0

export XR_RUNTIME_JSON=$HOME/Documents/godot/godot-bin/openxr_wivrn.json

./Godot_v4.7-stable_linux.x86_64 --path ~/Documents/godot/virtual-stras-mio/godot4pyvr-mio/ --xr-mode on --rendering-method forward_plus
```

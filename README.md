# STRAS XR arm visualization

The `hello-world` Godot project and sibling `stras-python` Python project
communicate over localhost UDP. Python is a separate process, not embedded in
Godot. Godot reads both XR controller poses relative to the `Station`, maps
each controller's bend, twist, and forward travel to a three-joint arm
configuration, then sends a 10-value STRAS configuration to the Python service.
The endoscope's four values are currently sent as zero. The mapping includes
initial calibration, deadzone, gain, clamping, and low-pass filtering; pressing
either controller's `grip_click` recalibrates both controllers.

The Python service loads `XRFollowerAdapter` and the STRAS kinematic model from
`stras-python/module_main`, computes the right and left arm shapes, and returns
32 points per arm. Godot renders those polylines as tubes in the floating
screen viewport. Set `send_test_configuration` on the UDP client to use the
configured fixed test values instead of live controller-derived values; it is
off by default.

The Godot telemetry HUD visualizes controller pose data. The Python
`XRLeaderAdapter` is not currently part of this data path: input comes directly
from Godot's XR controllers. This is a visualization/control-mapping prototype,
not a hardware driver; neither the Godot client nor Python service sends
commands to a physical STRAS robot. Camera-image rendering is not part of the
current Godot display path and the optional VTK renderer module is not included.

## Run the STRAS shape service

Start the Python model service in a separate terminal and leave it running while
Godot is active:

```console
cd ~/Documents/godot/stras-python
conda activate pyvr
python PythonFiles/stras_udp_server.py
```

The service listens on `127.0.0.1:4242`; Godot binds `127.0.0.1:4243`. The
server resolves its `module_main` path relative to the script, so the shown
working directory is recommended but not required. Then launch the Godot
project from `hello-world` in another terminal. With the default client setting,
live controller-derived joint values are sent; enable `send_test_configuration`
in the `PythonXRManager` node to use the fixed smoke-test configuration.

## Run in XR

The instructions below assume the `pyvr` conda environment, WiVRn, and a local
Godot binary. Replace paths if your installation differs.


Launch Godot under the `pyvr` conda environment (the Python UDP service should
also be running in its own terminal, as described above).


**Terminal session 1**

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

**Terminal session 2**

- Step 1: Go to the path of the binaries of the godot executable:
```console
cd ~/Documents/godot/godot-bin
```

- Step 2A: Launch godot while explicitly pointing out the path:
```console
cd ~/Documents/godot/godot-bin/

# 1. Clear out Intel Mesa targets from the loader scope completely
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

# 2. Enforce standard XR & Python variables
export XR_RUNTIME_JSON=$HOME/Documents/godot/godot-bin/openxr_wivrn.json
export PYTHONPATH=~/miniconda3/envs/pyvr/lib/python3.10/site-packages:$PYTHONPATH

# 3. Apply standard PRIME hardware parameters
export __NV_PRIME_RENDER_OFFLOAD=1
export __NV_PRIME_RENDER_OFFLOAD_PROVIDER=NVIDIA-G0
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# 4. Fire up the Editor
./Godot_v4.7-stable_linux.x86_64 -e --path ~/Documents/godot/hello-world/
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
export PYTHONPATH=~/miniconda3/envs/pyvr/lib/python3.10/site-packages:$PYTHONPATH

./Godot_v4.7-stable_linux.x86_64 --path ~/Documents/godot/hello-world/ --xr-mode on --rendering-method gl_compatibility
```

I’m narrowing it to the follower side now and separating the reusable XR logic from the hardware-specific control path so you can keep only the right pieces.

Searched for regex `class |def |socket|decode|computeModels|initializeModels|simulate|visual|getState|button|teleoperation` (`**/exp-esr14/module_main/*follower*.py`), 309 results

Read , lines 90 to 220

Read , lines 1320 to 1555

## Follower side: what to keep for XR

For the follower files, the split is the same as with the leader:

### Keep
These are the useful XR pieces from module_main/EASE_STRAS_follower.py and module_main/EASE_STRAS_follower_real.py:

- state containers:
  - `axisCurrVal`
  - `axisCurrValNormalized`
  - `axisCurrValVel`
  - `axisPastVal`
- config-space / joint-limit definitions:
  - `qlims_down_virtual`
  - `qlims_up_virtual`
  - `lowerCSlims`
  - `upperCSlims`
  - `lims_down_joint`
  - `lims_up_joint`
- ordering and remapping logic:
  - `order_follower2configspace`
  - `map_norm2range`
  - `orderRanges`
- safety/filtering utilities:
  - `checkQinrange`
  - `filterConfigurationState`
  - `filterConfigurationState_bendingsOnly`
- model/preview methods if you want an XR robot visualization:
  - `initializeModels`
  - `computeModels`
  - `passArmState`
  - `visualizeState`
  - `visualizeFrame`
  - `visualizeFrameTeleop`

### Drop
These are hardware-specific and not needed for XR unless you still control the real robot:

- socket layer:
  - `connect`
  - `closeSocket`
  - `setTimeOut`
  - `onCMD`
  - `offCMD`
  - `stopCMD`
  - `resetCMD`
- low-level command sending:
  - `sendCMDmsg2STRAS`
  - `send_VELIDX`
  - `send_GoToPos`
  - `send_GoToPosPerAxis`
  - `send_GoToPos_byLabel`
- protocol decoding:
  - `decodeMsg`
  - `msgCMD_AXIS_infos`
  - `msgCMD_RECV_STATE`
- real-time hardware state polling:
  - `getAxisInfo`
  - `getState` in the real-driver sense

---

## Practical rule

If the goal is XR-only interaction, keep only:

1. state normalization
2. limit clamping
3. config-space conversion
4. optional model preview

If the goal is XR + robot preview, also keep:
- `computeModels`
- `visualizeFrame`
- `visualizeState`

If the goal is real robot control, then you keep the socket and command methods too. But for XR, those should stay out of the app logic.

---

## Minimal XR subset I would keep from follower files

For a clean XR follower adapter, I would keep roughly:

- `axisCurrVal`
- `axisCurrValNormalized`
- `qlims_down_virtual`
- `qlims_up_virtual`
- `lowerCSlims`
- `upperCSlims`
- `order_follower2configspace`
- `map_norm2range`
- `checkQinrange`
- `computeModels`
- `initializeModels`
- `visualizeState`
- `visualizeFrame`

And I would remove everything that touches:
- TCP sockets
- STRAS message framing
- command packets
- raw hardware polling

If you want, I can create the follower version of the XR adapter next, using the same pattern as the leader XR file.
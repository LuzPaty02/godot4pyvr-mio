## XR-ready subset to keep

Based on the two leader files, I’d keep these as the practical XR subset:

### Core state and mapping
- `getState`
- `simulatedState`
- `simulatedInit`
- `deactivateTeleoperation_button`

### Robot model / preview
- `computeModels`
- `initializeModels`
- `visualizeState`
- `visualizeFrame`
- `initializeVisualization`

### Relevant state variables from the class
These are the ones that matter if you want to use the robot state in XR:
- `axisCurrVal`
- `axisCurrValNormalized`
- `axisButtons`
- `handInLeftHandle`
- `handInRightHandle`
- `buttonFlag`
- `buttonActivatedFlag`
- `axisMins`
- `axisMaxs`
- `masterOrder`
- `qpOrder`
- `slaveOrder`

### Relevant ordering/limit constants
- `qlims_down_joint_pos`
- `qlims_up_joint_pos`
- `qlims_down_master_pos`
- `qlims_up_master_pos`
- `qlims_down_slave_pos`
- `qlims_up_slave_pos`
- `lims_down_joint`
- `lims_up_joint`
- `lims_down_master`
- `lims_up_master`
- `lims_down_slave`
- `lims_up_slave`

---

## What I would drop
I would remove these for XR and replace them with a cleaner interface:
- `sendCMDmsg2STRAS`
- `decodeMsg`
- `msgCMD_AXIS_infos`
- `msgCMD_RECV_STATE`
- `onCMD`
- `offCMD`
- `stopCMD`
- `resetCMD`
- `getAxisInfo`
- `easyStart`
- `setTimeOut`
- `connect`
- `closeSocket`

These exist only to speak the STRAS binary protocol directly.

---

## Recommended XR design

The clean pattern would be:

1. XR controller input
2. map to robot state / normalized axes
3. use `axisCurrValNormalized` or equivalent
4. optionally preview with `visualizeState`
5. send final robot commands through a simpler adapter layer

So the “keep” list is really a state/kinematics layer, not a hardware driver layer.

If you want, I can turn that into a small, ready-to-use XR class skeleton with just the functions and variables you’d actually need.
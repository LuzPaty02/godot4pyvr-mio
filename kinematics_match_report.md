# STRAS Kinematics Match Report

This report lists the aspects of the implementation that match the STRAS description in the paper and the corresponding code locations.

## 1. Joint configuration ordering

The implementation uses the same configuration-space ordering described in the paper:

- right arm joints first
- left arm joints second
- endoscope joints last

The order is defined in:
- [robot_control/stras/simulation/constants.py](robot_control/stras/simulation/constants.py)

The explicit ordering is:

`BR, RR, TR, BL, RL, TL, EBV, EBH, ER, ET`

This matches the paper’s notation for the 10-DOF follower configuration vector:

$q = [q_r, q_l, q_e]^T$

with the arm and endoscope variables grouped as described in the manuscript.

## 2. Separation of endoscope, right arm, and left arm kinematics

The STRAS model computes the endoscope pose and Jacobian separately from the two instrument poses and Jacobians, matching the paper’s formalization:

- endoscope: `Te`, `Je`
- right arm: `Tr_e`, `Jr_e`, `Tr_0`, `Jr_0`
- left arm: `Tl_e`, `Jl_e`, `Tl_0`, `Jl_0`

This is implemented in:
- [module_main/model_utilities/stras_model.py](module_main/model_utilities/stras_model.py)
- [robot_control/stras/model/kinematics.py](robot_control/stras/model/kinematics.py)

The code preserves the same decomposition used in the paper: poses are expressed in the endoscope frame or in the fixed/world frame, depending on the variable.

## 3. Endoscope-first composition of arm poses

The model computes the endoscope transform first and then composes each arm pose relative to it. This is the same as the paper’s chained transformation structure:

- `${}^{w}T_r = {}^{w}T_e {}^{e}T_r$`
- `${}^{w}T_l = {}^{w}T_e {}^{e}T_l$`

This is implemented in:
- [module_main/model_utilities/stras_model.py](module_main/model_utilities/stras_model.py)

The code calls the endoscope model with the endoscope joint values first, then passes its transform into the right and left arm computations.

## 4. World- and endoscope-frame pose accessors

The wrapper deliberately exposes both local and world-frame poses, matching the document’s notation and usage.

Examples:
- `Tr_w` / `Tl_w` are the world-fixed arm poses
- `Tr_e` / `Tl_e` are the arm poses expressed in the endoscope frame
- `Te` is the endoscope pose

Defined in:
- [robot_control/stras/model/kinematics.py](robot_control/stras/model/kinematics.py)

This matches the paper’s distinction between the arm positions in `{e}` and `{w}`.

## 5. Jacobian structure consistent with the generalized optimization formulation

The paper describes a generalized Jacobian that links the commanded joint rates to the output variables, with a block structure for the dominant arm, world arm tracking, and non-dominant arm motion.

The implementation exposes the underlying Jacobians for each subsystem in the same separation:

- right-arm Jacobian in endoscope frame
- right-arm Jacobian in world frame
- left-arm Jacobian in endoscope frame
- left-arm Jacobian in world frame
- endoscope Jacobian

These are returned by the STRAS kinematics wrapper in:
- [robot_control/stras/model/kinematics.py](robot_control/stras/model/kinematics.py)

This is consistent with the paper’s use of Jacobian blocks for objective-based control.

## 6. Model-level validation already present in the repository

The project includes a dedicated kinematics test file checking the model’s behavior against the intended ordering and pose conventions:

- [robot_control/tests/test_kinematics.py](robot_control/tests/test_kinematics.py)

The tests confirm:
- the QP joint ordering is preserved,
- endoscope/world pose references remain consistent,
- arm shapes and tip positions match the model outputs,
- endoscope rotation changes the bent-plane orientation as expected.

## 7. Overall conclusion

The codebase preserves the same modeling structure described in the paper:

- same 10-DOF configuration ordering,
- same right-/left-arm/endoscope decomposition,
- same endoscope-relative and world-relative pose treatment,
- same Jacobian-based formulation compatible with the optimization scheme in the manuscript.

The implementation is therefore consistent with the STRAS kinematics description in the paper, with only naming/notation differences rather than actual model discrepancies.

## Verification

The implementation was validated with:

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 conda run -n pyvr pytest -q robot_control/tests/test_kinematics.py`

Result:
- 8 passed in 0.25s

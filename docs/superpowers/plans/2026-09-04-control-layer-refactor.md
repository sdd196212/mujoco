# MuJoCo Control Layer Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the MuJoCo control layer into focused controller helpers, remove the leg-body-angle lock feature completely, and translate Python comments/docstrings to Chinese without changing control behavior.

**Architecture:** `Simulation.py` remains the executable entry point and `apply_lqr()` orchestration layer. `Controller.py` owns control constants, PID classes, VMC state update, LQR sign mapping, force correction, and actuator mapping helpers with explicit object-in/object-out interfaces. `environment.py`, `VMC.py`, and `lqr_controller.py` retain their domain responsibilities and receive only comment/import cleanup.

**Tech Stack:** Python 3.10, MuJoCo 3.4.0, NumPy, SciPy optional for gain loading, standard-library `unittest`.

## Global Constraints

- Preserve all existing control parameter values, LQR gain files, MJCF files, actuator order, sign conventions, and `python Simulation.py` startup behavior.
- Remove every occurrence and execution path for `LOCK_LEG_BODY_ANGLE_ZERO`, `LEG_BODY_ANGLE_KP`, and `LEG_BODY_ANGLE_KD`.
- Keep `FORCE_LQR_THETA_ZERO` as an independent LQR diagnostic switch.
- Do not launch a MuJoCo viewer from automated tests.
- Follow TDD: each behavior change starts with a failing test, then the smallest implementation, then a green refactor.
- Keep all maintained Python comments and docstrings in Chinese; do not alter user-facing numeric labels or controller formulas.

---

### Task 1: Create failing regression tests and test fixtures

**Files:**
- Create: `tests/__init__.py`
- Create: `tests/test_controller.py`
- Create: `tests/test_cleanup.py`

**Interfaces:**
- Tests consume the planned `Controller.RollPID`, `Controller.leg_force_f0`, `Controller.split_torque_pid`, and `Controller.compute_lqr_outputs` interfaces defined in later tasks.
- Cleanup tests inspect `Simulation.py` source and require forbidden lock symbols to be absent.

- [ ] **Step 1: Write the failing cleanup test**

```python
from pathlib import Path
import unittest


class CleanupTest(unittest.TestCase):
    def test_leg_body_angle_lock_code_is_removed(self):
        source = (Path(__file__).parents[1] / "Simulation.py").read_text(encoding="utf-8")
        for symbol in (
            "LOCK_LEG_BODY_ANGLE_ZERO",
            "LEG_BODY_ANGLE_KP",
            "LEG_BODY_ANGLE_KD",
        ):
            self.assertNotIn(symbol, source)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Write failing controller behavior tests**

```python
from types import SimpleNamespace
import unittest

import Controller


class ControllerTest(unittest.TestCase):
    def test_roll_pid_clamps_output_and_reset_clears_integral(self):
        RollPID = getattr(Controller, "RollPID", None)
        self.assertIsNotNone(RollPID)
        pid = RollPID(kp=10.0, ki=2.0, kd=0.0, output_limit=3.0, integral_limit=1.0)
        self.assertEqual(pid.update(1.0, 0.0, 0.1), 3.0)
        self.assertNotEqual(pid.integral, 0.0)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)

    def test_leg_force_pid_resets_when_disabled(self):
        leg_force_f0 = getattr(Controller, "leg_force_f0", None)
        self.assertIsNotNone(leg_force_f0)
        vmc = SimpleNamespace(L0=0.2, d_L0=0.0)
        force = leg_force_f0(vmc, 0.01, enabled=True)
        self.assertGreater(force, 0.0)
        self.assertEqual(leg_force_f0(vmc, 0.01, enabled=False), 0.0)
        self.assertEqual(vmc.length_integral, 0.0)

    def test_split_pid_returns_zero_and_resets_when_disabled(self):
        split_torque_pid = getattr(Controller, "split_torque_pid", None)
        self.assertIsNotNone(split_torque_pid)
        robot = SimpleNamespace()
        right = SimpleNamespace(theta=0.1, d_theta=0.0)
        left = SimpleNamespace(theta=0.0, d_theta=0.0)
        torque, error = split_torque_pid(robot, right, left, 0.01, enabled=True)
        self.assertGreater(torque, 0.0)
        self.assertAlmostEqual(error, 0.1)
        self.assertEqual(split_torque_pid(robot, right, left, 0.01, enabled=False), (0.0, 0.0))
        self.assertEqual(robot.split_integral, 0.0)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 3: Run the new tests and verify the expected RED state**

Run: `python -m unittest discover -s tests -v`

Expected: cleanup test fails because the lock symbols still exist; controller tests fail with `AssertionError` because the extracted attributes do not yet exist. There must be no `ImportError` or test-discovery error.

- [ ] **Step 4: Commit the failing tests**

```bash
git add tests
git commit -m "test: define control layer regression coverage"
```

### Task 2: Move controller constants and PID helpers into `Controller.py`

**Files:**
- Modify: `Controller.py`
- Modify: `Simulation.py:12-124`
- Test: `tests/test_controller.py`

**Interfaces:**
- `RollPID(kp=ROLL_KP, ki=ROLL_KI, kd=ROLL_KD, output_limit=ROLL_F0_MAX, integral_limit=5.0)` exposes `reset()` and `update(roll, roll_rate, dt, target=ROLL_TARGET) -> float`.
- `leg_force_f0(vmc, dt, enabled=True) -> float` updates `vmc.length_integral`/`vmc.length_error` and returns a clamped radial force.
- `split_torque_pid(robot, vmc_r, vmc_l, dt, enabled=True) -> tuple[float, float]` updates `robot.split_integral` and returns `(correction_torque, error)`.

- [ ] **Step 1: Add the controller constants, `RollPID`, `leg_force_f0`, and `split_torque_pid` to `Controller.py`**

Copy the existing numeric values and formulas exactly. Translate their docstrings/comments to Chinese. Keep integral initialization, anti-windup limits, disabled reset behavior, and output clipping unchanged.

- [ ] **Step 2: Import the helpers from `Controller.py` in `Simulation.py`**

Replace the local class/functions and their duplicated constants with imports. Keep `CONTROL_DIVIDER` in `Simulation.py` because it belongs to the simulation loop timing. Do not remove `FORCE_LQR_THETA_ZERO` yet; move it with the remaining controller constants in the next task.

- [ ] **Step 3: Run the focused tests and verify GREEN**

Run: `python -m unittest tests.test_controller -v`

Expected: all three controller tests pass with no warnings or errors.

- [ ] **Step 4: Commit the extracted PID helpers**

```bash
git add Controller.py Simulation.py tests/test_controller.py
git commit -m "refactor: centralize PID control helpers"
```

### Task 3: Extract VMC update and LQR sign mapping helpers

**Files:**
- Modify: `Controller.py`
- Modify: `Simulation.py:127-185`
- Modify: `tests/test_controller.py`

**Interfaces:**
- `update_vmc_positions(robot, vmc_r, vmc_l, control_dt) -> None` calls both VMC position updates with the existing joint-index, pitch, and gyro signs.
- `compute_lqr_outputs(robot, vmc_r, vmc_l, lqr, enabled=True) -> tuple[tuple[float, float], tuple[float, float], float, float]` returns `((T_r, Tp_r), (T_l, Tp_l), theta_r_lqr, theta_l_lqr)` and preserves the existing MATLAB `K @ state` row-specific signs.

- [ ] **Step 1: Add a fake LQR and VMC fixture to `tests/test_controller.py`**

Use a real `SimpleNamespace` fixture with `theta`, `d_theta`, and `L0`, plus a fake LQR object that records `control()` arguments and returns a deterministic two-element NumPy array. The fixture must not import or start MuJoCo.

- [ ] **Step 2: Write a failing test for the left/right LQR mapping**

Import `Controller` as a module and first assert `getattr(Controller, "compute_lqr_outputs", None)` is not `None`, so the pre-implementation failure is an assertion rather than an import error. After that guard, assert that `compute_lqr_outputs()` sends the right and left states with the current mirrored signs, returns the wheel result from index `0`, the virtual-leg result from index `1`, and returns zero printable theta values when `enabled=False`.

- [ ] **Step 3: Implement `update_vmc_positions()` and `compute_lqr_outputs()`**

Move the existing code without changing arithmetic, argument order, or the `FORCE_LQR_THETA_ZERO` switch semantics. Keep `FORCE_LQR_THETA_ZERO` as a module-level constant in `Controller.py` and import it in `Simulation.py`.

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run: `python -m unittest tests.test_controller -v`

Expected: PID and LQR helper tests pass.

- [ ] **Step 5: Commit the VMC/LQR helpers**

```bash
git add Controller.py Simulation.py tests/test_controller.py
git commit -m "refactor: extract VMC and LQR mapping helpers"
```

### Task 4: Simplify `apply_lqr()` and remove the leg-body-angle lock

**Files:**
- Modify: `Controller.py`
- Modify: `Simulation.py:187-262`
- Modify: `tests/test_controller.py`
- Modify: `tests/test_cleanup.py`

**Interfaces:**
- `map_vmc_torques(robot, vmc_r, vmc_l) -> None` calls each `vmc_calc_torque()` and writes the existing four-element `robot.joint_torque` list in XML order.
- `apply_lqr()` remains callable with its current signature and returns `((T_r, Tp_r), (T_l, Tp_l))`.

- [ ] **Step 1: Write a failing mapping test**

Import `Controller` as a module and first assert `getattr(Controller, "map_vmc_torques", None)` is not `None`, so the pre-implementation failure is an assertion rather than an import error. After that guard, create two VMC fixtures with deterministic `torque_set` values and a robot fixture whose `actuator_set_torque()` records invocation. Assert the helper writes `[right.phi1, right.phi4, -left.phi4, -left.phi1]` exactly.

- [ ] **Step 2: Implement `map_vmc_torques()` in `Controller.py`**

Move only the existing VMC torque calculation and actuator-order mapping. Do not add new clipping or sign changes.

- [ ] **Step 3: Rewrite `apply_lqr()` as orchestration**

Call `update_vmc_positions()`, `compute_lqr_outputs()`, `split_torque_pid()`, `RollPID.update()`/reset, `leg_force_f0()`, and `map_vmc_torques()` in the documented order. Keep the existing roll and leg-length `F0` formulas, `Tp` split correction, wheel torque assignment, diagnostic print fields, and return value.

- [ ] **Step 4: Delete the lock feature completely**

Remove `LOCK_LEG_BODY_ANGLE_ZERO`, `LEG_BODY_ANGLE_KP`, `LEG_BODY_ANGLE_KD`, the commented duplicate assignment, and the `if LOCK_LEG_BODY_ANGLE_ZERO ... else ...` branch. `split_torque_pid()` must now run unconditionally after LQR output calculation.

- [ ] **Step 5: Run the focused and cleanup tests and verify GREEN**

Run: `python -m unittest tests.test_controller tests.test_cleanup -v`

Expected: all tests pass, and cleanup assertions confirm that no forbidden lock symbols remain in `Simulation.py`.

- [ ] **Step 6: Commit the simplified control flow**

```bash
git add Controller.py Simulation.py tests/test_controller.py tests/test_cleanup.py
git commit -m "refactor: remove leg body angle lock path"
```

### Task 5: Translate comments and remove obvious source detritus

**Files:**
- Modify: `Simulation.py`
- Modify: `Controller.py`
- Modify: `environment.py`
- Modify: `VMC.py`
- Modify: `lqr_controller.py`
- Modify: `keyboard.py`
- Modify: `caculation.py`
- Test: `tests/test_cleanup.py`

**Interfaces:**
- No public function signatures or numeric behavior change.

- [ ] **Step 1: Extend cleanup tests for source hygiene**

Scan each Python source file and assert that none of these exact legacy comment fragments remains: `model timestep`, `Virtual-leg force controller`, `Roll PID output`, `Anti-split PID`, `Keep the original`, `The wheel (Wt)`, `The left XML joints`, `Positive MATLAB`, `Direct length PID`, `Run the MuJoCo`, `The imported MuJoCo`, and `Positive wheel_vel`. Also assert there is no `#robot.wheel_torque` and no `from caculation import *`.

- [ ] **Step 2: Translate comments and docstrings**

Translate English comments/docstrings in all listed files to concise Chinese. Preserve technical identifiers, units, actuator names, and formulas. Replace the humorous multi-line VMC string with a normal Chinese docstring describing completion of forward kinematics.

- [ ] **Step 3: Remove only unambiguous detritus**

Remove `time` from `environment.py` if unused, replace `from caculation import *` with `from caculation import orientation2euler`, remove unused `numpy`/`sin`/`cos` imports where confirmed, remove commented-out print/torque code, and collapse redundant blank lines. Do not remove imports used by the `keyboard.py` executable example.

- [ ] **Step 4: Run source hygiene tests**

Run: `python -m unittest tests.test_cleanup -v`

Expected: all cleanup assertions pass.

- [ ] **Step 5: Commit comment and import cleanup**

```bash
git add Simulation.py Controller.py environment.py VMC.py lqr_controller.py keyboard.py caculation.py tests/test_cleanup.py
git commit -m "chore: translate comments and remove unused code"
```

### Task 6: Full verification and final documentation check

**Files:**
- Modify: `README.md` only if the refactor changes a documented module/API name
- Test: `tests/test_controller.py`, `tests/test_cleanup.py`

- [ ] **Step 1: Run the complete unit-test suite**

Run: `python -m unittest discover -s tests -v`

Expected: all tests pass without warnings, viewer windows, or unhandled exceptions.

- [ ] **Step 2: Compile every Python module**

Run: `python -m compileall -q .`

Expected: exit code `0` and no syntax errors.

- [ ] **Step 3: Review the final diff for behavioral scope**

Run: `git diff HEAD~5 --stat` and `git diff HEAD~5 -- Simulation.py Controller.py environment.py VMC.py lqr_controller.py keyboard.py caculation.py`

Confirm that MJCF files, gain data, control constants, actuator order, LQR signs, and simulation entrypoint were not changed unintentionally.

- [ ] **Step 4: Perform a headless MuJoCo smoke check when dependencies are available**

Run:

```bash
python -c "from environment import LegWheelRobot; robot = LegWheelRobot('MJCF/env.xml', visualize=False); robot.sensor_read_data(); print('headless MuJoCo smoke check passed')"
```

Expected: the command prints `headless MuJoCo smoke check passed` and exits with code `0` without opening a viewer. If MuJoCo is unavailable, record the dependency error and rely on compile/unit-test results; do not alter production code to bypass it.

- [ ] **Step 5: Commit any documentation-only update and report verification**

If the README needs an API-name correction, commit it with `docs: update control layer notes`; otherwise leave README untouched. Report exact test and compile commands and their outcomes.

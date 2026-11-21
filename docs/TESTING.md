# Testing and verification — v1.4

What is tested, how to run it, and how each claim of this release was verified. Every check is labelled with **where** it ran:

- **Sandbox:** run by the development assistant in a Linux VM without ROS (plain CMake builds, pytest with local stand-ins for the ROS Python API).
- **User's machine:** run in the two Docker containers (ROS 2 Jazzy) on Windows 11 + Docker Desktop.
- **Gazebo:** a full simulation run on the user's machine.

---

## 1. Run the tests

```bash
# ids container: control law, IDS core, IDS node parameters
cd /ws && colcon build --packages-up-to ids_ros
colcon test --packages-select iiwa_control_law ids_core ids_ros
colcon test-result --verbose

# robot container: controller, simulation, planner and attacker, bringup
cd /ws && colcon build --packages-up-to ids_bringup ids_evaluation
colcon test --packages-select iiwa_controllers iiwa_sim ids_attacks ids_experiments ids_bringup ids_evaluation
colcon test-result --verbose

# Python style (the repository's .flake8 is not mounted into the containers, so pass its settings)
flake8 --max-line-length 99 --extend-ignore E203 src
```

The ROS-free libraries also build and test with plain CMake outside ROS (`ids_core`, `iiwa_control_law`, the parameter library of `ids_ros`, the tracking core of `iiwa_controllers`, the friction model of `iiwa_sim`).

## 2. Test suites

| Package | Suite | Tests | Proves |
|---|---|---|---|
| `iiwa_control_law` | `test_control_law` | 7 | Hold between samples, PD + gravity, clamp, a setpoint attack appears as Kp·γ on one joint |
| `ids_core` | `test_robot_model` | 9 | Dynamics, derivatives (vs finite differences), payload added and removed exactly |
| | `test_closed_loop_model` | 5 | Jacobians in both modes, tracking, disturbance inversion recovers the right joint |
| | `test_ekf` | 9 | Disturbance on its joint, friction learning, χ² gate, rollback, re-sync, refinement |
| | `test_dual_ewma` | 8 | Dissertation λ values, exact variance, drift offset, freeze, rollback |
| | `test_detector` | 7 | Debounce, hysteresis, latch, channels, limits meet their false-alarm target |
| | `test_calibration` | 5 | SHA-256, what changes the fingerprint and what does not, file round trip |
| | `test_pipeline` | 7 | Modes, calibrate once, refusal, command check, halted windows, warm-up, attribution |
| | `test_scenarios` | 7 | Synthetic cell: no alarm under wear; step on each joint < 100 ms and attributed; ramp; torque tamper; thesis mode blind; payload handover |
| | `test_realtime` | 2 | No heap allocation while armed (and the counter works) |
| | `test_replay` | 6 | `.npy` round trip and rejection, column checks, calibration from a recording, quiet nominal replay, step on a4 detected and attributed (v1.4) |
| `ids_ros` | `test_parameters`, `test_conversions`, `test_parameter_file` | 5 + 4 + 5 | Every setting is a parameter; message conversions are exact; the parameter YAML files load for `ids_replay` |
| `iiwa_controllers` | `test_tracking_core` | 15 | State machine, payload gravity, halt/resume, clamp, deadline, tracking on a plant, no allocation, windows, tick clock |
| | `test_encoder_model` | 4 | Quantisation, seeded noise, within the IDS's noise model |
| | `test_ids_contract` | 2 | Controller + IDS: no false alarm; MitM on each joint detected, halted, resumed without a new alarm |
| | `test_load_controller` | 1 | The plugin loads through `controller_manager` |
| `iiwa_sim` | `test_friction_model` | 7 | Wear schedule; friction never stacked twice |
| | `test_description` | 11 | The simulated robot equals the IDS's model; plugins, gripper, profiles, meshes |
| `ids_attacks` | pytest | 40 | Every attack kind, transparency outside the window, ground truth, seeded noise, input safety, the torque cover (v1.4) |
| `ids_experiments` | pytest | about 50 | Trajectories, sequencer and handover order, scenarios (every kind, cover flag), IK, payload, start-up gate, shutdown |
| `ids_bringup` | pytest | 36 | Campaign, sweeps, cover wiring, run files, Gazebo clean-up, run monitor, campaign runner |
| `ids_evaluation` | pytest | 51 | Layouts, detection and false-alarm rules, export, baselines (momentum observer on a simulated arm), replay calls, reports, commands (v1.4) |

## 3. Verification status of v1.4

| Check | Where | Result |
|---|---|---|
| C++ builds (`iiwa_control_law`, `ids_core`, `ids_ros`, `iiwa_controllers`) and gtests: 7, 65, 14, 21 | Sandbox | Passed, 0 warnings |
| Python tests on pytest 7.4.4 and 9: `ids_attacks` 40, `ids_bringup` 36, `ids_evaluation` 51, `ids_experiments` 46 (ROS-free modules); flake8 | Sandbox | Passed; flake8 clean |
| MitM node against an `rclpy` stand-in: every kind, the cover, a sweep entry | Sandbox | 13/13 scenarios passed |
| `ids_replay`, `evaluate` and `compare_baselines` on the v1.2 recordings | Sandbox | Passed; replay limits equal the live ones |
| IDS container: build and `colcon test` `ids_core`, `ids_ros` | User's machine | 100 tests, 0 failures |
| Robot container: build (now including `ids_core` and `ids_ros`) and `colcon test` of 5 packages | User's machine | 227 tests, 0 failures |
| Sweep plan (`--dry-run`) | User's machine | 28 runs planned |
| Regression set `nominal` 600 s and `step_a4`, plus `step_a4_cover`, seed 1 | Gazebo | All complete, no saturation; results in `docs/v1.4.md` |
| Evaluation, replay (≥ 98.9 % agreement) and baselines on those runs | User's machine | Passed |
| Behaviour lock of the commented release | Sandbox | Only comments differ from the comment-free code (C++, Python AST, XML, YAML/JSON) |
| Nine attack kinds, sweeps, ramp, other joints, halt chain in simulation | — | Implemented and unit-tested, **not run in simulation** |

## 3a. Verification status of v1.2

| Check | Where | Result |
|---|---|---|
| C++ builds with `-Wall -Wextra -Wpedantic` (ROS-free parts) | Sandbox | 0 warnings |
| C++ unit tests: control law 7, `ids_core` 59, `ids_ros` 9, controllers 22 | Sandbox | 97/97 passed |
| Python tests on pytest 7.4.4 (Jazzy) and 9; flake8 | Sandbox | 87/87 passed; flake8 clean |
| Launch files and nodes against stand-ins for `launch` and `rclpy` | Sandbox | 16/16 passed |
| `colcon build`, all 7 packages, both containers | User's machine | Passed, no compiler output |
| IDS container tests (v16 code) | User's machine | 89/89 passed |
| Robot container tests: controllers 22, `iiwa_sim` 18, `ids_experiments` 50, `ids_bringup` 30 | User's machine | Passed (v12–v15 code; re-run before tagging) |
| Measurement windows exactly 10 ms | Gazebo | Every step 10.000 ms (bag check) |
| Arm at home at controller start, tracking error ≤ 0.004 rad, 0 % saturation | Gazebo | Passed |
| IDS model matches Gazebo: d̂ mean ≤ 0.006 N·m; friction a1–a6 within 2 % | Gazebo (calibration) | Passed |
| Wear reaches the joints | Gazebo | Confirmed (thesis-mode d̂ follows the friction torque, slope 1.00–1.02) |
| Regression set: `nominal` 600 s and `step_a4`, seed 1 | Gazebo | Passed (results in `docs/v1.2.md`) |
| Behaviour lock of the commented release: only comments added to the code | Sandbox | C++ compared after comment removal, Python by AST, XML by canonical form, YAML/JSON by value: identical |
| Ramp attacks, other joints, halt chain in simulation | — | Implemented and unit-tested, **not run in simulation** |
| IDS step time on the development PC | — | Not measured |
| GitHub Actions CI | — | Not run yet (first push) |

## 4. Regression set for each release

From the fast-track plan, each release is checked in Gazebo with at least:

```bash
# ids container
ros2 launch ids_ros ids.launch.py run_mode:=calibrate      # then, in the robot container:
ros2 launch ids_bringup experiment.launch.py scenario:=calibration
# ids container
ros2 launch ids_ros ids.launch.py
# robot container
ros2 run ids_bringup run_campaign --only nominal step_a4 --seeds 1
```

Pass criteria: both runs `complete` with no warnings and ≈ 0 % torque saturation; v2 raises no alarm in `nominal`; v2 alarms on a4 within 100 ms of the attack in `step_a4` with a4 as the primary joint.

From v1.4 the runs are scored with the evaluation package (add `--rerun` to repeat runs that are already complete):

```bash
R=$(ls -d /ws/data/runs/*_nominal_s1 | tail -1); S=$(ls -d /ws/data/runs/*_step_a4_s1 | tail -1)
ros2 run ids_evaluation evaluate $R $S --out /ws/data/eval
ros2 run ids_evaluation compare_baselines $R $S --train $R --out /ws/data/compare
```

## 5. Benchmark

```bash
/ws/build/ids_core/bench_ids_step 5000     # prints median, p90, p99 and max in microseconds
```

Sandbox (shared 2-vCPU VM): median 181–233 µs, p99 237–427 µs per 10 ms window.

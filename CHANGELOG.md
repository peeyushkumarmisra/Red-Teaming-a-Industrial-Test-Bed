# Changelog

All notable changes to this project are listed here, newest first.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow [Semantic Versioning](https://semver.org/).
Change IDs (C01, C02, …) and problem IDs (B1, B2, …) refer to the project's refinement plan and iteration log.

---

## [1.4.0] — 2026-11-21 · `v1.4-ids-attacks-eval`

**Harder attacks, and a proper way to score them.** Ten attack kinds, an attacker who also falsifies the robot's torque report, sweeps, an offline replay of recorded runs, an evaluation package and five baseline detectors. The IDS maths is unchanged, so v1.2 calibrations stay valid. Release notes, results and graphs: [`docs/v1.4.md`](docs/v1.4.md).

> **Results are narrow:** seed 1, joint a4 only (step and covered step), 0.165 h of nominal data. Nine attack kinds and the sweeps are implemented and unit-tested, not run in simulation.

### Highlights

- **v2 IDS:** 0 false alarms in 600 s of wear; the 0.02 rad step on a4 detected in **44 ms**, and in **50 ms** when the attacker also falsified the torque report (physics channel only), both on a4.
- **Baselines on the same data:** static and single-EWMA thresholds are as fast but raised a false alarm in 0.131 h of held-out wear; the momentum observer (v1's residual) misses the setpoint attack.

### Added

- `ids_attacks` (C19): step, ramp, sine, scaling, freeze, replay, delay, noise, coordinated and drift-mimic attacks with per-sample ground truth; `TorqueCover` (C24).
- MitM torque-report cover: scenario flag `cover_torque`; the controller then publishes `/robot/measurement_raw`, the MitM relays a falsified `/robot/measurement` (C24).
- Campaign: scenarios for every new kind on a4, `coordinated_a2a4`, `step_a4_cover`; `sweeps:` (`step_sweep`, `ramp_sweep`) expanded to named runs (C20).
- `ids_core` offline replay and `.npy` I/O; `ids_replay` command in `ids_ros`, reading the node's own parameter YAML with `--set` overrides (C21).
- `ids_evaluation` (C18, C22): `export_run`, `evaluate`, `compare_baselines`, `replay_run`, `split_legacy`; metrics, baselines, figures, Markdown/CSV reports.
- `docs/v1.4.md` with the measured results and graphs.

### Changed

- `ids_experiments`: scenarios accept every kind, several joints (`joints`), `magnitudes`, `frequency`, `lag` and `cover_torque`; the MitM uses `ids_attacks`.
- `ids_bringup`: `/robot/measurement_raw` recorded; covered runs move the controller's measurement topic behind the MitM.
- `.gitignore`: `/data/` anchored to the repository root (B13).

### Removed

- `ids_experiments/attacks.py` and its test (replaced by `ids_attacks`).

### Fixed during testing

- Replay test limit (B1), momentum-observer integration (B2), payload mass in the baseline (B3), friction-truth columns and clock (B4), detection after the attack end counted separately (B8), training-run matching (B10), missing test URDF on the v1.2 branch (B13), code broken by hand-edited comments (B14, B15).

### Known issues

- B11 replay agreement 98.9–100 %; B12 commanded-torque baseline unstable; B16 two a1 alarms after the covered attack (one run); B17 shutdown warning; B24 friction lag (a4 −24.6 % at 600 s); a7 friction unobservable.

---

## [1.2.0] — 2026-11-20 · `v1.2-ids-core-sim`

**The dissertation experiment, rebuilt so its results can be trusted.** The IDS now predicts the arm's motion from the trusted PLC reference, watches all seven joints, learns friction wear, and is calibrated once. The dissertation's design runs alongside it on the same data. Release notes, results and graphs: [`docs/v1.2.md`](docs/v1.2.md).

> **Results are narrow:** one seed, one attack on joint a4, one nominal run. There is no per-hour false-alarm rate, no ramp result and no result for the other joints yet.

### Highlights

- **v2 IDS:** no false alarm in 600 s of friction wear; the 0.02 rad setpoint attack on a4 detected **50 ms** after the first falsified setpoint, on the right joint.
- **Thesis mode** (the dissertation's design): missed the same attack, which confirms the blind spot found in the v1 audit, now in Gazebo.

### Added

- **Containers and CI** (C01): `robot` and `ids` Docker images, a Compose file with seven services (five idle placeholders under the `planned` profile), a non-root user, a dev container that attaches to `robot`, and a GitHub Actions workflow that builds, tests and lints.
- **Messages** (C02): `ids_interfaces` with `JointReference`, `JointMeasurement`, `IdsStatus` and `AttackTruth`.
- **Shared control law** (C03): `iiwa_control_law`, a PD tracking law with gravity feed-forward for the trusted payload context, a torque clamp and a first-order hold. The controller and the IDS use the same code.
- **Robot model** (C04): payload composition, analytic derivatives and preallocated buffers, so there is no heap allocation while the IDS is armed.
- **Closed-loop model** (C05): replays the 10 controller ticks of each 10 ms window and builds the Jacobians and the disturbance map Φ.
- **EKF** (C06): v2 mode with a viscous-friction state per joint, and thesis mode without; Joseph-form update, χ² (NIS) gate, friction freeze and rollback.
- **Disturbance estimate** (C07): d̂ = Φ⁻¹·r with one Gauss–Newton refinement, standardised by its covariance in every window.
- **Dual-EWMA** (C08): exact time-varying variance, a two-sided statistic, a slow-channel detector, and in v2 mode a fast-channel baseline that freezes during an alarm and rolls back 50 windows at its start.
- **Detector** (C09): limits from a false-alarm budget (0.125 per hour), a 5-window debounce, hysteresis, latch and `/ids/reset`, the attacked joint named at every alarm onset, and a 4 s warm-up.
- **Command check** (C10): the torque the controller applied, against the torque the trusted reference implies.
- **Calibrate once** (C11): a calibration file stamped with a fingerprint of the parsed robot model and the IDS settings.
- **IDS node** (C12): `ids_ros`, driven by sim time and message stamps, with a trusted-reference tap, status and alarm topics, a calibration mode, and two instances (`ids` and `ids_thesis`).
- **Tracking controller** (C13): `iiwa_controllers/ReferenceTrackingController`, a `ros2_control` plugin at 1 kHz with a 20-bit encoder model, a deadline guard, a halt input and a tick clock that gives exact 10 ms windows in simulation.
- **Simulation** (C14): `iiwa_sim`, with a launch-time xacro description, the FrictionDrift plugin (all joints, four friction profiles, ground truth on `/sim/friction_truth`), pruned meshes with a NOTICE file, and the `cell` world.
- **PLC-style planner** (C15): a 10 ms scan on sim time, a step sequencer, seeded inverse kinematics, quintic trajectories streamed at 100 Hz, a trusted payload context, a gripper handshake and part re-seating at each station.
- **Setpoint MitM** (C16): a dormant on-path node with step and ramp attacks on any joint, plus `/attack/truth`.
- **Bringup** (C17): `ids_bringup`, with simulation and experiment launches, a scenario and campaign file, a run supervisor (`summary.yaml` with a torque-saturation check), a campaign runner, MCAP recording and clean-up of leftover Gazebo servers.
- **Repository hygiene:** `.gitattributes` (LF line endings; meshes and images stored as binary) and `.editorconfig` (shared indentation rules).
- **Documentation:** [`README_REFINED.md`](README_REFINED.md) (the full story: objective, v1 problems, changes, algorithms, results, limitations, future work, challenges), a landing `README.md`, [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md), [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md), [`docs/TESTING.md`](docs/TESTING.md), release notes [`docs/v1.2.md`](docs/v1.2.md) with four result graphs, `CONTRIBUTING.md` and the Apache-2.0 `LICENSE` declared by every package.
- **Plain-language comments in every source file:** a header explaining what the file does and why it matters for detecting attacks, plus comments on design decisions. Comments only: the code was checked to be unchanged (C++ after comment removal, Python by syntax tree, XML, YAML and JSON by parsed content).

### Changed

- The IDS predicts from the **trusted PLC reference** instead of the commanded torque. Thesis mode keeps the commanded-torque input for comparison.
- The robot is driven by a C++ `ros2_control` controller with gravity feed-forward, instead of the v1 Python PD controller on a 1 kHz `rclpy` timer.
- Friction wear is applied by a rewritten FrictionDrift plugin that reaches every joint and reports what it applies.
- `.flake8` ignores E203, so Black-formatted code passes the CI lint.

### Removed

- v1 packages `context_aware_ids`, `vulnerable_controller`, `custom_gazebo_plugins` and `iiwa_description` (replaced by `iiwa_sim`). The v1 `ids_bringup` and `ids_experiments` were replaced by new packages with the same names.
- v1 root files `iiwa.urdf`, `controllers.yaml`, `run_exp.sh`, `experiment_data.csv`, `exp_combined.png` and a committed `.venv`.
- `.devcontainer/Dockerfile`; the dev container now attaches to the `robot` service.

Everything removed is still available at tag `v1.0.0`.

### Fixed during testing

Twenty-one problems (B1–B23, with B13 diagnosed and fixed as B14) were found and fixed between the first sandbox build and the final runs, among them: timing of the measurement windows (B19), a Gazebo server surviving between runs (B20), the arm collapsing at start-up (B21), false alarms at every grab and alarms that never cleared (B23), and others. The main ones are summarised in [`docs/v1.2.md` §6](docs/v1.2.md#6-problems-found-and-fixed).

### Known issues

- **B4:** an attack that does not start on a window boundary can also flag a second joint for a few windows (a7 in `step_a4`). The attacked joint is still named correctly.
- **B24:** under wear, the v2 friction estimates on a1–a4 fall behind the truth (up to 18 % on a4 after 240 s). Detection was not affected in the measured run.
- Joint a7's friction cannot be learned while a7 does not move.
- Thesis mode alarms at the first move from home and later under wear; it has no friction model.
- Ramp attacks, attacks on the other six joints and the halt-on-alarm chain are implemented and unit-tested but **not run in simulation**.

### Moved to later releases

- **C18** `ids_evaluation` (latency, false alarms per hour, per-joint table, plots) → `v1.4.0`.
- The v1.2 campaign (7 joints × step and ramp × seeds 1–3, plus nominal × 3) → the results backlog after `v2.0.0`.

---

## [1.0.0] · dissertation code

The original MSc dissertation code (EMATM0055), as submitted. Tag `v1.0.0`, commit `b560e49`.

[1.2.0]: https://github.com/peeyushkumarmisra/red-teaming-a-industrial-test-bed/compare/v1.0.0...v1.2.0
[1.0.0]: https://github.com/peeyushkumarmisra/red-teaming-a-industrial-test-bed/releases/tag/v1.0.0

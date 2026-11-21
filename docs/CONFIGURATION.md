# Configuration reference — v1.4

Every setting of the testbed, where it lives, its default and what it does. Values are the ones used for the v1.2 and v1.4 results unless stated.

> [!WARNING]
> **Any change to an IDS setting marked 🔒 changes the calibration fingerprint.** The IDS then refuses the old calibration (mode `fault`) until you calibrate again. The controller's gains and limits must match the IDS's model settings, otherwise the IDS predicts a different robot.

---

## 1. IDS (`ids_ros`)

Files: `src/ids_ros/config/ids.yaml` (instance `/ids`, v2) and `ids_thesis.yaml` (instance `/ids_thesis`, dissertation design). Defaults: `src/ids_core/include/ids_core/config.hpp`. Unknown names and invalid values stop the node at start-up.

### 1.1 Node settings (not part of the fingerprint)

| Parameter | Default | Meaning |
|---|---|---|
| `run_mode` | `armed` | `armed`: detect with the stored calibration. `calibrate`: record and save a new one |
| `calibration_file` | (required) | `/ws/data/calibration/ids_v2.yaml` or `ids_thesis.yaml` |
| `calibration_duration` | `120.0` | Seconds of simulation time recorded for a calibration |
| `arm_after_calibration` | `false` | Start detecting right after saving a calibration |
| `exit_after_calibration` | `false` | Exit after saving (the launch sets this in calibrate mode) |
| `warmup` | `2.0` | Seconds after the first reference that are ignored (start-up) |
| `model_urdf` | `""` | Robot model file; empty = take `/robot_description` |
| `use_sim_time` | `true` | Run on Gazebo's clock |

### 1.2 Estimator and timing 🔒

| Parameter | v2 | Thesis | Meaning |
|---|---|---|---|
| `input_source` | `trusted_reference` | `commanded_torque` | What drives the model. The commanded torque is under the attacker's control too |
| `estimator_mode` | `extended` | `thesis` | `extended` learns viscous friction per joint; `thesis` has no friction states |
| `nominal_rate_hz` | `100.0` | same | IDS windows per second |
| `max_steps_per_window` | `100` | same | Longer gaps are re-synced, not detected |
| `context_guard` | `0.3` | same | Seconds after a trusted payload change during which the IDS re-syncs instead of detecting |

### 1.3 Robot model 🔒 (must match the controller)

| Parameter | Default | Meaning |
|---|---|---|
| `model.controller_period` | `0.001` | Controller tick (1 kHz) |
| `model.reference_hold` | `0.05` | How long a reference sample is extrapolated |
| `model.reference_latency` | `0.0` | Delay between a reference's stamp and its use by the controller |
| `model.tip_joint` | `joint_a7` | Joint the payload is attached to |
| `model.kp` | `800, 800, 600, 600, 250, 150, 50` | Position gains (N·m/rad) |
| `model.kd` | `64.6, 81.6, 32.4, 32.0, 6.4, 3.8, 1.4` | Velocity gains (N·m·s/rad) |
| `model.effort_limit` | `320, 320, 176, 176, 110, 40, 40` | Torque limits (N·m) |
| `model.payload_mass` | `5.0` | Nominal payload mass (kg); the actual mass comes from the trusted reference |
| `model.payload_com` | `0, 0, 0.204` | Payload centre of mass in the link_7 frame (m) |
| `model.payload_inertia` | `0.008333` × 3 | Payload inertia diagonal (kg·m²) |

### 1.4 Kalman filter 🔒

| Parameter | Default | Meaning |
|---|---|---|
| `ekf.q_position` | `1e-11` | Process noise on position, per tick |
| `ekf.q_velocity` | `1e-7` | Process noise on velocity, per tick |
| `ekf.q_friction` | `1e-9` | How fast friction may change, per tick (see open item B24) |
| `ekf.r_position` | `1e-10` | Encoder position noise (rad²) |
| `ekf.r_velocity` | `1e-6` | Velocity noise ((rad/s)²) |
| `ekf.p0_position`, `ekf.p0_velocity`, `ekf.p0_friction` | `1e-8`, `1e-6`, `1.0` | Initial uncertainty |
| `ekf.friction_max` | `50.0` | Upper bound on a friction estimate (N·m·s/rad) |
| `ekf.nis_gate` | `36.1` | Friction is learned only below this χ² value (14 dof, about 0.1 % tail) |
| `ekf.rollback_steps` | `50` | Windows of friction learning and baseline undone at an alarm onset (v2) |
| `ekf.refine_iterations` | `1` | Gauss–Newton refinements of the disturbance estimate |

### 1.5 Statistics and detector 🔒

| Parameter | v2 | Thesis | Meaning |
|---|---|---|---|
| `ewma.tau_fast` | `0.0448` | same | Fast average time constant (s): λ = 0.2 at 100 Hz |
| `ewma.tau_slow` | `1.995` | same | Slow average time constant (s): λ = 0.005 at 100 Hz |
| `detector.false_alarm_budget_per_hour` | `0.125` | same | Target false alarms per hour, shared by all channels and joints |
| `detector.debounce` | `5` | same | Windows in a row above the limit before an alarm |
| `detector.clear_count` | `5` | same | Windows in a row below `hysteresis × limit` before it clears |
| `detector.hysteresis` | `0.5` | same | Clear level as a fraction of the limit |
| `detector.latch` | `true` | same | Keep the alarm until `/<ns>/reset` |
| `detector.use_slow_detector` | `true` | (not used) | Slow channel; only in extended mode |
| `detector.use_command_check` | `true` | (not used) | Applied vs expected torque; only in extended mode |
| `detector.limit_override` | `0.0` | same | > 0 replaces the computed fast limit |
| `detector.slow_limit_override` | `0.0` | same | > 0 replaces the computed slow limit |
| `detector.sigma_floor` | `1e-3` | same | Smallest standard deviation used in standardisation |

**Calibration output** (`data/calibration/*.yaml`): fingerprint, duration, samples, `limit`, `slow_limit`, per-joint means, standard deviations and statistic scales, and the friction learned at the end of calibration. v1.2 values: limits 4.127 / 5.093 (v2) and 3.968 / 4.866 (thesis).

---

## 2. Tracking controller (`iiwa_controllers`)

File: `src/iiwa_sim/config/controllers.yaml`. The experiment launch writes a copy per run with that run's `noise_seed` and `halt_on_alarm`.

| Parameter | Default | Meaning |
|---|---|---|
| `update_rate` (controller_manager) | `1000` | Control loop rate (Hz); the IDS assumes 1 kHz |
| `joints` | `joint_a1` … `joint_a7` | Controlled joints, in order |
| `tip_joint` | `joint_a7` | Joint the payload is attached to |
| `kp`, `kd`, `effort_limit` | as the IDS | Gains and torque limits (defaults equal the IDS's) |
| `payload_com`, `payload_inertia` | as the IDS | Payload geometry; the mass comes from the reference |
| `reference_hold` | `0.05` | Extrapolation time of a reference sample (s) |
| `reference_timeout` | `0.5` | No reference for this long: hold position (starved) |
| `resume_time` | `0.5` | Blend time back onto the reference after a hold (s) |
| `halt_on_alarm` | `false` | Hold the arm while `/ids/alarm` is true (off in v1.2 runs) |
| `tick_clock` | `true` (sim) | Time = first update + n × 1 ms (exact windows); `false` on hardware |
| `encoder_bits` | `20` | Position resolution of the encoder model |
| `velocity_noise` | `0.001` | Velocity noise σ (rad/s) |
| `noise_seed` | `1` (per run) | Seed of the sensor noise |
| `measurement_decimation` | `10` | Publish one measurement every N ticks (one IDS window) |
| `reference_topic` / `measurement_topic` / `alarm_topic` | `/robot/reference` / `/robot/measurement` / `/ids/alarm` | Topics |

---

## 3. Simulation (`iiwa_sim`)

**Friction profiles** (`config/friction.yaml`): coefficient(t) = initial + rate × max(0, t − delay), capped at max, with t in simulation time.

| Profile | Initial (a1…a7, N·m·s/rad) | Growth | Use |
|---|---|---|---|
| `none` | 0 | — | Tests |
| `nominal` | 4, 4, 2, 2, 0.4, 0.2, 0.1 | none | **Calibration** |
| `wear` | same | +0.1 % of initial per s from 30 s, up to 2× | **Experiments** |
| `thesis` | a4 only | v1 schedule from 60 s | Reproducing v1 |

**Robot description arguments** (`urdf/iiwa_sim.urdf.xacro`): `friction_profile`, `controllers_file`, `friction_file`, `initial_positions_file`, `mesh_root`. Home pose (`config/initial_positions.yaml`): 0, 0.5, 0, −1.2, 0, 0.8, 0 rad.

**World** (`worlds/cell.sdf`): 1 ms physics step; source pad at (−0.5, 0), destination pad at (0, 0.5).

---

## 4. PLC planner (`ids_experiments`, node `plc_planner`)

| Parameter | Default | Meaning |
|---|---|---|
| `scan_period` | `0.01` | PLC scan = reference period (s) |
| `start_delay` | `2.0` | Settle time after the gripper reports `detached` (s) |
| `cycles` | `0` | Pick-and-place cycles (A→B→A); 0 = forever |
| `payload_mass` | `5.0` | Mass of the part (kg) |
| `source` / `destination` | `[-0.5, 0.0]` / `[0.0, 0.5]` | Station positions (m) |
| `hover_height` / `grab_height` / `drop_height` | `0.25` / `0.10` / `0.11` | Tool heights (m) |
| `travel_time` / `approach_time` | `4.0` / `2.0` | Minimum move times (s), stretched if the limits require |
| `dwell` | `0.5` | Settle time at the grab and after release (s) |
| `speed_scale` | `0.5` | Fraction of the joint velocity limits used |
| `max_acceleration` | `2.0` | Joint acceleration limit (rad/s²) |
| `wait_for_gripper` | `true` | Repeat detach until the gripper reports `detached` |
| `gripper_warning` | `30.0` | Warn if the gripper has not reported after this long (s) |
| `seat_parts` | `true` | Re-seat the part on its pad after each release |
| `world` / `payload_name` | `cell` / `payload` | Gazebo names used by `set_pose` |

## 5. Attacker (`ids_experiments`, node `mitm`)

| Parameter | Default | Meaning |
|---|---|---|
| `scenario_file` | `""` | Scenario YAML with the attacks; empty = forward unchanged |
| `input_topic` / `output_topic` / `truth_topic` | `/plc/reference` / `/robot/reference` / `/attack/truth` | Topics |
| `measurement_input` / `measurement_output` | `/robot/measurement_raw` / `/robot/measurement` | Relay used only when the scenario sets `cover_torque` (v1.4) |
| `kp`, `kd` | controller gains | What the cover assumes about the controller |
| `reference_hold` | `0.05` s | Longest time a setpoint is held, for the cover's torque computation |

---

## 6. Scenarios and campaign (`ids_bringup`)

**Scenario format** (written per run as `scenario.yaml`):

```yaml
name: step_a4
seed: 1
friction_profile: wear          # none | nominal | wear | thesis
duration: 180.0                 # seconds after the first trusted reference
planner: {cycles: 0, payload_mass: 5.0}
attacks:
  - {id: 1, kind: step, joint: 4, magnitude: 0.02, start: 120.0, duration: 10.0}
```

`kind`: `step`, `ramp`, `sine`, `scaling`, `freeze`, `replay`, `delay`, `noise`, `coordinated` or `drift_mimic` (v1.4; effects in [`docs/v1.4.md`](v1.4.md#2-the-ten-attack-kinds-and-the-torque-cover)). `joint`: 1–7, or `joints: [2, 4]` with `magnitudes: [...]` for several joints. `frequency` (Hz) for `sine`, `lag` (s) for `replay` and `delay`; `freeze`, `replay` and `delay` need no magnitude. `cover_torque: true` adds the falsified torque report. Attacks must not overlap and must end within `duration`.

**Campaign** (`config/campaign.yaml`): `defaults`, single `scenarios` (`smoke` 60 s, `calibration` 130 s with nominal friction, `nominal` 600 s), `families` expanded per joint (`step_a1` … `step_a7`: 0.02 rad for 10 s; `ramp_a1` … `ramp_a7`: 0.001 rad/s for 30 s; both at 120 s in 180 s runs), and the default order. Seeds 1–3 by default. v1.4 adds families on a4 for every new kind, `coordinated_a2a4`, `step_a4_cover`, and `sweeps:` with `joints` and `magnitudes` lists, expanded to `<sweep>_a<joint>_m<magnitude>` (`step_sweep`: 0.0025–0.02 rad; `ramp_sweep`: 0.00025–0.001 rad/s; all 7 joints). Sweeps and the new kinds are not in the default order.

## 7. Launch and command-line options

| Command | Options |
|---|---|
| `ros2 launch ids_bringup experiment.launch.py` | `scenario` (`smoke`), `seed` (`1`), `campaign` (installed file), `output_dir` (`/ws/data/runs`), `gui` (`false`), `halt_on_alarm` (`false`) |
| `ros2 launch ids_bringup sim.launch.py` | `friction_profile` (`wear`), `controllers_file`, `payload_mass` (`5.0`), `gui` (`false`) |
| `ros2 launch ids_ros ids.launch.py` | `run_mode` (`armed` \| `calibrate`), `thesis` (`true`) |
| `ros2 run ids_bringup run_campaign` | `--only NAME/PATTERN …`, `--seeds N …`, `--dry-run`, `--rerun`, `--gui`, `--halt-on-alarm`, `--output-dir`, `--campaign`, `--pause` (5 s) |

| `ros2 run ids_evaluation evaluate` | `RUN… --out DIR`, `--legacy FILE`, `--campaign`, `--calibration-dir` (`/ws/data/calibration`), `--limits V2 V2_SLOW THESIS THESIS_SLOW`, `--include-saturated`, `--grace` (1 s), `--tail` (30 s), `--no-plots` |
| `ros2 run ids_evaluation compare_baselines` | as `evaluate`, plus `--train RUN`, `--urdf` (default: xacro of `iiwa_sim`), `--duration` (120 s), `--skip-replay`, `--skip-momentum`, `--replay-binary` |
| `ros2 run ids_evaluation export_run` / `replay_run` / `split_legacy` | `RUN… [--force]` / `RUN --out OUT.npz (--calibrate C \| --calibration C) [--params ids.yaml] [--set name=value]` / `EXPORT --out DIR` |
| `ids_replay` (installed by `ids_ros`) | `--urdf --params --reference --measurement --output`, `--calibrate OUT [--duration S]` or `--calibration IN`, `--warmup` (2 s), `--set name=value` |

**Run supervisor** (`run_supervisor`, set by the experiment launch): `duration`, `summary_file`, `start_timeout` (300 s wall time), `progress_period` (30 s), `namespaces` (`ids`, `ids_thesis`), `effort_limits`, `saturation_warning` (0.01 = 1 % of windows).

## 8. Containers (`compose.yaml`)

| Setting | Value | Why |
|---|---|---|
| `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` | `0` / `rmw_cyclonedds_cpp` | Same DDS everywhere |
| `CYCLONEDDS_URI` | interface `10.203.20.0` | Use the `monitor_net` interface, so the IDS sees the robot (B17) |
| `monitor_net` subnet | `10.203.20.0/24` | Fixed, so the interface can be selected by address |
| Volumes | `./src` → `/ws/src`, `./data` → `/ws/data`, named build volumes | Edit on the host, build in the container, keep results on the host |

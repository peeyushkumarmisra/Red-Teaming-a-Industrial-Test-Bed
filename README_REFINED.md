# Red-Teaming an Industrial Robot Testbed — v1.2 `mp-core-sim`

### Physics-informed intrusion detection for a KUKA LBR iiwa, rebuilt so its results can be trusted

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-F58113)](https://gazebosim.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-20-00599C)](#9-technologies-retained-introduced-and-replaced)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB)](#9-technologies-retained-introduced-and-replaced)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue)](LICENSE)
[![Release](https://img.shields.io/badge/release-v1.2.0-success)](CHANGELOG.md)

> **In one sentence.** A model of the robot's physics watches all seven joints, compares how the arm actually moves with what the trusted PLC asked for, and raises an alarm when someone falsifies the setpoints on their way to the robot, while slow friction wear is learned instead of being mistaken for an attack.

> [!IMPORTANT]
> **The measured results in this release are narrow:** one seed, one attack on joint a4 and one 600 s nominal run. There is no per-hour false-alarm rate, no ramp-attack result and no result for the other six joints yet. Everything else is labelled *implemented and unit-tested, not run in simulation*. See [§12](#12-measured-improvements) and [§18](#18-current-limitations).

---

## Contents

| | | |
|---|---|---|
| [1. Project overview](#1-project-overview) | [8. Architecture evolution](#8-architecture-evolution) | [15. Configuration](#15-configuration) |
| [2. Original objective](#2-original-objective) | [9. Technologies](#9-technologies-retained-introduced-and-replaced) | [16. Testing & verification](#16-testing--verification) |
| [3. Original capabilities](#3-original-capabilities-v100) | [10. Algorithms & decisions](#10-algorithms--implementation-decisions) | [17. Benchmarks](#17-benchmarks) |
| [4. Original architecture](#4-original-architecture--technologies-v100) | [11. Final capabilities](#11-final-capabilities-v120) | [18. Current limitations](#18-current-limitations) |
| [5. Original limitations](#5-original-limitations) | [12. Measured improvements](#12-measured-improvements) | [19. Future work](#19-future-work) |
| [6. Problems identified](#6-engineering-problems-identified) | [13. Installation & setup](#13-installation--setup) | [Challenges & how we solved them](#challenges-faced--how-we-solved-them) |
| [7. Key changes](#7-key-changes--rationale) | [14. Running experiments](#14-running-experiments) | [Repository layout](#repository-layout) |

---

## 1. Project overview

Industrial robots take their motion commands from a PLC over a network. An attacker who sits on that network (a man-in-the-middle, MitM) can change the setpoints slightly; the robot obeys, the product is damaged or the cell becomes unsafe, and the controller itself sees nothing wrong because it is simply following orders.

This project is a **simulated industrial robot cell used as a red-team testbed** for that threat, and an **intrusion detection system (IDS)** that catches it from physics:

- the IDS taps the PLC's reference on the **trusted** side of the network;
- it replays the robot's own controller and rigid-body dynamics to predict how each joint should move;
- it estimates, for every joint and every 10 ms, the extra torque that would explain what the encoders actually saw;
- a dual-timescale statistic separates **slow wear** (learned as friction) from **fast attacks** (alarm), and names the attacked joint.

This repository started as the code of an MSc dissertation (EMATM0055, tag `v1.0.0`). Release **v1.2.0** (branch `mp-core-sim`) rebuilds that experiment from the ground up: correct maths, a deterministic simulation, wear that really reaches the joints, containers, tests and CI. Later releases add more attack types (v1.4), SROS2 security (v1.6) and a full OT network with OpenPLC and Modbus (v1.8).

| | |
|---|---|
| **Robot** | KUKA LBR iiwa 14 R820 (7 joints), Gazebo Harmonic, `ros2_control` at 1 kHz |
| **Task** | Pick-and-place of a 5 kg cube between two stations, forever |
| **Wear** | Viscous friction on every joint grows 0.1 % of its initial value per second from 30 s |
| **Attack (measured)** | Setpoint MitM, +0.02 rad on joint a4 from 120 s to 130 s |
| **IDS** | EKF + disturbance inversion + Dual-EWMA, two instances side by side (v2 and the dissertation's design) |
| **Headline (one run each)** | v2: **no false alarm in 600 s of wear; attack detected 50 ms after the first falsified setpoint, on a4.** Dissertation design: missed the attack. |

---

## 2. Original objective

From the dissertation, in its own terms:

> Decouple slow mechanical friction drift from fast false-data-injection attacks (FDIAs) using a physics-informed state observer with Dual-EWMA adaptive thresholding, keeping the **false-positive rate below 1 %** during drift and the **detection latency below 100 ms**.

v1.2 keeps this objective unchanged and adds what is needed to test it honestly: every joint monitored, the attacked joint named, calibrate once, and the dissertation's own design measured side by side on the same data.

---

## 3. Original capabilities (v1.0.0)

What the committed v1 code did, verified against the code and its CSV log during the Phase 1 audit:

| Aspect | v1.0.0 as committed |
|---|---|
| Robot and simulation | KUKA iiwa in Gazebo, a Python PD controller on a 1 kHz `rclpy` timer, no gravity compensation |
| Attack | Feedback spoof on `/joint_states` (0.8 rad step on joint 4, 0.5 s), 6 events |
| Detection | Norm of a momentum-observer residual over all joints; the EKF ran but its result was discarded |
| Thresholding | Constant limit from a 0–30 s burn-in; slow average silently frozen while alarmed |
| Wear | FrictionDrift plugin, at most about 0.44 N·m of torque by the end of the run |
| Reported metrics | Reproduced exactly from the CSV: FPR 0.55 %, recall 57.78 %, precision 72.90 %, F1 0.6446, mean latency 25 ms |
| Build and tests | No clean build from the repository; no automated tests |

---

## 4. Original architecture & technologies (v1.0.0)

```mermaid
flowchart LR
    TP["task_planner (Python, ikpy, wall-clock sleeps)"] -- "/joint_trajectory" --> VC["vulnerable_controller (Python PD, 1 kHz rclpy)"]
    VC -- "effort command" --> GZ[("Gazebo + FrictionDrift + DetachableJoint")]
    GZ -- "/joint_states" --> ATK["attacker_node (feedback spoof)"]
    ATK -- "/spoofed_joint_states" --> VC
    VC -- "commanded torque" --> IDS["ids_node (momentum observer + EKF + Dual-EWMA)"]
    GZ -- "/joint_states" --> IDS
    IDS -- "CSV log" --> EVAL["evaluation script"]
```

Technologies: ROS 2, Gazebo, Pinocchio, a C++ IDS node, Python controller, planner and attacker, a hand-edited `iiwa.urdf`, `run_exp.sh`, one dev container.

---

## 5. Original limitations

- **The observer could not see the attack it was meant for.** It was driven by the commanded torque, which a setpoint attacker also controls (finding **M1**).
- **The alarm did not come from the EKF**, and the momentum observer had a sign error (`C·q̇` instead of `Cᵀ·q̇`).
- **Wear was almost absent**: about 0.44 N·m at most, and the plugin's torque was probably overwritten by the controller interface.
- **All 87 false positives** came within 0.26 s after an attack ended, none at friction onset, so the wear claim was never actually tested.
- **Timing ran on the wall clock**, while the simulation ran 1.4–1.55× slower than real time.
- **The control limit was inflated about 6.9×** by the start-up transient in the burn-in window.
- **No clean build** was possible (missing dependencies, 30 hard-coded paths), there were **no tests**, and flake8 reported **218 findings**.

---

## 6. Engineering problems identified

The Phase 1 audit listed 26 defects (D1–D26) and six maths findings (M1–M6). The ones that shaped v1.2:

| ID | Problem | Consequence |
|---|---|---|
| **M1** | The innovation stays zero under a setpoint attack when the model input is the commanded torque | Drive the model from the **trusted reference** |
| **M2** | Per-joint residuals are coupled through the inverse mass matrix (a joint-4 attack appears at 82 % on joint 6) | **Disturbance inversion** d̂ = Φ⁻¹·r gives the attacked joint exactly |
| **M3** | The threshold used σ of the fast average, not of fast − slow | Exact, time-varying variance of the difference |
| **M4** | Under a linear drift fast − slow settles at 195 × slope, so wear and a slow ramp look identical | Learn friction in the EKF, plus a slow-channel detector |
| **M5** | L = 3 on correlated EWMA output gives about 700 alarm episodes per hour | Limits chosen from a **false-alarm budget per hour**, with debounce |
| **M6** | Encoder noise of 1e-4 rad² corresponds to a 7.5-bit encoder | A realistic 20-bit encoder, modelled in the controller |
| D1–D3 | Wrong residual, ill-posed EKF, wrong momentum-observer dynamics | New estimator (C05–C07) |
| D4, D5 | Burn-in transient; wall-clock timebase | Calibrate once on sim time (C11, C12) |
| D6, D24 | Payload replaced the link's mass instead of being added | Payload composition (C04) |
| D10, D12 | Wear negligible and possibly never applied | Rewritten FrictionDrift with ground truth (C14) |
| D11, D13, D25 | Torque clipping, no gravity compensation, jitter | C++ `ros2_control` controller (C13) |
| D14, D23 | Wall-clock planner, unseeded IK (wrist flips) | PLC-style planner on sim time, seeded IK (C15) |
| D16–D20 | No clean build, hard-coded paths, hand-edited URDF, no tests | Containers, xacro, tests, CI (C01, C14) |

---

## 7. Key changes & rationale

| ID | Change | Why |
|---|---|---|
| C01 | Docker images (`robot`, `ids`), 7-service Compose file, CI | Anyone can rebuild the exact environment; the IDS is isolated on the trusted network |
| C02 | `ids_interfaces` messages | One explicit contract between the cell and the IDS |
| C03 | Shared control law (`iiwa_control_law`) | The IDS replays exactly the controller's law; they can never drift apart |
| C04 | Robot model with payload composition and derivatives | Correct dynamics with the part in the gripper (D6) |
| C05 | Closed-loop model, 10 ticks per window, analytic Jacobians | Predict the arm under its own controller from the trusted reference (M1) |
| C06 | EKF with thesis and extended modes, friction states, Joseph form, χ² gate, freeze and rollback | Learn wear without learning attacks (M4) |
| C07 | Disturbance inversion d̂ = Φ⁻¹·r with one refinement | Torque per joint: the attacked joint is named (M2) |
| C08 | Dual-EWMA with exact variance, slow-channel detector, baseline freeze and rollback | Correct statistics from the first window (M3, M4) |
| C09 | Limits from a false-alarm budget, debounce, hysteresis, latch, attribution | A real false-alarm rate instead of a per-sample α (M5) |
| C10 | Command check τ_applied − τ_nominal | Second, independent check of the controller's output |
| C11 | Calibrate once, with a fingerprint | Reuse a calibration safely; refuse it for another model or other settings |
| C12 | `ids_node` on sim time, trusted tap, status/alarm/reset | Deterministic, auditable decisions, one per window |
| C13 | `ReferenceTrackingController` (C++, 1 kHz, encoder model, tick clock) | Realistic sensing and an exact timeline for the IDS |
| C14 | `iiwa_sim`: xacro at launch, FrictionDrift with ground truth | Wear really applied, and measurable |
| C15 | PLC-style planner: 10 ms scans on sim time, seeded IK, trusted context | Repeatable motion; grabs that do not cause false alarms |
| C16 | Setpoint MitM node with ground truth | The dissertation's actual threat model |
| C17 | Bringup: launches, scenarios, campaign runner, run supervisor | Unattended, recorded, health-checked runs |
| C18 | `ids_evaluation` | **Moved to v1.4** (fast-track decision) |

The full list is in [`CHANGELOG.md`](CHANGELOG.md).

---

## 8. Architecture evolution

**v1.2 data flow** (the trusted tap is the key difference from v1):

```mermaid
flowchart LR
    PLC["PLC planner<br/>plc_planner"] -- "/plc/reference (100 Hz)" --> MITM["Setpoint MitM<br/>mitm"]
    MITM -- "/robot/reference" --> CTRL["Tracking controller<br/>1 kHz"]
    CTRL -- "joint torque" --> SIM[("Gazebo Harmonic<br/>KUKA iiwa + wear")]
    SIM -- "joint state" --> CTRL
    CTRL -- "/robot/measurement (100 Hz)" --> IDS["IDS v2<br/>ids"]
    CTRL -- "/robot/measurement" --> THESIS["IDS thesis mode<br/>ids_thesis"]
    PLC -. "trusted tap" .-> IDS
    PLC -. "trusted tap" .-> THESIS
    IDS -. "/ids/alarm (halt off in these runs)" .-> CTRL
```

**Containers and networks:**

```mermaid
flowchart TB
    subgraph control_net["control_net (robot side)"]
        robot["robot: Gazebo, controller, planner, MitM, supervisor, recorder"]
        attacker["attacker (placeholder)"]
        plc["plc (placeholder)"]
    end
    subgraph monitor_net["monitor_net (trusted)"]
        ids["ids: IDS v2 + thesis mode"]
        range["range (placeholder)"]
    end
    subgraph ops_net["ops_net (office side)"]
        hmi["hmi (placeholder)"]
        engws["eng-ws (placeholder)"]
    end
    robot --- ids
```

`robot` sits on `control_net` and `monitor_net` (the `plc` placeholder on all three); `ids` sits only on `monitor_net`, so the attacker's network never reaches it. Details: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

| | v1.0.0 | v1.2.0 |
|---|---|---|
| Model input | Commanded torque | Trusted PLC reference (thesis mode kept for comparison) |
| Residual | Norm of a momentum-observer residual | Disturbance torque per joint, d̂ = Φ⁻¹·r |
| Wear handling | Slow EWMA only | EKF friction states plus the slow channel |
| Limits | Constant, from a burn-in | From a false-alarm budget, calibrated once |
| Timing | Wall clock | Simulation time, exact 10 ms windows |
| Controller | Python PD, no gravity compensation | C++ `ros2_control`, gravity feed-forward, encoder model |
| Attack | Feedback spoof | Setpoint MitM (step and ramp, any joint) |
| Build | Not reproducible | Two Docker images, CI, about 200 tests |

---

## 9. Technologies retained, introduced and replaced

| Retained | Introduced | Replaced or dropped |
|---|---|---|
| ROS 2 and Gazebo | ROS 2 **Jazzy**, **Gazebo Harmonic** (gz-sim 8), pinned in Docker | Python PD controller → C++ `ros2_control` plugin |
| Pinocchio rigid-body dynamics | `ros2_control` + `gz_ros2_control` | Wall-clock task planner → PLC-style planner on sim time |
| EKF + Dual-EWMA concept | `ids_core`: ROS-free C++ detector library | `context_aware_ids` → `ids_core` + `ids_ros` |
| KUKA iiwa 14 meshes (15 of 21) | Docker Compose (`robot`, `ids` + 5 placeholders), Cyclone DDS | Hand-edited `iiwa.urdf` → launch-time xacro |
| FrictionDrift and DetachableJoint | GitHub Actions CI, gtest, pytest, flake8 | `custom_gazebo_plugins` → `iiwa_sim` FrictionDrift |
| ikpy (pinned 4.5.0) | MCAP recording, run supervisor, YAML calibration | `run_exp.sh` + CSV → launches, campaign runner, bags |
| — | `ros_gz_bridge`, `yaml-cpp` | **Dropped:** RLS estimator; `.devcontainer/Dockerfile` |

---

## 10. Algorithms & implementation decisions

### 10.1 Closed-loop prediction (C03, C05)

Every 10 ms window holds 10 controller ticks. For each tick the IDS computes the controller torque from the **trusted** reference with the shared law

$$\tau = K_p\,(q_{ref} - q) + K_d\,(\dot q_{ref} - \dot q) + g(q), \quad \text{clamped to the effort limits,}$$

subtracts viscous friction $b \odot \dot q$, and integrates the rigid-body dynamics (semi-implicit Euler, 1 ms). The analytic Jacobians give the state transition $F$ and the disturbance map $G$ of the window.

### 10.2 Extended Kalman filter (C06)

State $x = [q;\ \dot q;\ b]$ (21 numbers), measurement $[q;\ \dot q]$ from the encoders. Joseph-form covariance update. Friction is updated **only** in extended mode, outside alarms, and when the normalised innovation squared is under a χ² gate (36.1 for 14 degrees of freedom, about the 0.1 % tail), so an attack is not learned as friction.

### 10.3 Disturbance per joint (C07)

The prediction is anchored at the **measured** state at the start of the window, so its velocity residual $r$ only contains what happened inside the window. With $\Phi$, the map from a constant torque per joint to the end-of-window velocities,

$$\hat d = \Phi^{-1} r_{\dot q},$$

followed by one Gauss–Newton refinement (replay the window with $\hat d$ and correct the rest). For a setpoint attack of γ rad on joint *i*, $\hat d_i \approx K_{p,i}\,\gamma$ and the other joints stay near zero. $\hat d$ is standardised by its propagated covariance, so a heavy payload (which makes the arm harder to move) does not inflate it (B10).

### 10.4 Dual-EWMA statistics (C08)

Fast average (λ = 0.2, τ = 0.045 s) and slow average (λ = 0.005, τ = 2 s) of each standardised $\hat d_i$:

$$s_{fast} = \frac{|Z_{fast} - Z_{slow}|}{\sigma_D(k)}, \qquad s_{slow} = \frac{|Z_{slow}^{live}|}{\sigma_S(k)}$$

with the **exact** standard deviations after $k$ samples. During an alarm the baseline is frozen (v2 only) and, at onset, rolled back 50 windows, so the start of an attack is not absorbed.

### 10.5 Detector and limits (C09, C10, C11)

- **Alarm:** 5 windows in a row above the limit (50 ms). **Clear:** 5 windows below half the limit. **Latch:** until `/ids/reset`.
- **Limits:** a Monte Carlo of 20 hours of white noise through exactly this detector gives the limit for a budget of **0.125 false alarms per hour**, shared by every channel and joint. The scale of each statistic on calibration data accounts for real, correlated noise. Results: v2 limits 4.127 (fast) and 5.093 (slow); thesis mode 3.968 (fast; its slow statistic is computed but does not decide).
- **Warm-up:** no alarms in the first 399 windows (2 × τ_slow) after arming or reset.
- **Attribution:** at each onset, the joint whose statistic is furthest above its limit.
- **Command check (v2):** the applied torque minus the torque the trusted reference implies, through its own Dual-EWMA.
- **Context guard:** for 0.3 s after a trusted payload change the IDS re-syncs instead of detecting (grabs are not modelled).
- **Fingerprint:** SHA-256 of the parsed robot model and every detection setting. A calibration is refused if either changed.

### 10.6 Thesis mode

Same pipeline, driven by the **commanded torque**, no friction states, no baseline freeze or rollback, and only the fast channel decides. This is the dissertation's design, kept to show its blind spot on the same data.

### 10.7 Simulation decisions

- **Tick clock (B19):** the controller counts its own 1 ms ticks instead of reading the late `/clock`, so every measurement window is exactly 10 ms.
- **FrictionDrift (B14, B21):** adds $-b(t)\,\dot q$ only to a torque the controller has written, after it (priority 100), and publishes the ground truth.
- **Handover order (B23):** settle → payload context on → attach → lift on the next scan, so the grab falls inside the context guard. After each release the part is re-seated on its pad (a locating nest).

---

## 11. Final capabilities (v1.2.0)

| Capability | Status |
|---|---|
| Setpoint MitM (step) on a4 detected and attributed in Gazebo | **Measured** (one run) |
| No false alarm under wear in Gazebo | **Measured** (one 600 s run) |
| Dissertation design measured side by side (blind spot reproduced) | **Measured** (one run) |
| IDS model matches Gazebo within its noise floor; friction learned within 2 % at constant friction | **Measured** (calibration run) |
| Wear really reaches the joints (ground truth published) | **Measured** |
| Step attack on every joint detected within 100 ms and attributed | Unit-tested on a synthetic cell; **not run in simulation** |
| Slow ramp attack detected by physics alone | Unit-tested; **not run in simulation** |
| Torque tampering after the controller detected (command check) | Unit-tested; **not run in simulation** |
| IDS → robot halt chain (halt, reset, resume without a new alarm) | Unit-tested; **not run in simulation** |
| Calibrate once, fingerprint-protected | Implemented and used in every run |
| Unattended campaign with health checks and resumable runs | Implemented; used for the two regression runs |
| No memory allocation in the real-time paths | Unit-tested |

---

## 12. Measured improvements

Only measured values; "not comparable" where the two releases measured different things.

| Metric | v1.0.0 (baseline) | v1.2.0 (measured) |
|---|---|---|
| Clean build from the repository | Expected to fail (missing dependencies) | All 7 packages build in both containers, no compiler output |
| Automated tests | 0 | About 200, all passing in their last reported runs (sandbox and user's machine) |
| flake8 findings | 218 | 0 |
| Measurement timing | Wall clock; simulation 1.4–1.55× slower than the log | Every window exactly 10.000 ms |
| Heap allocations in the real-time path | Per cycle | None (tested) |
| Wear torque at the joints | At most about 0.44 N·m, probably never applied | Applied and confirmed: thesis-mode d̂ follows the true friction torque with slope 1.00–1.02 |
| Model fit (v2, calibration) | — | d̂ mean ≤ 0.006 N·m on every joint; friction a1–a6 within 2 % |
| False alarms under wear | 87 false-positive samples, all just after attacks; wear negligible | **0 alarm samples in 600 s**, friction up 57 % by the end (one run) |
| Latency, setpoint attack | Not measured (v1 attacked the feedback, 0.8 rad) | **50 ms** after the first falsified setpoint, 0.02 rad on a4 (one run) |
| Dissertation design vs setpoint MitM | Not tested | Missed it (one run) |
| IDS time per 10 ms window | Not measured | Median 181–233 µs (shared 2-core VM) |

### Results of the two regression runs

| Run | IDS | Measured |
|---|---|---|
| `nominal`, 600 s, wear | v2 | 0 alarm samples. Highest readings in the first 240 s: fast 0.955×, slow 0.745× the limit |
| | thesis | 2947 alarm samples: first-move alarm at 6.05 s (a4), then short a6 alarms from 177 s as wear grows |
| `step_a4`, +0.02 rad, 120–130 s | v2 | Alarm at 120.055 s, 50 ms after the first falsified setpoint, on a4. Peaks: fast 111×, slow 429×, command check 9022× the limit. a7 also flagged briefly at the attack edges (B4). Cleared 24.3 s after onset |
| | thesis | No alarm during the attack (4 windows over the limit at the start, one short of the debounce); 0.14 s alarm when the setpoint stepped back |

![IDS statistics, step_a4 run](docs/v1.2/v1.2_statistics_step_a4.png)

*Joint a4's fast and slow readings divided by their limits (dashed line = alarm limit), with the highest reading of the other joints. Grey band: the attack. Red ticks: alarms. v2 (top) jumps two orders of magnitude above the limit; thesis mode (bottom) barely reacts.*

More graphs and the full discussion of these results: [`docs/v1.2.md`](docs/v1.2.md).

---

## 13. Installation & setup

**You need:** Docker (Docker Desktop with WSL2 on Windows, or Docker Engine on Linux), Git, about 15 GB of disk and 8 GB of RAM for the containers. No GPU is needed; Gazebo runs headless. Optional: VS Code with the Dev Containers extension, and an X server on Windows for the Gazebo GUI.

```bash
git clone https://github.com/peeyushkumarmisra/red-teaming-a-industrial-test-bed.git
cd red-teaming-a-industrial-test-bed
git checkout mp-core-sim

docker compose build robot ids     # builds the two images
docker compose up -d robot ids     # start both containers
```

**Build the workspace inside each container:**

```bash
# robot container (VS Code "Reopen in Container", or: docker compose exec robot bash)
cd /ws && colcon build --packages-up-to ids_bringup && source install/setup.bash

# ids container (docker compose exec ids bash)
cd /ws && colcon build --packages-up-to ids_ros && source install/setup.bash
```

The source folder `src/` and the results folder `data/` are shared with your machine; build output stays inside the containers.

---

## 14. Running experiments

All times are simulation time. Each run writes `data/runs/<date-time>_<scenario>_s<seed>/` with `summary.yaml` and an MCAP bag.

| Step | IDS container | Robot container |
|---|---|---|
| 1. Smoke test (60 s, IDS optional) | — | `ros2 launch ids_bringup experiment.launch.py scenario:=smoke` |
| 2. Calibrate (once) | `ros2 launch ids_ros ids.launch.py run_mode:=calibrate` | `ros2 launch ids_bringup experiment.launch.py scenario:=calibration` |
| 3. Arm the IDS | `ros2 launch ids_ros ids.launch.py` | — |
| 4. One experiment | (keep running) | `ros2 launch ids_bringup experiment.launch.py scenario:=step_a4 seed:=1` |
| 5. Regression set | (keep running) | `ros2 run ids_bringup run_campaign --only nominal step_a4 --seeds 1` |
| 6. Full campaign (45 runs, about 3 h) | (keep running) | `ros2 run ids_bringup run_campaign` |

**Check a run:**

```bash
cat /ws/data/runs/*_step_a4_s1/summary.yaml      # status, alarm counts, first alarm, warnings
ros2 bag info /ws/data/runs/*_step_a4_s1/bag
```

A healthy run has `status: complete`, `torque_saturation` near 0 and no warnings. The calibration files are `data/calibration/ids_v2.yaml` and `ids_thesis.yaml`; calibrate again after any change to the robot model or the IDS settings (the IDS refuses an old one).

Scenarios: `smoke`, `calibration`, `nominal`, `step_a1` … `step_a7`, `ramp_a1` … `ramp_a7` (see `src/ids_bringup/config/campaign.yaml`).

---

## 15. Configuration

| What | Where |
|---|---|
| IDS settings (v2 and thesis) | `src/ids_ros/config/ids.yaml`, `ids_thesis.yaml`; defaults in `src/ids_core/include/ids_core/config.hpp` |
| Controller | `src/iiwa_sim/config/controllers.yaml` |
| Friction profiles | `src/iiwa_sim/config/friction.yaml` |
| Start pose | `src/iiwa_sim/config/initial_positions.yaml` |
| Planner | parameters of `plc_planner` (station positions, heights, move times) |
| Scenarios and campaign | `src/ids_bringup/config/campaign.yaml` |
| Containers and networks | `compose.yaml` |

Every IDS setting is a ROS parameter with the same dotted name (`detector.debounce`, `ekf.q_friction`, …). The full reference, with defaults and what each one does, is in [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md).

---

## 16. Testing & verification

```bash
# ids container
colcon test --packages-select iiwa_control_law ids_core ids_ros && colcon test-result --verbose

# robot container
colcon test --packages-select iiwa_controllers iiwa_sim ids_experiments ids_bringup
colcon test-result --verbose
flake8 --max-line-length 99 --extend-ignore E203 src
```

| Package | What the tests prove |
|---|---|
| `iiwa_control_law` (7) | The control law, hold, clamp and the Kp·γ signature of a setpoint attack |
| `ids_core` (59) | Dynamics and derivatives, closed-loop model, EKF, Dual-EWMA, detector, limits, fingerprint, pipeline, end-to-end attack scenarios, no allocation |
| `ids_ros` (9) | Every parameter reaches the configuration; message conversions are exact |
| `iiwa_controllers` (22) | Controller state machine, encoder model, tick clock, plugin loading, and the controller–IDS contract (attack → detect → halt → resume) |
| `iiwa_sim` (18) | Wear schedule, and that the simulated robot equals the IDS's model |
| `ids_experiments`, `ids_bringup` | Trajectories, handover order, attacks, scenarios, IK, start-up and shutdown, campaign, run files, health checks |

How each check was verified (sandbox, user's machine, or Gazebo), and what is still open: [`docs/TESTING.md`](docs/TESTING.md).

---

## 17. Benchmarks

`bench_ids_step` times the IDS on one 10 ms window, the real-time budget:

```bash
# after building ids_core with tests (ids container)
/ws/build/ids_core/bench_ids_step 5000
```

| Machine | Median | p90 | p99 |
|---|---|---|---|
| Shared 2-vCPU VM (Xeon 2.1 GHz), several runs | 181–233 µs | 246–324 µs | 237–427 µs |
| Development PC (Ryzen 7 5800H) | not measured yet | | |

That is about 2 % of the 10 ms budget. The GPU is not used.

---

## 18. Current limitations

| Limitation | Status / plan |
|---|---|
| **Narrow results**: one seed, attacks on a4 only, one nominal run; no per-hour false-alarm rate, no ramp in simulation | Campaign scheduled after v2.0 |
| **B4**: an attack onset that is not on a window boundary can also flag a second joint for a few windows (a7 in `step_a4`); the main joint is still right | v1.4 |
| **B24**: under wear, the friction estimates on a1–a4 trail the truth (up to −18 % on a4 after 240 s); detection was unaffected in the measured run | Estimator decision (needs approval) |
| a7 friction cannot be learned while a7 does not move | Candidate: stop learning on joints without motion |
| Thesis mode alarms at the first move from home and later under wear | Expected: no friction model |
| 0.3 s blind window after each trusted payload change | Accepted, documented |
| The simulated gripper does not centre the part; the planner re-seats it | Simulation shortcut |
| White-box: the IDS knows the controller and taps the trusted PLC output; the PLC itself is trusted | Design assumption |
| The IDS model and Gazebo use the same URDF; only viscous wear (no Coulomb, stiction, backlash) | Real hardware needs identification |
| Graphs were produced with offline scripts that are not yet in the repository | `ids_evaluation` (C18) in v1.4 |
| Ramp attacks, other joints and the halt chain are implemented and unit-tested, not run in simulation | Results backlog |
| CI has not run yet | First push of this branch |

---

## 19. Future work

| Release | Branch | Adds |
|---|---|---|
| **v1.4.0** | `mp-attacks-eval` | `ids_evaluation` (latency, false alarms per hour, per-joint table, plots); ten attack kinds (step, ramp, sine, scaling, freeze, replay, delay, noise, coordinated, drift-mimic); sweep runner; offline replay; baselines; falsified torque report; onset timing inside a window (B4) |
| **v1.6.0** | `mp-sros2` | SROS2 keystore and permissions; a rogue node without credentials is blocked; a stolen-credential MitM passes SROS2 and the IDS still catches it; IDS → robot halt chain |
| **v1.8.0** | `mp-ot-network` | OpenPLC with a structured-text sequencer, Modbus gateway, engineering-workstation update attack, attacker proxy, web HMI, recording range |
| **v2.0.0** | `mp-release` | Hardened CI, full documentation, merge into `main`, `legacy/v1` branch |

**Results backlog (after v2.0):** the 45-run v1.2 campaign (seeds 1–3, all joints, step and ramp), every attack kind and sweep, minimum-detectable-attack curves, SROS2 overhead, Modbus attacks.

**Open estimator decisions:** a cap on the slow channel's input (post-attack tail), a7 friction observability, the position-noise setting (NIS median 4.6 against 14 expected), and friction tracking under wear (B24). MEWMA for coordinated attacks is planned for v1.4.

---

## Challenges faced & how we solved them

Every problem below is logged with its root cause and fix in the project's iteration log. Status: ✅ resolved, ⚠️ mitigated or open.

### In the algorithm (found in unit tests)

| # | Problem | Root cause | How we solved it | |
|---|---|---|---|---|
| B1 | A disturbance on one joint leaked up to 5.7 N·m into the others | The estimate came from the innovation, which accumulates filter error | Estimate from a prediction anchored at the measured state | ✅ |
| B2 | Friction learning gave inconsistent values in a test | The test fed 120 s of reference at once, so the arm hardly moved | Feed references as time advances, like the runtime | ✅ |
| B3 | Large disturbances on light joints leaked into others | The window map Φ is first order | One Gauss–Newton refinement (leak 0.123 → 0.00016 N·m) | ✅ |
| B4 | An attack starting mid-window can flag a second joint for a few windows | Φ assumes the disturbance lasts the whole window | Attribution of the main joint made robust; onset timing within the window is v1.4 work | ⚠️ |
| B6 | Resuming after a halt and reset raised the alarm again | The IDS predicted tracking while the controller held or blended | The controller flags any non-tracking window; the IDS skips it | ✅ |
| B8 | A calibration made with constant friction was refused in wear runs | The fingerprint hashed raw URDF text with simulator-only tags | Hash the parsed model instead | ✅ |
| B9, B10 | Grabbing the 5 kg part raised alarms | The physical grab is not modelled, and the noise scale ignored the payload | 0.3 s context guard; noise from the propagated covariance | ✅ |

### In the build and the simulation (found on the user's machine and in Gazebo)

| # | Problem | Root cause | How we solved it | |
|---|---|---|---|---|
| B12 | 4 compiler warnings in the controller | Written against an older `hardware_interface` API | `get_optional()` and checked `set_value()` | ✅ |
| B13, B14 | Gazebo ignored the friction plugin's run order | Wrong `ConfigurePriority` signature | Parameterless override, priority 100 | ✅ |
| B15 | A new run could inherit the last run's latched alarm | The node did not reset its published alarm on restart | Publish `false` on every restart | ✅ |
| B16 | The part could stay attached to the flange from start-up | The gripper attaches by default and drops early detaches | Repeat detach until the gripper reports `detached` | ✅ |
| B17 | The IDS container received nothing | DDS chose the wrong network interface | Fixed subnet and interface selection | ✅ |
| B18 | Tests errored on Jazzy's pytest 7 | A helper named `setup` is treated as a setup hook | Renamed; tests run on pytest 7 and 9 | ✅ |
| B19 | Windows of 9 or 11 ms instead of 10 | The controller timed itself by the late `/clock` | Tick clock: exactly 1 ms per update | ✅ |
| B20 | A new run attached to the previous run's Gazebo | The Gazebo server survived shutdown under a new name | Find and stop leftover servers before each run | ✅ (2 attempts) |
| B21 | Every early run started with the arm collapsed | The friction plugin's force command disabled Gazebo's joint hold | Only add to a force the controller wrote; flag saturated runs | ✅ |
| B22 | Traceback at shutdown | An interrupted message conversion raised `RuntimeError` | One tolerant shutdown helper | ✅ |
| B23 | False alarms at grabs, alarms that never cleared, part drifting | Late baseline freeze, no warm-up, grab order, no part seating | Live slow detector with rollback, warm-up, new grab order, part re-seating | ✅ (2 attempts) |
| B24 | Friction estimates trail the wear on a1–a4 | Not diagnosed; candidate: friction process noise too small | Open: needs an estimator decision | ⚠️ |

---

## Repository layout

```
.
├── README_REFINED.md          this document
├── CHANGELOG.md               release history
├── CONTRIBUTING.md            how to work on the project
├── LICENSE                    Apache License 2.0
├── compose.yaml               containers and networks
├── docker/                    robot and IDS images, entrypoint
├── docs/
│   ├── ARCHITECTURE.md        nodes, topics, timing, containers
│   ├── CONFIGURATION.md       every parameter
│   ├── TESTING.md             test suites and verification status
│   ├── v1.2.md                release notes, results and graphs
│   └── v1.2/                  result graphs
└── src/
    ├── ids_interfaces/        messages
    ├── iiwa_control_law/      shared control law and robot model
    ├── ids_core/              the IDS algorithm (no ROS)
    ├── ids_ros/               the IDS node and its configs
    ├── iiwa_controllers/      the ros2_control tracking controller
    ├── iiwa_sim/              Gazebo robot, wear plugin, world
    ├── ids_experiments/       PLC planner, MitM attacker, scenarios
    └── ids_bringup/           launches, runs, campaign
```

Every source file starts with a short header explaining what it does and why it matters for detecting attacks. Four files carry no comments because their formats or roles do not allow it: `src/iiwa_sim/NOTICE` (legal text), `src/iiwa_sim/env-hooks/iiwa_sim.dsv.in` (colcon hook format) and the two empty `resource/` marker files.

---

## Acknowledgements

Robot meshes and kinematic and inertial parameters come from the `iiwa_description` package of ICube-Robotics/iiwa_ros2 (Apache-2.0); see `src/iiwa_sim/NOTICE`. The detection idea comes from the author's MSc Robotics dissertation, *Physics-Informed Intrusion Detection System (IDS) for Industrial Manipulators using Adaptive Dynamic Thresholding* (EMATM0055, University of Bristol and University of the West of England, 2026).

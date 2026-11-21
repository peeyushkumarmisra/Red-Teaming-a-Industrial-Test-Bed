# Contributing

Thank you for helping. This project is a research testbed: a change is only useful if its effect on detection can be measured and reproduced. The rules below keep it that way.

## Branches and releases

- Each release lives on its own branch, started from the previous tag: `v1.2-ids-core-sim` (v1.2) → `v1.4-ids-attacks-eval` (v1.4) → `mp-sros2` (v1.6) → `mp-ot-network` (v1.8) → `mp-release` (v2.0, merged into `main`).
- `v1.0.0` is the original dissertation code and is never changed.
- Every release has a short spec, an implementation, a test phase in Gazebo (at least the regression set in `docs/TESTING.md`), and documentation. Record what changed in `CHANGELOG.md`.

## Environment

Work inside the containers (`docker compose up -d robot ids`, then VS Code "Reopen in Container" or `docker compose exec`). Build with `colcon build`; never commit `build/`, `install/`, `log/` or `data/`.

## Style

| Language | Rule | Tool |
|---|---|---|
| C++ | Google style, 100 columns | `clang-format -i <file>` (`.clang-format`) |
| Python | PEP 8, 99 columns; Black formatting is fine | `flake8 src` (`.flake8`, E203 ignored for Black) |
| All | UTF-8, LF line endings, final newline | `.editorconfig`, `.gitattributes` |

**Comments.** Every source file starts with a short header: what the file does and why it matters for detecting attacks. Comment the intent and the design decisions (why), not each line (what). Keep comments in plain English, ASCII only (ROS message generation and some tools reject other characters), and never put `--` or `${...}` inside an XML or xacro comment.

## Tests

- Every change to the IDS, controller or planner logic comes with a unit test. Keep the logic in the ROS-free libraries (`ids_core`, the tracking core, the friction model, the Python modules) so it can be tested without ROS or Gazebo.
- The real-time paths (`IdsPipeline::process`, `TrackingCore::update`) must not allocate memory: `test_realtime` and `UpdateNeverAllocates` check this.
- Python tests must pass on pytest 7.4.4 (Ubuntu 24.04 / Jazzy) and on current pytest. Do not name a helper `setup` at module level.

## Changing the IDS

- A new IDS setting needs: a field in `ids_core/config.hpp`, a line in the binding table of `ids_ros/src/parameters.cpp`, the new expected count in `test_parameters.cpp` (39 parameters in v1.2), and an entry in `docs/CONFIGURATION.md`. If it changes detection, add it to `configSignature` so the fingerprint changes.
- Any change that affects detection invalidates existing calibrations: recalibrate before comparing results.
- Changes to the estimator or detector maths (noise settings, MEWMA, slow-channel caps, friction observability) need an explicit decision before implementation, and a before-and-after measurement.

## Adding an experiment

Add a scenario or a family to `src/ids_bringup/config/campaign.yaml` (format in `docs/CONFIGURATION.md` §6). `test_campaign.py` checks that every scenario is valid for the MitM.

## Reporting results

State the scope with every number: seeds, joints, attack, run length. Measure the false-alarm rate per hour over the campaign, not over a single run. Exclude runs that the supervisor flags as not nominal (torque saturation above 1 %).

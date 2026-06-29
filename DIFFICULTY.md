# Difficulties & Know-how

## D-001: CAN bridge Twist→track speed conversion bug — steering impossible

- **Date**: 2026-03-16
- **Situation**: The CAN bridge node receives ROS2 `cmd_vel` (Twist) and sends CAN commands to the left/right track motors
- **Issue**: In `_periodic_send()`, the same `linear.x * 3.6` was assigned to both left and right tracks. `angular.z` (rotation) was completely ignored, so **on the real vehicle only straight driving was possible, steering was impossible**
- **Trial and error**: Initially suspected a CAN protocol problem and re-analyzed the DBC file. It was actually a simple logic error in the ROS→CAN conversion layer
- **Resolution**: Implemented Twist → left/right differential speed conversion via `SkidSteerModel.twist_to_tracks()`. Straight (L=R), left turn (L<R), right turn (L>R), pivot (L=-R)
- **Alternative**: Sending angular as a separate field at the CAN message level → inconsistent with the existing MD2K protocol, rejected
- **Know-how**: For skid-steer vehicles you must not use Twist directly. **You must always go through twist_to_tracks() conversion**. Unit tests must include the 6 cases of straight/reverse/left turn/right turn/pivot/stop
- **Retrospective**: When first implementing the CAN bridge, only the straight-driving test was run. Had a rotation test been included from the start, it would have been caught immediately
- **Related files**: `src/ad_can_bridge/ad_can_bridge/can_bridge_node.py`, `src/ad_can_bridge/test/test_can_bridge_logic.py`

## D-002: Curtis→MD2K motor controller misidentification — full simulation rework

- **Date**: 2026-03-25
- **Situation**: While writing the C49 cross-validation report, the motor controller was described as a Curtis 1226BL
- **Issue**: The real vehicle uses an **MDROBOT MD2K** (dual-channel DC motor driver). The control characteristics, protection features, and CAN protocol written based on Curtis were all wrong
- **Trial and error**: Precisely tuned simulation parameters based on the Curtis 1226BL datasheet → fundamentally the wrong controller baseline
- **Resolution**: Fully corrected 5 places in the C49 report + re-implemented based on MD2K in C50 (SS/SD acceleration/deceleration, protection features, CAN codec, thermal model)
- **Alternative**: Overlaying only MD2K correction values on top of the Curtis parameters → impossible because the architecture differs, rewrote from scratch
- **Know-how**: **Verifying hardware components by checking the actual vehicle in person comes before the datasheet**. Even for similarly named parts, if the manufacturer/model differs, the control characteristics are completely different. When reviewing HIH-2 project materials, cross-check the actually installed part by the content's context (CAN ID, parameter ranges), not by the file name
- **Retrospective**: Had I just asked Inhyeok "Which motor controller do you use?" at the start, I could have saved a day. Do not estimate from the datasheet alone
- **Related files**: `docs/프로젝트/task/C49_simulation_vs_actual_comparison.md`, `docs/프로젝트/task/C50_md2k_motor_controller.md`

## D-003: Gazebo SDF IMU noise type attribute missing — 8 syntax errors

- **Date**: 2026-03-22
- **Situation**: Built the Gazebo Harmonic environment on the home PC (WSL2) and started the simulation
- **Issue**: The `type` attribute was missing from the IMU/GPS sensor noise tags in model.sdf, so Gazebo could not load the sensors. The error message was vague, making it hard to pinpoint the cause
- **Trial and error**: Suspected a Gazebo version compatibility problem and tried reinstalling the plugin. A Nav2 parameter problem also occurred at the same time, causing confusion (needed to replace SmacPlannerLattice → SmacPlannerHybrid)
- **Resolution**: Added the `type="gaussian"` or `type="gaussian_quantized"` attribute in 8 places in the SDF file. Also fixed Nav2 at the same time
- **Alternative**: None, complying with the SDF spec is required
- **Know-how**: Gazebo SDF sensor noise tags must be in the `<noise type="gaussian">` form. `<noise>` cannot be used on its own. Validate SDF in advance with `gz sdf -k model.sdf`
- **Retrospective**: When manually editing the SDF file, I should have run XML schema validation first. Adding an SDF validation step to CI would prevent recurrence
- **Related files**: `src/ad_bringup/models/ss500/model.sdf`, `src/ad_bringup/config/nav2_params.yaml`

## D-004: ROS2 parameter double declaration — declare vs get confusion

- **Date**: 2026-03-30
- **Situation**: Adding a crop_row publisher to PerceptionNode
- **Issue**: A parameter already declared in the YAML parameter file was re-declared in code via `declare_parameter()` → raised `ParameterAlreadyDeclaredException`
- **Trial and error**: Suspected a parameter name typo and compared the YAML and code names. It was actually a problem with the declaration method itself
- **Resolution**: Changed `declare_parameter()` → `get_parameter()`. Declare in YAML, and only read in code
- **Alternative**: Removing the parameter from YAML and declaring it only in code → violates the YAML central-management principle
- **Know-how**: ROS2 parameter rule: **declare in YAML → read with `get_parameter()` in code**. Use `declare_parameter()` in code only to define a default value for a parameter not present in YAML. Mixing them causes a double-declaration error
- **Retrospective**: Had I decided the parameter management policy (YAML-first or code-first) at the start of the project, this confusion would not have happened
- **Related files**: `src/ad_perception/ad_perception/perception_node.py`, `src/ad_perception/config/perception_params.yaml`

## D-005: Simulation parameters off by 4x — developed using only estimated values

- **Date**: 2026-03-23
- **Situation**: Since February, the simulation had been running on estimated values (mass=200kg, gear_ratio=30, RPM=3000)
- **Issue**: When cross-validating against HIH-2 real-vehicle data (DB130-48 datasheet, CAN DBC, BLACKTAN tests), discovered errors of **vehicle mass 4x (200→800kg), gear ratio 1.5x (30→20), motor RPM 1.5x (3000→2000)**. The reliability of all simulation results was in doubt
- **Trial and error**: Spent a month tuning control parameters and testing path planning based on the estimated values. Because the results "looked plausible," the error went unnoticed
- **Resolution**: Fully corrected 6 files in C48 (VehicleDynamicsConfig, DrivetrainConfig, SensorNoiseModel, SDF). Confirmed all 218 tests passed again
- **Alternative**: Setting parameters as ranges (min/max) and running Monte Carlo simulation → excessive complexity, rejected
- **Know-how**: **Cross-validate against real-vehicle data from the start of the simulation**. If you use estimated values with a "this should be roughly right" attitude, you face full rework later. In particular, mass, gear ratio, and motor rating have a nonlinear effect on simulation results. Secure the datasheet first, and if it's unavailable, explicitly track it as a TODO
- **Retrospective**: I should have analyzed the HIH-2 data first on Day 1 of the project. Run sim-vs-real cross-validation as a continuous process rather than a milestone
- **Related files**: `src/ad_core/ad_core/vehicle_dynamics.py`, `src/ad_core/ad_core/drivetrain_model.py`, `src/ad_bringup/models/ss500/model.sdf`, `docs/프로젝트/task/C48_hih2_parameter_update.md`

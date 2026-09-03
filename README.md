# trash_modular

A modular, independently-testable trash-sorting pipeline for the Hiwonder JetRover, built as a clean rewrite of `trash_sorter`. Every hardware/sensor/logic concern is its own module with one responsibility, and every module can be tested on its own before the full pipeline ever runs.

## Project Overview

**What it does:** the robot scans for an object (apple or tissue paper), detects and classifies it recyclable/non-recyclable using either a VLM (Claude/ChatGPT) or a local YOLO model, drives up to it, fine-aligns with the arm, grasps it, drives to the matching bin, and drops it.

**Robot:** Hiwonder JetRover (mecanum/skid-steer base, 5-DOF arm + gripper, RGB-D camera, IMU, LiDAR).

**Sensors used:**
- RGB-D camera (`/depth_cam/rgb/image_raw`, `/depth_cam/depth/image_raw` + `camera_info`)
- IMU (`/ros_robot_controller/imu_raw`) — used for heading during in-place turns, since wheel odometry is unreliable under wheel slip on this base
- Wheel odometry (`/odom`) — used for x/y position and as a heading fallback
- LiDAR (`/scan`) — used for standoff-distance approach to the object and the bin

**Main software components** (see Architecture below): `hardware` (base/arm/gripper), `perception` (camera/depth/imu/lidar/detector), `intelligence` (VLM clients), `navigation` (movement/position/alignment), `manipulation` (grasp/place), `pipeline` (state machine + orchestrator node).

**Relationship to the old `trash_sorter` package:** `trash_sorter` was a research prototype where the entire robot (all sensors, all logic, VLM calls, navigation, grasping) lived in one ~14,000-line file, with dead historical versions kept as comments instead of using git. `trash_modular` reimplements the same capabilities as ~15 small, single-responsibility, independently testable modules. See each module's docstring for what specifically changed and why.

## Architecture

```
trash_modular/
  config/params.py           YAML config loader (config.yaml -> dict), no framework
  utils/transforms.py        quaternion->yaw, angle normalize, pixel->angle (ONE implementation)
  utils/watchdog.py          command-timeout safety watchdog
  hardware/base.py           RobotBase - the only class that publishes cmd_vel, with speed clamp + watchdog
  hardware/arm.py            named-pose + raw servo commands
  hardware/gripper.py        open/close + grasp verification via servo feedback
  perception/camera.py       RGB frame provider
  perception/depth.py        pixel -> 3D point, using LIVE camera_info intrinsics
  perception/imu.py          gyro-integrated heading
  perception/lidar.py        front-cone distance
  perception/object_detector.py   Detection dataclass + claude/chatgpt/yolo_hybrid strategies
  intelligence/vlm.py        Claude/ChatGPT API clients only - no robot logic
  navigation/movement.py     forward/backward/rotate/drive-until-lidar primitives
  navigation/position.py     odometry tracking + the ONE closed-loop go_to_pose()
  navigation/alignment.py    pixel->angle math + coarse (body) / fine (arm turret) alignment loops
  manipulation/grasp.py      GraspCalibrator (IDW pose correction) + grasp execution
  manipulation/place.py      navigate-to-bin + drop
  pipeline/state_machine.py  pure state machine, no ROS - unit-testable
  pipeline/trash_sorter_pipeline.py   orchestrator node - wires the above together, no logic of its own
  test_nodes/test_*.py       ros2 run trash_modular test_X - one per module
  scripts/calibrate_*.py     calibration utilities
```

Every class above takes a plain `config: dict` (loaded once from `config.yaml`) plus, where it needs ROS, the owning `rclpy.node.Node`. Nothing reaches into global state.

## Requirements

- Ubuntu 20.04/22.04 (matches your JetRover image)
- ROS2 (Foxy/Humble - whatever your JetRover image ships)
- Python 3.8+
- ROS2 packages: `rclpy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `cv_bridge`, `servo_controller_msgs`, `ament_index_python` (all standard on the JetRover image; `servo_controller_msgs` comes from `src/driver/` in this workspace)
- Python packages: `pyyaml`, `numpy`, `opencv-python` (or `opencv-python-headless`)
- Only if using VLM detection: `anthropic` and/or `openai`, with `ANTHROPIC_API_KEY` / `OPENAI_API_KEY` set in the environment
- Only if using `yolo_hybrid` detection: `ultralytics`, plus a trained YOLO weights file
- Hardware: Hiwonder JetRover with depth camera, IMU, LiDAR, and the arm/gripper servo bus already brought up (see `src/bringup` in this workspace)

## Installation

```bash
cd ~/ros2_ws/src
# this package already lives here as trash_modular/

pip3 install pyyaml numpy opencv-python-headless --break-system-packages
# only if you'll use VLM detection:
pip3 install anthropic openai --break-system-packages
# only if you'll use yolo_hybrid detection:
pip3 install ultralytics --break-system-packages

cd ~/ros2_ws
colcon build --packages-select trash_modular
source install/setup.bash
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select trash_modular
source install/setup.bash
```

After changing Python code under `trash_modular/`, you must rebuild (or symlink-build once with `colcon build --symlink-install --packages-select trash_modular` so edits take effect without rebuilding) and re-source. After changing `config/config.yaml`, just restart the node - no rebuild needed if you used `--symlink-install`, otherwise rebuild so the installed copy picks up the change.

## Running the Robot

1. Bring up the base hardware drivers (camera, IMU, LiDAR, servo bus) - this is provided by the existing JetRover bringup, e.g.:
   ```bash
   ros2 launch bringup bringup.launch.py
   ```
2. Set VLM credentials if using `claude`/`chatgpt`/`yolo_hybrid` detection:
   ```bash
   export ANTHROPIC_API_KEY=sk-...
   ```
3. Test each module independently first (see below) before running the full pipeline.

## Testing Individual Modules

Every module has a standalone test. None of them move the robot/arm/gripper unless you pass an explicit flag - safe by default.

```bash
ros2 run trash_modular test_camera                    # waits for an RGB frame, reports shape, PASS/FAIL
ros2 run trash_modular test_depth                      # waits for depth+camera_info, samples centre pixel
ros2 run trash_modular test_imu                        # waits for IMU data, reports integrated heading
ros2 run trash_modular test_lidar                       # waits for a scan, reports front-cone distance
ros2 run trash_modular test_detector                    # runs the configured detection strategy on one frame

ros2 run trash_modular test_arm                          # reports configured poses only, moves nothing
ros2 run trash_modular test_arm -- --pose home             # moves the arm to the "home" pose

ros2 run trash_modular test_gripper                      # reports servo feedback only, moves nothing
ros2 run trash_modular test_gripper -- --open               # opens the gripper
ros2 run trash_modular test_gripper -- --close               # closes the gripper, reports grasp verification

ros2 run trash_modular test_movement                      # reports IMU/LiDAR readiness only, moves nothing
ros2 run trash_modular test_movement -- --forward            # drives forward 1s (--duration to change)
ros2 run trash_modular test_movement -- --rotate 30            # rotates 30deg in place, IMU closed-loop
ros2 run trash_modular test_movement -- --approach               # drives to the configured LiDAR standoff

ros2 run trash_modular test_alignment                      # reports one detection only, moves nothing
ros2 run trash_modular test_alignment -- --coarse             # runs body-rotation coarse alignment
ros2 run trash_modular test_alignment -- --fine                 # runs arm-turret fine alignment

ros2 run trash_modular test_bin -- --material recyclable         # jog to HOME, find+approach the bin, simulate a drop
ros2 run trash_modular test_bin -- --material non-recyclable

ros2 run trash_modular test_pipeline                       # constructs every module, reports sensor readiness
ros2 run trash_modular test_pipeline -- --start --duration 30    # runs the full state machine for 30s, then stops
```

Every test node logs `TEST START`, then its steps, then a `RESULT: PASS/FAIL` banner, and exits with code 0/1 accordingly - so they're also usable as CI-style smoke checks.

### Calibration utilities

```bash
ros2 run trash_modular calibrate_home_bin -- --record --material recyclable
ros2 run trash_modular calibrate_home_bin -- --test   --material recyclable
ros2 run trash_modular calibrate_gripper -- --label empty      # then close the gripper on nothing
ros2 run trash_modular calibrate_gripper -- --label grasped      # then close it on an object
ros2 run trash_modular calibrate_grasp                          # collect real grasp-pose samples (see below)
```

`calibrate_home_bin` records a bin's position as an **offset from HOME**, not an absolute odom coordinate - `/odom` re-zeros every time its driver restarts, so absolute coordinates from a previous session are meaningless. Copy the printed offset into `config.yaml` under `bins.targets` (the pipeline reads from `config.yaml`, not from the saved calibration JSON - that file is only what `--test` reads back).

Both `--record` and `--test` drop you into a live jog prompt (`w`/`s`/`a`/`d` to drive, `space` to stop, `ENTER` when you're at the target) to position the robot - no second terminal needed. `--record` prompts once for HOME, then again for the bin (showing live distance from HOME while you jog). `--test` prompts once for HOME, then drives autonomously to the saved bin offset using the same `go_to_pose` the pipeline itself uses.

`calibrate_gripper` logs the gripper servo's commanded-vs-actual position to `gripper_calibration_log.csv` so you can pick a real `gripper.resist_threshold` from empty-vs-grasped distributions instead of guessing.

`calibrate_grasp` is what makes the arm's grasp pose actually depend on the object's pixel position and measured depth instead of always using the fixed baseline pose. `grasp_calibration.csv` ships empty, so `GraspCalibrator` has nothing to interpolate from and every grasp uses the same `baseline_lift`/`baseline_reach` regardless of `z3d` until you run this and collect samples. It detects an object, sends the arm to the baseline pose, then you jog it (`a`/`d`=turret, `w`/`s`=lift, `q`/`e`=reach) until the gripper is actually positioned to grasp it, optionally test-close with `space` to check the grasp-verification deficit, then `ENTER` saves `(px, py, z3d, delta_j1, delta_lift, delta_reach)` and moves on to the next object. Collect several samples spanning the pixel/depth range you actually see at grasp time - `GraspCalibrator` only trusts nearby samples (`arm.grasp.max_trusted_distance_px`) and falls back to the baseline pose otherwise.

## ROS2 Topics

```bash
ros2 topic list
ros2 topic echo <topic>
ros2 topic hz <topic>
ros2 topic info <topic>
```

Topics this package uses (all names are configurable in `config.yaml`):

| Topic | Type | Direction | Used by |
|---|---|---|---|
| `/controller/cmd_vel` | `geometry_msgs/Twist` | publish | `hardware.base.RobotBase` |
| `servo_controller` | `servo_controller_msgs/ServosPosition` | publish | `hardware.arm.Arm` |
| `/depth_cam/rgb/image_raw` | `sensor_msgs/Image` | subscribe | `perception.camera.Camera` |
| `/depth_cam/depth/image_raw` | `sensor_msgs/Image` | subscribe | `perception.depth.DepthSensor` |
| `/depth_cam/depth/camera_info` | `sensor_msgs/CameraInfo` | subscribe | `perception.depth.DepthSensor` |
| `/odom` | `nav_msgs/Odometry` | subscribe | `navigation.position.PositionTracker` |
| `/scan` | `sensor_msgs/LaserScan` | subscribe | `perception.lidar.Lidar` |
| `/ros_robot_controller/imu_raw` | `sensor_msgs/Imu` | subscribe | `perception.imu.ImuSensor` |
| `/controller_manager/servo_states` | `servo_controller_msgs/ServoStateList` | subscribe | `hardware.gripper.Gripper` |
| `/trash_sorter/command` | `std_msgs/String` | subscribe | `pipeline.trash_sorter_pipeline` (`"start"`/`"stop"`) |

There is no cross-node string-parsed detection protocol like the old project's `/trash_detection` - detection and depth lookup happen as direct in-process calls within whichever node is running (test node or the pipeline).

## Debugging

```bash
ros2 node list
ros2 topic list
ros2 topic info <topic>
ros2 topic echo <topic>
ros2 topic hz <topic>
```

Common problems:

- **A test node reports FAIL "no frame/scan/imu data received"** - check the driver is actually running (`ros2 topic hz <topic>`) and that the topic name in `config.yaml` matches `ros2 topic list`.
- **Robot rotates but `rotate_by_angle`/alignment times out** - check `ros2 topic hz /ros_robot_controller/imu_raw`; if the IMU never arrives, `Movement.rotate_by_angle` refuses to rotate blind rather than guessing (see Sensor Failure Handling below).
- **`drive_until_lidar` never converges** - check `ros2 topic echo /scan` and that `lidar.front_cone_deg`/`min_valid_range_m` in `config.yaml` match your LiDAR's actual field of view.
- **Grasp always uses the "baseline (uncalibrated)" pose** - you haven't populated `config/grasp_calibration.csv` yet, or every sample in it is farther than `arm.grasp.max_trusted_distance_px` from the live detection. This is expected until you collect real samples.
- **`test_detector` fails immediately** - check `ANTHROPIC_API_KEY`/`OPENAI_API_KEY` is exported in the same shell you ran `ros2 run` from, and that `detection.strategy` in `config.yaml` matches which key you set.

## Sensor Failure Handling

- **Camera unavailable** -> the pipeline transitions to `SAFE_STOP` (stops the base) rather than searching blind.
- **IMU unavailable** -> `Movement.rotate_by_angle` refuses to rotate; `PositionTracker.go_to_pose` falls back to (less reliable) wheel-odometry heading and logs a warning, it does not silently pretend the IMU is fine.
- **Depth unavailable** -> the pipeline returns to `SCAN` instead of approaching an object with no distance estimate, unless `pipeline.allow_depth_fallback: true` is explicitly set in config.
- **Detection fails / low confidence** -> the pipeline never leaves `SCAN`; it does not approach an unconfirmed target.
- **Grasp/gripper feedback missing** -> `Gripper.is_grasped()` logs a warning and returns `grasped=False` (informational) rather than raising or blocking the state machine - it's not gated on retry, since `pipeline.grasp_max_attempts` controls retry behavior explicitly in config, not a hidden fallback.

## Full Pipeline

```bash
ros2 launch trash_modular trash_modular.launch.py
```

Then, in another terminal:

```bash
ros2 topic pub --once /trash_sorter/command std_msgs/String "data: start"
# ... let it run ...
ros2 topic pub --once /trash_sorter/command std_msgs/String "data: stop"
```

The pipeline always returns to `IDLE` after a `stop` or a `SAFE_STOP` - you must publish `start` again to resume. State transitions are logged as `State: X -> Y`.

## Configuration

Every tunable lives in `config/config.yaml` - speeds, tolerances, topics, detection strategy/confidence, VLM models, arm poses, gripper thresholds, bin offsets, and safety timeouts. Nothing requires touching Python code or rebuilding to recalibrate (aside from re-sourcing if you didn't use `--symlink-install`).

## Development Order

This project was built module-by-module, each tested standalone before the next was written:

1. Skeleton, config loader, logging conventions
2. Hardware primitives (`base`, `arm`, `gripper`) - tested via `test_movement`/`test_arm`/`test_gripper`
3. Sensors (`camera`, `depth`, `imu`, `lidar`) - tested via their `test_*` nodes
4. Perception (`object_detector` strategies) - tested via `test_detector`
5. Navigation (`movement`, `position`, `alignment`) - tested via `test_movement`/`test_alignment`
6. Manipulation (`grasp`, `place`)
7. Integration combinations (detection + navigation + alignment + grasp)
8. Full pipeline + state machine (`pipeline/trash_sorter_pipeline.py`), tested via `test_pipeline`

## Known Limitations / Carried-Forward Caveats

- `gripper.resist_threshold` and the fine-alignment vertical (lift) correction are uncalibrated by default, same as in the old project - use `calibrate_gripper` to pick a real threshold before trusting grasp verification.
- `arm.grasp.calibration_csv` ships empty (header only) - every grasp uses the fixed baseline pose regardless of depth until you run `calibrate_grasp` and collect real samples.
- Optional email trial reporting was intentionally **not** ported - the old project had a Gmail app password committed in source, which is a real credential leak. If you want reporting back, wire it up behind `reporting.enabled` in config, sourcing credentials from the environment variables already named in `config.yaml` (`reporting.*_env`) - never commit credentials.

## Technical Notes

- The ROS2 package's actual installed name is `trash_modular` (see `package.xml`, `setup.py`, and every `ros2 run` command). This README uses the working name `trash_sorter` in prose, while command examples retain the real package identifier.

- `bins.location_mode` is currently set to `detect` in `config.yaml`. The robot's primary bin-finding behaviour is the VLM visual-search-and-approach path. The odometry-offset path is available as a configured alternative and its offsets are populated — switch `bins.location_mode` to `static` to use it.

- `grasp_calibration.csv` contains only its header row. The grasp-pose correction feature is fully implemented and unit-tested but has no data to act on. Every grasp currently uses one fixed baseline pose regardless of the object's position or depth. Run `scripts/calibrate_grasp.py` to populate it.

- `gripper.resist_threshold` is still the code default (`60`). There is no evidence that `calibrate_gripper`'s output has been applied. Run `scripts/calibrate_gripper.py` and update this value in `config.yaml` before trusting grasp verification.

- A hardware-independent `pytest` suite exists covering 16 tests across the state machine, coordinate transforms, and grasp calibrator logic. Run it without a robot: `pytest test/`.

- Trial results are logged to the console and held in memory only. Nothing is written to disk, so there is no way to answer "how well has this performed over N runs" from the project alone. See Section 13 (Future Work) for the planned fix.

- Every major sensor-loss path (camera, IMU, depth, LiDAR, gripper feedback) has an explicit logged fallback rather than a silent assumption of success. This is a consistent design pattern across the codebase.

- The most important behavioural fixes (LiDAR scan-plane dropout guard, bin-approach overrun guard, watchdog re-send pattern, outlier rejection in object and bin alignment) are documented in code comments as responses to specific previously observed failures. Preserve these comments in any future refactor.

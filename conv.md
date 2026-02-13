# Conversation Log (`conv.md`)

This file is a running summary of our work.
Timestamps are recorded in local ISO-8601 format.

## Entries

- `2026-02-11T21:04:37+0530` | User asked to review `PORT_TO_ROS2.md` and align on ROS1->ROS2 migration scope.
- `2026-02-11T21:04:37+0530` | We aligned on migration phases: create `vop2el_ros2`, port build system/node/launch/scripts, then validate with `colcon`.
- `2026-02-11T21:04:37+0530` | User requested Docker Compose changes for ROS2 testing; compose file was added and validated, with host mount resolved to `.../vop2el_ws -> /vop2el_ws`.
- `2026-02-11T21:04:37+0530` | User confirmed compose path was fine and requested focus on ROS2 coding.
- `2026-02-11T21:04:37+0530` | `vop2el_ros2` package was created/ported from ROS1 and build-system moved to `ament_cmake`.
- `2026-02-11T21:04:37+0530` | ROS2 node, launch files, config, and scripts were ported; `colcon` build for `vop2el_ros2` succeeded in isolated build dirs.
- `2026-02-11T21:04:37+0530` | User requested ongoing conversation tracking in `conv.md`; logging is now enabled and will be updated as we proceed.
- `2026-02-11T21:06:20+0530` | User confirmed `vop2el_ros1/*` and old `docker-compose.yml` deletions are intentional; requested repo sanity check and exact Docker Compose run command.
- `2026-02-11T21:06:20+0530` | Verified `docker-compose.yaml` parses cleanly and binds host `/home/tejas/project_workspaces/personal_github/vop2el_conversion_ros2/vop2el_ws` to container `/vop2el_ws`.
- `2026-02-11T21:18:06+0530` | User requested additional dataset mount; updated `docker-compose.yaml` to bind `/home/tejas/project_workspaces/datasets` to `/datasets` and validated with `docker compose config`.
- `2026-02-11T21:27:11+0530` | User asked for the exact commands to run after creating the container (build and launch workflow for ROS2 package).
- `2026-02-11T21:38:58+0530` | User reported container build failure: `CeresConfig.cmake` not found while `Vop2elEnv` was being built. Next action is to install native deps in container and build only `vop2el_ros2` via explicit `--base-paths`.
- `2026-02-11T21:46:05+0530` | User asked why `Vop2elEnv` is being built and why `vop2el_ros2` is not directly detected by plain `colcon build`.
- `2026-02-11T21:50:07+0530` | Runtime launch failure investigated. Root cause: packaged `Vop2elParameters.txt` hardcoded old `/volp2el_ws/...` calibration file path. Plan: switch to relative calibration path and resolve relative-to-ini directory in loader; also expose `ini_file` launch arg for txt/ini override.
- `2026-02-11T21:52:15+0530` | Fixed launch/runtime path issues: changed `Vop2elParameters` calibration reference to relative path, added relative-path resolution in `Utils::GenerateVop2elParamsFromIniFile`, and added explicit `ini_file` launch argument so `.txt` or `.ini` can be selected at runtime.
- `2026-02-11T21:52:15+0530` | Rebuilt `vop2el_ros2` successfully after fixes.
- `2026-02-11T21:53:10+0530` | Verified runtime fix with timed launch: node starts successfully and loads installed `Vop2elParameters.txt` without calibration-path crash.
- `2026-02-11T23:07:12+0530` | Compared ROS1 and ROS2 configs: runtime parameters are effectively the same; only YAML `ini_file` default differs (ROS2 launch injects packaged default), and parameter file calibration path was made relative for install portability.
- `2026-02-11T23:07:12+0530` | Investigated 9 Hz vs expected 10 Hz: likely causes are strict stereo desync gate (`max_stereo_dt_sec=0.002` with `sync_policy=approximate`) or compute saturation; advised using debug counters to distinguish drops vs processing limit.
- `2026-02-11T23:22:22+0530` | User reported debug mode hurting Hz/drift. Applied performance patch: debug no longer triggers per-frame keypoint copy, and debug/features publishers switched to SensorDataQoS (best-effort) to reduce publisher backpressure.
- `2026-02-11T23:22:22+0530` | Rebuilt `vop2el_ros2` successfully after debug-overhead optimization.
- `2026-02-11T23:29:13+0530` | User noted prior 10 Hz/good trajectory baseline; investigated for wrapper regression and found ROS2 subscriptions using fixed SensorData QoS depth (5) instead of configured `queue_size` (10).
- `2026-02-11T23:29:13+0530` | Patched ROS2 image/camera_info subscriptions to use SensorData QoS with depth=`queue_size` and rebuilt successfully.
- `2026-02-11T23:31:08+0530` | User clarified `use_camera_info=false`. Clarified that queue-depth/QoS patch still affects image-only mode because left/right image subscriptions and message_filters are always active even without camera_info subscriptions.
- `2026-02-11T23:48:03+0530` | Added two performance-oriented changes: `vop2el_ros2` now defaults to `Release` when build type is unspecified, and Vop2el Ceres thread auto-selection no longer caps at 4 (env override still supported via `VOP2EL_NUM_THREADS`).
- `2026-02-11T23:48:03+0530` | Rebuilt `vop2el_ros2` successfully after performance patches.
- `2026-02-11T23:55:51+0530` | User reported latest performance tweaks worsened runtime without Hz gain. Rolled back solver thread auto-scaling to prior capped behavior (`min(hw,4)`) and rebuilt `vop2el_ros2`.
- `2026-02-11T23:59:44+0530` | User requested to pause and continue later with full logging. Session paused. Current code state: ROS2 package exists and builds; ini/calibration path fix is in place; solver thread auto-scaling was rolled back to capped behavior (min(hw,4)) after performance regression report.
- `2026-02-12T00:03:45+0530` | Investigated runtime warnings (`/tf_static` QoS incompatibility and stereo desync skips). Patched node to avoid creating TF listener when not needed (`use_camera_info=false` path), aligned ApproximateTime max interval with `max_stereo_dt_sec`, and made desync drop-check optional when `max_stereo_dt_sec<=0`.
- `2026-02-12T00:03:45+0530` | Rebuilt `vop2el_ros2` successfully after warning-path and sync-filter patches.
- `2026-02-12T00:11:36+0530` | User observed output odometry remains around 8.5 Hz with slight drift and suspects near-but-not-every-frame processing. Next step is to isolate input-rate limits vs wrapper frame drops (desync/filter/queue) using targeted runtime counters and input-topic Hz checks.
- `2026-02-13T11:13:52+0530` | User requested a codebase review focused on GPU acceleration opportunities for ~8.5 Hz runtime. Reviewed `conv.md`, ROS2 node wrapper, matcher, optical-flow stage, and build/config paths.
- `2026-02-13T11:13:52+0530` | Added experimental GPU acceleration path via OpenCL toggles: ROS2 params `use_opencl` and `opencv_num_threads`, OpenCV runtime initialization/logging in node startup, and OpenCL-backed `UMat` execution for `calcOpticalFlowPyrLK` and `matchTemplate` hot paths (CPU fallback preserved). Verified by building `vop2el_ros2` successfully.
- `2026-02-13T11:14:59+0530` | Added launch-time overrides for OpenCL/thread tuning (`use_opencl`, `opencv_num_threads`) to both normal and debug launch files, then revalidated with cached `colcon` rebuild and Python launch syntax checks.

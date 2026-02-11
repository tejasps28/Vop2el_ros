# Vop2el

The goal of this repository is to provide a stereo visual odometry algorithm based on the  methods described in [SOFT2](https://lamor.fer.hr/images/50036607/2022-cvisic-soft2-tro.pdf) paper.

Note: This algorithm relies on a known camera calibration matrix and a known extrinsic transformation between the left and right cameras of the stereo camera.

![alt text](doc/result_sequence_00_kitti.gif)

## Dependencies

This project has been tested and verified to build successfully on Ubuntu 20.04 LTS with the following dependencies.

- Eigen 3.4

- Ceres 2.2

- OpenMP

- For OpenCV, we use a slightly modified version that will be built during the build of this project.

## OpenCV (custom fork)

This repo uses a custom OpenCV fork as a git submodule at `third-party/opencv`:

- URL: `https://github.com/Vop2elToolkit/opencv.git`
- Branch: `feat/RansacReturnsAllInliers`

To download the exact version pinned by this repo:

```bash
git submodule update --init --recursive
```

To verify which commit you have:

```bash
git -C third-party/opencv rev-parse --short HEAD
```

If you explicitly want the latest commit from the forked branch instead of the pinned version:

```bash
git -C third-party/opencv fetch origin feat/RansacReturnsAllInliers
git -C third-party/opencv checkout feat/RansacReturnsAllInliers
git -C third-party/opencv pull
```

## Build and install

To build and install this project on linux, follow the steps below:

```bash
git clone https://github.com/Vop2elToolkit/Vop2el.git

cd Vop2el && git submodule update --init --recursive

mkdir ../Vop2el-build && cd ../Vop2el-build
```

#### Notes:
- ```vop2el``` has two build modes: with and without Rerun SDK 
- When building without Rerun SDK, users will receive a ```csv``` file containing the estimated poses after processing a sequence of images.
- When building with Rerun SDK, users will get a real time visualization of estimated poses, and receive a ```csv``` file containing the final poses after processing a sequence of images

### Build without Rerun SDK

```bash
cmake ../Vop2el -DBUILD_WITH_RERUN=OFF

cmake --build .
```    

### Build with Rerun SDK for visualization

Download the Rerun viewer (version 0.17.0) from https://github.com/rerun-io/rerun/releases

```bash
cmake ../Vop2el

cmake --build .
```

## Docker (ROS Noetic)

If you want to run inside Docker using the provided compose file:

```bash
ROS_IMAGE=osrf/ros:noetic-desktop-full docker compose -f Vop2el_ros/docker-compose.yml up -d
docker compose -f Vop2el_ros/docker-compose.yml exec ros1 bash
```

This mounts:

- Workspace: `/home/tejas/project_workspaces/personal_github/volp2el_ws` -> `/workspaces/volp2el_ws`
- Datasets: `/home/tejas/project_workspaces/datasets` -> `/datasets` (read-only)

If you need GUI tools (rviz/rqt), run `xhost +local:root` on the host before starting the container.

## ROS1 Wrapper (Realtime Buffer)

The wrapper is configured as **INI-only** for algorithm parameters to keep core VO behavior identical to upstream Vop2el.
YAML is used only for ROS topics, publishing, buffering, and frame-selection policy.

For realtime ROS setup, configure in `vop2el_ros1/config/vop2el.yaml`:

- `ini_file: /volp2el_ws/src/Vop2el_ros/my_files/Vop2elParameters.txt`
- `use_camera_info: false`
- `queue_size: 50`
- `input_buffer_size: 60`
- `drop_oldest_when_full: true`
- `sync_policy: exact` (recommended for rosbag/KITTI-style playback)
- `max_stereo_dt_sec: 0.002` (skip badly paired stereo frames)
- `force_grayscale: true` (convert input to MONO8 before VO)
- `selector_mode: stride` (`stride` or `quality_buffer`)
- `process_every_n: 1` (for `stride` mode)
- `target_process_rate_hz: 0.0` (optional cap; 0 disables)
- `selection_buffer_size: 3` (for `quality_buffer` mode)
- `selection_max_latency_sec: 0.15` (for `quality_buffer` mode)
- `min_sharpness: 0.0`, `min_brightness: -1.0`, `max_brightness: 256.0` (quality gates)
- `skip_publish_on_fallback: false` (if true, fallback frames are not published)
- `path_publish_stride: 5` (reduce path publishing overhead while keeping odom every frame)

For best tracking fidelity (closest to folder-mode behavior), keep heavy debug publishers off during VO runs:

- `publish_debug: false`
- `publish_features: false`
- `publish_features_image: false`

Optional for tighter realtime behavior under recording/load:

- Set `VOP2EL_NUM_THREADS=4` (or lower) before launch to cap Ceres solver threads and avoid CPU oversubscription.

Build and run inside the container:

```bash
source /opt/ros/noetic/setup.bash
cd /volp2el_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
export VOP2EL_NUM_THREADS=4
roslaunch vop2el_ros1 vop2el.launch
```

Published outputs:

- Odometry: `/vo/odom`
- Path: `/vo/path`
- Features: `/vo/features` (`sensor_msgs/PointCloud`, x/y are image pixel coordinates)
- Feature overlay image: `/vo/features_image` (`sensor_msgs/Image`, green dots over left image)
- Debug stats: `/vo/debug` (`diagnostic_msgs/DiagnosticArray`), including:
- `match_count`, `inlier_count`, `fallback_used`, `extrapolated_on_failure`, `failure_reason`
- Selector/pipeline counters: `frames_received`, `frames_skipped_selector`, `frames_skipped_desync`, `frames_rejected_quality`, `frames_selected_quality_buffer`, `frames_enqueued`, `frames_processed`, `frames_published`, `frames_skipped_publish_fallback`, `frames_dropped`

Record full-trajectory debug stats with debug publishing forced from launch:

```bash
rosrun vop2el_ros1 record_debug_stats.sh \
  /volp2el_ws/src/Vop2el_ros/vop2el_ros1/config/vop2el.yaml \
  /volp2el_ws/src/debug_bags
```

This runs `vop2el_record_debug.launch`, sets `publish_debug=true`, and records:
- `/vo/debug`
- `/vo/odom`
- `/vo/path`
- `/vo/features`

To record TUM format from odometry, run the separate recorder node:

```bash
rosrun vop2el_ros1 odom_to_tum.py _odom_topic:=/vo/odom _tum_output_file:=/volp2el_ws/src/vo_tum.txt
```

Parameters for `odom_to_tum.py`:

- `_odom_topic` (default: `/vo/odom`)
- `_tum_output_file` (default: `vo_tum.txt`)
- `_append` (default: `false`)
- `_flush_interval` (default: `100`)

Example rosbag playback:

```bash
rosbag play /datasets/<your_bag>.bag --clock \
  /kitti/camera_gray_left/image_raw \
  /kitti/camera_gray_right/image_raw
```

## Run

To run this project on linux, follow the steps below (Skip the initial two steps when building without Rerun):

- Launch the Rerun viewer

- In the Rerun viewer, open the blueprint file ```vop2el/vop2el_src/test/vop2el.rbl```

- Run Vop2el

```bash
./bin/Vop2elTester /path/to/left/images/folder \
                   /path/to/right/images/folder \
                   /path/to/config/ini/file \
                   /path/to/estimated/poses/text/file \
                   /path/to/ground/truth/poses/text/file(optional)
```


![alt text](doc/rerun_sequence_00_kitti.gif)


### Notes:
- Users should rely on parameters ini file provided in test folder.
- Set use_ground_plane_correction to false in the parameters file if the robot is non-terrestrial or if the distance and normal to the ground plane are unknown.
- To improve processing time, the simplest way is to reduce value of max_number_matches in parameters file.
- In the estimated poses text file, each pose will be written as a single line representing a row-major 3x4 matrix.
- The last argument mentioned in the command above is optional. When provided, it is used solely to compute translation and rotation error metrics between the estimated and ground truth poses.

## Differences with SOFT2
| SOFT2 | Ours |
| ---                              | ---        |
| Automatic computing of ground plane normal vector and distance to ground plane | The user needs to provide the ground plane normal vector and distance to ground plane if he would like to use patch correction feature |
| Use SOFT algorithm to estimate initial relative pose | Use Lucas–Kanade optical flow to estimate initial matches, followed by a RANSAC to estimate initial relative pose |
| Bundle adjustment | No bundle adjustment
| Extrinsic camera rotation is optimized during scale computing | For now, the extrinsic camera rotation is deemed satisfactory and will not undergo optimization during the algorithm

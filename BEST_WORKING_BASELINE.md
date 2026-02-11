# Best Working Baseline (Frozen)

Timestamp: `2026-02-10 23:45:57 +0530`

This file captures the current repo parameter state that is considered "working best so far".

## Active Launch Command

Use this for normal realtime runs:

```bash
roslaunch vop2el_ros1 vop2el.launch publish_debug:=false
```

Note:
- `vop2el.launch` currently has `publish_debug` launch arg default set to `true`, so explicitly passing `publish_debug:=false` is recommended for best performance.

## Config File Checksums

- `vop2el_ros1/config/vop2el.yaml`:
  - `631303810088334c3af4cde3da1591a630ecd90e47135e85fc043845e562f82b`
- `my_files/Vop2elParameters.txt`:
  - `dcd128f4e53776a7335db65921f0ae55da111b76a5275af2ccd7f7956a72130d`
- `my_files/StereoCameraParameters.txt`:
  - `198f90d74b7ef9a7d4f054aa57ebd7b780586494316ada672ab7b464c57a50f0`
- `vop2el_ros1/launch/vop2el.launch`:
  - `190457e7c87741dfcb5670086924383f7d9b213f4910fc252d2712b32c28cb7f`

## Current `vop2el_ros1/config/vop2el.yaml`

```yaml
# Topics (override these with your dataset-specific values)
left_image_topic: /kitti/camera_gray_left/image_raw #/stereo/left/image_rect
right_image_topic: /kitti/camera_gray_right/image_raw #/stereo/right/image_rect
left_camera_info_topic: /kitti/camera_gray_left/camera_info #/stereo/left/camera_info
right_camera_info_topic: /kitti/camera_gray_right/camera_info #/stereo/right/camera_info

# Optional: path to the original Vop2el INI file. If set, all algorithm parameters
# (optical flow, matcher, cost, stereo handler, ground plane) are loaded from the INI.
ini_file: /volp2el_ws/src/Vop2el_ros/my_files/Vop2elParameters.txt
# true: read intrinsics/extrinsics from CameraInfo/TF (ROS-native mode)
# false: use original INI/TXT camera parameters; only image topics are needed
use_camera_info: false

# Frame overrides (optional). If empty, use frame_id from CameraInfo.
left_frame_override: ""
right_frame_override: ""

# Output
odom_topic: /vo/odom
path_topic: /vo/path
features_topic: /vo/features
features_image_topic: /vo/features_image
debug_topic: /vo/debug
odom_frame: odom
base_frame: camera_left
publish_tf: true
publish_odom: true
publish_path: true
publish_features: false
publish_features_image: false
publish_debug: false

# Sync / buffering / frame selection
queue_size: 10
# Active stereo frame buffer (push/pop queue used by wrapper before VO processing)
input_buffer_size: 5
drop_oldest_when_full: true
# Stereo synchronization policy:
# - exact: strict timestamp pairing (recommended for offline rosbag / KITTI-like data)
# - approximate: relaxed pairing
sync_policy: approximate
# Additional protection: skip stereo pairs with |left_stamp-right_stamp| larger than this.
max_stereo_dt_sec: 0.002
# Convert incoming images to MONO8 before VO for consistency and speed.
force_grayscale: true
# selector_mode:
# - stride: fixed decimation with process_every_n
# - quality_buffer: keep a short buffer of good frames and select the sharpest
selector_mode: stride
# stride mode: process only every Nth synchronized stereo pair (1 = every frame)
process_every_n: 1 #2
# optional upper bound on selected processing rate in Hz (0.0 disables)
target_process_rate_hz: 0.0
# quality_buffer mode parameters
selection_buffer_size: 3
selection_max_latency_sec: 0.15
min_sharpness: 0.0
min_brightness: -1.0
max_brightness: 256.0
# if true, do not publish odom/path/tf/features for fallback frames
skip_publish_on_fallback: false
# Publish full /vo/path every N processed frames to reduce runtime overhead.
path_publish_stride: 10

# Camera-info / TF path is disabled in INI-only mode, but these are kept for compatibility.
use_rectified: true
# TF lookup timeout in seconds (relevant only if camera-info mode is enabled in future).
tf_lookup_timeout: 0.1
```

## Current `my_files/Vop2elParameters.txt`

```ini
; Camera parameters
[stereo_camera_parameters]
; See example of stereo camera parameters txt file in test folder
stereo_camera_parameters = /volp2el_ws/src/Vop2el_ros/my_files/StereoCameraParameters.txt
; Image columns
image_cols = 1241
; Image rows
image_rows = 376

; Initial matches parameters
[optical_flow_parameters]
; Parameter of cv::calcOpticalFlowPyrLK
of_window_rows = 31
; Parameter of cv::calcOpticalFlowPyrLK
of_window_cols =  31
; Parameter of cv::calcOpticalFlowPyrLK
of_pyramid_level = 3
; Parameter of cv::calcOpticalFlowPyrLK
of_eigen_treshold = 0.001
; Parameter of cv::calcOpticalFlowPyrLK
of_criteria_max_count = 50
; Parameter of cv::calcOpticalFlowPyrLK
of_criteria_epsilon = 0.05
; Treshold of difference between forward and backward optical flow to consider an initial match valid
of_forward_backward_treshold = 1.5

; Non linear cost functions parameters
[cost_functions_parameters]
; Maximum number of iterations
cost_functions_max_num_iterations = 500
; If true, use Tukey instead of square loss function (Tukey requires more processing time than square loss)
use_tukey_estimator = true
; the point at which the Tukey loss transitions from quadratic to linear behavior
tukey_parameter = 1.0

; Vop2el Matcher parameters
[vop2el_matcher_parameters]
; Maximum number of matches to use to compute relative pose, -1 to use all matches available
max_number_matches = -1
; Normalized cross-correlation score above which the match is considered valid
ncc_treshold = 0.75
; Interval of search on the epipolar line, depends on the speed of the robot
epipolar_line_search_interval = 100
; Maximum number of stereo keypoints candidates to search their matches in previous frame
max_stereo_points_to_process = 10
; Total number of rows of a patch is (HalfPatchRows * 2 + 1)
half_patch_rows = 4
; Total number of columns of a patch is (HalfPatchCols * 2 + 1)
half_patch_cols =4
; Total number of rows of the narrow region to search in is (HalfVerticalSearch * 2 + 1)
half_vertical_search = 4
half_horizontal_search = 4
; Treshold first/last stereo point ncc score to consider a left image keypoint not ambiguous
max_thresh = 0.25

; Stereo images handler parameters
[stereo_images_handler_parameters]
; Capacity of the stereo images handler, reset once exceeded
num_frames_capacity = 2
; GFTT bin width
bin_width = 50
; GFTT bin height
bin_height = 50
; Maximum number of keypoints to have in a single BinWidth x BinHeight bin
max_key_points_per_bin = 3

; Ground plane parameters, see "Demo 3" in https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html
[ground_plane_parameters]
; Set it to true if known ground plane normal and distance
use_ground_plane_correction = true
; Ground plane normal vector x component of a vector
plane_normal_x = 0.0
; Ground plane normal vector y component of a vector
plane_normal_y = -1.0
; Ground plane normal vector z component of a vector
plane_normal_z = 0.0
; Distance between camera and ground plane on ground plane normal (meters)
camera_ground_plane_distance = 1.65
```

## Current `my_files/StereoCameraParameters.txt`

```text
7.188560000000e+02 0.000000000000e+00 6.071928000000e+02 |
0.000000000000e+00 7.188560000000e+02 1.852157000000e+02 | intrinsic calibration matrix
0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 |
1.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.5371657189       |
0.000000000000e+00 1.000000000000e+00 0.000000000000e+00 0.000000000000e+00 | extrinsic stereo camera 4x3 transform
0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 0.000000000000e+00 | from left camera to right camera
```

## Current `vop2el_ros1/launch/vop2el.launch`

```xml
<launch>
  <arg name="config" default="$(find vop2el_ros1)/config/vop2el.yaml"/>
  <arg name="publish_debug" default="true"/>

  <node pkg="vop2el_ros1" type="vop2el_node" name="vop2el" output="screen">
    <rosparam command="load" file="$(arg config)"/>
    <param name="publish_debug" value="$(arg publish_debug)"/>
  </node>
</launch>
```


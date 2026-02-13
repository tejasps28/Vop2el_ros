# Port `Vop2el_ros` From ROS1 To ROS2

This document is a practical runbook for porting the current `ros1` branch to a ROS2-compatible branch.

## 1. Recommended Repo/Workspace Setup

Use a fresh clone and a separate ROS2 workspace. Do not mix catkin and colcon in the same active workspace.

```bash
# Example location (change as needed)
mkdir -p ~/project_workspaces/personal_github
cd ~/project_workspaces/personal_github

git clone https://github.com/<your-org-or-fork>/Vop2el_ros.git
cd Vop2el_ros

# If ros2 branch already exists remotely
git checkout ros2

# If ros2 branch does not exist yet
git checkout -b ros2
```

Create a ROS2 workspace and place the repo under `src`:

```bash
mkdir -p ~/volp2el_ros2_ws/src
ln -sf ~/project_workspaces/personal_github/Vop2el_ros ~/volp2el_ros2_ws/src/Vop2el_ros
cd ~/volp2el_ros2_ws
```

## 2. Environment Prerequisites

Pick one ROS2 distro and stay on it for the whole migration.

- Ubuntu 22.04 -> ROS2 Humble
- Ubuntu 24.04 -> ROS2 Jazzy

Base dependencies to install before building:

- `rclcpp`, `rclpy`
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `diagnostic_msgs`
- `tf2_ros`, `tf2_geometry_msgs`
- `cv_bridge`, `image_transport`, `message_filters`
- `OpenCV`, `Eigen3`, `Ceres`, `OpenMP`

## 3. Source Layout Strategy

Keep core VO algorithm unchanged first, port only the ROS wrapper.

- Keep as-is: `vop2el_src/`
- Existing ROS1 wrapper: `vop2el_ros1/`
- New ROS2 wrapper target: `vop2el_ros2/`

Start by copying ROS1 package structure:

```bash
cd ~/project_workspaces/personal_github/Vop2el_ros
cp -r vop2el_ros1 vop2el_ros2
```

Then port `vop2el_ros2` incrementally.

## 4. Build System Port (`catkin` -> `ament_cmake`)

### 4.1 `vop2el_ros2/package.xml`

- Change package name to `vop2el_ros2`
- `buildtool_depend`: `ament_cmake` (not `catkin`)
- Replace ROS1 deps:
  - `roscpp` -> `rclcpp`
  - `rospy` -> `rclpy`
- Keep message/tf/image deps in ROS2 form

### 4.2 `vop2el_ros2/CMakeLists.txt`

- Remove `find_package(catkin ...)`, `catkin_package(...)`
- Add:
  - `find_package(ament_cmake REQUIRED)`
  - `find_package(rclcpp REQUIRED)`
  - message/tf/image packages
- Keep external libs (`OpenCV`, `Eigen3`, `Ceres`, `OpenMP`)
- Link with `ament_target_dependencies(...)`
- Install rules:
  - node binary to `lib/${PROJECT_NAME}`
  - config/launch to `share/${PROJECT_NAME}`
  - python scripts with `install(PROGRAMS ...)`
- End with `ament_package()`

## 5. C++ Node API Port (`vop2el_node.cpp`)

Primary source: `vop2el_ros1/src/vop2el_node.cpp`

Porting map:

- `ros::NodeHandle` -> `rclcpp::Node`
- `ros::Publisher` -> `rclcpp::Publisher<T>::SharedPtr`
- `ros::Time` -> `rclcpp::Time`
- `ros::Duration` -> `rclcpp::Duration`
- `ROS_INFO/WARN/ERROR/FATAL` -> `RCLCPP_INFO/WARN/ERROR/FATAL`
- `ros::ok()` -> `rclcpp::ok()`
- `ros::spin()` -> `rclcpp::spin(node)`

Message namespaces:

- `sensor_msgs::Image` -> `sensor_msgs::msg::Image`
- `sensor_msgs::CameraInfo` -> `sensor_msgs::msg::CameraInfo`
- `nav_msgs::Odometry` -> `nav_msgs::msg::Odometry`
- `nav_msgs::Path` -> `nav_msgs::msg::Path`
- `geometry_msgs::TransformStamped` -> `geometry_msgs::msg::TransformStamped`
- `diagnostic_msgs::*` -> `diagnostic_msgs::msg::*`

## 6. Parameters + YAML Port

### 6.1 C++ parameter handling

Replace `pnh_.param(...)` with:

1. `declare_parameter("name", default_value)`
2. `get_parameter("name", variable)`

### 6.2 YAML format

ROS1:

```yaml
left_image_topic: /...
```

ROS2:

```yaml
vop2el:
  ros__parameters:
    left_image_topic: /...
```

Where `vop2el` must match the runtime node name.

## 7. Message Filters and QoS

ROS2 `message_filters` requires QoS-compatible subscriptions. For camera streams use SensorData QoS.

Actions:

- Port `image_transport::SubscriberFilter` and `message_filters::Subscriber`
- Use explicit sensor-data QoS for image + camera_info topics
- Keep exact/approx sync policies as in ROS1 behavior

## 8. TF2 Port

Keep the same logic but update constructors/usages for ROS2:

- `tf2_ros::Buffer` uses ROS2 clock
- `tf2_ros::TransformListener` takes ROS2 node interfaces
- `tf2_ros::TransformBroadcaster` constructed with ROS2 node

## 9. Launch Port (`.launch` -> `.launch.py`)

Port files:

- `vop2el_ros1/launch/vop2el.launch`
- `vop2el_ros1/launch/vop2el_record_debug.launch`

To ROS2:

- `vop2el_ros2/launch/vop2el.launch.py`
- `vop2el_ros2/launch/vop2el_record_debug.launch.py`

Use `launch_ros.actions.Node` for node execution and pass parameters from YAML.

## 10. Script Port

### 10.1 `odom_to_tum.py`

Port from `rospy` to `rclpy`:

- `rospy.Subscriber` -> `create_subscription`
- `rospy.get_param` -> `declare_parameter/get_parameter`
- `rospy.spin()` -> `rclpy.spin(node)`

### 10.2 `record_debug_stats.sh`

Replace ROS1 commands:

- `roslaunch` -> `ros2 launch`
- `rosbag record` -> `ros2 bag record`

## 11. Build and Run Commands (ROS2)

```bash
source /opt/ros/<humble-or-jazzy>/setup.bash
cd ~/volp2el_ros2_ws

colcon build --packages-select vop2el_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch vop2el_ros2 vop2el.launch.py
```

## 12. Validation Checklist (Must Pass)

- Package builds cleanly with `colcon`
- Node starts and subscribes to stereo topics
- Outputs publish:
  - `/vo/odom`
  - `/vo/path`
  - `/vo/debug` (when enabled)
  - `/vo/features` and `/vo/features_image` (when enabled)
- TF publish works (`odom` -> `camera_left` by default)
- `ini_file` loading path works in ROS2
- Frame selector behavior matches ROS1 (`stride` and `quality_buffer`)

## 13. Common Pitfalls

- Mixing ROS1 bag tooling with ROS2 node tests
- Parameter YAML missing `ros__parameters`
- Node name mismatch between YAML root key and launch node name
- QoS mismatch causing "no messages received" symptoms
- Assuming ROS1 launch substitution syntax works in ROS2

## 14. Suggested Migration Order (Do Not Skip)

1. Create `vop2el_ros2` package skeleton + successful configure step
2. Compile `vop2el_node.cpp` with ROS2 API (may fail at runtime initially)
3. Fix parameters and launch
4. Fix QoS/sync and TF behavior
5. Port scripts and recording workflow
6. Run parity test vs ROS1 on same dataset

## 15. Handoff Prompt For New Codex Session

Use this prompt in the new session:

```text
We are porting Vop2el_ros from ROS1 to ROS2 on branch ros2.
Please follow PORT_TO_ROS2.md and implement the migration in phases:
1) create/port vop2el_ros2 package files (package.xml + CMakeLists.txt),
2) port vop2el_node.cpp to rclcpp with ROS2 message types and parameters,
3) port launch and YAML,
4) port scripts,
5) run colcon build and fix compile/runtime issues.
Keep vop2el_src algorithm logic unchanged unless required for compilation.
```


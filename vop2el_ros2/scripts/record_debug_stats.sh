#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ros2 run vop2el_ros2 record_debug_stats.sh [config_yaml] [output_dir]
#
# Example:
#   ros2 run vop2el_ros2 record_debug_stats.sh \
#     /vop2el_ws/src/Vop2el_ros/vop2el_ros2/config/vop2el.yaml \
#     /vop2el_ws/src/debug_bags

CONFIG_FILE="${1:-$(ros2 pkg prefix vop2el_ros2)/share/vop2el_ros2/config/vop2el.yaml}"
OUTPUT_DIR="${2:-$HOME/.ros/vop2el_debug}"

mkdir -p "${OUTPUT_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="${OUTPUT_DIR}/vop2el_debug_${STAMP}"

echo "Config file : ${CONFIG_FILE}"
echo "Output bag  : ${BAG_DIR}"
echo "Recording topics: /vo/debug /vo/odom /vo/path /vo/features"
echo
echo "Press Ctrl-C to stop recording."

exec ros2 launch vop2el_ros2 vop2el_record_debug.launch.py \
  config:="${CONFIG_FILE}" \
  bag_file:="${BAG_DIR}"

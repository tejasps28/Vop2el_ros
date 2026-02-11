#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   rosrun vop2el_ros1 record_debug_stats.sh [config_yaml] [output_dir]
#
# Example:
#   rosrun vop2el_ros1 record_debug_stats.sh \
#     /volp2el_ws/src/Vop2el_ros/vop2el_ros1/config/vop2el.yaml \
#     /volp2el_ws/src/debug_bags

CONFIG_FILE="${1:-$(rospack find vop2el_ros1)/config/vop2el.yaml}"
OUTPUT_DIR="${2:-$HOME/.ros/vop2el_debug}"

mkdir -p "${OUTPUT_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_FILE="${OUTPUT_DIR}/vop2el_debug_${STAMP}.bag"

echo "Config file : ${CONFIG_FILE}"
echo "Output bag  : ${BAG_FILE}"
echo "Recording topics: /vo/debug /vo/odom /vo/path /vo/features"
echo
echo "Press Ctrl-C to stop recording."

exec roslaunch vop2el_ros1 vop2el_record_debug.launch \
  config:="${CONFIG_FILE}" \
  bag_file:="${BAG_FILE}"

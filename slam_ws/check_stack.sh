#!/usr/bin/env bash
set -euo pipefail

SLAM_WS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SLAM_WS_ROOT/setup.bash"
cloud_topic="${LIDAR_CLOUD_TOPIC:-/livox/lidar}"
lidar_frame="${LIDAR_FRAME:-livox_frame}"

echo "关键话题："
ros2 topic list -t | grep -E "^(${cloud_topic}|/scan|/odom|/map|/cmd_vel|/robot_description|/joint_states)" || true

echo
echo "关键节点："
ros2 node list | grep -E 'livox|pointcloud_to_laserscan|slam_toolbox|amcl|planner|controller|bt_navigator|robot_state_publisher|joint_state_publisher' || true

echo
echo "点云/扫描频率："
timeout 3s ros2 topic hz "$cloud_topic" || true
timeout 3s ros2 topic hz /scan || true

echo
echo "TF 检查："
timeout 3s ros2 run tf2_ros tf2_echo base_link "$lidar_frame" || true
timeout 3s ros2 run tf2_ros tf2_echo base_footprint base_link || true
timeout 3s ros2 run tf2_ros tf2_echo odom base_footprint || true

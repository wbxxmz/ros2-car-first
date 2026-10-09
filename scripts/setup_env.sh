#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用 source 加载环境："
  echo "  source scripts/setup_env.sh"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
BINARY_SETUP="${WORKSPACE_DIR}/binary_install/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "未找到 ${ROS_SETUP}，请先运行 bash scripts/00_install_ros2_gazebo.sh" >&2
  return 1
fi

if [[ ! -f "${BINARY_SETUP}" ]]; then
  echo "未找到 ${BINARY_SETUP}，请先运行 bash scripts/01_setup_inspection_workspace.sh" >&2
  return 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "未找到 ${WORKSPACE_SETUP}，请先运行 bash scripts/01_setup_inspection_workspace.sh" >&2
  return 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${BINARY_SETUP}"
# shellcheck disable=SC1090
source "${WORKSPACE_SETUP}"

export ICAN_INSPECTION_WS="${WORKSPACE_DIR}"
# Gazebo 材质/贴图搜索路径（world 里材质 URI 的兜底，避免路径失效导致贴图丢失）
export GAZEBO_RESOURCE_PATH="${WORKSPACE_DIR}/binary_install/share/ican_inspection_sim:${GAZEBO_RESOURCE_PATH:-}"
echo "已加载 ROS 2 Humble 和 iCAN 任务一巡检仿真工作区。"

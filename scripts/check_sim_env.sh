#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
BINARY_SETUP="${WORKSPACE_DIR}/binary_install/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"
FAILED=0

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

check() {
  local label="$1"
  shift
  if "$@" >/tmp/ican_inspection_check.out 2>/tmp/ican_inspection_check.err; then
    printf '\033[1;32m[OK]\033[0m %s\n' "${label}"
  else
    printf '\033[1;31m[FAIL]\033[0m %s\n' "${label}" >&2
    sed -n '1,20p' /tmp/ican_inspection_check.err >&2
    FAILED=1
  fi
}

check_optional() {
  local label="$1"
  shift
  if "$@" >/tmp/ican_inspection_check.out 2>/tmp/ican_inspection_check.err; then
    printf '\033[1;32m[OK]\033[0m %s\n' "${label}"
  else
    printf '\033[1;33m[WARN]\033[0m %s\n' "${label}" >&2
    sed -n '1,20p' /tmp/ican_inspection_check.err >&2
  fi
}

clean_terminal_env() {
  env \
    -u SNAP \
    -u SNAP_ARCH \
    -u SNAP_INSTANCE_NAME \
    -u SNAP_NAME \
    -u SNAP_REVISION \
    -u SNAP_USER_COMMON \
    -u SNAP_USER_DATA \
    -u GTK_PATH \
    -u GI_TYPELIB_PATH \
    -u GDK_PIXBUF_MODULE_FILE \
    -u GDK_PIXBUF_MODULEDIR \
    "$@"
}

check "ROS 2 Humble setup" test -f "${ROS_SETUP}"
check "仿真/Nav2 二进制环境 setup" test -f "${BINARY_SETUP}"
check "工作区 setup" test -f "${WORKSPACE_SETUP}"

if [[ -f "${ROS_SETUP}" ]]; then
  source_setup_file "${ROS_SETUP}"
fi

if [[ -f "${BINARY_SETUP}" ]]; then
  source_setup_file "${BINARY_SETUP}"
fi

if [[ -f "${WORKSPACE_SETUP}" ]]; then
  source_setup_file "${WORKSPACE_SETUP}"
fi

check "ros2 命令" command -v ros2
check "Gazebo 命令" command -v gazebo
check "colcon 命令" command -v colcon
check "gazebo_ros 包" ros2 pkg prefix gazebo_ros
check "Navigation2 bringup 包" ros2 pkg prefix nav2_bringup
check "Nav2 地图服务包" ros2 pkg prefix nav2_map_server
check_optional "pointcloud_to_laserscan 包，可选 Livox 路径" ros2 pkg prefix pointcloud_to_laserscan
check "cv_bridge 包" ros2 pkg prefix cv_bridge
check "OpenCV Python 模块" python3 -c "import cv2"
check "gnome-terminal 命令" command -v gnome-terminal
check "gnome-terminal 可运行" clean_terminal_env gnome-terminal --version
check "ican_inspection_sim 包" ros2 pkg prefix ican_inspection_sim
check "ican_inspection_navigation 包" ros2 pkg prefix ican_inspection_navigation
check "ican_inspection_examples 包" ros2 pkg prefix ican_inspection_examples
check "仿真 launch 参数" ros2 launch ican_inspection_sim inspection.launch.py --show-args
check "地面导航 launch 参数" ros2 launch ican_inspection_navigation nav2_ground.launch.py --show-args

rm -f /tmp/ican_inspection_check.out /tmp/ican_inspection_check.err
exit "${FAILED}"

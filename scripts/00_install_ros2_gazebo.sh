#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_NAME="$(basename "$0")"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"

log() {
  printf '\033[1;34m[%s]\033[0m %s\n' "${SCRIPT_NAME}" "$*"
}

warn() {
  printf '\033[1;33m[%s]\033[0m %s\n' "${SCRIPT_NAME}" "$*" >&2
}

die() {
  printf '\033[1;31m[%s]\033[0m %s\n' "${SCRIPT_NAME}" "$*" >&2
  exit 1
}

usage() {
  cat <<EOF
用法：
  bash scripts/00_install_ros2_gazebo.sh

说明：
  安装本任务一巡检仿真工作区所需的 Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic
  基础依赖。脚本会使用系统当前 APT 源；如需国内镜像源，可先在系统中配置好镜像，
  或直接复用上个项目的镜像安装脚本。

环境变量：
  ROS_DISTRO_NAME   默认值：${ROS_DISTRO_NAME}
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "未知选项：$1"
      ;;
  esac
done

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
else
  die "无法读取 /etc/os-release"
fi

if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  cat >&2 <<EOF
${SCRIPT_NAME}：当前脚本只面向 Ubuntu 22.04 LTS。

当前检测结果：
  ID=${ID:-unknown}
  VERSION_ID=${VERSION_ID:-unknown}
  PRETTY_NAME=${PRETTY_NAME:-unknown}
EOF
  exit 1
fi

if [[ $EUID -eq 0 ]]; then
  SUDO=()
else
  command -v sudo >/dev/null 2>&1 || die "缺少 sudo 命令"
  SUDO=(sudo)
fi

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

log "准备安装 ROS 2 Humble、Gazebo Classic、RViz、地面导航、图像处理和构建工具。"
"${SUDO[@]}" apt-get update

packages=(
  "ros-${ROS_DISTRO_NAME}-desktop"
  "ros-${ROS_DISTRO_NAME}-gazebo-ros-pkgs"
  "ros-${ROS_DISTRO_NAME}-gazebo-plugins"
  "ros-${ROS_DISTRO_NAME}-navigation2"
  "ros-${ROS_DISTRO_NAME}-nav2-bringup"
  "ros-${ROS_DISTRO_NAME}-nav2-msgs"
  "ros-${ROS_DISTRO_NAME}-tf-transformations"
  "ros-${ROS_DISTRO_NAME}-robot-state-publisher"
  "ros-${ROS_DISTRO_NAME}-joint-state-publisher"
  "ros-${ROS_DISTRO_NAME}-rviz2"
  "ros-${ROS_DISTRO_NAME}-cv-bridge"
  "ros-${ROS_DISTRO_NAME}-image-transport"
  "ros-${ROS_DISTRO_NAME}-camera-info-manager"
  "ros-${ROS_DISTRO_NAME}-sensor-msgs-py"
  gazebo
  libgazebo-dev
  gnome-terminal
  python3-colcon-common-extensions
  python3-rosdep
  python3-vcstool
  python3-pip
  python3-opencv
  build-essential
  cmake
)

"${SUDO[@]}" DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${packages[@]}"

optional_packages=(
  "ros-${ROS_DISTRO_NAME}-pointcloud-to-laserscan"
)

for package in "${optional_packages[@]}"; do
  log "正在尝试安装可选依赖：${package}"
  if ! "${SUDO[@]}" DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${package}"; then
    warn "可选依赖 ${package} 未安装；只有实体 Livox 点云转 /scan 路径会受影响。"
  fi
done

if command -v rosdep >/dev/null 2>&1; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "正在初始化 rosdep。"
    "${SUDO[@]}" rosdep init || warn "rosdep init 未完成；如果提示已初始化，可忽略。"
  fi
  log "正在更新 rosdep 数据。"
  rosdep update || warn "rosdep update 未完成；网络恢复后可手动重试。"
fi

if [[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  source_setup_file "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
else
  die "未找到 /opt/ros/${ROS_DISTRO_NAME}/setup.bash，请检查 ROS 2 安装结果。"
fi

log "基础环境安装完成。建议继续运行：bash scripts/01_setup_inspection_workspace.sh"

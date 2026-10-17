#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WORKSPACE_DIR}"

set +u
# shellcheck disable=SC1091
source scripts/setup_env.sh
set -u

# 清理残留进程：上一轮任务的 Gazebo/Nav2 若还活着，同名节点会抢答
# change_state 服务（旧 map_server 已 active，收到 configure 非法转换直接失败），
# 导致 lifecycle_manager 报 "Failed to change state for node: map_server" 中止 bringup
echo "[run_inspection_env] 清理残留的仿真/导航进程..."
for pat in minimal_user_node gzserver gzclient rviz2 map_server amcl \
           planner_server controller_server smoother_server behavior_server \
           bt_navigator waypoint_follower lifecycle_manager static_transform_publisher \
           "ros2 launch" "ros2 topic"; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 2

SETUP_CMD="cd \"${WORKSPACE_DIR}\"; source \"${WORKSPACE_DIR}/scripts/setup_env.sh\""

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

clean_terminal_env gnome-terminal \
  --window --title="inspection sim" --command="bash -lc \"${SETUP_CMD}; ros2 launch ican_inspection_sim inspection.launch.py gui:=true ground_demo:=false camera_rviz:=false; exec bash\"" \
  --tab --title="user minimal node" --command="bash -lc \"sleep 10; ${SETUP_CMD}; echo '[run_inspection_env] 正在等待 UAV 里程计 /uav/odom...'; timeout 45 ros2 topic echo /uav/odom --once >/dev/null 2>&1 || true; ros2 run ican_inspection_examples minimal_user_node; exec bash\"" \
  --tab --title="ground nav2" --command="bash -lc \"sleep 8; ${SETUP_CMD}; echo '[run_inspection_env] 正在等待 Gazebo 发布 /clock...'; timeout 30 ros2 topic echo /clock --once >/dev/null 2>&1 || true; ros2 launch ican_inspection_navigation nav2_ground.launch.py rviz:=true; exec bash\""

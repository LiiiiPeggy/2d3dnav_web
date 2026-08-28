#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd -- "${script_dir}/../.." && pwd)"

lidar_device="${1:-${DOG_LIDAR_DEVICE:-/dev/ttyUSB0}}"
image_tag="${DOG_IMAGE_TAG:-dog-cartographer-nav2:humble-arm64}"
container_name="${DOG_CONTAINER_NAME:-dog_cartographer_nav2}"
container_workspace="/root/work/tracker/cartographer_nav2_ws"
ros_domain_id="${ROS_DOMAIN_ID:-0}"

if ! command -v docker >/dev/null 2>&1; then
  echo "错误：没有找到 docker。" >&2
  exit 1
fi

if [[ ! -f "${workspace_root}/install/setup.bash" ]]; then
  echo "错误：工作空间尚未编译，找不到 ${workspace_root}/install/setup.bash" >&2
  echo "请先按照 README_CN.txt 在容器内完成 colcon build。" >&2
  exit 1
fi

if ! docker image inspect "${image_tag}" >/dev/null 2>&1; then
  echo "错误：找不到 Docker 镜像 ${image_tag}" >&2
  echo "请先执行 load_image_on_robot.bash。" >&2
  exit 1
fi

if docker container inspect "${container_name}" >/dev/null 2>&1; then
  container_state="$(
    docker container inspect --format '{{.State.Status}}' "${container_name}"
  )"
  echo "错误：容器 ${container_name} 已存在，当前状态：${container_state}" >&2
  echo "运行中可直接使用；停止后可执行：docker start ${container_name}" >&2
  echo "如需重新创建，请先明确执行：docker rm ${container_name}" >&2
  exit 1
fi

mkdir -p "${workspace_root}/maps"

docker_args=(
  run
  --detach
  --name "${container_name}"
  --restart unless-stopped
  --init
  --stop-timeout 25
  --network host
  --ipc host
  --env "ROS_DOMAIN_ID=${ros_domain_id}"
  --volume "${workspace_root}:${container_workspace}"
  --workdir "${container_workspace}"
  --log-opt max-size=10m
  --log-opt max-file=3
)

if [[ -e "${lidar_device}" ]]; then
  # 容器内统一使用 /dev/ttyUSB0，避免宿主机设备名影响 launch 白名单。
  docker_args+=(--device "${lidar_device}:/dev/ttyUSB0")
else
  echo "警告：没有找到雷达 ${lidar_device}，将只启动 Web 控制入口。" >&2
  echo "网页仍可访问，但当前容器不能启动真实雷达建图。" >&2
  echo "连接雷达后需重新创建容器并传入正确设备。" >&2
fi

docker "${docker_args[@]}" \
  "${image_tag}" \
  ros2 launch dog_cartographer_nav2_bringup \
    nav2_web_persistent.launch.py \
    enable_launch_control:=True \
    map_save_directory:="${container_workspace}/maps"

echo "Docker Web 控制服务已启动：${container_name}"
echo "开机恢复策略：unless-stopped"
echo "状态：docker ps --filter name=${container_name}"
echo "日志：docker logs -f ${container_name}"
echo "手机：打开 http://机器人IP:8081"

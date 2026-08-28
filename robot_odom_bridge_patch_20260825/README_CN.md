# 图 SLAM 导航补充 `/odom`

这个补丁解决以下问题：

- Cartographer 已经发布 `odom -> base_footprint` TF；
- Nav2/RPP 需要的 `/odom`（`nav_msgs/msg/Odometry`）却没有发布者；
- 控制器因此一直把当前速度当成 0，容易反复给出很小的转向速度。

补丁中的 `cartographer_tf_to_odom` 会以 50 Hz 发布 `/odom`。位置来自
Cartographer TF；选择手机中的“图 SLAM + IMU”时，角速度来自 YIS320
陀螺仪。它**不会发布 TF**，所以不会和 Cartographer 抢 TF。

## 唯一部署方法

先把 `robot_odom_bridge_patch_20260825.tar.gz` 复制到机器狗：

```text
/home/siasun/panda3_2026_08_07/panda3/src/nav2/robot_odom_bridge_patch_20260825.tar.gz
```

然后只在机器狗宿主机执行：

```bash
cd /home/siasun/panda3_2026_08_07/panda3/src/nav2

tar -xzf robot_odom_bridge_patch_20260825.tar.gz

cp -a \
  robot_odom_bridge_patch_20260825/workspace/src/. \
  nav2_arm64_offline_deploy_20260824/workspace/src/

docker exec dog_nav2 bash -lc '
set -e
source /opt/ros/humble/setup.bash
cd /root/work/nav2/cartographer_nav2_ws
source install/setup.bash
colcon build \
  --packages-select dog_cartographer_nav2_bringup nav2_web \
  --parallel-workers 8
'

docker restart dog_nav2
```

这里只重新编译两个小包，然后只重启 `dog_nav2`。不会修改或重启
`bottom_control`、强化学习等其他容器。

## 怎么使用

`dog_nav2` 重启后，Web 服务仍会自动启动。随后在手机上选择：

- `图 SLAM 纯激光 + MPPI`：`/odom` 的线速度和角速度均由
  Cartographer TF 计算；
- `图 SLAM + IMU + MPPI`：位置和线速度来自 Cartographer TF，角速度
  使用 `/Devices/Imu/Data`；
- AMCL 或 RF2O 启动方式不受这个补丁影响。

只有启动图 SLAM 导航后，`cartographer_tf_to_odom` 才会运行。Web 服务
单独运行时 `/odom` 没有发布者是正常的。

## 启动图 SLAM 后验证

```bash
docker exec -it dog_nav2 bash

source /opt/ros/humble/setup.bash
source /root/work/nav2/cartographer_nav2_ws/install/setup.bash

ros2 node list | grep cartographer_tf_to_odom
ros2 topic info /odom --verbose
timeout 6 ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 param get /controller_server odom_topic
```

正确结果应满足：

- 有 `/cartographer_tf_to_odom`；
- `/odom` 的 `Publisher count` 为 1；
- `/odom` 频率接近 50 Hz；
- `header.frame_id` 是 `odom`；
- `child_frame_id` 是 `base_footprint`；
- 控制器的 `odom_topic` 是 `/odom` 或 `odom`。

如果 `/controller_server` 显示 `Node not found`，说明 Nav2 没有成功启动，
需要先检查导航 launch 日志；这不是 `/odom` 节点参数问题。

## 现有 TF 告警

你日志中的 `TF_OLD_DATA ... frame base` 是另一条名为 `base` 的旧 TF。
本补丁使用的是 `base_footprint`，不会生成 `base`，因此这条告警需要另行
检查 `rl_real_panda3`、两个同名 `/robot_control_node` 或未知 TF 发布者。
它不是此前 `/odom` 没有发布者的直接原因。

## 关于 EKF

这版没有引入 `robot_localization`。目前没有可靠的腿式里程计或轮速里程计，
只有 IMU 和 Cartographer 位姿；直接再套一层 EKF 不会凭空得到更准确的
平移速度，还容易形成重复融合或 TF 所有权冲突。现在的做法保留
Cartographer 的位姿，并只用 IMU 陀螺仪增强角速度，是当前离线环境下更稳妥
的第一步。以后底层能提供独立腿式 `/odom_raw` 后，再加入 EKF 才有明显价值。

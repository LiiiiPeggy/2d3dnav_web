# nav2_web

> 0.3.0 adds the Android `SCAN 3D` scene protocol: sampled PointCloud2,
> Marker, Path, Odometry and TF data are relayed to the bundled offline WebGL
> renderer. Physical SCAN-Planner defaults are available through
> `ros2 launch nav2_web scanplanner_3d.launch.py`. The full topic/TF and phone
> instructions are in `docs/SCANPLANNER_3D_APP.md` at the workspace root.

面向手机横屏的 Nav2 全流程 Web 控制台，直接覆盖“SLAM 建图 → AMCL/Cartographer
图定位 → 轨迹规划”。它不启动或嵌入 RViz，通过 ROS 2 订阅地图、激光、
路径和速度，通过 TF 获取机器人位置，并使用 Nav2 API 保存地图、发布
初始位置、发送或取消导航目标。

## 功能

- 自动识别建图、AMCL/Cartographer 图定位和轨迹规划三个流程阶段
- 导航区提供 AMCL/图 SLAM 两个定位选项，并标识当前实际运行的后端；选择
  不匹配时暂停定位和目标按钮，提示先停止当前 launch 再切换，避免 TF 冲突
- 显示 `/map` (`nav_msgs/OccupancyGrid`) 和建图行走轨迹
- 显示建图健康度、已知面积、网格已知占比、地图稳定度和激光有效率
- 检查 SLAM、LaserScan、TF 和地图更新是否正常
- 显示以 `base_scan` 为中心的局部激光点，并将端点变换到 `map` 检查重定位匹配
- 显示 `/particle_cloud` AMCL 粒子、粒子扩散半径和收敛建议
- Web 初始位置优先调用 AMCL `/set_initial_pose` 服务，服务不可用时自动回退到
  RViz 同样使用的 `/initialpose` 话题
- Web 重启且机器人静止时自动调用 `/request_nomotion_update`，恢复完整 AMCL
  粒子云，不必等待机器人再次移动
- 显示全局地图、局部代价地图、机器人和雷达的实际 Frame ID
- 分别叠加全局/局部 Costmap 膨胀代价，支持独立显示开关
- 地图按已知栅格边界自动适配可用空间，避免大片未知区让有效地图显得过小；
  同时支持“大地图”模式隐藏顶栏和状态面板
- 在网页动态调整全局/局部 `inflation_radius`、`cost_scaling_factor`、
  `enabled`、`inflate_unknown` 和 `inflate_around_unknown`
- 通过 `/map_saver/save_map` 在网页保存 YAML/PGM 地图
- 显示 `map -> base_link` 实时位置
- 显示底层接收话题 `/cmd_vel`
- 显示 NavFn 在 `map` 下发布的全局路径 `/plan`
- 显示 MPPI `/trajectories`：淡蓝候选采样和粉色最优局部轨迹；Web Bridge
  自动将其从局部 `odom` 坐标系变换到 `map`
- 地图点击/拖动发送 `NavigateToPose` 目标和朝向
- 地图点击/拖动发布 `/initialpose`
- 一键清除网页旧激光/粒子/路径显示，并可调用
  `/reinitialize_global_localization` 重置当前定位后端
- 取消当前导航并显示反馈、剩余距离和导航时间
- Web 白名单 Launch 管理：启动/停止建图或导航流程，并在手机实时显示日志
- 单指拖动、双指缩放，横竖屏按实际 WebView 尺寸自适应

## 手机 Launch 管理

本工作区使用独立 Web Bridge 作为“启动起点”：

```bash
source /opt/ros/humble/setup.bash
source /home/u/cartographer_nav2_ws/install/setup.bash
ros2 launch dog_cartographer_nav2_bringup nav2_web_persistent.launch.py
```

浏览器打开 `http://电脑IP:8081`，在“手机 Launch 控制”卡片中可以：

- 启动 Cartographer 纯激光建图；
- 启动 Cartographer + RF2O 先验建图；
- 从工作区 `maps` 目录选择 YAML，启动 AMCL + MPPI 导航；
- 当同名 PBSTREAM 存在时，启动 Cartographer 图定位 + MPPI；
- 停止当前受管流程并查看实时 ROS/launch 输出。

同一时间只允许一个受管 launch，启动另一个前必须先点击“停止当前
流程”。所有受管入口都强制使用 `start_web:=False`，不会抢占长驻 Web
Bridge 的 `8081/8891` 端口。网页不接受任意 shell 命令，地图路径也被限制在
`/home/u/cartographer_nav2_ws/maps`。

如果 Cartographer、Nav2 或雷达驱动已由其他终端启动，网页会拒绝再次
启动，避免两套 ROS 节点和 TF 同时运行。请先安全停止原命令行流程，再从
网页启动。

> 当前 HTTP/WebSocket 没有鉴权和 TLS。启用 Launch 控制后只能在可信的
> 机器人局域网使用，不要把 `8081/8891` 暴露到公网。

SSH 断开时 Web Bridge 本身仍需要存活；可先在 `screen`/`tmux` 中运行上述
唯一一条基础 launch。如需开机自动启动，应再将它配置为 `systemd`
服务。

## 依赖

目标机器需要安装 Nav2：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

如果还需要建图：

```bash
sudo apt install ros-humble-slam-toolbox
```

## 编译

```bash
source /opt/ros/humble/setup.bash
cd /home/u/jie_deamon/nav2_web_ws
colcon build --symlink-install
source install/setup.bash
```

## 无 RViz 完整流程

每个终端先加载 ROS 2 和两个工作空间：

```bash
source /opt/ros/humble/setup.bash
source /home/u/dog_ws/install/setup.bash
source /home/u/jie_deamon/nav2_web_ws/install/setup.bash
```

首先启动 Web，Web 可在 SLAM/Nav2 切换时保持运行：

```bash
ros2 launch nav2_web nav2_web.launch.py \
  use_sim_time:=True \
  scan_topic:=/scan \
  particle_topic:=/particle_cloud \
  local_costmap_topic:=/local_costmap/costmap \
  global_costmap_topic:=/global_costmap/costmap \
  map_save_directory:=/home/u/dog_ws/maps
```

手机和机器人处于同一局域网时访问：

```text
http://机器人IP:8081
```

例如：

```text
http://10.10.10.186:8081
```

默认端口与原有跟随网页错开：HTTP `8081`，WebSocket `8891`。

### 1. SLAM 建图

```bash
export GAZEBO_MODEL_PATH=/home/u/dog_ws/src/navigation2/nav2_system_tests/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}
ros2 launch nav2_bringup tb3_simulation_launch.py \
  slam:=True \
  headless:=False \
  use_rviz:=False
```

网页会自动进入“建图”阶段。建图完成后，在右侧输入地图名，点击
“保存地图”。默认保存为：

```text
/home/u/dog_ws/maps/<地图名>.yaml
/home/u/dog_ws/maps/<地图名>.pgm
```

> 建图健康度是数据链路和网格稳定性的启发式评估，不等于几何精度真值。
> 保存前仍应目视检查墙体重影、断裂和闭环错位。

### 2. AMCL 重定位

关闭建图 launch，再启动：

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py \
  slam:=False \
  map:=/home/u/dog_ws/maps/gazebo_map.yaml \
  headless:=False \
  use_rviz:=False
```

网页自动进入“AMCL 重定位”。点击“初始位置”，在地图上按下机器人
实际位置，拖动箭头设置朝向。当 `map -> base_link` 可用后，流程自动进入
“轨迹规划”。

如果刚启动时红色激光匹配点与地图相差很大：

1. 先点击“清除旧显示”。它只清除 Web 中保留的激光端点、AMCL 粒子、
   路径和目标，不修改地图、TF 或里程计；新的实时数据会继续出现。
2. 如果新出现的红点仍然错位，先取消正在执行的导航，再点击“重置 AMCL”。
   网页会调用 `/reinitialize_global_localization`，暂停 `map` 下的激光投影，
   并自动切换到“初始位置”模式。
3. 在地图上重新设置机器人的真实位置和朝向，随后缓慢原地转动，观察红色
   激光端点与黑色墙体重合、AMCL 粒子逐步收敛。

“重置 AMCL”不会删除已知地图，也不会清零 `odom`。导航执行期间为避免
坐标系突变，该按钮会被禁用，必须先取消导航。

### 3. 轨迹规划

点击“导航目标”，在地图空闲区域拖出目标朝向。网页会显示 `/plan`
全局路径、MPPI 局部轨迹、剩余距离和导航时间。地图左上角可以分别开关
“全局路径”和“MPPI 轨迹”：

- 绿色线：`/plan`，全局规划器生成的整条几何路径，Frame ID 为 `map`。
- 淡蓝点：`/trajectories` 中 MPPI 每轮采样的候选局部轨迹。
- 粉色线：`/trajectories` 中 MPPI 本轮选出的最优局部轨迹，原始 Frame ID
  通常为 `odom`，网页显示前会变换到 `map`。

`/plan` 不是直接发送给底盘的带时间轨迹。MPPI 以它作为参考，在局部代价
地图中反复采样和评分，最后由 `controller_server` 发布 `/cmd_vel` 驱动底盘。

## 参数

```bash
ros2 launch nav2_web nav2_web.launch.py \
  cmd_vel_topic:=/cmd_vel \
  scan_topic:=/scan \
  particle_topic:=/particle_cloud \
  local_costmap_topic:=/local_costmap/costmap \
  global_costmap_topic:=/global_costmap/costmap \
  local_costmap_node:=/local_costmap/local_costmap \
  global_costmap_node:=/global_costmap/global_costmap \
  map_topic:=/map \
  path_topic:=/plan \
  mppi_trajectories_topic:=/trajectories \
  set_initial_pose_service:=/set_initial_pose \
  save_map_service:=/map_saver/save_map \
  reset_localization_service:=/reinitialize_global_localization \
  nomotion_update_service:=/request_nomotion_update \
  map_save_directory:=/home/u/dog_ws/maps \
  map_frame:=map \
  odom_frame:=odom \
  base_frame:=base_link
```

Gazebo 必须使用 `use_sim_time:=True`，否则 Web 发布的初始位置时间戳和
Nav2/AMCL 的仿真时钟不一致。真实机器人运行时改为 `use_sim_time:=False`。

## Frame 关系

```text
map                     已知全局地图、导航目标、AMCL 粒子和全局路径
 └─ odom                连续的局部里程计坐标系，局部代价地图的 global_frame
     └─ base_footprint  机器人在地面的二维投影
         └─ base_link   机器人本体
             └─ base_scan 雷达原始 LaserScan
```

`/scan` 的点本来在 `base_scan`。Web Bridge 查询 `map -> base_scan` TF 后，
把激光端点画到全局 `map` 上。红色端点与静态地图中的黑色墙体越
重合，AMCL 位姿越合理。`/particle_cloud` 是 AMCL 位姿候选集，不是障碍物点云。

## 膨胀层调参

网页会读取并设置以下两个节点的动态参数：

```text
/global_costmap/global_costmap
/local_costmap/local_costmap
```

- `inflation_radius`：障碍物外向生成代价的最大距离，越大走得越保守。
- `cost_scaling_factor`：代价随距离衰减的速度，越大衰减越快。
- `inflate_unknown`：是否把未知空间本身当作需膨胀区。
- `inflate_around_unknown`：是否在未知边界周围生成膨胀代价。

修改后点击“应用全局参数”或“应用局部参数”，Nav2 运行中立即生效。
这些动态值不会自动回写 `nav2_params.yaml`；确认效果后应把最终数值同步到配置文件。

如果底层实际接收 `/cmd_vel_out`：

```bash
ros2 launch nav2_web nav2_web.launch.py cmd_vel_topic:=/cmd_vel_out
```

## 运行前检查

```bash
ros2 topic info /map -v
ros2 topic info /scan -v
ros2 topic info /plan -v
ros2 topic info /trajectories -v
ros2 topic info /cmd_vel -v
ros2 service type /map_saver/save_map
ros2 service type /reinitialize_global_localization
ros2 action info /navigate_to_pose
ros2 run tf2_ros tf2_echo map base_link
```

没有 `/map` 时网页会显示“等待地图”；没有 `map -> base_link` 时地图可以显示，
但机器人位置显示为未就绪。Web 服务仅监听，不会替代 Nav2 或底盘节点。

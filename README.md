# Cartographer Nav2 四足机器人操作文档

YDLidar 二维激光雷达 + Cartographer SLAM（纯激光 / IMU / RF2O 建图与图定位）+ Nav2（AMCL、NavFn、MPPI）+ 手机 Web/Android 控制台的操作说明。

## 目录结构

| 目录 | 说明 |
|------|------|
| `src/dog_cartographer_nav2_bringup/` | 实物四足机器人建图、定位、导航和 Web Bridge 启动入口 |
| `src/drivers/ydlidar_ros2_driver/` | YDLidar ROS 2 驱动及雷达参数 |
| `src/localization/rf2o_laser_odometry/` | RF2O 激光里程计 |
| `src/navigation2/` | Nav2、NavFn、MPPI 等导航源码 |
| `src/web/nav2_web/` | 手机浏览器使用的 Nav2 Web 控制台 |
| `nav2_android/` | Dog Nav2 Android 客户端源码 |
| `artifacts/` | Android APK、机器人端离线部署包及校验文件 |
| `maps/` | 运行时保存的 `.yaml`、`.pgm` 和 `.pbstream` 地图（首次使用前创建） |
| `robot_odom_bridge_patch_20260825/` | 历史补丁备份，不参与当前工作区编译 |

## 前置条件

- Ubuntu 22.04
- ROS 2 Humble
- 已安装 `colcon`、`rosdep`、Cartographer ROS 和工作区依赖
- 已安装 YDLidar-SDK
- YDLidar 默认串口为 `/dev/ttyUSB0`
- IMU 模式默认订阅 `/Devices/Imu/Data`，消息坐标系为 `YIS320`

安装常用依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-cartographer ros-humble-cartographer-ros \
  ros-humble-navigation2 ros-humble-nav2-bringup
```

如果系统尚未安装 YDLidar-SDK，需要先编译并安装官方 SDK，否则 `ydlidar_ros2_driver` 会在 CMake 阶段提示找不到 `ydlidar_sdk`。

### YDLidar 设备权限

```bash
cd /home/w/cartographer_nav2_ws
sudo sh src/drivers/ydlidar_ros2_driver/startup/initenv.sh
sudo usermod -aG dialout "$USER"
```

重新插拔雷达并重新登录终端，然后检查设备：

```bash
ls -l /dev/ttyUSB0 /dev/ydlidar 2>/dev/null
```

如果实际设备不是 `/dev/ttyUSB0`，启动时通过 `serial_port:=实际设备` 指定。

## 1. 编译工作区

```bash
cd /home/w/cartographer_nav2_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
mkdir -p maps
```

这里必须保留 `--base-paths src`，避免 `robot_odom_bridge_patch_20260825/` 中的同名备份包被 Colcon 重复发现。

以后每个新终端都需要加载环境：

```bash
source /opt/ros/humble/setup.bash
source /home/w/cartographer_nav2_ws/install/setup.bash
```

多台设备组成 ROS 2 网络时，各设备的 `ROS_DOMAIN_ID` 必须一致，例如：

```bash
export ROS_DOMAIN_ID=71
```

## 2. 启动长驻 Web 控制台（推荐）

先在独立终端启动 Web Bridge：

```bash
source /opt/ros/humble/setup.bash
source /home/w/cartographer_nav2_ws/install/setup.bash
ros2 launch dog_cartographer_nav2_bringup nav2_web_persistent.launch.py
```

手机或电脑浏览器访问：

```text
http://机器人IP:8081
```

默认端口：HTTP `8081`，WebSocket `8891`。在网页中可启动或停止建图、选择地图启动导航、保存地图、设置初始位置、发送/取消目标并查看实时日志。

同一时间只允许一个受 Web 管理的建图或导航流程。长驻 Bridge 已运行时，不要给其他 launch 传入 `start_web:=True`，否则会抢占 `8081/8891` 端口。当前 Web 服务没有鉴权和 TLS，只能用于可信的机器人局域网，不能直接暴露到公网。

## 3. 建图

### 网页方式

进入网页“建图”页，根据传感器选择一种模式：

| 模式 | 用途 |
|------|------|
| `Cartographer 纯激光建图` | 只使用 `/scan`，默认且最简单的入口 |
| `Cartographer + IMU 建图` | 融合 `/scan` 与 `/Devices/Imu/Data` |
| `Cartographer + RF2O 建图` | RF2O 发布 `/odom_rf2o` 作为 Cartographer 里程计先验 |

确认雷达、TF 和地图状态正常后遥控机器人缓慢巡场；经过走廊、墙角等有辨识度的区域，并尽量回到起点形成回环。

### 命令行方式

纯激光建图：

```bash
ros2 launch dog_cartographer_nav2_bringup cartographer_mapping.launch.py \
  serial_port:=/dev/ttyUSB0
```

激光 + IMU 建图：

```bash
ros2 launch dog_cartographer_nav2_bringup cartographer_mapping.launch.py \
  serial_port:=/dev/ttyUSB0 \
  publish_imu_tf:=True \
  cartographer_configuration_basename:=ydlidar_cartographer_2d_imu.lua
```

激光 + RF2O 先验建图：

```bash
ros2 launch dog_cartographer_nav2_bringup cartographer_rf2o_mapping.launch.py \
  serial_port:=/dev/ttyUSB0
```

如果雷达驱动已由其他进程启动，增加 `start_lidar:=False`；如果机器人自身已经发布 `base_footprint -> laser_frame`，增加 `publish_laser_tf:=False`，避免重复 TF。

## 4. 保存地图

推荐在网页建图页输入地图名并点击保存。以 `dog_map` 为例，系统会同时生成：

```text
/home/w/cartographer_nav2_ws/maps/dog_map.yaml
/home/w/cartographer_nav2_ws/maps/dog_map.pgm
/home/w/cartographer_nav2_ws/maps/dog_map.pbstream
```

- AMCL 导航需要同名 `.yaml` 和 `.pgm`。
- Cartographer 图定位除 `.yaml/.pgm` 外，还需要同名 `.pbstream`。

命令行备用保存方式：

```bash
mkdir -p /home/w/cartographer_nav2_ws/maps

ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: '/map', map_url: '/home/w/cartographer_nav2_ws/maps/dog_map', image_format: 'pgm', map_mode: 'trinary', free_thresh: 0.25, occupied_thresh: 0.65}"

ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/w/cartographer_nav2_ws/maps/dog_map.pbstream', include_unfinished_submaps: true}"
```

确认三个文件保存成功后再停止建图流程。

## 5. 定位与 MPPI 导航

启动导航前必须停止建图，防止多个节点同时发布 `map -> odom` 或 `odom -> base_footprint`。

### AMCL + MPPI

网页“导航”页选择 `AMCL + MPPI 导航` 和对应 YAML 地图；命令行等效命令为：

```bash
ros2 launch dog_cartographer_nav2_bringup nav2_mppi_navigation.launch.py \
  map:=/home/w/cartographer_nav2_ws/maps/dog_map.yaml \
  serial_port:=/dev/ttyUSB0
```

该模式默认由 AMCL 发布 `map -> odom`，RF2O 发布 `/odom` 和 `odom -> base_footprint`。如果底盘已经提供可靠的 `/odom` 与该 TF，增加 `start_rf2o:=False`。

### Cartographer 图定位 + MPPI

纯激光图定位：

```bash
ros2 launch dog_cartographer_nav2_bringup cartographer_mppi_navigation.launch.py \
  map:=/home/w/cartographer_nav2_ws/maps/dog_map.yaml \
  pbstream:=/home/w/cartographer_nav2_ws/maps/dog_map.pbstream \
  serial_port:=/dev/ttyUSB0
```

激光 + IMU 图定位：

```bash
ros2 launch dog_cartographer_nav2_bringup cartographer_mppi_navigation.launch.py \
  map:=/home/w/cartographer_nav2_ws/maps/dog_map.yaml \
  pbstream:=/home/w/cartographer_nav2_ws/maps/dog_map.pbstream \
  serial_port:=/dev/ttyUSB0 \
  publish_imu_tf:=True \
  odom_use_imu_angular_velocity:=True \
  cartographer_configuration_basename:=ydlidar_cartographer_2d_localization_imu.lua
```

图定位模式由 Cartographer 加载冻结的 `.pbstream`，`cartographer_tf_to_odom` 只根据 TF 生成 `/odom` 消息，不重复发布 TF。

### 设置初始位置与导航目标

1. 在网页地图上选择“初始位置”，按住并拖动设置机器人的实际位置和朝向。
2. 等待激光与静态地图重合、定位状态稳定。
3. 选择“导航目标”，按住并拖动设置目标位置和最终朝向。
4. 需要停止任务时先在网页点击“取消导航”；紧急情况应使用机器人底层急停或切断运动使能。

## 6. Android 客户端安装与操作

### 安装 App

开发电脑通过 ADB 安装工作区中现成的 APK：

```bash
adb devices -l
adb install -r /home/w/cartographer_nav2_ws/artifacts/dog-nav2-0.2.6-debug.apk
```

机器人端必须先保持 Web Bridge 运行：

```bash
source /opt/ros/humble/setup.bash
source /home/w/cartographer_nav2_ws/install/setup.bash
ros2 launch dog_cartographer_nav2_bringup nav2_web_persistent.launch.py
```

### 首次连接机器人

1. 确保手机和机器人连接到同一个可信局域网。
2. 打开 `Dog Nav2`，点击“配置机器人连接”。
3. “机器人 IP / 主机名”只填写机器人地址，例如 `192.168.1.100`，不要填写路径。
4. HTTP 端口填写 `8081`，WebSocket 端口填写 `8891`。
5. 点击“保存并连接”，顶部状态显示“已连接”或控制台显示“Web 已连接”后再操作。

App 会保存上次填写的地址。需要更换机器人时，点击顶部的“已连接”或“连接设置”重新配置。Web Bridge 暂时重启时 App 页面不会退出，连接恢复后会自动重连。

### 使用 App 建图

1. 点击底部“建图”。
2. 在 Launch 控制中选择建图模式：
   - 普通使用选择 `Cartographer 纯激光建图`；
   - IMU 和 `YIS320` TF 已配置好时选择 `Cartographer + IMU 建图`；
   - 需要激光里程计先验时选择 `Cartographer + RF2O 建图`。
3. 点击“启动建图”，观察实时日志，并等待 SLAM、激光、TF、地图流进入正常状态。
4. 缓慢遥控机器人巡场，避免急转弯，并尽量回到起点形成回环。
5. 在“地图名称”中填写名称，例如 `dog_map`，点击“保存地图”。
6. 等待界面提示 YAML/PGM 和 PBSTREAM 均保存成功，再点击“停止当前流程”。

地图名只能包含中英文、数字、下划线和短横线。保存结果位于机器人工作区的 `maps/` 目录。

### 使用 App 定位与导航

1. 确认建图流程已经停止，然后点击底部“导航”。
2. 选择定位方法和对应 Launch：
   - `AMCL`：选择 `AMCL + MPPI 导航`，只要求存在 YAML/PGM 地图；
   - `图 SLAM`：选择纯激光或 IMU 的 Cartographer + MPPI，要求存在同名 YAML/PGM/PBSTREAM。
3. 在“导航地图”中选择地图，点击“启动导航”，等待地图、定位器和规划器就绪。
4. 点击地图上方“初始位置”，在机器人真实位置按住并拖动，箭头方向代表机器人朝向，松手发送。
5. 观察红色激光端点是否与黑色墙体重合；匹配稳定后再发送目标。
6. 点击“导航目标”，在目标位置按住并拖动设置最终朝向，松手后机器人开始导航。
7. 通过全局路径、MPPI 轨迹、剩余距离、导航时间和 `/cmd_vel` 判断运行状态。
8. 需要停止任务时点击“取消当前导航”；确认机器人停止后，才能点击“停止当前流程”或切换定位方式。

如果初始定位明显错误，先点击“清除旧显示”排除页面残留；仍然错位时点击“重置定位”，然后重新设置初始位置。导航运行中必须先取消导航，才能重置定位。

### 地图触控与页面功能

- “浏览”模式下单指拖动地图，双指缩放，也可使用 `+`、`-`。
- “适配”显示整张地图，“居中”将视图移动到机器人，“大地图”隐藏部分面板。
- “图层”可显示或隐藏激光点、定位粒子、全局/局部膨胀层、全局路径和 MPPI 轨迹。
- “状态”页可查看并调整 Costmap 膨胀参数；不熟悉参数含义时不要在机器人运动中修改。
- “设置”页可调整地图与控制面板比例、按钮大小并保存横竖屏布局。
- 横屏下可拖动地图、控制面板和底部摘要之间的分隔条调整窗口大小。

强制关闭 App 不等于取消导航。离开控制界面前应先点击“取消当前导航”，再停止当前受管流程；紧急情况使用机器人底层急停或切断运动使能。

### 从源码构建 App

需要从源码构建 Android 客户端时：

```bash
cd /home/w/cartographer_nav2_ws/nav2_android
source ./android-env.sh
./gradlew :nav2-app:assembleDebug
```

APK 输出到 `nav2_android/nav2-app/build/outputs/apk/debug/`。

## 常用 Launch 与参数

| Launch | 用途 |
|------|------|
| `nav2_web_persistent.launch.py` | 长驻 Web/Android Bridge 和白名单 Launch 管理 |
| `cartographer_mapping.launch.py` | Cartographer 纯激光或 IMU 建图 |
| `cartographer_rf2o_mapping.launch.py` | Cartographer + RF2O 里程计先验建图 |
| `nav2_mppi_navigation.launch.py` | YAML 地图 + AMCL + RF2O + Nav2 MPPI |
| `cartographer_mppi_navigation.launch.py` | YAML/PBSTREAM + Cartographer 图定位 + Nav2 MPPI |

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyUSB0` | 雷达串口 |
| `scan_topic` | `/scan` | 激光话题 |
| `start_lidar` | `True` | 是否由当前 launch 启动雷达 |
| `publish_laser_tf` | `True` | 是否发布 `base_footprint -> laser_frame` |
| `laser_z` | `0.35` | 雷达相对底盘高度，单位为米 |
| `laser_yaw` | `3.141592653589793` | 雷达相对底盘偏航角 |
| `map` | `maps/dog_map.yaml` | Nav2 占据栅格地图 |
| `pbstream` | `maps/dog_map.pbstream` | Cartographer 冻结图 |
| `start_rf2o` | `True` | AMCL 导航时是否由 RF2O 提供里程计 |
| `start_web` | `False` | 是否随当前流程启动 Web；长驻 Bridge 运行时保持 `False` |

## 关键话题与 TF

| 名称 | 说明 |
|------|------|
| `/scan` | YDLidar 激光数据 |
| `/map` | Cartographer 或 Map Server 发布的占据栅格地图 |
| `/odom_rf2o` | 建图模式下提供给 Cartographer 的 RF2O 先验 |
| `/odom` | Nav2 控制器使用的里程计 |
| `/plan` | NavFn 全局路径 |
| `/trajectories` | MPPI 候选轨迹与最优局部轨迹 |
| `/cmd_vel` | 速度平滑后的底盘控制指令 |
| `/initialpose` | Web/RViz 设置的初始位置 |
| `/navigate_to_pose` | Nav2 单点导航 Action |

标准 TF 链为：

```text
map -> odom -> base_footprint -> laser_frame
                              -> YIS320（IMU 模式）
```

每一条 TF 只能有一个发布者。建图、AMCL 导航和 Cartographer 图定位不能同时运行；机器人自身已有静态 TF 时，应关闭对应的 `publish_laser_tf` 或 `publish_imu_tf`。

## 常用检查与故障排查

```bash
# 雷达是否正常输出
ros2 topic hz /scan
ros2 topic echo /scan --once

# 地图和里程计
ros2 topic hz /map
ros2 topic hz /odom

# 检查关键 TF
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_frame

# 检查导航接口与速度输出
ros2 action info /navigate_to_pose
ros2 topic echo /cmd_vel

# 检查 Web 端口是否被重复占用
ss -lntp | grep -E ':8081|:8891'
```

- 找不到雷达：检查串口名、USB 连接、udev 规则和 `dialout` 用户组。
- 网页打不开：确认机器人 IP 可达，并检查 `nav2_web_bridge` 日志及 `8081/8891` 端口。
- 网页提示没有地图：确认建图节点或 Map Server 正在发布 `/map`。
- Cartographer 图定位无法启动：确认 `.yaml`、`.pgm`、`.pbstream` 同名且路径正确。
- 机器人位置跳变或 TF 报冲突：检查是否同时运行了两套建图/定位流程，或底盘与 launch 是否重复发布同一 TF。
- 有路径但机器人不运动：检查 `/odom`、`/cmd_vel`、底盘运动使能和急停状态。

# SCAN-Planner_WAB 操作文档

Livox Mid-360S 激光雷达 + FAST-LIO（建图 / 定位）+ PCT（全局规划）+
SCAN-Planner（局部避障 / 实机控制）的操作说明。

正常使用时，电脑端只需启动一次 `planner` Server。之后雷达、FAST-LIO、建图、
重定位、PCT 和 SCAN-Planner 都由手机 App 启动或停止，不需要再开多个算法终端。

```text
电脑启动 planner Server → 手机连接 → App 选择运行模式 → 查看三维画面和日志
```

## 目录结构

| 目录 | 说明 |
|------|------|
| `FAST_LIO-ROS2/` | FAST-LIO 实时定位与 PCD 建图 |
| `Fast-LIO2-Localization/` | 历史 PCD 地图 ICP 重定位 |
| `PCT_planner/` | PCD 层析建图与三维全局路径规划 |
| `SCAN-Planner-Ros2/` | SCAN-Planner 局部规划、避障和速度控制 |
| `web/nav2_web/` | 手机/Web 三维监控与 ROS Launch 管理服务 |
| `nav2_android/` | `planner` Android App 工程 |

## 前置条件

- Ubuntu 22.04 + ROS 2 Humble
- Livox Mid-360S 及 `livox_ros_driver2` 工作区
- 电脑与手机位于同一可信局域网
- 电脑与机器狗底层使用相同的 `ROS_DOMAIN_ID`
- 机器狗底层订阅 `/cmd_vel`，消息类型为 `geometry_msgs/msg/Twist`
- 默认地图库：`/home/w/scanplanner_maps`
- 数据流：

```text
Mid-360S → FAST-LIO → PCT 全局路径 → SCAN-Planner 局部避障 → /cmd_vel
```

## 1. 编译

```bash
source /opt/ros/humble/setup.bash
source /home/w/ws_livox/install/setup.bash

cd /home/w/scanplanner_wab
rosdep install --from-paths src --ignore-src -r -y

# 第一次使用 PCT 时编译第三方库
cd src/PCT_planner && ./build_3rdparty.sh

cd /home/w/scanplanner_wab
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 2. 启动 planner Server

每次电脑开机后，在运行算法的电脑上执行下面这一条启动流程：

```bash
source /opt/ros/humble/setup.bash
source /home/w/ws_livox/install/setup.bash
source /home/w/scanplanner_wab/install/setup.bash

export ROS_DOMAIN_ID=71          # 按机器狗实际配置修改
export ROS_LOCALHOST_ONLY=0
ros2 launch scan_planner planner_app.launch.py
```

`planner_app.launch.py` 是唯一的常驻 Server 入口。它会启动 HTTP、WebSocket、三维
数据桥和受限的 ROS Launch 管理器；保持该终端运行即可。

Server 启动后，不需要手动启动 Livox、FAST-LIO、PCT 或 SCAN-Planner，这些流程
全部在 App 的“系统启动”页面中选择。

手机连接同一局域网，打开 `planner` App，填写算法电脑 IP：

```text
HTTP：8081
WebSocket：8891
```

也可以在浏览器打开 `http://电脑IP:8081`。连接后可使用“三维监控”“系统启动”
和“运行终端”。Server 本身不会自动启动雷达或控制机器狗。

## 3. 雷达与 FAST-LIO 检查

Server 保持运行，先在 App“系统启动”中选择“Mid-360S 雷达终端”，确认雷达和
IMU 持续输出：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

停止雷达终端，再选择“FAST-LIO 定位终端”。启动后保持机器人静止 5～10 秒，
等待 IMU 初始化，在“三维监控”中确认点云、机器人位姿和地面方向正确。

```bash
ros2 topic hz /cloud_registered
ros2 run tf2_ros tf2_echo map trunk
ros2 run tf2_ros tf2_echo trunk livox_frame
```

点云、机身位置或 TF 不正确时，应先检查安装外参，不得启动实机控制。

## 4. 实时规划（不使用历史地图）

无需打开新终端，直接在 App“系统启动”中选择对应模式。先运行“安全预览”，
确认轨迹正确后再运行“实机控制”。

| 模式 | 用途 |
|------|------|
| 模式 1 | App 发送单目标到 `/move_base_simple/goal` |
| 模式 2 | 按 YAML 中的 `fsm.waypoints` 顺序导航 |
| 模式 3 | 接收 `/initial_path`，沿全局参考路径进行局部避障 |

模式 2 启动 App 服务时必须指定真实航点文件：

```bash
ros2 launch scan_planner planner_app.launch.py \
  keypoints_file:=/absolute/path/to/keypoints.yaml
```

模式 3 可指定参考路径文件；未指定时等待外部节点发布 `/initial_path`：

```bash
ros2 launch scan_planner planner_app.launch.py \
  reference_path_file:=/absolute/path/to/reference_path.yaml
```

## 5. 建图与保存 PCD

在 App 中选择“FAST-LIO 全局 PCD 建图”：

1. 输入地图名，例如 `building_a`；
2. 启动建图，让机器人缓慢走遍需要导航的区域；
3. 确认点云完整且没有明显重影；
4. 点击“保存地图并停止”。

地图保存为：

```text
/home/w/scanplanner_maps/building_a.pcd
```

如果保存失败，建图会继续运行。根据 App 终端提示排查后重新保存，不要直接关闭
App 常驻服务。

## 6. PCD 转 PCT 全局图

在 App 中选择“PCD → PCT 全局图”，选择刚保存的 `building_a`，设置最低可走
地面高度和垂直分层间隔，然后启动。

终端出现 `Tomogram exported` 后停止流程，生成文件：

```text
/home/w/scanplanner_maps/building_a.pickle
```

PCD 与 PCT 文件必须同名，App 才会识别为一套地图：

```text
building_a.pcd
building_a.pickle
```

可先运行“PCT 示例地图 · App 两点测试”，使用虚拟起点检查目标点和多点路线。
该模式不启动雷达，也不会发布速度。

## 7. 历史地图重定位与导航

在 App 中选择“历史地图模式 3 · 安全预览”：

1. 将机器人放在历史地图覆盖区域；
2. 选择同名的 PCD/PCT 地图并启动；
3. 保持机器人静止，等待雷达和定位节点就绪；
4. 在“三维监控”中点击“粗定位”；
5. 点击机器人的实际地面位置，拖动箭头设置朝向；
6. 等待 ICP 收敛，确认实时点云与历史点云重合；
7. 使用“点云目标”发送单目标，或使用“点云路线”发送多途经点；
8. 确认 PCT 全局路径和 SCAN-Planner 局部轨迹正确。

预览正常后停止当前流程，再选择“历史地图模式 3 · 实机控制”，使用同一地图并
重新完成粗定位。ICP 未收敛、TF 异常或点云未重合时禁止启动控制。

## 8. 实机控制与停止

实机控制前确认急停可用、场地清空，并在机器狗底层检查 `/cmd_vel` 通信：

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /scan_planner/fastlio_inputs_ready --once
ros2 topic echo /scan_planner/body_pose --once
```

- “安全预览”只规划和显示，不发布 `/cmd_vel`
- “实机控制”会发布 `/cmd_vel`，运行期间必须有人监控
- 普通模式点击“停止当前流程”结束
- PCD 建图模式点击“保存地图并停止”结束
- 完全关闭 App 服务时，在电脑启动终端按 `Ctrl+C`

> 手机控制接口没有鉴权和 TLS，只能在可信局域网使用，禁止将 `8081` 和 `8891`
> 暴露到公网。

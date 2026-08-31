# planner Android App

当前 App 名称为 **planner**，版本 `0.6.2`，Debug 包 ID 为
`com.dog.planner.debug`。它与原来的 Dog Nav2
（`com.dog.nav2controller.debug`）是两个独立 App，可以同时安装。

## 当前界面

App 已经针对 Mid-360S + FAST-LIO + SCAN-Planner 实机流程重构为三个模式：

- **三维监控**：用离线 WebGL 显示注册点云、PCT 可通行区域、占据点云、规划 Marker、
  轨迹、机器狗位姿和 TF；默认单指 360° 环绕，双指平移缩放，也可切到单指平移；
  PCT 模式的目标、粗定位和多途经点只从绿色可通行区域拾取；
- **系统启动**：选择雷达终端、FAST-LIO 定位，以及 SCAN-Planner 模式 1/2/3
  的安全预览或实机控制；每个入口按组件、TF、外参、ICP、控制输出分组显示本次启动
  参数，参数通过服务端白名单校验并保存在手机本地；
- **运行终端**：显示受管 `ros2 launch` 的实时标准输出、PID 和退出码，并可停止流程。

所有 HTML、CSS、JavaScript 和 WebGL 渲染代码均在 APK 内。ROS launch 重启时界面不会
消失，WebSocket 断开后会自动重连。点云在机器人端只为手机显示进行抽样，算法仍使用
原始全量 topic。

## 构建与安装

```bash
cd /home/u/scanplanner_wab/src/nav2_android
source ./android-env.sh
./gradlew :nav2-app:spotlessApply :nav2-app:testDebugUnitTest :nav2-app:assembleDebug
adb install -r nav2-app/build/outputs/planner/planner.apk
```

生成文件：

```text
/home/u/scanplanner_wab/src/nav2_android/nav2-app/build/outputs/planner/planner.apk
```

## 只用手机启动

首次安装时先在电脑上安装用户服务：

```bash
cd /home/u/scanplanner_wab
sudo loginctl enable-linger "$USER"
./deployment/install_planner_app_user_service.bash
```

以后电脑开机后无需打开 RViz、终端或 SSH，直接打开手机 App 即可。这个常驻服务只启动
App 桥接和受限的 launch 管理器，不会自动启动雷达，也不会让机器狗运动。可用下面命令
检查或重启：

```bash
systemctl --user status planner-app.service
systemctl --user restart planner-app.service
journalctl --user -u planner-app.service -f
```

若 Livox 工作空间不在默认的 `/home/u/ws_livox`，安装服务前为
`PLANNER_LIVOX_SETUP` 设置它的 `install/setup.bash` 路径。

连接后，在 App 的“系统启动”中选择模式。每次只允许一个受管流程：

每个可运行模式的“本次启动参数”都有“电脑同时打开 RViz”开关，默认关闭；手机会把
选择传给对应 ROS launch。该开关只控制电脑显示，不影响手机三维画面。电脑必须已经
登录图形桌面，否则 RViz 无法创建窗口，但 ROS 算法仍可正常运行。

| 模式 | 组件 | 是否发布 `/cmd_vel` |
|---|---|---|
| Mid-360S 雷达终端 | Livox Mid-360S 驱动 | 否 |
| FAST-LIO 定位终端 | Livox + FAST-LIO | 否 |
| 模式 1/2/3 · 安全预览 | Livox + FAST-LIO + Planner | 否 |
| 模式 1/2/3 · 实机控制 | 完整链路 + 闭环控制器 | **是** |
| FAST-LIO 全局 PCD 建图 | Livox + 稳定版 FAST-LIO + `/map_save` | 否 |
| PCD → PCT 全局图 | CUDA tomography | 否 |
| PCT 示例地图 · App 两点测试 | 固定虚拟起点 + PCD + PCT | 否 |
| 历史地图 PCT 模式 3 | ICP + Localization FAST-LIO + PCT + SCAN | 预览否/控制是 |

`FAST-LIO 全局 PCD 建图`运行时，普通停止按钮会自动显示为“保存地图并停止”。点击后
App 先调用 FAST-LIO `/map_save`，等待服务返回成功并确认所选的
`/home/u/scanplanner_maps/地图名.pcd` 是非空文件，然后才停止建图。若保存失败，建图
保持运行，手机会显示失败原因，可以排除问题后再次点击；不要在保存过程中关闭常驻 Web。

三种 `navi_mode` 的输入分别是：模式 1 的 `/move_base_simple/goal`、模式 2 参数文件中的
`fsm.waypoints`、模式 3 的 `/initial_path`。模式 2 不会加载仓库示例坐标，必须在
常驻入口中显式配置真实航点文件：

```bash
ros2 launch scan_planner planner_app.launch.py \
  keypoints_file:=/absolute/path/to/keypoints.yaml
```

PCT 启动后发布绿色 `/pct/traversable` 可通行点云。App 的“点云目标”和“多点路线”只会
吸附到这些 PCT 已判定可通行的单元，再发布 `/move_base_simple/goal` 或
`/pct_waypoints`。PCT 从当前 `/scan_planner/body_pose` 开始按列表逐段规划，全部成功后
拼接为一条 `/initial_path` 给 SCAN 模式 3。选中可通行单元可避免点落在障碍物上；若
起点与目标位于互不连通的区域，PCT 仍会正常拒绝该路线。

历史地图重定位需要一个大致初始位姿。在 PCT 模式中点“粗定位”，触摸绿色可通行区域
确定地面位置并拖动箭头确定朝向，App 会发布 `/initialpose`；无需在启动参数里填写
`initial_x/y/yaw`。地图要选对，粗位置和朝向越接近真实位置，ICP 越容易收敛。

`PCD → PCT 全局图` 在手机上开放“最低可走地面 Z”和“PCT 垂直分层间隔”；PCT 示例及
历史地图模式开放“地面到 trunk 高度”。这些值随当前模式保存在手机本地，修改后由手机
启动流程时传入，不需要再到终端手写。

地图相关模式会显示“本次使用的地图”。App 自动扫描机器人端安全目录
`/home/u/scanplanner_maps`，按同名规则配对 `名字.pcd` 与 `名字.pickle`。可以从列表选择
已有地图；FAST-LIO 建图入口也可以直接输入一个新的地图名。App 不接受任意绝对路径，
因此不能越过安全目录读取机器人上的其他文件。

不接机器人时，可以给常驻入口传入示例 `building2_9.pcd` 和已经生成的
`building2_9.pickle`，再选择“PCT 示例地图 · App 两点测试”。该入口只发布虚拟固定
`body_pose`、示例 `/prior_map` 和 PCT `/initial_path`，绝不会启动雷达或发布速度。

控制模式在 App 中有红色提示和二次确认。仍应保证场地清空、急停可用，并先在安全预览
模式确认定位、点云、TF 和规划轨迹正常。

App 首次打开时填写运行算法电脑的 IP，HTTP 保持 `8081`，WebSocket 保持 `8891`。手机/平板和
机器人应在同一可信局域网；当前接口为无鉴权明文连接，不能映射到公网。

完整 topic、TF 和性能说明见
[`SCANPLANNER_3D_APP.md`](../../docs/SCANPLANNER_3D_APP.md)。

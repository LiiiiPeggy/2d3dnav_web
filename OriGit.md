# cartographer_nav2_ws 变更记录（OriGit）

> 记录日期：2026-08-28
> 本工作空间相对各子仓库原始（origin）分支的本地改动汇总。
> 分支均为本地已跟踪状态：改动分「已修改」与「未跟踪（新增）」两类。

---

## 1. nav2_android（分支 `main`，Android 机器人控制应用）

基于 Google Now-in-Android 示例工程改造，新增了一个独立的机器人控制模块 `:nav2-app`，
并将手机端 WebView UI 与 ROS Bridge 打通。

### 已修改文件

| 文件 | 修改内容 |
| --- | --- |
| `README.md` | 顶部新增说明：本仓库同时包含 `:nav2-app` 机器人控制器，并链接 `NAV2_APP.md` 文档 |
| `gradle.properties` | 末尾新增 `org.gradle.tooling.parallel=true`（启用 Gradle 9.4+ 并行同步），并补齐文件结尾换行 |
| `settings.gradle.kts` | 模块列表新增 `include(":nav2-app")`，使新模块参与构建 |

### 未跟踪（新增）文件

| 文件/目录 | 说明 |
| --- | --- |
| `NAV2_APP.md` | `nav2-app` 机器人控制应用的架构说明：WebView 从 APK 内加载 `nav2_web` 前端，只通过 WebSocket 连机器人；UI 独立于 ROS 进程存活；断线自动重连 |
| `nav2-app/` | 新增 Android 机器人控制模块（Kotlin + Jetpack 组件）：`MainActivity.kt`、`Nav2ViewModel.kt`、`Nav2WebView.kt`、`RobotEndpoint.kt`、`ConnectionStore.kt`；内嵌前端 `assets/nav2_web/*`（index.html / app.js / style.css / scene3d.js）；含编译产物 APK |
| `android-env.sh` | 设置 `JAVA_HOME`（android-studio/jbr）与 `ANDROID_HOME`/`ANDROID_SDK_ROOT` 等 Android 构建环境变量 |
| `install-phone.sh` | 编译 `:nav2-app` 并 `adb install` 到手机（无设备时给出提示） |
| `sync-nav2-ui.sh` | 把 `nav2_web` 前端同步到 APK assets 的脚本 |
| `install/` | colcon 构建产物目录（含 `COLCON_IGNORE`，非源码） |
| `log/` | colcon 构建日志目录（`build_2026-08-18_11-24-16` 等，非源码） |

---

## 2. src/localization/rf2o_laser_odometry（分支 `ros2`，2D 激光里程计 RF2O）

为配合 Cartographer 定位，做了大量健壮性改动：核心目标是把发给 Cartographer 的
odometry 时间戳保证为「严格递增」，并对低端雷达变点数扫描做重采样。

### 已修改文件

| 文件 | 修改内容 |
| --- | --- |
| `include/rf2o_laser_odometry/CLaserOdometry2DNode.hpp` | 新增 `#include <cstdint>`；新增成员 `odom_has_been_published`、`last_published_odom_time_ns`，用于记录/校验已发布 odom 时间戳 |
| `package.xml` | 移除 `cmake_modules` 构建/运行依赖，新增 `nav_msgs` 依赖 |
| `src/CLaserOdometry2D.cpp` | 节点构造改为 `use_global_arguments(false)` 避免误解析外部参数；修复初始化 bug：`laser_oldpose_ = laser_pose_`（原来写成自赋值 `laser_oldpose_ = laser_oldpose_`）；`execution time`、`Laser odom`、`Robot-base odom` 三条 INFO 日志降为 DEBUG；Eigensolver 失败告警改为 5 秒节流输出 |
| `src/CLaserOdometry2DNode.cpp` | 默认初始姿态 `orientation.w` 由 0 改为 1（合法四元数）；新增 `new_scan_available=false` 初始化；激光回调增加「束数变化」检测——异常跳变（<0.8× 或 >1.2×）直接丢帧，正常小幅波动则按首帧角度栅格重采样到固定宽度；首次初始化时 `setLaserPoseFromTf()` 失败直接返回不初始化；`process()` 先把扫描标记为已消费，`odometryCalculation()` 返回失败（Eigensolver 未解出）时不再发布，避免重复时间戳；`publish()` 末尾新增时间戳校验，拒绝发布非递增时间戳并节流报错 |

### 未跟踪（新增）文件

无

# scanplanner_wab 变更记录（OriGit）

> 记录日期：2026-08-28
> 本工作空间相对各子仓库原始（origin）分支的本地改动汇总。
> 共 4 个 git 子仓库：`PCT_planner`、`nav2_android`、`Fast-LIO2-Localization`、`SCAN-Planner-Ros2`。

---

## 1. src/PCT_planner（分支 `feat/ros2pkg`，全局 3D 规划器）

从「静态场景演示」改造成「实时定位 + 点击目标规划」的 ROS2 全局规划节点，并摆脱了对
第三方库硬编码路径的依赖（改为本地 3rdparty 自动构建）。

### 已修改文件

| 文件 | 修改内容 |
| --- | --- |
| `.gitignore` | 新增忽略 `/3rdparty/` |
| `build_3rdparty.sh` | 完全重写：改为自动 `git clone` 指定 tag 的 gtsam-4.2.0 / osqp-v1.0.0 源码到 `3rdparty/src`，独立 `build`/`install` 目录；克隆失败自动重试 3 次；支持 `PCT_BUILD_JOBS`；用法收敛为 `{gtsam\|osqp\|all}` |
| `pct_planner/CMakeLists.txt` | 无构建类型时强制 `Release`；新增 `PCT_THIRDPARTY_ROOT` 指向 `../3rdparty/install`，并把 gtsam/osqp 加入 `CMAKE_PREFIX_PATH` |
| `pct_planner/launch/planner.launch.py` | 重写：去掉 `rsg_root`/`scene_name`，改为 `tomogram_path`、`map_frame`、`body_height` 参数，并对 `body_pose`/`goal`/`global_path` 做话题重映射 |
| `pct_planner/lib/src/common/smoothing/CMakeLists.txt` | 移除硬编码的 `/media/alex/.../osqp-1.0.0` 路径，改用 `find_package(osqp REQUIRED CONFIG)`，并删除写死的 include 目录 |
| `pct_planner/lib/src/common/smoothing/solver/osqp/osqp_interface.h` | include 由 `<osqp/osqp.h>` 改为 `<osqp.h>` |
| `pct_planner/lib/src/common/smoothing/solver/osqp/osqp_sparse_matrix.h` | 同上，include 改为 `<osqp.h>` |
| `pct_planner/package.xml` | 更新描述；新增 `rclpy`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`sensor_msgs_py`、`std_msgs`、`python3-numpy` 运行依赖 |
| `pct_planner/pct_planner/planner_node.py` | 完全重写：从「固定场景 + 起终点一键规划」改为 ROS2 实时节点——订阅 `body_pose`（Odometry）/`goal`（PoseStamped）/`waypoints`（Path），发布 `global_path`、`traversable_cloud`；新增参数校验（tomogram 文件存在性、body_height>0、航点数上限等） |
| `pct_planner/pct_planner/planner_wrapper.py` | 相对导入改为 `from .lib import ...`；`loadTomogram` 支持绝对路径（无 rsg_root 时）；新增 `selectLayer`（按地面高度选层）与 `traversablePoints`（导出可选点云）；`plan()` 支持 `start_ground_z/end_ground_z`、失败返回 `None`；`pos2idx` 修复中心点减法 |
| `pct_planner/pct_planner/utils/convertion.py` | 移除轨迹高度硬编码 `+0.5m` 偏移（改为 0，避免真实地图被整体抬升） |
| `pct_planner/pct_planner/utils/vis_ros.py` | `traj2ros` 支持自定义 `frame_id`/`stamp`；新增按相邻点航向计算 yaw 的位姿姿态 |
| `pct_planner/scripts/planner_node` | 改为可执行（100644→100755），入口 `sys.exit(main())` |
| `tomography/launch/tomography.launch.py` | 重写：去掉 `rsg_root`/`scene_name`，改为 `pcd_path`、`tomogram_path` 及一系列可遍历性参数（resolution、slope、step、clearance、inflation 等） |
| `tomography/package.xml` | 移除 `open3d`、`tomogram_rsc` 依赖，新增 `python3-scipy` |
| `tomography/setup.py` | 打包 rviz 配置（从 tomogram_rsc 引用），整理 `console_scripts` 格式 |
| `tomography/tomography/tomography_node.py` | 移除 open3d：PCD 改用自写 `pcd_io.load_xyz` 读取；新增 CUDA/CuPy 不可用时自动回退 CPU 后端（`compute_backend` = auto/cuda/cpu）；大量可遍历性参数改为可配置并做参数校验；`benchmark_repeats` 替代写死的 10 次 |

### 未跟踪（新增）文件

| 文件 | 说明 |
| --- | --- |
| `tomography/tomography/pcd_io.py` | 轻量 PCD 读取器（支持 ASCII / 未压缩二进制，读取 x/y/z 字段） |
| `tomography/tomography/tomogram_cpu.py` | CPU 版 Tomogram（NumPy/SciPy），无 CUDA/CuPy 环境的离线后备实现，输出格式与 CUDA 版一致 |

---

## 2. src/nav2_android（分支 `main`，Android 机器人控制应用）

与 cartographer_nav2_ws 中的 `nav2_android` 同源同步，改动内容一致，但未跟踪文件略少。

### 已修改文件

| 文件 | 修改内容 |
| --- | --- |
| `README.md` | 顶部新增 `:nav2-app` 机器人控制器说明，链接 `NAV2_APP.md` |
| `gradle.properties` | 末尾新增 `org.gradle.tooling.parallel=true`（Gradle 9.4+ 并行同步） |
| `settings.gradle.kts` | 模块列表新增 `include(":nav2-app")` |

### 未跟踪（新增）文件

| 文件/目录 | 说明 |
| --- | --- |
| `NAV2_APP.md` | `nav2-app` 机器人控制应用架构文档（WebView 内嵌前端 + WebSocket 连机器人，UI 独立于 ROS 存活） |
| `nav2-app/` | 新 Android 机器人控制模块：Kotlin 源码 `MainActivity.kt`、`Nav2ViewModel.kt`、`Nav2WebView.kt`、`RobotEndpoint.kt`、`ConnectionStore.kt`；内嵌 `assets/nav2_web/*` 前端；含 APK 编译产物 |
| `android-env.sh` | 设置 `JAVA_HOME` 与 `ANDROID_HOME`/`ANDROID_SDK_ROOT` 环境 |
| `install-phone.sh` | 编译并 `adb install` APK 到手机 |

> 注：此处没有 cartographer 版中额外的 `install/`、`log/` 目录和 `sync-nav2-ui.sh`。

---

## 3. src/Fast-LIO2-Localization（分支 `master`，FAST-LIO2 重定位）

把原 fast_lio 仓库复制为独立的 `fast_lio_localization` 包，专注于「历史地图重定位」，
并为 Mid-360S 真机加了「重力对齐水平 world」初始化和按需 QoS 调整。

### 已修改文件

| 文件 | 修改内容 |
| --- | --- |
| `FAST_LIO/CMakeLists.txt` | 项目名 `fast_lio` → `fast_lio_localization`；文件尾补换行 |
| `FAST_LIO/include/common_lib.h` | `fast_lio::msg::Pose6D` → `fast_lio_localization::msg::Pose6D` |
| `FAST_LIO/launch/mapping.launch.py` | 包名引用全部改为 `fast_lio_localization` |
| `FAST_LIO/launch/relocalization.launch.py` | 同上，包名改为 `fast_lio_localization` |
| `FAST_LIO/package.xml` | 包名改为 `fast_lio_localization`，描述改为「prior-PCD 重定位，与建图 fast_lio 隔离」 |
| `FAST_LIO/src/IMU_Processing.hpp` | 新增 `set_gravity_align_world()` 与 `gravity_align_world_` 标志：开启后 IMU 初始化把重力对齐到世界系 Z 轴，建立水平 `map`（`world_from_imu` 旋转）；并增加 mean_acc 有效性检查（非有限或 <1e-6 时跳过初始化） |
| `FAST_LIO/src/laserMapping.cpp` | 新增 `mapping.gravity_align_world` 参数；订阅 QoS 调整——LiDAR `keep_last(1)` 丢弃积压、IMU `keep_last(200)` 保留历史、输出点云改 `KeepLast(1).reliable`；SIGINT 处理补 `rclcpp::shutdown()`；prior PCD 加载失败由日志改为抛异常；`main()` 结束前补 `rclcpp::shutdown()` |
| `icp_relocalization/CMakeLists.txt` | 依赖 `octomap_ros`/`tf2_eigen` → `sensor_msgs`/`pcl_conversions` |
| `icp_relocalization/package.xml` | 依赖整理：新增 `sensor_msgs`、`pcl_conversions`、`livox_ros_driver2`、`tf2_ros`/`tf2_geometry_msgs` |
| `icp_relocalization/rviz/loam_livox.rviz` | 移除 octomap `OccupancyGrid` 显示；新增「PCT Traversable」点云、「PCT Global Path」路径显示和 `SetGoal`（2D Goal Pose）工具，话题指向 `/pct/traversable`、`/initial_path`、`/move_base_simple/goal` |
| `icp_relocalization/src/icp_node.cpp` | 大量重构：参数改名并调整默认值（`max_correspondence_distance` 1.0、`ransac_outlier_rejection_threshold` 0.5、`fitness_score_threshold` 0.30、`required_convergences` 5）；新增传感器外参 `sensor_translation/rpy_in_initial_frame`（初始猜测 = T_map_robot × T_robot_sensor）；结果发布到 `icp_sensor_result`；对齐成功后置 `localization_complete_` 停止重复 ICP；对坏帧不再污染初始猜测；`main()` 加异常捕获 |
| `icp_relocalization/src/transform_publisher.cpp` | 重写：订阅 `icp_sensor_result`，结合 `sensor_translation/rpy_in_odom` 外参计算 `T_map_odom = T_map_sensor × inverse(T_odom_sensor)`，发布校正后的 `icp_result`（transient_local）与 `map → odom` 静态 TF |

### 未跟踪（新增）文件

| 文件 | 说明 |
| --- | --- |
| `FAST_LIO/config/mid360_scanplanner_localization.yaml` | Mid-360 真机重定位专用配置：`locate_in_prior_map: true`、`gravity_align_world: true`、`send_odom_base_tf: false`（TF 由 fastlio_pose_adapter 发布） |

---

## 4. src/SCAN-Planner-Ros2（分支 `main`，局部轨迹规划/避障）

加入 FAST-LIO 真机数据链路、全局地图（PCT）与局部规划（SCAN 模式 3）的整合入口，
控制器增加健康检查联锁，可视化 frame 全部改为可配置。

### 已修改文件

| 文件 | 修改内容 |
| --- | --- |
| `README.md` | 新增大量「FAST-LIO 真机复现」章节：完整流程（建图→重定位→PCT→模式3）、TF 职责划分、Mid-360 标定参数说明、App 启动与安全预览流程 |
| `src/planner/plan_manage/CMakeLists.txt` | 新增 `fastlio_pose_adapter`、`global_frame_adapter` 两个可执行目标并安装；新增安装 `mode1_local.rviz` 及 `fastlio_input_monitor.py`、`pct_demo_pose_publisher.py` 脚本 |
| `src/planner/plan_manage/config/controllers.yaml` | `max_vyaw` 1.0→0.75（闭合/开环一致）；新增 `odom_timeout: 0.3`、`require_inputs_ready: false` |
| `src/planner/plan_manage/config/planner.yaml` | 新增 `fsm.reference_path_z_is_body: false`（地面高度参考路径；PCT 启动时覆盖为 true） |
| `src/planner/plan_manage/include/plan_manage/reference_path_utils.h` | `prepareReferenceWaypoints()` 新增 `path_z_is_body` 参数：为 true 时路径 z 已是身体高度不再加 `body_height` |
| `src/planner/plan_manage/include/plan_manage/scan_replan_fsm.h` | 新增成员 `reference_path_z_is_body_` |
| `src/planner/plan_manage/launch/run.launch.py` | 真机输入由 `/LIO/odom_vehicle`、`/LIO/clouds_lidar` 改为 FAST-LIO 默认 `/Odometry`、`/cloud_registered`，`world_frame=map`、`cloud_is_world=true`、`need_extrinsic=false`；新增 `body_pose_topic`/`sensor_pose_topic`/`cloud_topic`/`depth_topic`/`cmd_vel_topic`/`world_frame` 等覆盖参数；`publish_robot_state` 默认仿真 true / 真机 false；真机模式强制闭环控制器 |
| `src/planner/plan_manage/package.xml` | 新增 `livox_ros_driver2`、`nav2_web`、`icp_relocalization`、`pct_planner` 运行依赖 |
| `src/planner/plan_manage/src/closed_loop_controller.cpp` | 引入速度硬上限 `kMaxLinearSpeedLimit=0.75`（vx/vy）与 `kMaxVYawLimit=0.75`；新增 `odom_timeout` 里程计超时联锁（超时强制零速度）；新增 `require_inputs_ready`/`inputs_ready` 订阅——FAST-LIO 输入健康不通过时持续发布零 `cmd_vel` |
| `src/planner/plan_manage/src/scan_replan_fsm.cpp` | 模式 1 RViz 目标：忽略输入 z，改用当前机身高度（保持轨迹水平、兼容负地面 Z）；对 `initial_path` 与 RViz 目标做 `frame_id` 校验；`pathCallback` 传入 `reference_path_z_is_body_` |
| `src/planner/plan_manage/test/test_reference_path_utils.cpp` | 适配新参数签名，新增「身体高度路径保持不变」测试用例 |
| `src/planner/traj_utils/include/traj_utils/planning_visualization.h` | 新增 `frame_id_` 成员（含 `<string>` include） |
| `src/planner/traj_utils/src/planning_visualization.cpp` | 可视化 marker 的 frame_id 由硬编码 `world`/`map` 改为读取 `grid_map.frame_id` 参数 |
| `src/simulator/local_sensing/CMakeLists.txt` | `glm` 依赖移到 `USE_GPU` 分支内才查找（非 GPU 构建不再需要） |

### 未跟踪（新增）文件

| 文件 | 说明 |
| --- | --- |
| `launch/real_fastlio.launch.py` | 真机总入口：可选启动 Livox 驱动 / FAST-LIO / SCAN-Planner，默认 `start_livox_driver=false start_fastlio=false` |
| `launch/prior_map_navigation.launch.py` | 历史地图流程整合：PCD 重定位（ICP）→ PCT 全局规划 → SCAN 模式 3 |
| `launch/build_global_map.launch.py` | 用已验证的 FAST-LIO 建图流程生成并显式保存全局 PCD |
| `launch/build_pct_tomogram.launch.py` | 把保存的 FAST-LIO PCD 转成离线 PCT tomogram |
| `launch/pct_offline_demo.launch.py` | 基于 prior PCD + tomogram + 固定起始位姿的 PCT/App 安全演示 |
| `launch/planner_app.launch.py` | 手机 Planner App 的常驻启动入口（透传各模式 terminal） |
| `launch/mode1_local.rviz` | 模式 1 本地规划的 RViz 配置 |
| `scripts/fastlio_input_monitor.py` | FAST-LIO 输入健康检查节点（输出 `ready=true/false` 与 odom/cloud 频率） |
| `scripts/pct_demo_pose_publisher.py` | 为 PCT/App 离线演示发布固定机身位姿 |
| `src/fastlio_pose_adapter.cpp` | FAST-LIO 适配器：订阅 FAST-LIO IMU 位姿，按外参换算到机身体系，发布 `body_pose`、`/Odometry`、`trunk → livox_frame` 等 TF，并估计线速度 |
| `src/global_frame_adapter.cpp` | 全局帧适配器：把局部系 odom/点云转换到全局 `map` 系（`body_pose_global` 等） |

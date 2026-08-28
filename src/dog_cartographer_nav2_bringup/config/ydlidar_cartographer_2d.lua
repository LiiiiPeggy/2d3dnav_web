-- 四足机器人实物使用的 Cartographer 2D 建图配置。
-- 当前是“纯 2D 激光”模式：不使用 IMU、轮式/腿式里程计、Gazebo 或假 TF。
-- 建议每次只调整一组参数，并用同一段 rosbag 对比修改前后的建图效果。

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- 坐标系约定：map -> odom -> base_footprint -> laser。
  -- tracking_frame 必须能通过 TF 查到雷达坐标系。
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_footprint",
  odom_frame = "odom",

  -- true：Cartographer 根据激光匹配结果提供连续的 odom 相关 TF。
  -- 当前只有激光时保持 true。以后若底盘自己发布 odom -> base_footprint，
  -- 必须重新设计 TF 发布关系，不能让两个节点同时发布同一条 TF。
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,

  -- 使用位姿外推器提高 TF 发布连续性；当前明确不订阅外部 odometry。
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,

  -- YDLidar 发布一个 sensor_msgs/LaserScan，因此 num_laser_scans = 1。
  -- 若以后改成 PointCloud2，需相应修改 num_point_clouds，不能两边都随意开启。
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- TF 等待时间。出现偶发 extrapolation/lookup 错误可尝试增大到 0.3，
  -- 但根本问题通常仍是时间戳或 TF 发布不正确。
  lookup_transform_timeout_sec = 0.2,

  -- 发布周期只影响话题/TF 刷新速度，不直接提高扫描匹配精度。
  submap_publish_period_sec = 0.25,
  pose_publish_period_sec = 0.02,
  trajectory_publish_period_sec = 0.1,

  -- 激光采样比例：1.0 表示每帧都处理。算力不足时可试 0.8，
  -- 但纯激光无里程计模式不建议一开始就降低。
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 后台线程处理回环和图优化。机器人电脑较弱可改为 2；
-- 线程过多不一定更快，反而可能挤占 Nav2/MPPI 的控制线程。
MAP_BUILDER.num_background_threads = 4

-- 没有 IMU 时保持 false。
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 没有可靠 odom/IMU 时建议开启：在当前 submap 附近直接搜索激光最佳匹配。
-- 优点是对预测误差更稳健；缺点是增加 CPU 计算量。
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 每多少帧 LaserScan 合并后再匹配。1 表示每帧处理，更新最快；
-- 增大可降低处理频率，但机器人运动时更容易产生点云畸变。
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- 雷达有效范围，单位为米。应根据真实 /scan 数据质量设置。
-- 远距离噪点多时可把 max_range 和 missing_data_ray_length 一起降到 5~6 米。
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.

-- 点云体素滤波尺寸，单位为米。
-- 实物雷达只有约 6 Hz，使用 0.015 保留更多有效激光点。
-- 如果 CPU 持续过高，再回退到 0.02~0.025，不要直接增大到 0.04。
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.015

-- 实时相关扫描匹配的搜索窗口。
-- linear_search_window：平移搜索范围，angular_search_window：旋转搜索范围。
-- 快速转弯/机身晃动时跟丢，可尝试增大到 0.20~0.25 米、25~30 度；
-- 运行稳定但 CPU 高，可尝试减小到 0.10~0.12 米、10~15 度。
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- 对偏离运动预测的惩罚权重。权重越大，越不愿意离开预测位姿；
-- 纯激光且预测不可靠时不要盲目加大，否则可能压制正确的激光匹配结果。
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 运动滤波：位姿变化小于这些阈值时，不插入新的节点。
-- 6 Hz 雷达的帧间隔约为 0.167 s；时间阈值设为 0.15 s，
-- 并降低平移/旋转阈值，避免缓慢行走时再丢掉已经很低频的扫描。
-- 这会增加一些 CPU 占用，但不会伪造额外的雷达帧。
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.15
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

-- 每个 submap 插入的激光数据数量。当前 45 适合一般室内环境。
-- submap 需要包含足够多且有辨识度的墙角/结构，并与相邻 submap 保持重叠。
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45

-- 地图分辨率，单位为米/格。0.05 表示每格 5 cm。
-- 重要：生成 pbstream 后，纯定位必须使用相同分辨率；若修改需重新建图。
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- 是否插入激光穿过区域的空闲空间。建黑白占据栅格地图时应保持 true。
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true

-- 命中/未命中的概率更新强度。
-- hit 越大障碍越容易变黑；miss 越小，射线经过区域越容易被清为空闲。
-- 不建议仅凭一次显示效果大幅调整，动态物体和错误 TF 也会造成地图异常。
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.85
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

-- 每插入多少个节点执行一次全局图优化。减小会更频繁、更吃 CPU；
-- 增大会降低计算频率，但回环修正反映到地图上的速度更慢。
POSE_GRAPH.optimize_every_n_nodes = 30

-- 回环约束候选采样比例。CPU 高可降到 0.15~0.20；
-- 回环经常找不到则保持 0.30，并优先确认环境中是否有可辨识结构。
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- 普通约束搜索的最大距离。小型室内可降到 8~10 米以减少无效搜索。
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

-- 回环匹配最低得分。出现错误回环可逐步提高，例如 0.65/0.70；
-- 设置过高会导致正确回环也无法建立。
POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

-- 快速相关匹配器的全局搜索窗口。窗口越大，重定位/回环搜索范围越大，CPU 越高。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)

-- 鲁棒损失尺度，用于减弱异常约束对整张图的破坏；通常保持默认量级。
POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options

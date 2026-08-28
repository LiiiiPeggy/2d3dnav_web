-- 在已经保存的 Cartographer .pbstream 图上进行纯定位。
-- 本文件不是一套完全独立的参数：它先继承建图配置，再覆盖纯定位专用参数。
-- 因此坐标系、雷达有效范围、体素滤波、扫描匹配参数都来自下面 include 的文件。

include "ydlidar_cartographer_2d.lua"

-- 纯定位模式只保留少量当前 submap，避免像建图模式一样持续扩张占用内存。
-- 调大可保留更多局部历史，但会增加内存和计算量；机器人端通常保持 3。
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 纯定位的后台线程数。弱算力机器人保持 2；不要盲目设成 CPU 总核心数，
-- 还需要给雷达驱动、Web、Nav2 和 MPPI 留出计算资源。
MAP_BUILDER.num_background_threads = 2

-- 纯定位需要比建图更频繁地优化当前轨迹，因此这里覆盖为每 20 个节点优化一次。
-- 减小：定位修正更及时但 CPU 更高；增大：计算减少但修正延迟增加。
POSE_GRAPH.optimize_every_n_nodes = 20

-- 连续多少秒没有建立约束后，开始进行更大范围的全局约束搜索。
-- 初始位置误差较大、长时间无法匹配时可适当减小；过小会增加误匹配和 CPU。
POSE_GRAPH.global_constraint_search_after_n_seconds = 5.

-- 已冻结地图轨迹参与全局约束搜索的采样比例。
-- 数值很小是纯定位的正常设置，用于控制大型 pbstream 的搜索开销。
POSE_GRAPH.global_sampling_ratio = 0.003

-- 当前定位轨迹建立约束的采样比例。
-- 重定位偏慢但 CPU 有余量时可试 0.15；CPU 高时可降至 0.05。
-- 不要一次调整太大，否则难以判断定位改善来自哪一项参数。
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1

-- Ceres 图优化线程数。机器人端设置为 1 可以减少瞬时 CPU 峰值。
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 1

-- 重要：不要在本文件里覆盖地图 resolution、雷达方向或 tracking_frame。
-- 纯定位必须和生成该 pbstream 时使用的建图配置保持一致。
-- 如果修改了分辨率或关键激光预处理参数，应重新建图并生成新的 pbstream。

return options

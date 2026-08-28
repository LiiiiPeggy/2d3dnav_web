-- Cartographer 2D + RF2O 激光里程计先验配置。
--
-- 本文件继承纯激光建图配置，只打开 odometry 输入。RF2O 与 Cartographer
-- 使用的是同一个 /scan，因此 RF2O 不是独立真值，也仍然可能在无特征长走廊中
-- 退化；它的作用是提供连续的 scan-to-scan 前进预测，帮助 Cartographer 在
-- scan-to-submap 匹配前得到更合理的初始位姿。

include "ydlidar_cartographer_2d.lua"

-- 订阅 Cartographer 的 odom 输入话题。launch 会把 odom 重映射到
-- /odom_rf2o。RF2O 在此模式下 publish_tf=false，不能与 Cartographer 抢 TF。
options.use_odometry = true
options.odometry_sampling_ratio = 1.

-- RF2O 与 Cartographer 来自同一雷达。这里让 RF2O参与实时位姿外推，
-- 但不在全局位姿图中再次以高权重约束整条轨迹，避免同一份激光信息被
-- 重复计算并压制 Cartographer 的 submap 回环修正。
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.

return options

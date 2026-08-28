-- Cartographer 2D 激光 + IMU 建图配置。
--
-- 纯激光参数仍保留在 ydlidar_cartographer_2d.lua，避免本文件的
-- IMU 设置影响 RF2O 或纯激光入口。只有显式选择本文件时才使用 IMU。

include "ydlidar_cartographer_2d.lua"

-- Cartographer 要求 tracking_frame 与 IMU 共位；话题中的 frame_id
-- 为 YIS320，因此不能继续使用 base_footprint 作为 tracking_frame。
options.tracking_frame = "YIS320"
options.publish_frame_projected_to_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = true

return options

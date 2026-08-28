-- 冻结 pbstream 上的激光 + IMU 纯定位配置。
-- 纯激光定位仍使用 ydlidar_cartographer_2d_localization.lua。

include "ydlidar_cartographer_2d_localization.lua"

options.tracking_frame = "YIS320"
options.publish_frame_projected_to_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = true

return options

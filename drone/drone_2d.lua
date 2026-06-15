include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,           
  publish_frame_projected_to_2d = true,
  use_odometry = false,                
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.1,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  
  -- CÁC THÔNG SỐ BẮT BUỘC PHẢI THÊM CHO ROS 2 HUMBLE
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- BẬT IMU ĐỂ BÙ TRỪ GÓC NGHIÊNG & CHỐNG TRÔI
TRAJECTORY_BUILDER_2D.use_imu_data = true 
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 6.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5
-- Giảm độ phân giải để bản đồ mịn hơn
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- ÉP THUẬT TOÁN SO KHỚP ẢNH CHẠY MAX CÔNG SUẤT
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.0

-- Kích hoạt bộ tối ưu hóa Ceres (Ghì chặt các điểm ảnh vào nhau)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

-- Cấu hình độ nhạy của bộ lọc chuyển động (Ép vẽ map khi đi được 5cm)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.1

-- IMU tin cậy hơn Lidar ở môi trường trống
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

-- Tăng sức mạnh của tia Laser khi quét qua khoảng trống
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.45

return options

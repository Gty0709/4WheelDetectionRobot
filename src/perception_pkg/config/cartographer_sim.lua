-- cartographer_sim.lua
-- 仿真模式：tracking_frame=base_link，不使用IMU
-- 对齐模块A：/scan(laser_link) /odom(odom→base_footprint)

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder                       = MAP_BUILDER,
  trajectory_builder                = TRAJECTORY_BUILDER,
  map_frame                         = "map",
  tracking_frame                    = "base_footprint",  -- 仿真无IMU，直接追踪Gazebo原生帧
  published_frame                   = "odom",            -- Cartographer只发布map→odom
  odom_frame                        = "odom",
  provide_odom_frame                = false,             -- Gazebo已发布odom→base_footprint，不重复发布避免TF冲突
  publish_frame_projected_to_2d     = true,
  use_pose_extrapolator             = true,
  use_odometry                      = false,             -- WSL2里程计不准，关掉让Cartographer纯雷达匹配
  use_nav_sat                       = false,
  use_landmarks                     = false,
  num_laser_scans                   = 1,                 -- /scan 单雷达
  num_multi_echo_laser_scans        = 0,
  num_subdivisions_per_laser_scan   = 1,
  num_point_clouds                  = 0,
  lookup_transform_timeout_sec      = 0.2,
  submap_publish_period_sec         = 0.3,
  pose_publish_period_sec           = 5e-3,
  trajectory_publish_period_sec     = 30e-3,
  rangefinder_sampling_ratio        = 1.,
  odometry_sampling_ratio           = 1.,
  fixed_frame_pose_sampling_ratio   = 1.,
  imu_sampling_ratio                = 1.,
  landmarks_sampling_ratio          = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 激光雷达参数（对齐 mickrobot_ugv.urdf.xacro）
TRAJECTORY_BUILDER_2D.use_imu_data   = false
TRAJECTORY_BUILDER_2D.min_range      = 0.12
TRAJECTORY_BUILDER_2D.max_range      = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0
-- WSL2/Gazebo 实际 /scan 约 1.5Hz（非标称10Hz），每帧都处理
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 无里程计时加大搜索窗口，让帧间匹配更准
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- 每个子图只需 10 帧（1.5Hz × ~7s = 1个子图），适配低频雷达
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 10
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

return options

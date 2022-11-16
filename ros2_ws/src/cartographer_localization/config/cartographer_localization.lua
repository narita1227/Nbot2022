include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

TRAJECTORY_BUILDER_2D.use_imu_data = false


--added

TRAJECTORY_BUILDER_2D.min_range = 0.
TRAJECTORY_BUILDER_2D.max_range = 12.                     -- 自分の Lidar の性能に合わせる。 LD06 Lidar なら 12m + α
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5      --'max_range' を超える点は、この長さで空白部として挿入されます。

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true    -- 相関スキャンマッチャーというのを有効にします。
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1                 -- 1 (歪み対策。増やすとレスポンス低下したので 1 or 2?)

-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 50.
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100

-- TRAJECTORY_BUILDER_2D.use_imu_data = true  （既に前で false に設定済）
-- TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.

-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
-- TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90             -- 後で120 にする

-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"    -- or "TSDF"

-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
-- TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
-- TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
-- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 15.
-- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 60.

-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.truncation_distance = 0.3
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.maximum_weight = 10.
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_free_space = false
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.num_normal_samples = 4
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.sample_radius = 0.5
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.project_sdf_distance_to_scan_normal = true
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_range_exponent = 0
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5

-- POSE_GRAPH.optimize_every_n_nodes = 90

-- POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 5.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)

-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 8
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth = 3

-- POSE_GRAPH.constraint_builder.min_score = 0.55

-- POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
-- POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
-- POSE_GRAPH.matcher_translation_weight = 5e2
-- POSE_GRAPH.matcher_rotation_weight = 1.6e3
-- POSE_GRAPH.optimization_problem.*_weight
-- POSE_GRAPH.optimization_problem.ceres_solver_options

-- POSE_GRAPH.optimization_problem.log_solver_summary = false
-- POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = true

-- POSE_GRAPH.optimization_problem.huber_scale = 1e1

-- POSE_GRAPH.max_num_final_iterations = 200


-- POSE_GRAPH.optimize_every_n_nodes = 0    -- 90 -> 0  Global SLAM を OFF にする

-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2  -- 10 -> 200
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4e2     -- 40 -> 400 

-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025                    -- 増やす デフォルト： 0.025
-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05    -- 増やす デフォルト： 0.05
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200   -- 小さく デフォルト： 200
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.        -- 小さく デフォルト： 50.
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5       -- 増やす デフォルト： 0.5
-- TRAJECTORY_BUILDER_2D.max_range = 30.                              -- 小さく デフォルト： 30.
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20                  -- 小さく デフォルト： 20


-- POSE_GRAPH.optimize_every_n_nodes = 90                             -- 小さく デフォルト： 90
-- MAP_BUILDER.num_background_threads = 4                             -- 増やす(CPUコア数)デフォルト: 4
POSE_GRAPH.global_sampling_ratio = 0.0 --0.003                           -- 小さく デフォルト： 0.003
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.3                 -- 小さく デフォルト： 0.3
-- POSE_GRAPH.constraint_builder.min_score = 0.55                     -- 増やす デフォルト:  0.55

-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200   -- 小さく デフォルト： 200
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.        -- 小さく デフォルト： 50.
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5       -- 増やす デフォルト： 0.5
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025                    -- 増やす デフォルト： 0.025
-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05    -- 増やす デフォルト： 0.05
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20                  -- 小さく デフォルト： 20

-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 5. -- 小さく デフォルト： 5.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1. -- 小さく デフォルト： 1.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.) -- 小さく デフォルト： math.rad(30.)
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.) -- 小さく デフォルト： math.rad(20.)
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 10.          -- 増やす デフォルト:  10.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10 -- 小さく デフォルト： 10
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50 -- 小さく デフォルト： 50
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20 -- 小さく デフォルト： 20

-- from Mr.PINTO03091 's settings. Thanks!

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15 -- デフォルト: 0.1 -> 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.) -- デフォルト: math.rad(20.) -> math.rad(35.)
POSE_GRAPH.constraint_builder.min_score = 0.65                     -- デフォルト: 0.55 -> 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.0 --0.7  -- デフォルト: 0.6 -> 0.7
POSE_GRAPH.optimization_problem.huber_scale = 1e3                  -- デフォルト: 1e1 -> 1e3
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10 -- デフォルト: 1.  -> 10
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120                 -- デフォルト: 90  -> 120
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1      -- デフォルト: 0.2 -> 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2) -- デフォルト: math.rad(1.) -> math.rad(0.2)

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 2 --20

return options

-- @author: Marco Giberna
-- @email: marco.giberna@studenti.units.it
-- @email: marcogiberna@gmail.com

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map", --The ROS frame ID to use for publishing submaps
  tracking_frame = "Archimede_imu_link", -- The ROS frame ID of the frame that is tracked by the SLAM algorithm
  published_frame = "odom", -- The ROS frame ID to use as the child frame for publishing poses. For example “odom” if an “odom” frame is supplied by a different part of the system
  odom_frame = "odom", -- not used if provide_odom_frame = false
  provide_odom_frame = false, -- set to false if odom is already provided by another part of the system, otherwise transformation between odom_frame and published_frame will be provided
  publish_frame_projected_to_2d = false,
  --use_pose_extrapolator = true, -- the used version of cartographer doesn't have this flag
  use_odometry = true, -- subscribes to nav_msgs/Odometry on the topic “odom”, includes its information on SLAM
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  -- publishing rates on publisher topics 
  lookup_transform_timeout_sec = 0.2, --Timeout in seconds to use for looking up transforms (increase it if it gives lookup warnings, for examples due to different publishing frequencies within the tf_tree)
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  --consider to modify the following sampling ratios
  --check publishing rates with rostopic hz /topic
  rangefinder_sampling_ratio = 1., 
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 2 --if == 1 laser scan can't be exploit
-- how many step to update trajectory, and therefore to update map->odom
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 200
--TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = true
--TRAJECTORY_BUILDER_3D.imu_based.imu_acceleration_weight = 1.
--TRAJECTORY_BUILDER_3D.imu_based.imu_rotation_weight = 1.
--TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15 --it increases number of voxel, but it slows down the process

POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 480 --consider to set it to zero, it seems it gives a lot of problem at each optimization
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03 --0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.82
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.86
POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.optimization_problem.log_solver_summary = true
--POSE_GRAPH.ceres_scan_matcher_3d.rotation_weight = 4e3




POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e2
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e2

return options
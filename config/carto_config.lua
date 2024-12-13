-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.3,
  submap_publish_period_sec = 0.4,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
--前端参数
-- 是否使用imu数据
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 雷达数据的最远最近滤波, 保存中间值
TRAJECTORY_BUILDER_2D.min_range = 0.01
TRAJECTORY_BUILDER_2D.max_range = 2.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- 为了防止子图里插入太多数据, 在插入子图之前之前对数据进行过滤
-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
--构建子地图时使用的激光雷达扫描数据的数量，建议一个子图里插入雷达数据的个数的一半
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80
-- 几帧有效的点云数据进行一次扫描匹配，扫描匹配用于估计机器人位姿，匹配后能够及时纠正误差
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- 是否使用 real_time_correlative_scan_matcher 为ceres提供先验信息
-- 计算复杂度高 , 但是很鲁棒 , 在odom或者imu不准时依然能达到很好的效果
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
--用于在Ceres扫描匹配中调整平移和旋转的权重，增加这些权重可以使算法更倾向于相信来自IMU和里程计的先验信息，而不是完全依赖于扫描匹配的结果
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10000
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40000
--这两个参数用于实时相关性扫描匹配器，调整它们可以影响匹配过程中对平移和旋转变化的惩罚。如果地图抖动，可以尝试调整这些权重，以减少对小的运动变化的敏感性。
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 100
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 100




--后端参数
--这个参数控制全局优化的频率。如果设置得过高，可能会导致优化不及时，从而在地图上看到抖动。可以尝试减小这个值，使优化更频繁。
POSE_GRAPH.optimize_every_n_nodes = 160
--在姿势估计期间控制对旋转里程计数据的信任。
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight=1e10 --本地SLAM平移权重
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight=1e10 --本地SLAM旋转权重
POSE_GRAPH.optimization_problem.odometry_translation_weight=1e10 --里程计平移权重
POSE_GRAPH.optimization_problem.odometry_rotation_weight=1e10 --里程计旋转权重


POSE_GRAPH.constraint_builder.min_score = 0.5
POSE_GRAPH.constraint_builder.min_score = 0.75
POSE_GRAPH.optimization_problem.huber_scale = 5
--这两个参数用于确定何时接受新的约束。调整这些阈值可以影响全局优化过程中约束的添加，从而影响地图的稳定性。
-- POSE_GRAPH.constraint_builder.min_score = 0.65
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

 


-- POSE_GRAPH.optimize_every_n_nodes = 0

return options

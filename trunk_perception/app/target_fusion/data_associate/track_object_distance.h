/**
 * @file track_object_distance.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 计算跟踪目标与检测目标之间的距离
 * @version 0.1
 * @date 2024-10-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/target_fusion/data_fusion/tracker.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/fused_object.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

class TrackObjectDistance {
 public:
  TrackObjectDistance() = default;
  ~TrackObjectDistance() = default;

  uint32_t Init(const YAML::Node& config);

  /**
   * @brief 计算跟踪目标与激光雷达检测目标之间的距离
   *
   * @param tracker_ptr [IN] 跟踪目标
   * @param lidar_object [IN] 激光雷达检测目标
   * @return float [OUT] 距离
   */
  float Compute(const TrackerPtr& tracker_ptr, const LidarMeasureFrame::ConstPtr& lidar_object);

  /**
   * @brief 计算跟踪目标与前向毫米波雷达检测目标之间的距离
   *
   * @param tracker_ptr [IN] 跟踪目标
   * @param front_radar_object [IN] 前向毫米波雷达检测目标
   * @return float [OUT] 距离
   */
  float Compute(const TrackerPtr& tracker_ptr, const ars430::RadarMeasureFrame::ConstPtr& front_radar_object);

  /**
   * @brief 计算跟踪目标与前向视觉检测目标之间的距离   
   *
   * @param tracker_ptr [IN] 跟踪目标
   * @param front_vision_object [IN] 前向毫米波雷达检测目标
   * @return float [OUT] 距离
	 * @author zzg 2024-12-13
   */
  float Compute(const TrackerPtr& tracker_ptr, const VisionMeasureFrame::ConstPtr& front_vision_object);

 private:
  template <typename T>
  float Compute2DEuclideanDistance(const Eigen::Matrix<T, 3, 1>& des, const Eigen::Matrix<T, 3, 1>& src);

  template <typename T>
  float Compute2DRelEuclideanDistance(const Eigen::Matrix<T, 3, 1>& des, const Eigen::Matrix<T, 3, 1>& src);

  /**
   * @brief 计算跟踪目标与检测目标之间的正交距离和纵向距离
   *
   * @tparam T 数据类型
   * @param des [IN] 检测目标
   * @param src [IN] 跟踪目标
   * @param yaw [IN] 跟踪目标的航向角
   * @return Eigen::Vector2f [OUT] 距离 (orthogonal, longitudinal)
   */
  template <typename T>
  Eigen::Vector2f ComputeOrthogonalDistance(const Eigen::Matrix<T, 3, 1>& des, const Eigen::Matrix<T, 3, 1>& src,
                                            float yaw);

  bool use_position_ = false;
  float position_weight_ = 0.0;
  std::string position_type_ = "Orthogonal";
  const std::vector<std::string> position_type_list_ = {"Orthogonal", "Euclidean"};
  std::string position_point_ = "RearMiddle";
  const std::vector<std::string> position_point_list_ = {"Center", "RearMiddle", "Mix_CR"};

  bool use_velocity_ = false;
  float velocity_weight_ = 0.0;

  bool use_velocity_filter_ = false;
  float velocity_filter_abs_thresh_ = 5.0;
  float velocity_filter_rel_thresh_ = 30.0;

  bool use_position_filter_ = false;
  float position_filter_orthogonal_thresh_ = 3.0;
  float position_filter_longitudinal_thresh_ = 10.0;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
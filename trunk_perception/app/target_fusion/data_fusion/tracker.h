/**
 * @file tracker.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 跟踪序列
 * @version 0.1
 * @date 2024-10-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <jian/spdlog/fmt/bundled/chrono.h>
#include <yaml-cpp/yaml.h>
#include <cstdint>
#include <memory>

#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_base.h"
#include "trunk_perception/app/target_fusion/data_fusion/kalman_motion_fusion.h"
#include "trunk_perception/app/target_fusion/data_fusion/shape_fusion_base.h"
#include "trunk_perception/app/target_fusion/data_fusion/yaw_fusion_base.h"
#include "trunk_perception/app/target_fusion/data_fusion/type_fusion_base.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/fused_object.h"
#include "trunk_perception/common/types/object.h"
#include "trunk_perception/common/types/odometry.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

class Tracker {
 public:
  /**
   * @brief Construct a new Tracker object
   *
   * @param object [IN] 被跟踪的目标
   * @param motion_kf_config [IN] 运动Kalman Filter配置
   */
  explicit Tracker(const FusedObject::Ptr& object_ptr, const MotionFusionConfig& motion_kf_config,
                   const ShapeFusionConfig::ConstPtr& shape_fusion_config,
                   const ExistenceFusionConfig::ConstPtr& existence_fusion_config,
                   const TypeFusionConfig::ConstPtr& type_fusion_config,
                   const LidarMeasureFrame::ConstPtr& lidar_measure_ptr = nullptr,
                   const ars430::RadarMeasureFrame::ConstPtr& front_radar_measure_ptr = nullptr);
  ~Tracker();

  void Predict(const double& t);

  uint32_t Update(const std::vector<SensorMeasureFrame::ConstPtr>& measures_list);

  void Update(const LidarMeasureFrame::ConstPtr& lidar_measure_ptr);

  void Update(const ars430::RadarMeasureFrame::ConstPtr& front_radar_measure_ptr);

  void Update(const VisionMeasureFrame::ConstPtr& front_vision_measure_ptr);

  LidarMeasureFrame::ConstPtr GetLidarObject() const;

  ars430::RadarMeasureFrame::ConstPtr GetFrontRadarObject() const;

  VisionMeasureFrame::ConstPtr GetFrontVisionObject() const;

  FusedObject::ConstPtr GetFusedObject() const;

  void SetFusedObjectOdometry(Odometry::Ptr odo_ptr);

  Odometry::ConstPtr GetFusedObjectOdometry() const;

  int GetTrackID() const;

 private:
  Eigen::VectorXd GetStateFromFusedObject();

  void UpdateObjectPoseVelocity();

  void UpdateObjectShape();

  void UpdateObjectType();

  Eigen::VectorXd GetMeasurementFromLidar(const LidarMeasureFrame::ConstPtr& lidar_measure_ptr);

  Eigen::VectorXd GetMeasurementFromFrontRadar(const ars430::RadarMeasureFrame::ConstPtr& front_radar_measure_ptr);

  Eigen::VectorXd GetMeasurementFromFrontVision(const VisionMeasureFrame::ConstPtr& front_vision_measure_ptr);

  // 配置
  MotionFusionConfig motion_kf_config_;
  ShapeFusionConfig::ConstPtr shape_fusion_config_ = nullptr;
  ExistenceFusionConfig::ConstPtr existence_fusion_config_ = nullptr;
  TypeFusionConfig::ConstPtr type_fusion_config_ = nullptr;
  // 数据
  FusedObject::Ptr object_ptr_ = nullptr;  ///< 被跟踪的目标

  LidarMeasureFrame::ConstPtr object_lidar_ptr_ = nullptr;                ///< 最新的激光雷达观测
  ars430::RadarMeasureFrame::ConstPtr object_front_radar_ptr_ = nullptr;  ///< 最新的前向毫米波雷达观测
  VisionMeasureFrame::ConstPtr object_front_vision_ptr_ = nullptr;        ///< 最新的前向视觉观测

  std::shared_ptr<KalmanMotionFusion> motion_fusion_ = nullptr;
  std::shared_ptr<ShapeFusionBase> shape_fusion_ = nullptr;
  std::shared_ptr<ExistenceFusionBase> existence_fusion_ = nullptr;
  std::shared_ptr<TypeFusionBase> type_fusion_ = nullptr;
};

typedef std::shared_ptr<Tracker> TrackerPtr;

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
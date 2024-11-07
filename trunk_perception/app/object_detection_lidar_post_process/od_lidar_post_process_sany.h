/**
 * @file od_lidar_post_process_sany.h
 * @author Fan Dongsheng
 * @brief 激光检测后处理模块
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/algo/track/tracker_pipeline/tracker_pipeline_interface.h"
#include "trunk_perception/app/object_detection_lidar_post_process/algo/cluster_detection.h"
#include "trunk_perception/app/object_detection_lidar_post_process/od_lidar_post_process_base.h"
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/types/odometry.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief sany项目激光检测后处理
 *
 */
class OdLidarPostProcessSany : public OdLidarPostProcessBase {
 public:
  OdLidarPostProcessSany() = default;
  ~OdLidarPostProcessSany() = default;

  /**
   * @brief object detection lidar post process init param
   *
   * @param config yaml node
   * @return std::uint32_t
   */
  std::uint32_t Init(const YAML::Node& config) override;

  /**
   * @brief object detection lidar post process run pipeline
   *
   * @param ts current timestamp
   * @return std::uint32_t
   */
  std::uint32_t Run(const double& ts = 0.0) override;

  /**
   * @brief Get the Data object
   *
   * @param key string key
   * @return std::any
   */
  std::any GetData(const std::string& key) override;

 private:
  bool track_switch_ = false;                                    ///< 跟踪器开关
  bool cluster_detection_switch_ = false;                        ///< 聚类检测开关
  std::unique_ptr<TrackerPipelineInterface> tracker_ = nullptr;  ///< 跟踪器
  std::unique_ptr<ClusterDetection> cluster_detector_ = nullptr;  ///< 聚类检测器
  common::Odometry::ConstPtr last_odometry_ptr_ = nullptr;       ///< 上一帧里程计信息
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
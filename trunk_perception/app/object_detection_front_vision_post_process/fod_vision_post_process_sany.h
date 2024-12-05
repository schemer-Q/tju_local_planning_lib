/**
 * @file fod_vision_post_process_sany.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/algo/track/tracker_pipeline/tracker_pipeline_interface.h"
#include "trunk_perception/app/object_detection_front_vision_post_process/fod_vision_post_process_base.h"
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/types/odometry.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

class FodVisionPostProcessSany : public FodVisionPostProcessBase {
 public:
  FodVisionPostProcessSany() = default;
  ~FodVisionPostProcessSany() = default;

  /**
   * @brief object detection front vision post process init param
   *
   * @param config yaml node
   * @return std::uint32_t
   */
  std::uint32_t Init(const YAML::Node& config) override;

  /**
   * @brief object detection front vision post process run pipeline
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
  std::unique_ptr<TrackerPipelineInterface> tracker_ = nullptr;  ///< 跟踪器
  common::Odometry::ConstPtr last_odometry_ptr_ = nullptr;       ///< 上一帧里程计信息
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
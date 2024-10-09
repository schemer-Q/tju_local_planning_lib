/**
 * @file od_lidar_post_process_sany.cpp
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/app/object_detection_lidar_post_process/od_lidar_post_process_sany.h"
#include "trunk_perception/algo/track/tracker_pipeline/tracker_pipeline_manager.h"
#include "trunk_perception/app/object_detection_lidar_post_process/utils.hpp"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::uint32_t OdLidarPostProcessSany::Init(const YAML::Node& config) {
  try {
    // track param
    track_switch_ = config["Tracker"].IsDefined();
    if (track_switch_) {
      track_switch_ = config["Tracker"]["Switch"].as<bool>();
      if (track_switch_) {
        const auto& tracker_config = config["Tracker"];
        const std::string tracker_pipeline_method = tracker_config["TrackerPipelineMethod"].as<std::string>();
        tracker_ = TrackerPipelineManager::Create(tracker_pipeline_method);
        if (tracker_ == nullptr) {
          TFATAL << "[ObjectDetector] Init failed! tracker_ is nullptr!";
          return false;
        }
        auto res = tracker_->Init(tracker_config["TrackerPipelineParams"]);
        if (res) {
          TFATAL << "[ObjectDetector] Init failed! tracker_ init failed! " << res;
          return false;
        }
      }
    }
  } catch (const std::exception& e) {
    TFATAL << "OdLidarPostProcessSany::Init failed: " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }

  TINFO << "OdLidarPostProcessSany::Init success";
  return ErrorCode::SUCCESS;
}

std::uint32_t OdLidarPostProcessSany::Run(const double& ts) {
  auto frame = GET_OD_LIDAR_FRAME();

  std::shared_ptr<OdometryData> odometry_data_ptr = nullptr;
  uint32_t ret = GET_ODOMETRY_DATA_BY_TIME(frame->timestamp, odometry_data_ptr);
  if (ret != ErrorCode::SUCCESS) {
    TERROR << "OdLidarPostProcessSany::Run failed, get GET_ODOMETRY_DATA_BY_TIME failed";
    return ret;
  }

  // track
  if (track_switch_) {
    tracker_->Track(frame);
  } else {
    frame->tracked_objects.clear();
    frame->tracked_objects.assign(frame->detected_objects.begin(), frame->detected_objects.end());
  }

  return ErrorCode::SUCCESS;
}

std::any OdLidarPostProcessSany::GetData(const std::string& key) { return nullptr; }

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
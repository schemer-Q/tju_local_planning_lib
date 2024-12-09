/**
 * @file fod_vision_post_process_sany.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief 
 * @version 0.1
 * @date 2024-12-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "trunk_perception/app/object_detection_front_vision_post_process/fod_vision_post_process_sany.h"
#include "trunk_perception/algo/track/tracker_pipeline/tracker_pipeline_manager.h"
#include "trunk_perception/app/object_detection_front_vision_post_process/utils.hpp"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::uint32_t FodVisionPostProcessSany::Init(const YAML::Node& config) {
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
          TFATAL << "FodVisionPostProcessSany::Init create tracker failed!";
          return ErrorCode::UNINITIALIZED;
        }
        auto res = tracker_->Init(tracker_config["TrackerPipelineParams"]);
        if (res) {
          TFATAL << "FodVisionPostProcessSany::Init tracker init failed! " << res;
          return ErrorCode::UNINITIALIZED;
        }
        TINFO << "FodVisionPostProcessSany::Init turn on tracker pipeline method: " << tracker_pipeline_method;
      }
    }
  } catch (const std::exception& e) {
    TFATAL << "FodVisionPostProcessSany::Init failed: " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }

  TINFO << "FodVisionPostProcessSany::Init success";
  return ErrorCode::SUCCESS;
}

std::uint32_t FodVisionPostProcessSany::Run(const double& ts) {
  auto frame = GET_FOD_VISION_FRAME();
  if (frame == nullptr) {
    TWARNING << "FodVisionPostProcessSany::Run frame is nullptr";
    return ErrorCode::FOD_VISION_FRAME_NOT_FOUND;
  }

  std::shared_ptr<OdometryData> odometry_data_ptr = nullptr;
  uint32_t ret = GET_ODOMETRY_DATA_BY_TIME(frame->timestamp, odometry_data_ptr);
  if (ret != ErrorCode::SUCCESS) {
    TERROR << "FodVisionPostProcessSany::Run failed, get GET_ODOMETRY_DATA_BY_TIME failed";
    return ret;
  }
  if (odometry_data_ptr == nullptr) {
    TERROR << "FodVisionPostProcessSany::Run failed, get odometry_data is nullptr";
    return ErrorCode::ODOMETRY_DATA_INVALID;
  }

  // 计算前后帧变换TF
  if (last_odometry_ptr_) {
    frame->tf = calculateTransformMatrix(last_odometry_ptr_, odometry_data_ptr->data);
  } else {
    frame->tf = Eigen::Isometry3f::Identity();
  }
  last_odometry_ptr_ = odometry_data_ptr->data;

  // track
  if (track_switch_) {
    tracker_->Track(frame->detected_objects, frame->tf, frame->timestamp, frame->tracked_objects);
  } else {
    frame->tracked_objects.clear();
    frame->tracked_objects.assign(frame->detected_objects.begin(), frame->detected_objects.end());
  }

  return ErrorCode::SUCCESS;
}

std::any FodVisionPostProcessSany::GetData(const std::string& key) { return nullptr; }

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
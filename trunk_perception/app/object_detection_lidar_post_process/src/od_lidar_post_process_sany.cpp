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
          TFATAL << "OdLidarPostProcessSany::Init create tracker failed!";
          return ErrorCode::UNINITIALIZED;
        }
        auto res = tracker_->Init(tracker_config["TrackerPipelineParams"]);
        if (res) {
          TFATAL << "OdLidarPostProcessSany::Init tracker init failed! " << res;
          return ErrorCode::UNINITIALIZED;
        }
        TINFO << "OdLidarPostProcessSany::Init turn on tracker pipeline method: " << tracker_pipeline_method;
      }
    }

    // cluster detection param
    cluster_detection_switch_ = config["ClusterDetection"].IsDefined();
    if (cluster_detection_switch_) {
      cluster_detection_switch_ = config["ClusterDetection"]["Switch"].as<bool>();
      if (cluster_detection_switch_) {
        cluster_detector_ = std::make_unique<ClusterDetection>();
        auto res = cluster_detector_->Init(config["ClusterDetection"]["Params"]);
        if (res) {
          TFATAL << "OdLidarPostProcessSany::Init cluster detector init failed! " << res;
          return ErrorCode::UNINITIALIZED;
        }
      }
      TINFO << "OdLidarPostProcessSany::Init turn on cluster detection!";
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
  if (frame == nullptr) {
    TWARNING << "OdLidarPostProcessSany::Run frame is nullptr";
    return ErrorCode::LIDAR_OD_FRAME_NOT_FOUND;
  }

  std::shared_ptr<OdometryData> odometry_data_ptr = nullptr;
  uint32_t ret = GET_ODOMETRY_DATA_BY_TIME(frame->timestamp, odometry_data_ptr);
  if (ret != ErrorCode::SUCCESS) {
    TERROR << "OdLidarPostProcessSany::Run failed, get GET_ODOMETRY_DATA_BY_TIME failed";
    return ret;
  }
  if (odometry_data_ptr == nullptr) {
    TERROR << "OdLidarPostProcessSany::Run failed, get odometry_data is nullptr";
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
    frame->id_manager_ptr = tracker_->GetIDManager();
  } else {
    frame->tracked_objects.clear();
    frame->tracked_objects.assign(frame->detected_objects.begin(), frame->detected_objects.end());
  }

  // cluster detection
  if (cluster_detection_switch_) {
    std::shared_ptr<PointCloudData> noground_cloud = nullptr;
    ret = GET_SENSOR_DATA_BY_TIME("LIDAR_0", "noground", frame->timestamp, noground_cloud);
    if (ret != ErrorCode::SUCCESS) {
      TERROR << "OdLidarPostProcessSany::Run failed, get noground cloud failed";
      return ret;
    }
    if (noground_cloud == nullptr) {
      TERROR << "OdLidarPostProcessSany::Run failed, get noground cloud is nullptr";
      return ErrorCode::POINT_CLOUD_INVALID;
    }

    frame->noground_cloud = noground_cloud->data;
    cluster_detector_->Process(frame);
  }

  return ErrorCode::SUCCESS;
}

std::any OdLidarPostProcessSany::GetData(const std::string& key) { return nullptr; }

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
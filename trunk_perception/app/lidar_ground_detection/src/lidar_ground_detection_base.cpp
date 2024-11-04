/**
 * @file lidar_ground_detection_base.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-11-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/app/lidar_ground_detection/lidar_ground_detection_base.h"
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/types/point.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::uint32_t LidarGroundDetectionBase::Init(const YAML::Node& config) {
  try {
    lidar_name_ = config["Name"].as<std::string>();
    ground_detection_switch_ = config["GroundDetection"].IsDefined();
    if (ground_detection_switch_) ground_detection_switch_ = config["GroundDetection"]["Switch"].as<bool>();
    if (ground_detection_switch_) {
      const std::string ground_detection_method = config["GroundDetection"]["Method"].as<std::string>();
      ground_detector_ = GroundDetectionManager<PointT>::Create(ground_detection_method);
      if (!ground_detector_) {
        TFATAL << "[LidarGroundDetectionBase::Init] Ground detector create failed for " << lidar_name_;
        return ErrorCode::UNINITIALIZED;
      }
      if (ground_detector_->init(config["GroundDetection"]["MethodParams"])) {
        TFATAL << "[LidarGroundDetectionBase::Init] Ground detector init failed for " << lidar_name_;
        return ErrorCode::UNINITIALIZED;
      }
      TINFO << "Ground detection using method " << ground_detection_method << " is set on for " << lidar_name_;
    }
  } catch (const std::exception& e) {
    TFATAL << "LidarGroundDetectionBase::Init failed, " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }

  return ErrorCode::SUCCESS;
}

std::uint32_t LidarGroundDetectionBase::Run(const double& ts) {
  // init
  ground_cloud_ = nullptr;
  no_ground_cloud_ = nullptr;
  uint32_t ret = ErrorCode::SUCCESS;

  // 获取tf变换
  auto tf = GET_SENSOR_POSE(lidar_name_);
  if (tf == nullptr) {
    TERROR << "LidarGroundDetectionBase::Run failed, get " << lidar_name_ << " tf failed";
    return ErrorCode::SENSOR_POSE_NOT_FOUND;
  }

  // 获取点云
  std::shared_ptr<PointCloudData> tf_cloud = nullptr;
  ret = GET_SENSOR_DATA_BY_TIME(lidar_name_, "tf", ts, tf_cloud);
  if (ret != ErrorCode::SUCCESS) {
    TERROR << "LidarGroundDetectionBase::Run failed, get " << lidar_name_ << " tf cloud failed";
    return ret;
  }
  if (tf_cloud == nullptr) {
    TERROR << "LidarGroundDetectionBase::Run failed, get " << lidar_name_ << " tf cloud is nullptr";
    return ErrorCode::POINT_CLOUD_INVALID;
  }

  // 地面检测
  if (ground_detection_switch_) {
    ground_detector_->setTF(*tf);
    auto res = ground_detector_->process(tf_cloud->data);
    if (res) {
      TERROR << lidar_name_ << " ground detection fail!";
      return ErrorCode::LIDAR_GROUND_DETECTION_FAILED;
    }

    ground_cloud_ = ground_detector_->getGroundCloud();
    no_ground_cloud_ = ground_detector_->getNoGroundCloud();

    if (ground_cloud_) {
      ret = PUSH_SENSOR_DATA(lidar_name_, "ground", tf_cloud->time, ground_cloud_);
      if (ret != ErrorCode::SUCCESS) {
        TERROR << "LidarGroundDetectionBase::Run failed, push " << lidar_name_ << " ground cloud failed";
        return ret;
      }
    }

    if (no_ground_cloud_) {
      ret = PUSH_SENSOR_DATA(lidar_name_, "noground", tf_cloud->time, no_ground_cloud_);
      if (ret != ErrorCode::SUCCESS) {
        TERROR << "LidarGroundDetectionBase::Run failed, push " << lidar_name_ << " noground cloud failed";
        return ret;
      }
    }
  }

  return ErrorCode::SUCCESS;
}

std::any LidarGroundDetectionBase::GetData(const std::string& key) {
  if (key == "ground") {
    return ground_cloud_;
  } else if (key == "noground") {
    return no_ground_cloud_;
  }
  return nullptr;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END

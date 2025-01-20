#include "trunk_perception/app/target_fusion/data_fusion/tracker_manager.h"
#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_1l1r.h"
#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_1l1r1v.h"
#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_1l5r1v.h"
#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_1l5r2v.h"
#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_base.h"
#include "trunk_perception/app/target_fusion/data_fusion/kalman_motion_fusion.h"
#include "trunk_perception/app/target_fusion/data_fusion/measurement_functions.h"
#include "trunk_perception/app/target_fusion/data_fusion/shape_fusion_lidar_only_impl.h"
#include "trunk_perception/app/target_fusion/data_fusion/type_fusion_sliding_window.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/types/fused_object.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

std::uint32_t TrackerManager::Init(const YAML::Node& config) {
  try {
    motion_kf_config_.motion_model = GetMotionModel(config["MotionFusion"]["MotionModel"].as<std::string>());
    motion_kf_config_.sensor_R = GetMeasurementR(config["MotionFusion"]["Measurement"].as<std::string>());
    motion_kf_config_.sensor_H = GetMeasurementH(config["MotionFusion"]["Measurement"].as<std::string>());

    std::string shape_fusion_type = config["ShapeFusion"]["Type"].as<std::string>();
    if (shape_fusion_type == "LidarOnly") {
      auto shape_fusion_config = std::make_shared<ShapeFusionLidarOnlyConfig>();
      shape_fusion_config->type = shape_fusion_type;
      shape_fusion_config->window_size = config["ShapeFusion"]["Params"]["WindowSize"].as<int>();
      shape_fusion_config->threshold = config["ShapeFusion"]["Params"]["Threshold"].as<float>();
      shape_fusion_config->min_valid_frame = config["ShapeFusion"]["Params"]["MinValidFrame"].as<int>();
      shape_fusion_config_ = shape_fusion_config;
    } else {
      TFATAL << "[TrackerManager] Init failed: shape_fusion_type is not supported";
      return ErrorCode::TARGET_FUSION_INIT_TRACKER_MANAGER_FAILED;
    }

    std::string existence_fusion_type = config["ExistenceFusion"]["Type"].as<std::string>();
    if (existence_fusion_type == "1L1R") {
      auto existence_fusion_config = std::make_shared<ExistenceFusion1L1RConfig>();
      existence_fusion_config->type = existence_fusion_type;
      existence_fusion_config_ = existence_fusion_config;
    } else if (existence_fusion_type == "1L1R1V") {
      auto existence_fusion_config = std::make_shared<ExistenceFusion1L1R1VConfig>();
      existence_fusion_config->type = existence_fusion_type;
      existence_fusion_config_ = existence_fusion_config;
    } else if (existence_fusion_type == "1L5R1V") {
      auto existence_fusion_config = std::make_shared<ExistenceFusion1L5R1VConfig>();
      existence_fusion_config->type = existence_fusion_type;
      existence_fusion_config_ = existence_fusion_config;
    } else if (existence_fusion_type == "1L5R2V") {
      auto existence_fusion_config = std::make_shared<ExistenceFusion1L5R2VConfig>();
      existence_fusion_config->type = existence_fusion_type;
      existence_fusion_config_ = existence_fusion_config;
    } else {
      TFATAL << "[TrackerManager] Init failed: existence_fusion_type is not supported";
      return ErrorCode::TARGET_FUSION_INIT_TRACKER_MANAGER_FAILED;
    }

    std::string type_fusion_type = config["TypeFusion"]["Type"].as<std::string>();
    if (type_fusion_type == "SlidingWindow") {
      auto type_fusion_config = std::make_shared<TypeFusionSlidingWindowConfig>();
      type_fusion_config->config_type = type_fusion_type;
      type_fusion_config->window_size = config["TypeFusion"]["Params"]["WindowSize"].as<int>();
      type_fusion_config->min_valid_frame = config["TypeFusion"]["Params"]["MinValidFrame"].as<int>();
      type_fusion_config_ = type_fusion_config;
    } else {
      TFATAL << "[TrackerManager] Init failed: type_fusion_type is not supported";  // @zzg 2024-12-26
      return ErrorCode::TARGET_FUSION_INIT_TRACKER_MANAGER_FAILED;
    }
  } catch (const std::exception& e) {
    TFATAL << "[TrackerManager] Init failed: " << e.what();
    return ErrorCode::TARGET_FUSION_INIT_TRACKER_MANAGER_FAILED;
  }
  return ErrorCode::SUCCESS;
}

TrackerPtr TrackerManager::CreateTracker(const int& track_id, const LidarMeasureFrame::ConstPtr& lidar_object) {
  FusedObject::Ptr object_ptr = std::make_shared<FusedObject>();
  object_ptr->track_id = track_id;
  object_ptr->timestamp = lidar_object->timestamp;
  object_ptr->theta = lidar_object->theta;
  object_ptr->size = lidar_object->size;
  object_ptr->center = lidar_object->center;
  object_ptr->rear_middle_point = lidar_object->rear_middle_point;
  object_ptr->velocity = lidar_object->velocity;
  object_ptr->acceleration = lidar_object->acceleration;
  object_ptr->confidence = lidar_object->confidence;
  object_ptr->type = lidar_object->type;
  object_ptr->InitTrackPoint();
  object_ptr->odo_lidar_ptr = lidar_object->odo_lidar_ptr;
  TrackerPtr tracker = std::make_shared<Tracker>(object_ptr, motion_kf_config_, shape_fusion_config_,
                                                 existence_fusion_config_, type_fusion_config_, lidar_object);
  return tracker;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
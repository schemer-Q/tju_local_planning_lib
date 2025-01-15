#include "trunk_perception/app/target_fusion/target_fusion_impl/target_fusion_a.h"
#include <cstdint>
#include <memory>
#include <numeric>

#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

#define CHECK_AND_RETURN(func_call)                                     \
  do {                                                                  \
    uint32_t ret = func_call;                                           \
    if (ret != ErrorCode::SUCCESS) {                                    \
      TWARNING << "TargetFusionA::" #func_call " failed, ret: " << ret; \
      return ret;                                                       \
    }                                                                   \
  } while (0)

TargetFusionA::TargetFusionA() = default;

TargetFusionA::~TargetFusionA() = default;

std::uint32_t TargetFusionA::Init(const YAML::Node& config) {
  unsigned int max_id = 10000;
  YAML::Node tracker_config;
  try {
    max_id = config["IDPool"]["MaxID"].as<unsigned int>();
    tracker_config = config["Tracker"];
  } catch (const std::exception& e) {
    TERROR << "TargetFusionA::Init Load config failed, error: " << e.what();
    return ErrorCode::TARGET_FUSION_INIT_LOAD_CONFIG_FAILED;
  }

  id_pool_ptr_ = std::make_shared<IDPool>(max_id);

  tracker_manager_ptr_ = std::make_shared<TrackerManager>();
  CHECK_AND_RETURN(tracker_manager_ptr_->Init(tracker_config));

  tracker_objects_match_ptr_ = std::make_shared<TrackerObjectsMatch>();
  CHECK_AND_RETURN(tracker_objects_match_ptr_->Init(config["Match"]));

  return ErrorCode::SUCCESS;
}

std::uint32_t TargetFusionA::Run(const double& ts) {
  frame_ptr_ = std::make_shared<common::TargetFusionFrame>();
  frame_ptr_->timestamp = ts;

  CHECK_AND_RETURN(GetInputData(ts));
  ConvertToLocalCoordinate();

  if (is_first_frame_) {
    frame_ptr_->unassigned_lidar_objects_.resize(frame_ptr_->lidar_tracked_objects_local.size());
    std::iota(frame_ptr_->unassigned_lidar_objects_.begin(), frame_ptr_->unassigned_lidar_objects_.end(), 0);
    frame_ptr_->unassigned_front_radar_objects_.resize(frame_ptr_->front_radar_objects_local.size());
    std::iota(frame_ptr_->unassigned_front_radar_objects_.begin(), frame_ptr_->unassigned_front_radar_objects_.end(),
              0);
    frame_ptr_->unassigned_front_vision_objects_.resize(frame_ptr_->front_vision_tracked_objects_local.size());
    std::iota(frame_ptr_->unassigned_front_vision_objects_.begin(), frame_ptr_->unassigned_front_vision_objects_.end(),
              0);
    GenerateNewTrackers();
    is_first_frame_ = false;
    UPDATE_TF_FRAME(frame_ptr_);
    return ErrorCode::SUCCESS;
  }

  Predict();
  Association();
  Update();
  GenerateNewTrackers();
  GateKeeper();
  GenerateFusedObject();

  UPDATE_TF_FRAME(frame_ptr_);
  return ErrorCode::SUCCESS;
}

std::any TargetFusionA::GetData(const std::string& key) {
  if (key == "AssociateDebugData") {
    return associate_debug_data_ptr_;
  }
  return std::any();
}

std::uint32_t TargetFusionA::GetInputData(const double& ts) {
  CHECK_AND_RETURN(GetLidarData());
  CHECK_AND_RETURN(GetFrontRadarData());
  CHECK_AND_RETURN(GetFrontVisionData());
  CHECK_AND_RETURN(GetRightFrontCubtektarRadarData());
  CHECK_AND_RETURN(GetRightRearCubtektarRadarData());
  CHECK_AND_RETURN(GetLeftRearCubtektarRadarData());
  CHECK_AND_RETURN(GetLeftFrontCubtektarRadarData());
  return ErrorCode::SUCCESS;
}

uint32_t TargetFusionA::GetOdometryData(const double& ts, std::shared_ptr<Odometry>& odometry_ptr,
                                        const bool& use_compensation) {
  std::shared_ptr<OdometryData> odometry_data_ptr = nullptr;
  GET_ODOMETRY_DATA_BY_TIME(ts, odometry_data_ptr);
  if (odometry_data_ptr == nullptr || odometry_data_ptr->data == nullptr) {
    TWARNING << "TargetFusionA::GetOdometryData GET_ODOMETRY_DATA_BY_TIME failed";
    return ErrorCode::TARGET_FUSION_GET_DATA_ODOMETRY_FAILED;
  }
  odometry_ptr = std::make_shared<Odometry>(*odometry_data_ptr->data);
  if (use_compensation) {
    double delta_t = ts - odometry_data_ptr->time;
    Eigen::Vector3d delta_v3d = delta_t * Eigen::Vector3d(odometry_data_ptr->data->wheel_speed, 0.0f, 0.0f);
    odometry_ptr->position += delta_v3d;
  }
  return ErrorCode::SUCCESS;
}

uint32_t TargetFusionA::GetLidarData() {
  // 获取激光跟踪数据
  auto od_lidar_frame_ptr = GET_OD_LIDAR_FRAME();
  if (od_lidar_frame_ptr == nullptr) {
    TWARNING << "TargetFusionA::GetLidarData GET_OD_LIDAR_FRAME failed";
    return ErrorCode::TARGET_FUSION_GET_DATA_LIDAR_OD_FRAME_FAILED;
  }
  frame_ptr_->lidar_tracked_objects.reserve(od_lidar_frame_ptr->tracked_objects.size());
  std::transform(od_lidar_frame_ptr->tracked_objects.begin(), od_lidar_frame_ptr->tracked_objects.end(),
                 std::back_inserter(frame_ptr_->lidar_tracked_objects),
                 [](const Object& obj) { return std::make_shared<LidarMeasureFrame>(obj); });
  frame_ptr_->lidar_timestamp = od_lidar_frame_ptr->timestamp;

  // 获取lidar时间戳对应的odometry数据
  CHECK_AND_RETURN(
      GetOdometryData(frame_ptr_->lidar_timestamp, frame_ptr_->odometry_lidar_ptr, if_time_compensate_lidar_odometry_));

  return ErrorCode::SUCCESS;
}

uint32_t TargetFusionA::GetFrontRadarData() {
  // 获取前向毫米波数据
  // 通过激光雷达时间找毫米波数据，因为时间补偿是以激光雷达时间为基准
  std::shared_ptr<ARS430RadarData> front_radar_data_ptr;
  GET_FRONT_RADAR_DATA_BY_TIME(frame_ptr_->lidar_timestamp, front_radar_data_ptr);
  if (front_radar_data_ptr == nullptr || front_radar_data_ptr->data == nullptr) {
    TWARNING << "TargetFusionA::GetFrontRadarData GET_FRONT_RADAR_DATA_BY_TIME failed";
    return ErrorCode::TARGET_FUSION_GET_DATA_FRONT_RADAR_FAILED;
  }
  frame_ptr_->front_radar_timestamp = front_radar_data_ptr->time;
  frame_ptr_->front_radar_objects.resize(front_radar_data_ptr->data->objects.size());
  for (size_t i = 0; i < frame_ptr_->front_radar_objects.size(); ++i) {
    frame_ptr_->front_radar_objects[i] = std::make_shared<ars430::RadarMeasureFrame>(
        front_radar_data_ptr->data->objects[i], frame_ptr_->front_radar_timestamp);
  }

  // 获取前向毫米波外参
  auto front_radar_pose = GET_SENSOR_POSE("RADAR_0");
  if (front_radar_pose == nullptr) {
    TERROR << "TargetFusionA::GetFrontRadarData GET_SENSOR_POSE failed";
    return ErrorCode::TARGET_FUSION_GET_DATA_FRONT_RADAR_POSE_FAILED;
  }

  // 获取前向毫米波时间戳对应的odometry数据，用于将相对速度转为绝对速度
  CHECK_AND_RETURN(GetOdometryData(frame_ptr_->front_radar_timestamp, frame_ptr_->odometry_front_radar_ptr,
                                   if_space_compensate_front_radar_));

  // 将坐标系转到车体系，将相对速度转为绝对速度
  Eigen::Matrix3f rotation_matrix = front_radar_pose->matrix().block<3, 3>(0, 0);

  frame_ptr_->front_radar_objects_compensated.reserve(frame_ptr_->front_radar_objects.size());
  for (const auto& obj : frame_ptr_->front_radar_objects) {
    ars430::RadarMeasureFrame::Ptr obj_compensated = std::make_shared<ars430::RadarMeasureFrame>(*obj);

    // distance2d
    Eigen::Vector4f v4f_distance2d =
        front_radar_pose->matrix() *
        Eigen::Vector4f(obj_compensated->radar_obj.distance2d.x, obj_compensated->radar_obj.distance2d.y, 0.0f, 1.0f);
    obj_compensated->radar_obj.distance2d.x = v4f_distance2d.x();
    obj_compensated->radar_obj.distance2d.y = v4f_distance2d.y();
    obj_compensated->local_distance2d = Eigen::Vector2d(v4f_distance2d.x(), v4f_distance2d.y());

    // velocity2d - 只做旋转变换，不做平移
    Eigen::Vector3f v3f_velocity2d =
        Eigen::Vector3f(obj_compensated->radar_obj.velocity2d.x, obj_compensated->radar_obj.velocity2d.y, 0.0f);
    v3f_velocity2d = rotation_matrix * v3f_velocity2d;
    obj_compensated->radar_obj.velocity2d.x = v3f_velocity2d.x();
    obj_compensated->radar_obj.velocity2d.y = v3f_velocity2d.y();

    // acceleration_x - 同样只做旋转变换
    Eigen::Vector3f v3f_acceleration_x = Eigen::Vector3f(obj_compensated->radar_obj.acceleration_x, 0.0f, 0.0f);
    v3f_acceleration_x = rotation_matrix * v3f_acceleration_x;
    obj_compensated->radar_obj.acceleration_x = v3f_acceleration_x.x();

    // 相对速度补偿为绝对速度
    obj_compensated->radar_obj.velocity2d.x += frame_ptr_->odometry_front_radar_ptr->wheel_speed;
    frame_ptr_->front_radar_objects_compensated.push_back(obj_compensated);
  }

  Eigen::Matrix4d space_comp_t_;  // 空间补偿，从radar时间自车位置到lidar时刻自车坐标系
  if (if_space_compensate_front_radar_) {
    space_comp_t_ = frame_ptr_->odometry_lidar_ptr->Matrix().inverse() * frame_ptr_->odometry_front_radar_ptr->Matrix();
  }

  if (if_time_compensate_front_radar_) {
    double delta_t = frame_ptr_->lidar_timestamp - frame_ptr_->front_radar_timestamp;
    for (auto& obj : frame_ptr_->front_radar_objects_compensated) {
      Eigen::Vector3f delta_v3f =
          delta_t * Eigen::Vector3f(obj->radar_obj.velocity2d.x, obj->radar_obj.velocity2d.y, 0.0f);
      obj->radar_obj.distance2d.x += delta_v3f.x();
      obj->radar_obj.distance2d.y += delta_v3f.y();

      if (if_space_compensate_front_radar_) {
        Eigen::Vector4d loc_vec(obj->radar_obj.distance2d.x, obj->radar_obj.distance2d.y, 0, 1);
        Eigen::Vector4d loc_vec_comp = space_comp_t_ * loc_vec;
        obj->radar_obj.distance2d.x = loc_vec_comp.x();
        obj->radar_obj.distance2d.y = loc_vec_comp.y();
        obj->local_distance2d = Eigen::Vector2d(obj->radar_obj.distance2d.x, obj->radar_obj.distance2d.y);
      }
    }
  }

  return ErrorCode::SUCCESS;
}

uint32_t TargetFusionA::GetFrontVisionData() {
  auto front_vision_frame_ptr = GET_FOD_VISION_FRAME();
  if (front_vision_frame_ptr == nullptr) {
    TWARNING << "TargetFusionA::GetFrontVisionData GET_FOD_VISION_FRAME failed";
    return ErrorCode::TARGET_FUSION_GET_DATA_FRONT_VISION_FAILED;
  }

  frame_ptr_->front_vision_tracked_objects.reserve(front_vision_frame_ptr->tracked_objects.size());
  std::transform(front_vision_frame_ptr->tracked_objects.begin(), front_vision_frame_ptr->tracked_objects.end(),
                 std::back_inserter(frame_ptr_->front_vision_tracked_objects),
                 [](const Object& obj) { return std::make_shared<VisionMeasureFrame>(obj); });

  frame_ptr_->front_vision_timestamp = front_vision_frame_ptr->timestamp;

  CHECK_AND_RETURN(GetOdometryData(frame_ptr_->front_vision_timestamp, frame_ptr_->odometry_front_vision_ptr,
                                   if_time_compensate_vision_odometry_));

  return ErrorCode::SUCCESS;
}

// @author zzg 2025-01-15 获取环视检测数据
uint32_t TargetFusionA::GetSideVisionData() {
  auto side_vision_frame_ptr = GET_SIDE_OD_VISION_FRAME();
  if (side_vision_frame_ptr == nullptr) {
    TWARNING << "TargetFusionA::GetSideVisionData GET_SIDE_OD_VISION_FRAME failed";
    return ErrorCode::TARGET_FUSION_GET_DATA_SIDE_VISION_FAILED;
  }

  frame_ptr_->side_vision_detected_objects.reserve(side_vision_frame_ptr->detected_objects.size());
  std::transform(side_vision_frame_ptr->detected_objects.begin(), side_vision_frame_ptr->detected_objects.end(),
                 std::back_inserter(frame_ptr_->side_vision_detected_objects),
                 [](const Object& obj) { return std::make_shared<VisionMeasureFrame>(obj); });

  frame_ptr_->side_vision_timestamp = side_vision_frame_ptr->timestamp;

  CHECK_AND_RETURN(GetOdometryData(frame_ptr_->side_vision_timestamp, frame_ptr_->odometry_side_vision_ptr,
                                   if_time_compensate_vision_odometry_));

  return ErrorCode::SUCCESS;
}

// @author zzg 2025-01-06 获取右前角毫米波雷达数据
uint32_t TargetFusionA::GetRightFrontCubtektarRadarData() {
  // 获取右前角毫米波数据
  // 通过激光雷达时间找右前角毫米波数据，时间补偿是以激光雷达时间为基准
  std::shared_ptr<CUBTEKTARRadarData> corner_radar1_data_ptr;
  GET_CORNER_RADAR_DATA_BY_TIME("RADAR_1", frame_ptr_->lidar_timestamp, corner_radar1_data_ptr);
  if (corner_radar1_data_ptr == nullptr || corner_radar1_data_ptr->data == nullptr) {
    TWARNING << "TargetFusionA::GetRightFrontCubtektarRadarData GET_CORNER_RADAR_DATA_BY_TIME RADAR_1 failed";
    return ErrorCode::TARGET_FUSION_GET_DATA_CORNER_RADAR1_FAILED;
  }
  frame_ptr_->corner_radar1_timestamp = corner_radar1_data_ptr->time;

  frame_ptr_->corner_radar1_objects.resize(corner_radar1_data_ptr->data->obj_num);
  for (size_t i = 0; i < frame_ptr_->corner_radar1_objects.size(); ++i) {
    frame_ptr_->corner_radar1_objects[i] = std::make_shared<cubtektar::RadarMeasureFrame>(
        corner_radar1_data_ptr->data->objects[i], frame_ptr_->corner_radar1_timestamp, CornerRadarName::RADAR1);
  }

  // 获取右前位置角毫米波雷达外参
  auto corner_radar1_pose = GET_SENSOR_POSE("RADAR_1");
  if (corner_radar1_pose == nullptr) {
    TERROR << "TargetFusionA::GetRightFrontCubtektarRadarData GET_SENSOR_POSE RADAR_1 failed";
    return ErrorCode::TARGET_FUSION_GET_DATA_CORNER_RADAR1_POSE_FAILED;
  }

  // 获取右前位置毫米波时间戳对应的odometry数据，用于将相对速度转为绝对速度
  CHECK_AND_RETURN(GetOdometryData(frame_ptr_->corner_radar1_timestamp, frame_ptr_->odometry_corner_radar1_ptr,
                                   if_space_compensate_front_radar_));
  // 相对速度补偿为绝对速度
  for (const auto& obj : frame_ptr_->corner_radar1_objects) {
    obj->radar_obj.vx += frame_ptr_->odometry_corner_radar1_ptr->wheel_speed;
  }

  return ErrorCode::SUCCESS;
}

// @author zzg 2025-01-06 获取右后角毫米波雷达数据
uint32_t TargetFusionA::GetRightRearCubtektarRadarData() {
  // 获取右后角毫米波数据
  // 通过激光雷达时间找右后角毫米波数据，时间补偿是以激光雷达时间为基准
  std::shared_ptr<CUBTEKTARRadarData> corner_radar5_data_ptr;
  GET_CORNER_RADAR_DATA_BY_TIME("RADAR_5", frame_ptr_->lidar_timestamp, corner_radar5_data_ptr);
  if (corner_radar5_data_ptr == nullptr || corner_radar5_data_ptr->data == nullptr) {
    TWARNING << "TargetFusionA::GetRightRearCubtektarRadarData GET_CORNER_RADAR_DATA_BY_TIME RADAR_5 failed";
    return ErrorCode::TARGET_FUSION_GET_DATA_CORNER_RADAR5_FAILED;
  }
  frame_ptr_->corner_radar5_timestamp = corner_radar5_data_ptr->time;

  frame_ptr_->corner_radar5_objects.resize(corner_radar5_data_ptr->data->obj_num);
  for (size_t i = 0; i < frame_ptr_->corner_radar5_objects.size(); ++i) {
    frame_ptr_->corner_radar5_objects[i] = std::make_shared<cubtektar::RadarMeasureFrame>(
        corner_radar5_data_ptr->data->objects[i], frame_ptr_->corner_radar5_timestamp, CornerRadarName::RADAR5);
  }

  // 获取右后位置毫米波时间戳对应的odometry数据，用于将相对速度转为绝对速度
  CHECK_AND_RETURN(GetOdometryData(frame_ptr_->corner_radar5_timestamp, frame_ptr_->odometry_corner_radar5_ptr,
                                   if_space_compensate_front_radar_));
  // 相对速度补偿为绝对速度
  for (const auto& obj : frame_ptr_->corner_radar5_objects) {
    obj->radar_obj.vx += frame_ptr_->odometry_corner_radar5_ptr->wheel_speed;
  }
  return ErrorCode::SUCCESS;
}

// @author zzg 2025-01-06 获取左后角毫米波雷达数据
uint32_t TargetFusionA::GetLeftRearCubtektarRadarData() {
  // 获取左后毫米波数据
  // 通过激光雷达时间找角毫米波数据，时间补偿是以激光雷达时间为基准
  std::shared_ptr<CUBTEKTARRadarData> corner_radar7_data_ptr;
  GET_CORNER_RADAR_DATA_BY_TIME("RADAR_7", frame_ptr_->lidar_timestamp, corner_radar7_data_ptr);
  if (corner_radar7_data_ptr == nullptr || corner_radar7_data_ptr->data == nullptr) {
    TWARNING << "TargetFusionA::GetLeftRearCubtektarRadarData GET_CORNER_RADAR_DATA_BY_TIME RADAR_7 failed";
    return ErrorCode::TARGET_FUSION_GET_DATA_CORNER_RADAR7_FAILED;
  }
  frame_ptr_->corner_radar7_timestamp = corner_radar7_data_ptr->time;
  frame_ptr_->corner_radar7_objects.resize(corner_radar7_data_ptr->data->obj_num);
  for (size_t i = 0; i < frame_ptr_->corner_radar7_objects.size(); ++i) {
    frame_ptr_->corner_radar7_objects[i] = std::make_shared<cubtektar::RadarMeasureFrame>(
        corner_radar7_data_ptr->data->objects[i], frame_ptr_->corner_radar7_timestamp, CornerRadarName::RADAR7);
  }

  // 获取左后位置毫米波时间戳对应的odometry数据，用于将相对速度转为绝对速度
  CHECK_AND_RETURN(GetOdometryData(frame_ptr_->corner_radar7_timestamp, frame_ptr_->odometry_corner_radar7_ptr,
                                   if_space_compensate_front_radar_));
  // 相对速度补偿为绝对速度
  for (const auto& obj : frame_ptr_->corner_radar7_objects) {
    obj->radar_obj.vx += frame_ptr_->odometry_corner_radar7_ptr->wheel_speed;
  }
  return ErrorCode::SUCCESS;
}

// @author zzg 2025-01-06 获取左前角毫米波雷达数据
uint32_t TargetFusionA::GetLeftFrontCubtektarRadarData() {
  // 获取左前毫米波数据
  // 通过激光雷达时间找角毫米波数据，时间补偿是以激光雷达时间为基准
  std::shared_ptr<CUBTEKTARRadarData> corner_radar11_data_ptr;
  GET_CORNER_RADAR_DATA_BY_TIME("RADAR_11", frame_ptr_->lidar_timestamp, corner_radar11_data_ptr);
  if (corner_radar11_data_ptr == nullptr || corner_radar11_data_ptr->data == nullptr) {
    TWARNING << "TargetFusionA::GetLeftFrontCubtektarRadarData GET_CORNER_RADAR_DATA_BY_TIME RADAR_11 failed";
    return ErrorCode::TARGET_FUSION_GET_DATA_CORNER_RADAR11_FAILED;
  }
  frame_ptr_->corner_radar11_timestamp = corner_radar11_data_ptr->time;
  frame_ptr_->corner_radar11_objects.resize(corner_radar11_data_ptr->data->obj_num);
  for (size_t i = 0; i < frame_ptr_->corner_radar11_objects.size(); ++i) {
    frame_ptr_->corner_radar11_objects[i] = std::make_shared<cubtektar::RadarMeasureFrame>(
        corner_radar11_data_ptr->data->objects[i], frame_ptr_->corner_radar11_timestamp, CornerRadarName::RADAR11);
  }

  // 获取右后位置毫米波时间戳对应的odometry数据，用于将相对速度转为绝对速度
  CHECK_AND_RETURN(GetOdometryData(frame_ptr_->corner_radar11_timestamp, frame_ptr_->odometry_corner_radar11_ptr,
                                   if_space_compensate_front_radar_));
  // 相对速度补偿为绝对速度
  for (const auto& obj : frame_ptr_->corner_radar11_objects) {
    obj->radar_obj.vx += frame_ptr_->odometry_corner_radar11_ptr->wheel_speed;
  }
  return ErrorCode::SUCCESS;
}

void TargetFusionA::ConvertToLocalCoordinate() {
  // 激光雷达数据转到局部坐标系下
  frame_ptr_->lidar_tracked_objects_local.resize(frame_ptr_->lidar_tracked_objects.size());
  for (size_t i = 0; i < frame_ptr_->lidar_tracked_objects.size(); ++i) {
    frame_ptr_->lidar_tracked_objects_local[i] =
        std::make_shared<LidarMeasureFrame>(*frame_ptr_->lidar_tracked_objects[i]);
    frame_ptr_->lidar_tracked_objects_local[i]->Transform(frame_ptr_->odometry_lidar_ptr->Matrix());
    frame_ptr_->lidar_tracked_objects_local[i]->odo_lidar_ptr = frame_ptr_->odometry_lidar_ptr;
  }

  // 毫米波雷达数据转到局部坐标系下
  frame_ptr_->front_radar_objects_local.resize(frame_ptr_->front_radar_objects_compensated.size());
  for (size_t i = 0; i < frame_ptr_->front_radar_objects_compensated.size(); ++i) {
    frame_ptr_->front_radar_objects_local[i] =
        std::make_shared<ars430::RadarMeasureFrame>(*frame_ptr_->front_radar_objects_compensated[i]);
    frame_ptr_->front_radar_objects_local[i]->Transform(frame_ptr_->odometry_lidar_ptr->Matrix());
  }

  // 前向视觉数据转到局部坐标系下 @author zzg 2024-12-13
  frame_ptr_->front_vision_tracked_objects_local.resize(frame_ptr_->front_vision_tracked_objects.size());
  for (size_t i = 0; i < frame_ptr_->front_vision_tracked_objects.size(); ++i) {
    frame_ptr_->front_vision_tracked_objects_local[i] =
        std::make_shared<VisionMeasureFrame>(*frame_ptr_->front_vision_tracked_objects[i]);
    frame_ptr_->front_vision_tracked_objects_local[i]->Transform(frame_ptr_->odometry_lidar_ptr->Matrix());
  }

  // 右前角毫米波雷达数据转到局部坐标系下
  frame_ptr_->corner_radar1_objects_local.resize(frame_ptr_->corner_radar1_objects.size());
  for (size_t i = 0; i < frame_ptr_->corner_radar1_objects.size(); ++i) {
    frame_ptr_->corner_radar1_objects_local[i] =
        std::make_shared<cubtektar::RadarMeasureFrame>(*frame_ptr_->corner_radar1_objects[i]);
    frame_ptr_->corner_radar1_objects_local[i]->Transform(frame_ptr_->odometry_corner_radar1_ptr->Matrix());
  }

  // 右后角毫米波雷达数据转到局部坐标系下
  frame_ptr_->corner_radar5_objects_local.resize(frame_ptr_->corner_radar5_objects.size());
  for (size_t i = 0; i < frame_ptr_->corner_radar5_objects.size(); ++i) {
    frame_ptr_->corner_radar5_objects_local[i] =
        std::make_shared<cubtektar::RadarMeasureFrame>(*frame_ptr_->corner_radar5_objects[i]);
    frame_ptr_->corner_radar5_objects_local[i]->Transform(frame_ptr_->odometry_corner_radar5_ptr->Matrix());
  }

  // 左后角毫米波雷达数据转到局部坐标系下
  frame_ptr_->corner_radar7_objects_local.resize(frame_ptr_->corner_radar7_objects.size());
  for (size_t i = 0; i < frame_ptr_->corner_radar7_objects.size(); ++i) {
    frame_ptr_->corner_radar7_objects_local[i] =
        std::make_shared<cubtektar::RadarMeasureFrame>(*frame_ptr_->corner_radar7_objects[i]);
    frame_ptr_->corner_radar7_objects_local[i]->Transform(frame_ptr_->odometry_corner_radar7_ptr->Matrix());
  }

  // 左前角毫米波雷达数据转到局部坐标系下
  frame_ptr_->corner_radar11_objects_local.resize(frame_ptr_->corner_radar11_objects.size());
  for (size_t i = 0; i < frame_ptr_->corner_radar11_objects.size(); ++i) {
    frame_ptr_->corner_radar11_objects_local[i] =
        std::make_shared<cubtektar::RadarMeasureFrame>(*frame_ptr_->corner_radar11_objects[i]);
    frame_ptr_->corner_radar11_objects_local[i]->Transform(frame_ptr_->odometry_corner_radar11_ptr->Matrix());
  }
}

void TargetFusionA::Predict() {
  for (auto& tracker : new_trackers_) {
    tracker->Predict(frame_ptr_->lidar_timestamp);
    tracker->SetFusedObjectOdometry(frame_ptr_->odometry_lidar_ptr);
  }
  for (auto& tracker : stable_trackers_) {
    tracker->Predict(frame_ptr_->lidar_timestamp);
    tracker->SetFusedObjectOdometry(frame_ptr_->odometry_lidar_ptr);
  }
  for (auto& tracker : lost_trackers_) {
    tracker->Predict(frame_ptr_->lidar_timestamp);
    tracker->SetFusedObjectOdometry(frame_ptr_->odometry_lidar_ptr);
  }
}

void TargetFusionA::Association() {
  // stable tracker数据关联
  tracker_objects_match_ptr_->Match(stable_trackers_, frame_ptr_->lidar_tracked_objects_local,
                                    stable_tracker_lidar_association_result_);
  tracker_objects_match_ptr_->Match(stable_trackers_, frame_ptr_->front_radar_objects_local,
                                    stable_tracker_radar_association_result_);
  tracker_objects_match_ptr_->Match(stable_trackers_, frame_ptr_->front_vision_tracked_objects_local,
                                    stable_tracker_front_vision_association_result_);
  tracker_objects_match_ptr_->Match(stable_trackers_, frame_ptr_->corner_radar1_objects_local,
                                    stable_tracker_corner_radar_1_association_result_);
  tracker_objects_match_ptr_->Match(stable_trackers_, frame_ptr_->corner_radar5_objects_local,
                                    stable_tracker_corner_radar_5_association_result_);
  tracker_objects_match_ptr_->Match(stable_trackers_, frame_ptr_->corner_radar7_objects_local,
                                    stable_tracker_corner_radar_7_association_result_);
  tracker_objects_match_ptr_->Match(stable_trackers_, frame_ptr_->corner_radar11_objects_local,
                                    stable_tracker_corner_radar_11_association_result_);

  // 整理剩余的未关联上的数据
  frame_ptr_->unassigned_lidar_objects_ = stable_tracker_lidar_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_front_radar_objects_ = stable_tracker_radar_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_front_vision_objects_ =
      stable_tracker_front_vision_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_corner_radar_1_objects_ =
      stable_tracker_corner_radar_1_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_corner_radar_5_objects_ =
      stable_tracker_corner_radar_5_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_corner_radar_7_objects_ =
      stable_tracker_corner_radar_7_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_corner_radar_11_objects_ =
      stable_tracker_corner_radar_11_association_result_.unassigned_measurment_indices;

  // new tracker数据关联
  std::vector<LidarMeasureFrame::Ptr> unassigned_lidar_objects_tmp;
  std::vector<ars430::RadarMeasureFrame::Ptr> unassigned_front_radar_objects_tmp;
  std::vector<VisionMeasureFrame::Ptr> unassigned_front_vision_objects_temp;
  std::vector<cubtektar::RadarMeasureFrame::Ptr> unassigned_corner_radar_1_objects_temp;
  std::vector<cubtektar::RadarMeasureFrame::Ptr> unassigned_corner_radar_5_objects_temp;
  std::vector<cubtektar::RadarMeasureFrame::Ptr> unassigned_corner_radar_7_objects_temp;
  std::vector<cubtektar::RadarMeasureFrame::Ptr> unassigned_corner_radar_11_objects_temp;

  unassigned_lidar_objects_tmp.reserve(frame_ptr_->unassigned_lidar_objects_.size());
  unassigned_front_radar_objects_tmp.reserve(frame_ptr_->unassigned_front_radar_objects_.size());
  unassigned_front_vision_objects_temp.reserve(frame_ptr_->unassigned_front_vision_objects_.size());
  unassigned_corner_radar_1_objects_temp.reserve(frame_ptr_->unassigned_corner_radar_1_objects_.size());
  unassigned_corner_radar_5_objects_temp.reserve(frame_ptr_->unassigned_corner_radar_5_objects_.size());
  unassigned_corner_radar_7_objects_temp.reserve(frame_ptr_->unassigned_corner_radar_7_objects_.size());
  unassigned_corner_radar_11_objects_temp.reserve(frame_ptr_->unassigned_corner_radar_11_objects_.size());

  for (const auto& idx : frame_ptr_->unassigned_lidar_objects_) {
    unassigned_lidar_objects_tmp.push_back(frame_ptr_->lidar_tracked_objects_local[idx]);
  }
  for (const auto& idx : frame_ptr_->unassigned_front_radar_objects_) {
    unassigned_front_radar_objects_tmp.push_back(frame_ptr_->front_radar_objects_local[idx]);
  }
  for (const auto& idx : frame_ptr_->unassigned_front_vision_objects_) {
    unassigned_front_vision_objects_temp.push_back(frame_ptr_->front_vision_tracked_objects_local[idx]);
  }
  for (const auto& idx : frame_ptr_->unassigned_corner_radar_1_objects_) {
    unassigned_corner_radar_1_objects_temp.push_back(frame_ptr_->corner_radar1_objects_local[idx]);
  }
  for (const auto& idx : frame_ptr_->unassigned_corner_radar_5_objects_) {
    unassigned_corner_radar_5_objects_temp.push_back(frame_ptr_->corner_radar5_objects_local[idx]);
  }
  for (const auto& idx : frame_ptr_->unassigned_corner_radar_7_objects_) {
    unassigned_corner_radar_7_objects_temp.push_back(frame_ptr_->corner_radar7_objects_local[idx]);
  }
  for (const auto& idx : frame_ptr_->unassigned_corner_radar_11_objects_) {
    unassigned_corner_radar_11_objects_temp.push_back(frame_ptr_->corner_radar11_objects_local[idx]);
  }

  tracker_objects_match_ptr_->Match(new_trackers_, unassigned_lidar_objects_tmp, new_tracker_lidar_association_result_);
  ConvertIdx(frame_ptr_->unassigned_lidar_objects_, new_tracker_lidar_association_result_);

  tracker_objects_match_ptr_->Match(new_trackers_, unassigned_front_radar_objects_tmp,
                                    new_tracker_radar_association_result_);
  ConvertIdx(frame_ptr_->unassigned_front_radar_objects_, new_tracker_radar_association_result_);

  tracker_objects_match_ptr_->Match(new_trackers_, unassigned_front_vision_objects_temp,
                                    new_tracker_front_vision_association_result_);
  ConvertIdx(frame_ptr_->unassigned_front_vision_objects_, new_tracker_front_vision_association_result_);

  tracker_objects_match_ptr_->Match(new_trackers_, unassigned_corner_radar_1_objects_temp,
                                    new_tracker_corner_radar_1_association_result_);
  ConvertIdx(frame_ptr_->unassigned_corner_radar_1_objects_, new_tracker_corner_radar_1_association_result_);

  tracker_objects_match_ptr_->Match(new_trackers_, unassigned_corner_radar_5_objects_temp,
                                    new_tracker_corner_radar_5_association_result_);
  ConvertIdx(frame_ptr_->unassigned_corner_radar_5_objects_, new_tracker_corner_radar_5_association_result_);

  tracker_objects_match_ptr_->Match(new_trackers_, unassigned_corner_radar_7_objects_temp,
                                    new_tracker_corner_radar_7_association_result_);
  ConvertIdx(frame_ptr_->unassigned_corner_radar_7_objects_, new_tracker_corner_radar_7_association_result_);

  tracker_objects_match_ptr_->Match(new_trackers_, unassigned_corner_radar_11_objects_temp,
                                    new_tracker_corner_radar_11_association_result_);
  ConvertIdx(frame_ptr_->unassigned_corner_radar_11_objects_, new_tracker_corner_radar_11_association_result_);

  // 整理剩余的未关联上的数据
  frame_ptr_->unassigned_lidar_objects_ = new_tracker_lidar_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_front_radar_objects_ = new_tracker_radar_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_front_vision_objects_ =
      new_tracker_front_vision_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_corner_radar_1_objects_ =
      new_tracker_corner_radar_1_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_corner_radar_5_objects_ =
      new_tracker_corner_radar_5_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_corner_radar_7_objects_ =
      new_tracker_corner_radar_7_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_corner_radar_11_objects_ =
      new_tracker_corner_radar_11_association_result_.unassigned_measurment_indices;

  // lost tracker数据关联
  std::vector<LidarMeasureFrame::Ptr> unassigned_lidar_objects_tmp2;
  std::vector<ars430::RadarMeasureFrame::Ptr> unassigned_front_radar_objects_tmp2;
  std::vector<VisionMeasureFrame::Ptr> unassigned_front_vision_objects_tmp2;
  std::vector<cubtektar::RadarMeasureFrame::Ptr> unassigned_corner_radar_1_objects_temp2;
  std::vector<cubtektar::RadarMeasureFrame::Ptr> unassigned_corner_radar_5_objects_temp2;
  std::vector<cubtektar::RadarMeasureFrame::Ptr> unassigned_corner_radar_7_objects_temp2;
  std::vector<cubtektar::RadarMeasureFrame::Ptr> unassigned_corner_radar_11_objects_temp2;
  unassigned_lidar_objects_tmp2.reserve(frame_ptr_->unassigned_lidar_objects_.size());
  unassigned_front_radar_objects_tmp2.reserve(frame_ptr_->unassigned_front_radar_objects_.size());
  unassigned_front_vision_objects_tmp2.reserve(frame_ptr_->unassigned_front_vision_objects_.size());
  unassigned_corner_radar_1_objects_temp2.reserve(frame_ptr_->unassigned_corner_radar_1_objects_.size());
  unassigned_corner_radar_5_objects_temp2.reserve(frame_ptr_->unassigned_corner_radar_5_objects_.size());
  unassigned_corner_radar_7_objects_temp2.reserve(frame_ptr_->unassigned_corner_radar_7_objects_.size());
  unassigned_corner_radar_11_objects_temp2.reserve(frame_ptr_->unassigned_corner_radar_11_objects_.size());

  for (const auto& idx : frame_ptr_->unassigned_lidar_objects_) {
    unassigned_lidar_objects_tmp2.push_back(frame_ptr_->lidar_tracked_objects_local[idx]);
  }
  for (const auto& idx : frame_ptr_->unassigned_front_radar_objects_) {
    unassigned_front_radar_objects_tmp2.push_back(frame_ptr_->front_radar_objects_local[idx]);
  }
  for (const auto& idx : frame_ptr_->unassigned_front_vision_objects_) {
    unassigned_front_vision_objects_tmp2.push_back(frame_ptr_->front_vision_tracked_objects_local[idx]);
  }
  for (const auto& idx : frame_ptr_->unassigned_corner_radar_1_objects_) {
    unassigned_corner_radar_1_objects_temp2.push_back(frame_ptr_->corner_radar1_objects_local[idx]);
  }
  for (const auto& idx : frame_ptr_->unassigned_corner_radar_5_objects_) {
    unassigned_corner_radar_5_objects_temp2.push_back(frame_ptr_->corner_radar5_objects_local[idx]);
  }
  for (const auto& idx : frame_ptr_->unassigned_corner_radar_7_objects_) {
    unassigned_corner_radar_7_objects_temp2.push_back(frame_ptr_->corner_radar7_objects_local[idx]);
  }
  for (const auto& idx : frame_ptr_->unassigned_corner_radar_11_objects_) {
    unassigned_corner_radar_11_objects_temp2.push_back(frame_ptr_->corner_radar11_objects_local[idx]);
  }
  tracker_objects_match_ptr_->Match(lost_trackers_, unassigned_lidar_objects_tmp2,
                                    lost_tracker_lidar_association_result_);
  tracker_objects_match_ptr_->Match(lost_trackers_, unassigned_front_radar_objects_tmp2,
                                    lost_tracker_radar_association_result_);
  tracker_objects_match_ptr_->Match(lost_trackers_, unassigned_front_vision_objects_tmp2,
                                    lost_tracker_front_vision_association_result_);
  tracker_objects_match_ptr_->Match(lost_trackers_, unassigned_corner_radar_1_objects_temp2,
                                    lost_tracker_corner_radar_1_association_result_);
  tracker_objects_match_ptr_->Match(lost_trackers_, unassigned_corner_radar_5_objects_temp2,
                                    lost_tracker_corner_radar_5_association_result_);
  tracker_objects_match_ptr_->Match(lost_trackers_, unassigned_corner_radar_7_objects_temp2,
                                    lost_tracker_corner_radar_7_association_result_);
  tracker_objects_match_ptr_->Match(lost_trackers_, unassigned_corner_radar_11_objects_temp2,
                                    lost_tracker_corner_radar_11_association_result_);
  ConvertIdx(frame_ptr_->unassigned_lidar_objects_, lost_tracker_lidar_association_result_);
  ConvertIdx(frame_ptr_->unassigned_front_radar_objects_, lost_tracker_radar_association_result_);
  ConvertIdx(frame_ptr_->unassigned_front_vision_objects_, lost_tracker_front_vision_association_result_);
  ConvertIdx(frame_ptr_->unassigned_corner_radar_1_objects_, lost_tracker_corner_radar_1_association_result_);
  ConvertIdx(frame_ptr_->unassigned_corner_radar_5_objects_, lost_tracker_corner_radar_5_association_result_);
  ConvertIdx(frame_ptr_->unassigned_corner_radar_7_objects_, lost_tracker_corner_radar_7_association_result_);
  ConvertIdx(frame_ptr_->unassigned_corner_radar_11_objects_, lost_tracker_corner_radar_11_association_result_);

  // 整理剩余的未关联上的数据
  frame_ptr_->unassigned_lidar_objects_ = lost_tracker_lidar_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_front_radar_objects_ = lost_tracker_radar_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_front_vision_objects_ =
      lost_tracker_front_vision_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_corner_radar_1_objects_ =
      lost_tracker_corner_radar_1_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_corner_radar_5_objects_ =
      lost_tracker_corner_radar_5_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_corner_radar_7_objects_ =
      lost_tracker_corner_radar_7_association_result_.unassigned_measurment_indices;
  frame_ptr_->unassigned_corner_radar_11_objects_ =
      lost_tracker_corner_radar_11_association_result_.unassigned_measurment_indices;

  // 打印当前tracker状态
  GenerateAssociateDebugData();
}

void TargetFusionA::GenerateNewTrackers() {
  // 航迹合并：对不同传感器数据进行合并
  // 最简单的做法是仅使用Lidar创建新的跟踪，但单L目标会损失精度
  // demo： 先使用单L目标
  // TODO: 航迹合并: L/C/R

  int id = -1;
  for (const auto& idx : frame_ptr_->unassigned_lidar_objects_) {
    id = id_pool_ptr_->GetID();
    if (id == -1) {
      TERROR << "TargetFusionA::GenerateNewTrackers id_pool_ptr_->GetID() failed";
      break;
    }
    auto tracker = tracker_manager_ptr_->CreateTracker(id, frame_ptr_->lidar_tracked_objects_local[idx]);
    new_trackers_.push_back(tracker);
  }
}

void TargetFusionA::ConvertIdx(const std::vector<size_t>& idx_map, AssociationResult& association_result) {
  for (auto& pair : association_result.track_measurment_pairs) {
    pair.second = idx_map[pair.second];
  }
  for (auto& idx : association_result.unassigned_measurment_indices) {
    idx = idx_map[idx];
  }
}

void TargetFusionA::Update() {
  {  // update new trackers
    // 预处理关联结果到map中,避免重复查找
    std::vector<std::vector<SensorMeasureFrame::ConstPtr>> tracker_measures(new_trackers_.size());
    // 添加激光雷达关联结果
    for (const auto& pair : new_tracker_lidar_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->lidar_tracked_objects_local[pair.second]);
    }

    // 添加毫米波雷达关联结果
    for (const auto& pair : new_tracker_radar_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->front_radar_objects_local[pair.second]);
    }

    // 添加前向视觉目标关联结果
    for (const auto& pair : new_tracker_front_vision_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->front_vision_tracked_objects_local[pair.second]);
    }

    // 添加右前角毫米波目标关联结果
    for (const auto& pair : new_tracker_corner_radar_1_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->corner_radar1_objects_local[pair.second]);
    }

    // 添加右后角毫米波目标关联结果
    for (const auto& pair : new_tracker_corner_radar_5_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->corner_radar5_objects_local[pair.second]);
    }

    // 添加左后角毫米波目标关联结果
    for (const auto& pair : new_tracker_corner_radar_7_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->corner_radar7_objects_local[pair.second]);
    }

    // 添加左前角毫米波目标关联结果
    for (const auto& pair : new_tracker_corner_radar_11_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->corner_radar11_objects_local[pair.second]);
    }

    // 更新tracker
    for (size_t i = 0; i < new_trackers_.size(); i++) {
      new_trackers_[i]->Update(tracker_measures[i]);
    }
  }

  {  // update stable trackers
    std::vector<std::vector<SensorMeasureFrame::ConstPtr>> tracker_measures(stable_trackers_.size());
    for (const auto& pair : stable_tracker_lidar_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->lidar_tracked_objects_local[pair.second]);
    }
    for (const auto& pair : stable_tracker_radar_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->front_radar_objects_local[pair.second]);
    }
    for (const auto& pair : stable_tracker_front_vision_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->front_vision_tracked_objects_local[pair.second]);
    }
    for (const auto& pair : stable_tracker_corner_radar_1_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->corner_radar1_objects_local[pair.second]);
    }
    for (const auto& pair : stable_tracker_corner_radar_5_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->corner_radar5_objects_local[pair.second]);
    }
    for (const auto& pair : stable_tracker_corner_radar_7_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->corner_radar7_objects_local[pair.second]);
    }
    for (const auto& pair : stable_tracker_corner_radar_11_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->corner_radar11_objects_local[pair.second]);
    }
    for (size_t i = 0; i < stable_trackers_.size(); i++) {
      stable_trackers_[i]->Update(tracker_measures[i]);
    }
  }

  {  // update lost trackers
    std::vector<std::vector<SensorMeasureFrame::ConstPtr>> tracker_measures(lost_trackers_.size());
    for (const auto& pair : lost_tracker_lidar_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->lidar_tracked_objects_local[pair.second]);
    }
    for (const auto& pair : lost_tracker_radar_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->front_radar_objects_local[pair.second]);
    }
    for (const auto& pair : lost_tracker_front_vision_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->front_vision_tracked_objects_local[pair.second]);
    }
    for (const auto& pair : lost_tracker_corner_radar_1_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->corner_radar1_objects_local[pair.second]);
    }
    for (const auto& pair : lost_tracker_corner_radar_5_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->corner_radar5_objects_local[pair.second]);
    }
    for (const auto& pair : lost_tracker_corner_radar_7_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->corner_radar7_objects_local[pair.second]);
    }
    for (const auto& pair : lost_tracker_corner_radar_11_association_result_.track_measurment_pairs) {
      tracker_measures[pair.first].push_back(frame_ptr_->corner_radar11_objects_local[pair.second]);
    }
    for (size_t i = 0; i < lost_trackers_.size(); i++) {
      lost_trackers_[i]->Update(tracker_measures[i]);
    }
  }
}

void TargetFusionA::GateKeeper() {
  // 起始集进入稳定集
  auto it = std::remove_if(new_trackers_.begin(), new_trackers_.end(), [this](const TrackerPtr& tracker) {
    // TODO: 这里使用lidar作为观测帧数，后续有图像后可以另考虑
    if (tracker->GetFusedObject()->lidar_consecutive_hit > new_to_stable_life_thresh_) {
      // 将符合条件的tracker移到stable集合
      stable_trackers_.push_back(tracker);
      return true;  // 从new_trackers_中移除
    } else if (tracker->GetFusedObject()->life > new_to_stable_life_thresh_) {
      // 如果tracker生命周期大于阈值，则删除
      id_pool_ptr_->ReleaseID(tracker->GetTrackID());
      return true;
    }
    return false;  // 保留在new_trackers_中
  });
  new_trackers_.erase(it, new_trackers_.end());

  // 稳定集进入丢失集
  it = std::remove_if(stable_trackers_.begin(), stable_trackers_.end(), [this](const TrackerPtr& tracker) {
    // if (tracker->GetFusedObject()->lidar_consecutive_lost > stable_to_lost_life_thresh_) {
    if (tracker->GetFusedObject()->existence < 0.3) {
      lost_trackers_.push_back(tracker);
      return true;
    }
    return false;
  });
  stable_trackers_.erase(it, stable_trackers_.end());

  // 丢失集进入稳定集
  it = std::remove_if(lost_trackers_.begin(), lost_trackers_.end(), [this](const TrackerPtr& tracker) {
    if (tracker->GetFusedObject()->lidar_consecutive_hit > lost_to_stable_life_thresh_) {
      stable_trackers_.push_back(tracker);
      return true;
    }
    return false;
  });
  lost_trackers_.erase(it, lost_trackers_.end());

  // 删除集删除
  it = std::remove_if(lost_trackers_.begin(), lost_trackers_.end(), [this](const TrackerPtr& tracker) {
    if (tracker->GetFusedObject()->lidar_consecutive_lost > lost_to_delete_life_thresh_) {
      id_pool_ptr_->ReleaseID(tracker->GetTrackID());
      return true;
    }
    return false;
  });
  lost_trackers_.erase(it, lost_trackers_.end());
}

void TargetFusionA::GenerateFusedObject() {
  // 转换矩阵 - 从局部坐标系到车体系
  const Eigen::Matrix4d local_to_car = frame_ptr_->odometry_lidar_ptr->Matrix().inverse();

  // 合并处理 new_trackers 和 stable_trackers
  auto process_trackers = [&](const std::vector<TrackerPtr>& trackers) {
    for (const auto& tracker : trackers) {
      if (auto fused_obj = tracker->GetFusedObject()) {
        if (fused_obj->existence > 0) {
          auto fused_obj_ptr = std::make_shared<FusedObject>(*fused_obj);
          fused_obj_ptr->Transform(local_to_car);
          frame_ptr_->fused_objects.push_back(fused_obj_ptr);
        }
      }
    }
  };

  // 处理两类 trackers
  // process_trackers(new_trackers_);
  process_trackers(stable_trackers_);
}

void TargetFusionA::GenerateAssociateDebugData() {
  associate_debug_data_ptr_ = std::make_shared<AssociateDebugData>();
  associate_debug_data_ptr_->new_trackers = new_trackers_;
  associate_debug_data_ptr_->stable_trackers = stable_trackers_;
  associate_debug_data_ptr_->lost_trackers = lost_trackers_;
  associate_debug_data_ptr_->new_tracker_lidar_association_result = new_tracker_lidar_association_result_;
  associate_debug_data_ptr_->stable_tracker_lidar_association_result = stable_tracker_lidar_association_result_;
  associate_debug_data_ptr_->lost_tracker_lidar_association_result = lost_tracker_lidar_association_result_;
  associate_debug_data_ptr_->new_tracker_radar_association_result = new_tracker_radar_association_result_;
  associate_debug_data_ptr_->stable_tracker_radar_association_result = stable_tracker_radar_association_result_;
  associate_debug_data_ptr_->lost_tracker_radar_association_result = lost_tracker_radar_association_result_;

  associate_debug_data_ptr_->new_tracker_front_vision_association_result = new_tracker_front_vision_association_result_;
  associate_debug_data_ptr_->stable_tracker_front_vision_association_result =
      stable_tracker_front_vision_association_result_;
  associate_debug_data_ptr_->lost_tracker_front_vision_association_result =
      lost_tracker_front_vision_association_result_;

  associate_debug_data_ptr_->new_tracker_corner_radar_1_association_result =
      new_tracker_corner_radar_1_association_result_;
  associate_debug_data_ptr_->stable_tracker_corner_radar_1_association_result =
      stable_tracker_corner_radar_1_association_result_;
  associate_debug_data_ptr_->lost_tracker_corner_radar_1_association_result =
      lost_tracker_corner_radar_1_association_result_;

  associate_debug_data_ptr_->new_tracker_corner_radar_5_association_result =
      new_tracker_corner_radar_5_association_result_;
  associate_debug_data_ptr_->stable_tracker_corner_radar_5_association_result =
      stable_tracker_corner_radar_5_association_result_;
  associate_debug_data_ptr_->lost_tracker_corner_radar_5_association_result =
      lost_tracker_corner_radar_5_association_result_;

  associate_debug_data_ptr_->new_tracker_corner_radar_7_association_result =
      new_tracker_corner_radar_7_association_result_;
  associate_debug_data_ptr_->stable_tracker_corner_radar_7_association_result =
      stable_tracker_corner_radar_7_association_result_;
  associate_debug_data_ptr_->lost_tracker_corner_radar_7_association_result =
      lost_tracker_corner_radar_7_association_result_;

  associate_debug_data_ptr_->new_tracker_corner_radar_11_association_result =
      new_tracker_corner_radar_11_association_result_;
  associate_debug_data_ptr_->stable_tracker_corner_radar_11_association_result =
      stable_tracker_corner_radar_11_association_result_;
  associate_debug_data_ptr_->lost_tracker_corner_radar_11_association_result =
      lost_tracker_corner_radar_11_association_result_;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
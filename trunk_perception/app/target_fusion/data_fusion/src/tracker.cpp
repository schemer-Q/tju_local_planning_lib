/**
 * @file tracker.cpp
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 跟踪序列
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/app/target_fusion/data_fusion/tracker.h"
#include <Eigen/src/Core/Matrix.h>
#include "trunk_perception/app/target_fusion/data_fusion/kalman_motion_fusion.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/types/fused_object.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

Tracker::Tracker(const FusedObject::Ptr& object_ptr, const MotionFusionConfig& motion_kf_config,
                 const ShapeFusionConfig::ConstPtr& shape_fusion_config,
                 const ExistenceFusionConfig::ConstPtr& existence_fusion_config,
                 const TypeFusionConfig::ConstPtr& type_fusion_config,
                 const LidarMeasureFrame::ConstPtr& lidar_measure_ptr,
                 const ars430::RadarMeasureFrame::ConstPtr& front_radar_measure_ptr)
    : motion_kf_config_(motion_kf_config),
      shape_fusion_config_(shape_fusion_config),
      existence_fusion_config_(existence_fusion_config),
      type_fusion_config_(type_fusion_config),
      object_ptr_(object_ptr),
      motion_fusion_(std::make_shared<KalmanMotionFusion>(motion_kf_config)) {
  if (!object_ptr_) {
    TFATAL << "[Tracker] object_ptr is nullptr";
    return;
  }

  // 初始化motion_fusion_初始状态
  switch (motion_kf_config.motion_model) {
    case MotionModel::CV:
      Eigen::VectorXd x0 = GetStateFromFusedObject();
      if (!motion_fusion_->Init(x0)) {
        TFATAL << "[Tracker] init motion fusion failed!";
      }
      break;
  }

  // 初始化shape_fusion_
  shape_fusion_ = ShapeFusionFactory::Create(shape_fusion_config_);
  if (!shape_fusion_) {
    TFATAL << "[Tracker] create shape fusion failed!";
  }
  shape_fusion_->Init(object_ptr_->size);

  // 初始化existence_fusion_
  existence_fusion_ = ExistenceFusionFactory::Create(existence_fusion_config_);
  if (!existence_fusion_) {
    TFATAL << "[Tracker] create existence fusion failed!";
  }

  // 初始化type_fusion_
  type_fusion_ = TypeFusionFactory::Create(type_fusion_config_);
  if (!type_fusion_) {
    TERROR << "[Tracker] create type fusion failed!";
  }
  type_fusion_->Init(object_ptr_->type);

  object_ptr_->life = 1;

  if (lidar_measure_ptr) {
    object_lidar_ptr_ = lidar_measure_ptr;
    object_ptr_->obj_lidar_ptr_ = lidar_measure_ptr;
    object_ptr_->lidar_total_life = 1;
    object_ptr_->lidar_consecutive_hit = 1;
    object_ptr_->lidar_consecutive_hit_his = object_ptr_->lidar_consecutive_hit;  // @author zzg 2024_12_04
    object_ptr_->lidar_consecutive_hit_his_ts = lidar_measure_ptr->timestamp;     // @author zzg 2024_12_04
  }
  if (front_radar_measure_ptr) {
    object_front_radar_ptr_ = front_radar_measure_ptr;
    object_ptr_->obj_front_radar_ptr_ = front_radar_measure_ptr;
    object_ptr_->front_radar_total_life = 1;
    object_ptr_->front_radar_consecutive_hit = 1;
  }
}

Tracker::~Tracker() {}

void Tracker::Predict(const double& t) {
  double dt = t - object_ptr_->timestamp;
  if (!motion_fusion_->Predict(dt)) {
    TERROR << "[Tracker] predict failed!";
    return;
  }

  UpdateObjectPoseVelocity();

  object_ptr_->timestamp = t;
  object_ptr_->life += 1;
  object_ptr_->flag_special_keep_stable = false;

  if (object_ptr_->lidar_consecutive_lost > 0) object_ptr_->lidar_consecutive_hit = 0;
  object_ptr_->lidar_consecutive_lost += 1;

  if (object_ptr_->front_radar_consecutive_lost > 0) object_ptr_->front_radar_consecutive_hit = 0;
  object_ptr_->front_radar_consecutive_lost += 1;

  if (object_ptr_->front_vision_consecutive_lost > 0) object_ptr_->front_vision_consecutive_hit = 0;
  object_ptr_->front_vision_consecutive_lost += 1;

  if (object_ptr_->side_vision_consecutive_lost > 0) object_ptr_->side_vision_consecutive_hit = 0;
  object_ptr_->side_vision_consecutive_lost += 1;
}

uint32_t Tracker::Update(const std::vector<SensorMeasureFrame::ConstPtr>& measures_list) {
  if (measures_list.empty()) {
    object_ptr_->existence = existence_fusion_->Compute(object_ptr_);
    return ErrorCode::SUCCESS;
  }

  for (const auto& measure_ptr : measures_list) {
    if (!measure_ptr) {
      TERROR << "[Tracker] measure_ptr is nullptr";
      continue;
    }

    if (measure_ptr->sensor_type == MeasureSensorType::Lidar) {
      auto lidar_measure_ptr = std::static_pointer_cast<const LidarMeasureFrame>(measure_ptr);
      if (!lidar_measure_ptr) {
        TERROR << "[Tracker] lidar_measure_ptr is nullptr";
        continue;
      }
      Update(lidar_measure_ptr);
    } else if (measure_ptr->sensor_type == MeasureSensorType::ARS430Radar) {
      auto front_radar_measure_ptr = std::static_pointer_cast<const ars430::RadarMeasureFrame>(measure_ptr);
      if (!front_radar_measure_ptr) {
        TERROR << "[Tracker] front_radar_measure_ptr is nullptr";
        continue;
      }
      Update(front_radar_measure_ptr);
    } else if (measure_ptr->sensor_type == MeasureSensorType::FrontVision) {
      auto front_vision_measure_ptr = std::static_pointer_cast<const VisionMeasureFrame>(measure_ptr);
      if (!front_vision_measure_ptr) {
        TERROR << "[Tracker] front_vision_measure_ptr is nullptr";
        continue;
      }
      Update(front_vision_measure_ptr);
    } else if (measure_ptr->sensor_type == MeasureSensorType::CUBTEKTARRadar) {
      auto corner_radar_ptr = std::static_pointer_cast<const cubtektar::RadarMeasureFrame>(measure_ptr);
      if (!corner_radar_ptr) {
        TERROR << "[Tracker] corner_radar_ptr is nullptr";
        continue;
      }
      Update(corner_radar_ptr);
    } else if (measure_ptr->sensor_type == MeasureSensorType::SideVision) {
      auto side_vision_measure_ptr = std::static_pointer_cast<const SideVisionMeasureFrame>(measure_ptr);
      if (!side_vision_measure_ptr) {
        TERROR << "[Tracker] side_vision_measure_ptr is nullptr";
        continue;
      }
      Update(side_vision_measure_ptr);
    }
  }

  object_ptr_->existence = existence_fusion_->Compute(object_ptr_);

  return ErrorCode::SUCCESS;
}

void Tracker::Update(const LidarMeasureFrame::ConstPtr& lidar_measure_ptr) {
  if (!lidar_measure_ptr) {
    TERROR << "[Tracker] lidar_measure_ptr is nullptr";
    return;
  }

  Eigen::VectorXd z = GetMeasurementFromLidar(lidar_measure_ptr);
  if (!motion_fusion_->Update("Lidar", z)) {
    TERROR << "[Tracker] update motion fusion with lidar measure failed!";
    return;
  }

  if (shape_fusion_->Update(lidar_measure_ptr) != ErrorCode::SUCCESS) {
    TERROR << "[Tracker] update shape fusion with lidar measure failed!";
    return;
  }

  UpdateObjectPoseVelocity();
  UpdateObjectShape();

  object_ptr_->type = lidar_measure_ptr->type;
  if (type_fusion_->Update(lidar_measure_ptr) != ErrorCode::SUCCESS) {
    TERROR << "[Tracker] update type fusion with lidar measure failed!";
  }
  UpdateObjectType();

  // @author zzg 2025-01-13 对于theta角前后帧变化较大的，使用上一帧融合的theta
  // 前后变化绝对值 > 0.4363弧度(25角度) ，或相对变化大于1.5倍
  // VTI-14538，解决 theta 角的突变，后续还需要更加合理的逻辑进行优化
  // 存在情形：远处激光测量一开始检测theta错误，后近处检测theta正确，前后theta相差很大，用 delta_theta_num
  // 标记变化次数，超过三次使用激光测量theta
  float temp_theta = object_ptr_->theta;
  if (std::fabs(lidar_measure_ptr->theta - object_ptr_->theta) > 0.22 &&
      std::fabs((lidar_measure_ptr->theta - object_ptr_->theta) / object_ptr_->theta) > 0.5) {
    // 正常转弯掉头会达到 6.0、1.9，使用激光测量的 theta
    if (std::fabs(lidar_measure_ptr->theta - object_ptr_->theta) > 5.8 &&
        std::fabs((lidar_measure_ptr->theta - object_ptr_->theta) / object_ptr_->theta) > 1.89) {
      object_ptr_->theta = lidar_measure_ptr->theta;
      object_ptr_->delta_theta_num = 0;
    } else {
      if (object_ptr_->delta_theta_num > 5) {
        object_ptr_->theta = lidar_measure_ptr->theta;
        object_ptr_->delta_theta_num = 0;
      } else {
        object_ptr_->delta_theta_num += 1;
        object_ptr_->theta = temp_theta;
      }
    }
  } else {
    object_ptr_->theta = lidar_measure_ptr->theta;
    object_ptr_->delta_theta_num = 0;
  }

  object_ptr_->confidence = lidar_measure_ptr->confidence;

  object_ptr_->lidar_consecutive_lost = 0;
  object_ptr_->lidar_total_life += 1;
  object_ptr_->lidar_consecutive_hit += 1;
  object_ptr_->lidar_consecutive_hit_his = object_ptr_->lidar_consecutive_hit;  // @author zzg 2024_12_04
  object_ptr_->lidar_consecutive_hit_his_ts = object_ptr_->timestamp;           // @author zzg 2024_12_04
  object_lidar_ptr_ = lidar_measure_ptr;
  object_ptr_->obj_lidar_ptr_ = lidar_measure_ptr;
}

void Tracker::Update(const ars430::RadarMeasureFrame::ConstPtr& front_radar_measure_ptr) {
  if (!front_radar_measure_ptr) {
    TERROR << "[Tracker] front_radar_measure_ptr is nullptr";
    return;
  }

  object_ptr_->front_radar_consecutive_lost = 0;
  object_ptr_->front_radar_total_life += 1;
  object_ptr_->front_radar_consecutive_hit += 1;
  object_front_radar_ptr_ = front_radar_measure_ptr;
  object_ptr_->obj_front_radar_ptr_ = front_radar_measure_ptr;

  Eigen::Vector2d radar_position =
      Eigen::Vector2d(front_radar_measure_ptr->radar_distance2d.x(), front_radar_measure_ptr->radar_distance2d.y());
  // 对于目标处在毫米波坐标系原点后的（毫米波坐标系下，纵向位置 < 0.1 的），不使用毫米波进行运动属性更新
  if (radar_position.x() < 0.1) {
    return;
  }
  // 对于近处毫米波目标，位置在左右侧的，暂不确定其反射的位置点是车的后中心点还是车的侧边点，暂不使用其位置、速度做运动属性更新
  if (radar_position.x() < 15 && (radar_position.y() / std::fabs(radar_position.x())) > sqrt(3)) {
    return;
  }

  Eigen::VectorXd z = GetMeasurementFromFrontRadar(front_radar_measure_ptr);

  // 此时因为只有radar观测，需要需要加快radar的位置收敛速度
  if (object_ptr_->flag_special_keep_stable) {
    Eigen::MatrixXd R = motion_kf_config_.sensor_R.at("Radar0");
    R(0, 0) = 1.0;
    R(1, 1) = 1.0;
    motion_fusion_->SetSensorR("Radar0", R);
  } else {
    motion_fusion_->SetSensorR("Radar0", motion_kf_config_.sensor_R.at("Radar0"));
  }

  // @author zzg 2025-01-13 解决盲区内，无激光测量后，只由前向毫米波测量维持的融合目标入侵自车车道
  Eigen::Matrix4d local_to_car = object_ptr_->odo_lidar_ptr->Matrix().inverse();
  Eigen::Vector4d center_vec(object_ptr_->center.x(), object_ptr_->center.y(), object_ptr_->center.z(), 1.0);
  center_vec = local_to_car * center_vec;
  Eigen::Vector3d temp_center = Eigen::Vector3d(center_vec.x(), center_vec.y(), center_vec.z());
  if (std::fabs(temp_center.x()) < 15 && std::fabs(temp_center.y()) < 15) {
    Eigen::MatrixXd R = motion_kf_config_.sensor_R.at("Radar0");
    R(0, 0) = 100.0;
    R(1, 1) = 100.0;
    R(0, 0) = 100.0;
    R(1, 1) = 100.0;
    motion_fusion_->SetSensorR("Radar0", R);
  }

  if (!motion_fusion_->Update("Radar0", z)) {
    TERROR << "[Tracker] update motion fusion with front radar measure failed!";
    return;
  }

  UpdateObjectPoseVelocity();
}

void Tracker::Update(const VisionMeasureFrame::ConstPtr& front_vision_measure_ptr) {
  if (!front_vision_measure_ptr) {
    TERROR << "[Tracker] front_vision_measure_ptr is nullptr";
    return;
  }
  // @author zzg 2024-12-30 暂时不使用 前向视觉 对 运动属性(位置、速度)做更新
  // Eigen::VectorXd z = GetMeasurementFromFrontVision(front_vision_measure_ptr);

  object_ptr_->type = front_vision_measure_ptr->type;
  if (type_fusion_->Update(front_vision_measure_ptr) != ErrorCode::SUCCESS) {
    TERROR << "[Tracker] update type fusion with front vision measure failed!";
  }
  UpdateObjectType();

  object_ptr_->front_vision_consecutive_lost = 0;
  object_ptr_->front_vision_total_life += 1;
  object_ptr_->front_vision_consecutive_hit += 1;
  object_front_vision_ptr_ = front_vision_measure_ptr;
  object_ptr_->obj_front_vision_ptr_ = front_vision_measure_ptr;
}

void Tracker::Update(const SideVisionMeasureFrame::ConstPtr& side_vision_measure_ptr) {
  if (!side_vision_measure_ptr) {
    TERROR << "[Tracker] side_vision_measure_ptr is nullptr";
    return;
  }
  object_ptr_->side_vision_consecutive_lost = 0;
  object_ptr_->side_vision_total_life += 1;
  object_ptr_->side_vision_consecutive_hit += 1;
  object_side_vision_ptr_ = side_vision_measure_ptr;
  object_ptr_->obj_side_vision_ptr_ = side_vision_measure_ptr;

  object_ptr_->type = side_vision_measure_ptr->type;
  if (type_fusion_->Update(side_vision_measure_ptr) != ErrorCode::SUCCESS) {
    TERROR << "[Tracker] update type fusion with side vision measure failed!";
  }
  UpdateObjectType();

  // VTI-14761 暂不使用环视视觉做运动属性更新：VTI-14761 环视位置不对
  // @author zzg 2025-01-15 使用 环视视觉 对 运动属性(位置、速度)做更新
  // 限制 对 运动属性(位置、速度)做更新 的目标位置
  // Eigen::Vector2d car_center =
  //     Eigen::Vector2d(side_vision_measure_ptr->car_center.x(), side_vision_measure_ptr->car_center.y());
  // if (std::fabs(car_center(0)) > 30 && std::fabs(car_center(1)) > 8) {
  //   return;
  // }
  // Eigen::VectorXd z = GetMeasurementFromSideVision(side_vision_measure_ptr);
  // if (!motion_fusion_->Update("SideVision0", z)) {
  //   TERROR << "[Tracker] update motion fusion with side vision measure failed!";
  //   return;
  // }
  // UpdateObjectPoseVelocity();
}

void Tracker::Update(const cubtektar::RadarMeasureFrame::ConstPtr& corner_radar_measure_ptr) {
  if (!corner_radar_measure_ptr) {
    TERROR << "[Tracker] corner_radar_measure_ptr is nullptr";
    return;
  }

  if (corner_radar_measure_ptr->corner_radar_name_ == CornerRadarName::RADAR1) {
    object_corner_radar1_ptr_ = corner_radar_measure_ptr;
    object_ptr_->obj_corner_radar1_ptr_ = corner_radar_measure_ptr;
    object_ptr_->corner_radar1_consecutive_lost = 0;
    object_ptr_->corner_radar1_consecutive_hit += 1;
    object_ptr_->corner_radar1_total_life += 1;
  } else if (corner_radar_measure_ptr->corner_radar_name_ == CornerRadarName::RADAR5) {
    object_corner_radar5_ptr_ = corner_radar_measure_ptr;
    object_ptr_->obj_corner_radar5_ptr_ = corner_radar_measure_ptr;
    object_ptr_->corner_radar5_consecutive_lost = 0;
    object_ptr_->corner_radar5_consecutive_hit += 1;
    object_ptr_->corner_radar5_total_life += 1;
  } else if (corner_radar_measure_ptr->corner_radar_name_ == CornerRadarName::RADAR7) {
    object_corner_radar7_ptr_ = corner_radar_measure_ptr;
    object_ptr_->obj_corner_radar7_ptr_ = corner_radar_measure_ptr;
    object_ptr_->corner_radar7_consecutive_lost = 0;
    object_ptr_->corner_radar7_consecutive_hit += 1;
    object_ptr_->corner_radar7_total_life += 1;
  } else if (corner_radar_measure_ptr->corner_radar_name_ == CornerRadarName::RADAR11) {
    object_corner_radar11_ptr_ = corner_radar_measure_ptr;
    object_ptr_->obj_corner_radar11_ptr_ = corner_radar_measure_ptr;
    object_ptr_->corner_radar11_consecutive_lost = 0;
    object_ptr_->corner_radar11_consecutive_hit += 1;
    object_ptr_->corner_radar11_total_life += 1;
  }

  // 暂粗略划定角毫米波使用区域，对于不在区域内的角毫米波目标，不更新融合目标的运动属性
  Eigen::Vector2d corner_radar_position =
      Eigen::Vector2d(corner_radar_measure_ptr->radar_obj.x, corner_radar_measure_ptr->radar_obj.y);
  // 该区域外不使用角毫米波目标更新融合目标的运动属性
  if ((corner_radar_position.x() < -35) || (corner_radar_position.x() > 30) ||
      (std::fabs(corner_radar_position.y()) < 1.25)) {
    return;
  }
  // @author zzg 2025-01-23 此处先做限制，后续调优
  if ((corner_radar_position.x() > -20 && corner_radar_position.x() < 15) &&
      (std::fabs(corner_radar_position.y()) < 1.6)) {
    return;
  }
  // 该区域可以使用角毫米波目标更新融合目标的运动属性，但反射点是在目标车辆的侧边，需要修改后使用，后续修改
  // 目前该区域不使用角毫米波目标更新融合目标的运动属性
  if ((corner_radar_position.x() < 10) && (corner_radar_position.x() > -8)) {
    return;
  }
  // 其它区域可以认为角毫米波反射点是在目标车辆后中点，可以用于融合目标的运动属性更新
  Eigen::VectorXd z = GetMeasurementFromCornerRadar(corner_radar_measure_ptr);
  // 设置角毫米波的测量噪声
  Eigen::MatrixXd R = motion_kf_config_.sensor_R.at("Radar0");
  R(0, 0) = 20.0;
  R(1, 1) = 100.0;
  R(2, 2) = 0.6;
  R(3, 3) = 10.0;
  motion_fusion_->SetSensorR("Radar0", R);

  if (!motion_fusion_->Update("Radar0", z)) {
    TERROR << "[Tracker] update motion fusion with side radar measure failed!";
    return;
  }

  UpdateObjectPoseVelocity();
}

Eigen::VectorXd Tracker::GetStateFromFusedObject() {
  switch (motion_kf_config_.motion_model) {
    case MotionModel::CV:
      Eigen::Vector4d x0;
      x0 << object_ptr_->track_point.x(), object_ptr_->track_point.y(), object_ptr_->velocity.x(),
          object_ptr_->velocity.y();
      return x0;
  }
  return Eigen::VectorXd();
}

Eigen::VectorXd Tracker::GetMeasurementFromLidar(const LidarMeasureFrame::ConstPtr& lidar_measure_ptr) {
  switch (motion_kf_config_.motion_model) {
    case MotionModel::CV:
      Eigen::Vector4d z;
      if (object_ptr_->track_point_type == TrackPointType::RearMiddle) {
        z << lidar_measure_ptr->rear_middle_point.x(), lidar_measure_ptr->rear_middle_point.y(),
            lidar_measure_ptr->velocity.x(), lidar_measure_ptr->velocity.y();
      } else if (object_ptr_->track_point_type == TrackPointType::Center) {
        z << lidar_measure_ptr->center.x(), lidar_measure_ptr->center.y(), lidar_measure_ptr->velocity.x(),
            lidar_measure_ptr->velocity.y();
      } else {
        TERROR << "[Tracker] GetMeasurementFromLidar track_point_type is not implemented!";
        return Eigen::VectorXd();
      }
      return z;
  }
  return Eigen::VectorXd();
}

Eigen::VectorXd Tracker::GetMeasurementFromFrontRadar(
    const ars430::RadarMeasureFrame::ConstPtr& front_radar_measure_ptr) {
  switch (motion_kf_config_.motion_model) {
    case MotionModel::CV:
      Eigen::Vector4d z;
      if (object_ptr_->track_point_type == TrackPointType::RearMiddle) {
        z << front_radar_measure_ptr->local_distance2d.x(), front_radar_measure_ptr->local_distance2d.y(),
            front_radar_measure_ptr->local_velocity2d.x(), front_radar_measure_ptr->local_velocity2d.y();
      } else if (object_ptr_->track_point_type == TrackPointType::Center) {
        double cos_yaw = std::cos(object_ptr_->theta);
        double sin_yaw = std::sin(object_ptr_->theta);
        double center_x = front_radar_measure_ptr->local_distance2d.x() + cos_yaw * object_ptr_->size.x() / 2.0;
        double center_y = front_radar_measure_ptr->local_distance2d.y() + sin_yaw * object_ptr_->size.x() / 2.0;
        z << center_x, center_y, front_radar_measure_ptr->local_velocity2d.x(),
            front_radar_measure_ptr->local_velocity2d.y();
      } else {
        TERROR << "[Tracker] GetMeasurementFromFrontRadar track_point_type is not implemented!";
        return Eigen::VectorXd();
      }

      return z;
  }
  return Eigen::VectorXd();
}

Eigen::VectorXd Tracker::GetMeasurementFromCornerRadar(
    const cubtektar::RadarMeasureFrame::ConstPtr& corner_radar_measure_ptr) {
  switch (motion_kf_config_.motion_model) {
    case MotionModel::CV:
      Eigen::Vector4d z;
      if (object_ptr_->track_point_type == TrackPointType::RearMiddle) {
        z << corner_radar_measure_ptr->local_distance2d.x(), corner_radar_measure_ptr->local_distance2d.y(),
            corner_radar_measure_ptr->local_velocity2d.x(), corner_radar_measure_ptr->local_velocity2d.y();
      } else if (object_ptr_->track_point_type == TrackPointType::Center) {
        double cos_yaw = std::cos(object_ptr_->theta);
        double sin_yaw = std::sin(object_ptr_->theta);
        double center_x = corner_radar_measure_ptr->local_distance2d.x() + cos_yaw * object_ptr_->size.x() / 2.0;
        double center_y = corner_radar_measure_ptr->local_distance2d.y() + sin_yaw * object_ptr_->size.x() / 2.0;
        z << center_x, center_y, corner_radar_measure_ptr->local_velocity2d.x(),
            corner_radar_measure_ptr->local_velocity2d.y();
      } else {
        TERROR << "[Tracker] GetMeasurementFromFrontRadar track_point_type is not implemented!";
        return Eigen::VectorXd();
      }
      return z;
  }
  return Eigen::VectorXd();
}

Eigen::VectorXd Tracker::GetMeasurementFromFrontVision(const VisionMeasureFrame::ConstPtr& front_vision_measure_ptr) {
  switch (motion_kf_config_.motion_model) {
    case MotionModel::CV:
      Eigen::Vector4d z;
      if (object_ptr_->track_point_type == TrackPointType::RearMiddle) {
        z << front_vision_measure_ptr->rear_middle_point.x(), front_vision_measure_ptr->rear_middle_point.y(),
            front_vision_measure_ptr->velocity.x(), front_vision_measure_ptr->velocity.y();
      } else if (object_ptr_->track_point_type == TrackPointType::Center) {
        z << front_vision_measure_ptr->center.x(), front_vision_measure_ptr->center.y(),
            front_vision_measure_ptr->velocity.x(), front_vision_measure_ptr->velocity.y();
      } else {
        TERROR << "[Tracker] GetMeasurementFromFrontVision track_point_type is not implemented!";
        return Eigen::VectorXd();
      }
      return z;
  }
  return Eigen::VectorXd();
}

Eigen::VectorXd Tracker::GetMeasurementFromSideVision(const SideVisionMeasureFrame::ConstPtr& side_vision_measure_ptr) {
  switch (motion_kf_config_.motion_model) {
    case MotionModel::CV:
      Eigen::Vector4d z;
      if (object_ptr_->track_point_type == TrackPointType::RearMiddle) {
        z << side_vision_measure_ptr->rear_middle_point.x(), side_vision_measure_ptr->rear_middle_point.y(),
            side_vision_measure_ptr->velocity.x(), side_vision_measure_ptr->velocity.y();
      } else if (object_ptr_->track_point_type == TrackPointType::Center) {
        z << side_vision_measure_ptr->center.x(), side_vision_measure_ptr->center.y(),
            side_vision_measure_ptr->velocity.x(), side_vision_measure_ptr->velocity.y();
      } else {
        TERROR << "[Tracker] GetMeasurementFromSideVision track_point_type is not implemented!";
        return Eigen::VectorXd();
      }
      return z;
  }
  return Eigen::VectorXd();
}

void Tracker::UpdateObjectPoseVelocity() {
  // TODO: 用的center，没有实现rear_middle
  Eigen::VectorXd state = motion_fusion_->GetState();
  object_ptr_->center.x() = state(0);
  object_ptr_->center.y() = state(1);
  object_ptr_->velocity.x() = state(2);
  object_ptr_->velocity.y() = state(3);

  double cos_yaw = std::cos(object_ptr_->theta);
  double sin_yaw = std::sin(object_ptr_->theta);
  object_ptr_->rear_middle_point = object_ptr_->center + Eigen::Vector3d(-cos_yaw * object_ptr_->size.x() / 2.0,
                                                                         -sin_yaw * object_ptr_->size.x() / 2.0, 0.0);
  object_ptr_->InitTrackPoint();
}

void Tracker::UpdateObjectShape() {
  Eigen::Vector3f shape = shape_fusion_->GetFusedSize();
  object_ptr_->size = shape;
}

void Tracker::UpdateObjectType() {
  ObjectType type = type_fusion_->GetFusedType();
  object_ptr_->type = type;
}

LidarMeasureFrame::ConstPtr Tracker::GetLidarObject() const { return object_lidar_ptr_; }

ars430::RadarMeasureFrame::ConstPtr Tracker::GetFrontRadarObject() const { return object_front_radar_ptr_; }

VisionMeasureFrame::ConstPtr Tracker::GetFrontVisionObject() const { return object_front_vision_ptr_; }

FusedObject::ConstPtr Tracker::GetFusedObject() const { return object_ptr_; }

void Tracker::SetFusedObjectOdometry(Odometry::Ptr odo_ptr) { object_ptr_->odo_lidar_ptr = odo_ptr; };

Odometry::ConstPtr Tracker::GetFusedObjectOdometry() const { return object_ptr_->odo_lidar_ptr; };

int Tracker::GetTrackID() const {
  if (!object_ptr_) {
    TERROR << "[Tracker] object_ptr_ is nullptr";
    return -1;
  }
  return object_ptr_->track_id;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END

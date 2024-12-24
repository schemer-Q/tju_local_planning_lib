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
                 const LidarMeasureFrame::ConstPtr& lidar_measure_ptr,
                 const ars430::RadarMeasureFrame::ConstPtr& front_radar_measure_ptr)
    : motion_kf_config_(motion_kf_config),
      shape_fusion_config_(shape_fusion_config),
      existence_fusion_config_(existence_fusion_config),
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

  object_ptr_->life = 1;

  if (lidar_measure_ptr) {
    object_lidar_ptr_ = lidar_measure_ptr;
		object_ptr_->obj_lidar_ptr_ = lidar_measure_ptr;
    object_ptr_->lidar_total_life = 1;
    object_ptr_->lidar_consecutive_hit = 1;
		object_ptr_->lidar_consecutive_hit_his = object_ptr_->lidar_consecutive_hit;      // @author zzg 2024_12_04
		object_ptr_->lidar_consecutive_hit_his_ts = lidar_measure_ptr->timestamp;         // @author zzg 2024_12_04
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

  object_ptr_->theta = lidar_measure_ptr->theta;
  object_ptr_->type = lidar_measure_ptr->type;
  object_ptr_->confidence = lidar_measure_ptr->confidence;
  
  object_ptr_->lidar_consecutive_lost = 0;
  object_ptr_->lidar_total_life += 1;
  object_ptr_->lidar_consecutive_hit += 1;
	object_ptr_->lidar_consecutive_hit_his = object_ptr_->lidar_consecutive_hit;          // @author zzg 2024_12_04
	object_ptr_->lidar_consecutive_hit_his_ts = object_ptr_->timestamp;                   // @author zzg 2024_12_04
  object_lidar_ptr_ = lidar_measure_ptr;
	object_ptr_->obj_lidar_ptr_ = lidar_measure_ptr;
}

void Tracker::Update(const ars430::RadarMeasureFrame::ConstPtr& front_radar_measure_ptr) {
  if (!front_radar_measure_ptr) {
    TERROR << "[Tracker] front_radar_measure_ptr is nullptr";
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

  if (!motion_fusion_->Update("Radar0", z)) {
    TERROR << "[Tracker] update motion fusion with front radar measure failed!";
    return;
  }

  UpdateObjectPoseVelocity();

  object_ptr_->front_radar_consecutive_lost = 0;
  object_ptr_->front_radar_total_life += 1;
  object_ptr_->front_radar_consecutive_hit += 1;
  object_front_radar_ptr_ = front_radar_measure_ptr;
	object_ptr_->obj_front_radar_ptr_ = front_radar_measure_ptr;
}

void Tracker::Update(const VisionMeasureFrame::ConstPtr& front_vision_measure_ptr) {
	if (!front_vision_measure_ptr) {
		TERROR << "[Tracker] front_vision_measure_ptr is nullptr";
		return;
	}
	Eigen::VectorXd z = GetMeasurementFromFrontVision(front_vision_measure_ptr);

  object_ptr_->type = front_vision_measure_ptr->type;
  
  object_ptr_->front_vision_consecutive_lost = 0;
  object_ptr_->front_vision_total_life += 1;
  object_ptr_->front_vision_consecutive_hit += 1;
  object_front_vision_ptr_ = front_vision_measure_ptr;
	object_ptr_->obj_front_vision_ptr_ = front_vision_measure_ptr;
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

Eigen::VectorXd Tracker::GetMeasurementFromFrontVision(
		const VisionMeasureFrame::ConstPtr& front_vision_measure_ptr) {
	switch (motion_kf_config_.motion_model) {
		case MotionModel::CV:
			Eigen::Vector4d z;
      if (object_ptr_->track_point_type == TrackPointType::RearMiddle) {
        z << front_vision_measure_ptr->rear_middle_point.x(), front_vision_measure_ptr->rear_middle_point.y(),
            front_vision_measure_ptr->velocity.x(), front_vision_measure_ptr->velocity.y();
      } else if (object_ptr_->track_point_type == TrackPointType::Center) {
        z << front_vision_measure_ptr->center.x(), front_vision_measure_ptr->center.y(), front_vision_measure_ptr->velocity.x(),
            front_vision_measure_ptr->velocity.y();
      } else {
        TERROR << "[Tracker] GetMeasurementFromFrontVision track_point_type is not implemented!";
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

LidarMeasureFrame::ConstPtr Tracker::GetLidarObject() const { return object_lidar_ptr_; }

ars430::RadarMeasureFrame::ConstPtr Tracker::GetFrontRadarObject() const { return object_front_radar_ptr_; }

VisionMeasureFrame::ConstPtr Tracker::GetFrontVisionObject() const { return object_front_vision_ptr_; }

FusedObject::ConstPtr Tracker::GetFusedObject() const { return object_ptr_; }

int Tracker::GetTrackID() const {
  if (!object_ptr_) {
    TERROR << "[Tracker] object_ptr_ is nullptr";
    return -1;
  }
  return object_ptr_->track_id;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END

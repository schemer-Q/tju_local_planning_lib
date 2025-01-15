#include "trunk_perception/app/target_fusion/data_associate/track_object_distance.h"
#include <Eigen/src/Core/Matrix.h>
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

uint32_t TrackObjectDistance::Init(const YAML::Node& config) {
  try {
    if (config["Position"].IsDefined()) {
      use_position_ = config["Position"]["Switch"].as<bool>();
      if (use_position_) {
        position_weight_ = config["Position"]["Weight"].as<float>();
        position_type_ = config["Position"]["Type"].as<std::string>();
        position_point_ = config["Position"]["Point"].as<std::string>();
        if (std::find(position_type_list_.begin(), position_type_list_.end(), position_type_) ==
            position_type_list_.end()) {
          TFATAL << "TrackObjectDistance::Init failed, position_type is invalid";
          return ErrorCode::YAML_CONFIG_ERROR;
        }
        if (std::find(position_point_list_.begin(), position_point_list_.end(), position_point_) ==
            position_point_list_.end()) {
          TFATAL << "TrackObjectDistance::Init failed, position_point is invalid";
          return ErrorCode::YAML_CONFIG_ERROR;
        }
      }
    }

    if (config["Velocity"].IsDefined()) {
      use_velocity_ = config["Velocity"]["Switch"].as<bool>();
      if (use_velocity_) velocity_weight_ = config["Velocity"]["Weight"].as<float>();
    }

    if (config["Filter"].IsDefined()) {
      if (config["Filter"]["VelocityThresh"].IsDefined()) {
        use_velocity_filter_ = config["Filter"]["VelocityThresh"]["Switch"].as<bool>();
        velocity_filter_abs_thresh_ = config["Filter"]["VelocityThresh"]["Abs"].as<float>();
        velocity_filter_rel_thresh_ = config["Filter"]["VelocityThresh"]["Rel"].as<float>();
      }
      if (config["Filter"]["PositionThresh"].IsDefined()) {
        use_position_filter_ = config["Filter"]["PositionThresh"]["Switch"].as<bool>();
        position_filter_orthogonal_thresh_ = config["Filter"]["PositionThresh"]["Orthogonal"].as<float>();
        position_filter_longitudinal_thresh_ = config["Filter"]["PositionThresh"]["Longitudinal"].as<float>();
      }
    }
  } catch (const std::exception& e) {
    TFATAL << "TrackObjectDistance::Init failed, " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }

  if (!use_position_ && !use_velocity_) {
    TFATAL << "TrackObjectDistance::Init failed, use_position_ and use_velocity_ are both false";
    return ErrorCode::YAML_CONFIG_ERROR;
  }

  return ErrorCode::SUCCESS;
}

float TrackObjectDistance::Compute(const TrackerPtr& tracker_ptr, const LidarMeasureFrame::ConstPtr& lidar_object) {
  float distance = std::numeric_limits<float>::max();
  if (!tracker_ptr || !lidar_object) {
    TERROR << "TrackObjectDistance::Compute input is nullptr";
    return distance;
  }

  FusedObject::ConstPtr fused_obj = tracker_ptr->GetFusedObject();

  // filter
  if (use_velocity_filter_) {
    float velocity_rel_distance = Compute2DRelEuclideanDistance(fused_obj->velocity, lidar_object->velocity);
    float velocity_abs_distance = Compute2DEuclideanDistance(fused_obj->velocity, lidar_object->velocity);
    if (velocity_rel_distance > velocity_filter_rel_thresh_ && velocity_abs_distance > velocity_filter_abs_thresh_) {
      return distance;
    }
  }

  if (use_position_filter_) {
    Eigen::Vector2f position_distance =
        ComputeOrthogonalDistance(fused_obj->center, lidar_object->center, lidar_object->theta);
    if (position_distance.x() > position_filter_orthogonal_thresh_ ||
        position_distance.y() > position_filter_longitudinal_thresh_) {
      return distance;
    }
  }

  // 2025-01-12 若激光测量类型与融合目标类型差距过大，不允许匹配上
  bool use_type_filter_ = true;
  if (use_type_filter_) {
    ObjectType f_type = fused_obj->type;
    ObjectType l_type = lidar_object->type;
    if (f_type != l_type) {
      if ((f_type == ObjectType::PEDESTRIAN || f_type == ObjectType::CONE || f_type == ObjectType::BARREL) &&
          !(l_type == ObjectType::PEDESTRIAN || l_type == ObjectType::CONE || l_type != ObjectType::BARREL)) {
        return distance;
      }
      if ((l_type == ObjectType::PEDESTRIAN || l_type == ObjectType::CONE || l_type == ObjectType::BARREL) &&
          !(f_type == ObjectType::PEDESTRIAN || f_type == ObjectType::CONE || f_type == ObjectType::BARREL)) {
        return distance;
      }
    }
  }

  // 2025-01-12 若激光测量size与融合目标size差距过大，不允许匹配上
  bool use_size_filter_ = true;
  if (use_size_filter_) {
    Eigen::Vector3f f_size = fused_obj->size;
    Eigen::Vector3f l_size = lidar_object->size;
    if ((f_size[0] / l_size[0] > 5) || (f_size[1] / l_size[1] > 5) || (f_size[2] / l_size[2] > 5)) {
      return distance;
    }
  }

  // cal distance
  float position_distance = 0.0;
  if (use_position_) {
    if (position_type_ == "Orthogonal") {
      if (position_point_ == "Center") {
        auto dis_2d = ComputeOrthogonalDistance(fused_obj->center, lidar_object->center, lidar_object->theta);
        position_distance = dis_2d.x();
      } else if (position_point_ == "RearMiddle") {
        auto dis_2d = ComputeOrthogonalDistance(fused_obj->rear_middle_point, lidar_object->rear_middle_point,
                                                lidar_object->theta);
        position_distance = dis_2d.x();
      } else if (position_point_ == "Mix_CR") {
        auto dis_center = ComputeOrthogonalDistance(fused_obj->center, lidar_object->center, lidar_object->theta);
        auto dis_rear_middle = ComputeOrthogonalDistance(fused_obj->rear_middle_point, lidar_object->rear_middle_point,
                                                         lidar_object->theta);
        position_distance = std::min(dis_center.x(), dis_rear_middle.x());
      }
    } else if (position_type_ == "Euclidean") {
      if (position_point_ == "Center") {
        position_distance = Compute2DEuclideanDistance(fused_obj->center, lidar_object->center);
      } else if (position_point_ == "RearMiddle") {
        position_distance = Compute2DEuclideanDistance(fused_obj->rear_middle_point, lidar_object->rear_middle_point);
      } else if (position_point_ == "Mix_CR") {
        auto dis_center = Compute2DEuclideanDistance(fused_obj->center, lidar_object->center);
        auto dis_rear_middle =
            Compute2DEuclideanDistance(fused_obj->rear_middle_point, lidar_object->rear_middle_point);
        position_distance = std::min(dis_center, dis_rear_middle);
      }
    }
  }

  float velocity_distance = 0.0;
  if (use_velocity_) {
    velocity_distance = Compute2DEuclideanDistance(fused_obj->velocity, lidar_object->velocity);
  }

  distance = position_distance * position_weight_ + velocity_distance * velocity_weight_;

  return distance;
}

float TrackObjectDistance::Compute(const TrackerPtr& tracker_ptr,
                                   const ars430::RadarMeasureFrame::ConstPtr& front_radar_object) {
  float distance = std::numeric_limits<float>::max();
  if (!tracker_ptr || !front_radar_object) {
    TERROR << "TrackObjectDistance::Compute input is nullptr";
    return distance;
  }

  FusedObject::ConstPtr fused_obj = tracker_ptr->GetFusedObject();
  Eigen::Vector3d radar_obj_pos(front_radar_object->local_distance2d.x(), front_radar_object->local_distance2d.y(), 0);
  Eigen::Vector3f radar_obj_velocity(front_radar_object->local_velocity2d.x(), front_radar_object->local_velocity2d.y(),
                                     0);

  // filter
  if (use_velocity_filter_) {
    float velocity_rel_distance = Compute2DRelEuclideanDistance(fused_obj->velocity, radar_obj_velocity);
    float velocity_abs_distance = Compute2DEuclideanDistance(fused_obj->velocity, radar_obj_velocity);
    if (velocity_rel_distance > velocity_filter_rel_thresh_ && velocity_abs_distance > velocity_filter_abs_thresh_) {
      return distance;
    }
  }

  if (use_position_filter_) {
    Eigen::Vector2f position_distance =
        ComputeOrthogonalDistance(fused_obj->rear_middle_point, radar_obj_pos, fused_obj->theta);
    if (position_distance.x() > position_filter_orthogonal_thresh_ ||
        position_distance.y() > position_filter_longitudinal_thresh_) {
      return distance;
    }
  }

  // cal distance
  float position_distance = 0.0;
  if (use_position_) {
    // position_distance = ComputeOrthogonalDistance(fused_obj->rear_middle_point, radar_obj_pos, fused_obj->theta).x();
    // @author zzg 修改距离计算方式 为 Compute2DEuclideanDistance
    position_distance = Compute2DEuclideanDistance(fused_obj->rear_middle_point, radar_obj_pos);
  }

  float velocity_distance = 0.0;
  if (use_velocity_) {
    velocity_distance = Compute2DEuclideanDistance(fused_obj->velocity, radar_obj_velocity);
  }

  distance = position_distance * position_weight_ + velocity_distance * velocity_weight_;

  return distance;
}

// 计算 前向视觉目标 与 航迹目标 的匹配距离
float TrackObjectDistance::Compute(const TrackerPtr& tracker_ptr,
                                   const VisionMeasureFrame::ConstPtr& front_vision_object) {
  float distance = std::numeric_limits<float>::max();
  if (!tracker_ptr || !front_vision_object) {
    TERROR << "TrackObjectDistance::Compute input is pullptr";
    return distance;
  }

  FusedObject::ConstPtr fused_object = tracker_ptr->GetFusedObject();

  // filter
  if (use_velocity_filter_) {
    float velocity_rel_distance = Compute2DRelEuclideanDistance(fused_object->velocity, front_vision_object->velocity);
    float velocity_abs_distance = Compute2DEuclideanDistance(fused_object->velocity, front_vision_object->velocity);
    if (velocity_rel_distance > velocity_filter_rel_thresh_ && velocity_abs_distance > velocity_filter_abs_thresh_) {
      return distance;
    }
  }

  if (use_position_filter_) {
    Eigen::Vector2f position_distance =
        ComputeOrthogonalDistance(fused_object->center, front_vision_object->center, front_vision_object->theta);
    if (position_distance.x() > position_filter_orthogonal_thresh_ ||
        position_distance.y() > position_filter_longitudinal_thresh_) {
      return distance;
    }
  }

  // cal distance
  float position_distance = 0.0;
  if (use_position_) {
    if (position_type_ == "Orthogonal") {
      if (position_point_ == "Center") {
        auto dis_2d =
            ComputeOrthogonalDistance(fused_object->center, front_vision_object->center, front_vision_object->theta);
        position_distance = dis_2d.x();
      } else if (position_point_ == "RearMiddle") {
        auto dis_2d = ComputeOrthogonalDistance(fused_object->rear_middle_point, front_vision_object->rear_middle_point,
                                                front_vision_object->theta);
        position_distance = dis_2d.x();
      } else if (position_point_ == "Mix_CR") {
        auto dis_center =
            ComputeOrthogonalDistance(fused_object->center, front_vision_object->center, front_vision_object->theta);
        auto dis_rear_middle = ComputeOrthogonalDistance(
            fused_object->rear_middle_point, front_vision_object->rear_middle_point, front_vision_object->theta);
        position_distance = std::min(dis_center.x(), dis_rear_middle.x());
      }
    } else if (position_type_ == "Euclidean") {
      if (position_point_ == "Center") {
        position_distance = Compute2DEuclideanDistance(fused_object->center, front_vision_object->center);
      } else if (position_point_ == "RearMiddle") {
        position_distance =
            Compute2DEuclideanDistance(fused_object->rear_middle_point, front_vision_object->rear_middle_point);
      } else if (position_point_ == "Mix_CR") {
        auto dis_center = Compute2DEuclideanDistance(fused_object->center, front_vision_object->center);
        auto dis_rear_middle =
            Compute2DEuclideanDistance(fused_object->rear_middle_point, front_vision_object->rear_middle_point);
        position_distance = std::min(dis_center, dis_rear_middle);
      }
    }
  }

  float velocity_distance = 0.0;
  if (use_velocity_) {
    velocity_distance = Compute2DEuclideanDistance(fused_object->velocity, front_vision_object->velocity);
  }

  distance = position_distance * position_weight_ + velocity_distance * velocity_weight_;

  return distance;
}

float TrackObjectDistance::Compute(const TrackerPtr& tracker_ptr,
                                   const cubtektar::RadarMeasureFrame::ConstPtr& corner_radar_object) {
  float distance = std::numeric_limits<float>::max();
  if (!tracker_ptr || !corner_radar_object) {
    TERROR << "TrackObjectDistance::Compute input is nullptr";
    return distance;
  }

  FusedObject::ConstPtr fused_obj = tracker_ptr->GetFusedObject();
  Eigen::Vector3d radar_obj_pos(corner_radar_object->local_distance2d.x(), corner_radar_object->local_distance2d.y(),
                                0);
  Eigen::Vector3f radar_obj_velocity(corner_radar_object->local_velocity2d.x(),
                                     corner_radar_object->local_velocity2d.y(), 0);

  // filter
  if (use_velocity_filter_) {
    float velocity_rel_distance = Compute2DRelEuclideanDistance(fused_obj->velocity, radar_obj_velocity);
    float velocity_abs_distance = Compute2DEuclideanDistance(fused_obj->velocity, radar_obj_velocity);
    if (velocity_rel_distance > velocity_filter_rel_thresh_ && velocity_abs_distance > velocity_filter_abs_thresh_) {
      return distance;
    }
  }

  if (use_position_filter_) {
    Eigen::Vector2f position_distance =
        ComputeOrthogonalDistance(fused_obj->rear_middle_point, radar_obj_pos, fused_obj->theta);
    if (position_distance.x() > position_filter_orthogonal_thresh_ ||
        position_distance.y() > position_filter_longitudinal_thresh_) {
      return distance;
    }
  }

  // cal distance
  float position_distance = 0.0;
  if (use_position_) {
    // position_distance = ComputeOrthogonalDistance(fused_obj->rear_middle_point, radar_obj_pos, fused_obj->theta).x();
    // @author zzg 修改距离计算方式 为 Compute2DEuclideanDistance
    position_distance = Compute2DEuclideanDistance(fused_obj->rear_middle_point, radar_obj_pos);
  }

  float velocity_distance = 0.0;
  if (use_velocity_) {
    velocity_distance = Compute2DEuclideanDistance(fused_obj->velocity, radar_obj_velocity);
  }

  distance = position_distance * position_weight_ + velocity_distance * velocity_weight_;

  return distance;
}

template <typename T>
float TrackObjectDistance::Compute2DEuclideanDistance(const Eigen::Matrix<T, 3, 1>& des,
                                                      const Eigen::Matrix<T, 3, 1>& src) {
  Eigen::Matrix<T, 3, 1> diff_pos = des - src;
  float distance = static_cast<float>(std::sqrt(diff_pos.head(2).cwiseProduct(diff_pos.head(2)).sum()));
  return distance;
}

template <typename T>
float TrackObjectDistance::Compute2DRelEuclideanDistance(const Eigen::Matrix<T, 3, 1>& des,
                                                         const Eigen::Matrix<T, 3, 1>& src) {
  Eigen::Matrix<T, 3, 1> diff_pos = des - src;
  float distance = static_cast<float>(std::sqrt(diff_pos.head(2).cwiseProduct(diff_pos.head(2)).sum()));
  float des_norm = static_cast<float>(des.head(2).norm());
  if (des_norm < 1e-6) {
    return std::numeric_limits<float>::max();
  }
  return distance / des_norm;
}

template <typename T>
Eigen::Vector2f TrackObjectDistance::ComputeOrthogonalDistance(const Eigen::Matrix<T, 3, 1>& des,
                                                               const Eigen::Matrix<T, 3, 1>& src, float yaw) {
  // 计算两点之间的相对位置向量
  Eigen::Matrix<T, 2, 1> delta_pos(des.x() - src.x(), des.y() - src.y());

  // 构建偏航角方向的单位向量
  Eigen::Matrix<T, 2, 1> direction(std::cos(yaw), std::sin(yaw));
  direction.normalize();

  // 计算垂直距离：使用叉积的模长来计算点到直线的距离
  // 二维叉积可以用 x1*y2 - x2*y1 计算
  float orthogonal_distance = std::abs(delta_pos.x() * direction.y() - delta_pos.y() * direction.x());

  float longitudinal_distance = std::abs(delta_pos.x() * direction.x() + delta_pos.y() * direction.y());

  return Eigen::Vector2f(orthogonal_distance, longitudinal_distance);
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
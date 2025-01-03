#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_1l1r.h"
#include "trunk_perception/common/types/object.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

ExistenceFusion1L1R::ExistenceFusion1L1R(const ExistenceFusionConfig::ConstPtr& config) : ExistenceFusionBase(config) {}

float ExistenceFusion1L1R::Compute(const FusedObject::Ptr& fused_object_ptr) {
  if (fused_object_ptr == nullptr) {
    TERROR << "ExistenceFusion1L1R::Compute: fused_object_ptr is nullptr";
    return 0.0;
  }

  const int& lidar_consecutive_hit = fused_object_ptr->lidar_consecutive_hit;
  const int& lidar_consecutive_lost = fused_object_ptr->lidar_consecutive_lost;
  const int& front_radar_consecutive_hit = fused_object_ptr->front_radar_consecutive_hit;

  float score = 0.0;

  if (lidar_consecutive_hit <= 0) {
    if (lidar_consecutive_lost > 3) {
      score = 0.0;
    } else {
      if (front_radar_consecutive_hit > 0) {
        score = 0.3;
      } else {
        score = 0.0;
      }
    }
  } else if (lidar_consecutive_hit <= 1) {
    if (front_radar_consecutive_hit > 0) {
      score = 1.0;
    } else {
      score = 0.3;
    }
  } else {
    score = 1.0;
  }

  // 计算是否满足特殊情况设定
  // 为了保持对激光丢失，但毫米波雷达连续命中的情况，保持稳定 （远距离检测能力提升）
  fused_object_ptr->flag_special_keep_stable = false;
  if (score < 0.3 && fused_object_ptr->lidar_total_life > 30 && fused_object_ptr->front_radar_consecutive_hit > 10 &&
      fused_object_ptr->velocity.norm() > 7.0) {
    fused_object_ptr->flag_special_keep_stable = true;
    score = 0.3;
  }

  // @author zzg 2024_12_04 增加逻辑：目标类型为 TRUCK、VEHICLE， 激光雷达总生命周期 35 帧以上，且 毫米波一直连续命中 25
  // 帧以上，则维持融合目标 为了解决近距离怼脸货车由于 激光连续丢失多帧 导致的目标丢失；   解决特定问题
  if ((score < 0.3) && (fused_object_ptr->front_radar_consecutive_hit > 25) &&
      (fused_object_ptr->lidar_total_life > 35)) {
    if (fused_object_ptr->type == ObjectType::TRUCK || fused_object_ptr->type == ObjectType::VEHICLE) {
      Eigen::Matrix4d local_to_car = fused_object_ptr->odo_lidar_ptr->Matrix().inverse();
      Eigen::Vector4d center_vec(fused_object_ptr->center.x(), fused_object_ptr->center.y(),
                                 fused_object_ptr->center.z(), 1.0);
      center_vec = local_to_car * center_vec;
      Eigen::Vector3d temp_center = Eigen::Vector3d(center_vec.x(), center_vec.y(), center_vec.z());
      if (std::fabs(temp_center.x()) < 25 && std::fabs(temp_center.y()) < 2.5) {
        score = 0.3;
      }
    }
  }

  return score;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
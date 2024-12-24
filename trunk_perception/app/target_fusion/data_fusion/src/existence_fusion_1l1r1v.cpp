#include "trunk_perception/app/target_fusion/data_fusion/existence_fusion_1l1r1v.h"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/common/types/object.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

ExistenceFusion1L1R1V::ExistenceFusion1L1R1V(const ExistenceFusionConfig::ConstPtr& config) : ExistenceFusionBase(config) {}

float ExistenceFusion1L1R1V::Compute(const FusedObject::Ptr& fused_object_ptr) {

  if (fused_object_ptr == nullptr) {
    TERROR << "ExistenceFusion1L1R1V::Compute: fused_object_ptr is nullptr";
    return 0.0;
  }

  const int& lidar_consecutive_hit = fused_object_ptr->lidar_consecutive_hit;
  const int& lidar_consecutive_lost = fused_object_ptr->lidar_consecutive_lost;
  const int& front_radar_consecutive_hit = fused_object_ptr->front_radar_consecutive_hit;
	const int& front_vision_consecutive_hit = fused_object_ptr->front_vision_consecutive_hit;
	const int& front_vision_consecutive_lost = fused_object_ptr->front_vision_consecutive_lost;

  float score = 0.0;

  if (lidar_consecutive_hit <= 0) {
		if (lidar_consecutive_lost <= 3) {
			if (front_vision_consecutive_hit >= 1) {
				if (front_radar_consecutive_hit > 3) {
					score = 1.0;
				} else if ((front_radar_consecutive_hit >=1 && front_radar_consecutive_hit <= 3) 
								&& (front_radar_consecutive_hit <= 3)) {
					score = 0.3;
				}
			}
		} else if (lidar_consecutive_lost > 3) {
			if (front_radar_consecutive_hit > 5) {
				if ((front_vision_consecutive_hit >= 1) && (front_vision_consecutive_hit <=3)) {
					score = 0.3;
				} else if (front_vision_consecutive_hit > 3) {
					score = 1.0;
				}
			}
		}
  } else if (lidar_consecutive_hit <= 1) {
		score = 0.3;
		if ((front_radar_consecutive_hit > 1) || (front_vision_consecutive_hit > 1)) {
			score = 1.0;
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

  return score;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
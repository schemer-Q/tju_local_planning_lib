/**
 * @file lane_quality.h
 * @brief 车道线质量评估
 * @author chenjiansong
 * @date 2025-01-16
 */

#pragma once

#include <vector>
#include <cmath>
#include "trunk_perception/common/types/lane.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_post{

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;


class LaneQualityEvaluator {
public:
  LaneQualityEvaluator(float check_range_near_x,
                       float check_range_far_x,
                       float line_outliner_thres,
                       float norm_max_std_deviation):
    check_range_near_x_(check_range_near_x),
    check_range_far_x_(check_range_far_x),
    line_outliner_thres_(line_outliner_thres),
    norm_max_std_deviation_(norm_max_std_deviation){};
  
  /**
   * @brief 评估车道线质量
   * @param lane_line 车道线实例
   * @return 返回车道线的质量评分（0-1之间，1表示质量最好）
   */
  float EvaluateLaneQuality(const LaneLineVision& lane_line);

private:
  /**
   * @brief 计算点的离散程度的quality得分
   * @param lane_line 车道线实例
   * @return 返回点的离散程度（标准差）得分
   */
  float PointDispersionQuality(const LaneLineVision& lane_line);

private:
  float check_range_near_x_ = -5; 
  float check_range_far_x_ = 70;
  float line_outliner_thres_ = 1.0f;    ///< unit: m
  float norm_max_std_deviation_ = 1.0f;  ///< unit: m 

};

}  // namespace ld_post

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
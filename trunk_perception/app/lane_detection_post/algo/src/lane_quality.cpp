/**
 * @file lane_quality.cpp
 * @brief 车道线质量评估实现
 * @author chenjiansong
 * @date 2025-01-16
 */
#include <numeric>
#include <cmath>
#include "trunk_perception/app/lane_detection_post/algo/lane_quality.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_post{

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

float LaneQualityEvaluator::EvaluateLaneQuality(const LaneLineVision& lane_line) {
    float quality_score = 0.0f;

    // 1. 计算点的离散程度
    float point_dispersion_quality = PointDispersionQuality(lane_line);
    quality_score = point_dispersion_quality; // 离散程度越小，质量越高
    // TODO other evaluation metrics
    
    return quality_score;
}

float LaneQualityEvaluator::PointDispersionQuality(const LaneLineVision& lane_line) {
    if (lane_line.OriginPointsWorld.empty()) {
        return 0.0f; // 如果点集为空，返回最低质量
    }

    int valid_num = 0;   // 有效点数量
    int outlier_num = 0; // 外点数量

    // 提取拟合参数
    float a0 = lane_line.a0;
    float a1 = lane_line.a1;
    float a2 = lane_line.a2;
    float a3 = lane_line.a3;

    // 计算每个点到拟合曲线的偏差
    std::vector<float> deviations;
    for (const auto& point : lane_line.OriginPointsWorld) {
        // 过滤超出范围的点
        if (point.x > check_range_far_x_ || point.x < check_range_near_x_) {
            continue;
        }

        valid_num++;

        float x = point.x;
        float y_fit = a0 + a1 * x + a2 * x * x + a3 * x * x * x;
        float y_actual = point.y;
        float deviation = std::abs(y_actual - y_fit);

        if (deviation > line_outliner_thres_) {
            outlier_num++;
        } else {
            deviations.push_back(deviation);
        }
    }

    // 如果有效点数量不足，返回最低质量
    if (valid_num <= 2) {
        return 0.0f;
    }

    // 计算外点比例
    float outlier_ratio = static_cast<float>(outlier_num) / valid_num;

    // 如果没有非外点，返回最低质量
    if (deviations.empty()) {
        return 0.0f;
    }

    float mean_deviation = 0.0f;
    for (float deviation : deviations) {
        mean_deviation += deviation;
    }
    mean_deviation /= deviations.size();

    float variance = 0.0f;
    for (float deviation : deviations) {
        variance += std::pow(deviation - mean_deviation, 2);
    }
    variance /= deviations.size();
    float std_deviation = std::sqrt(variance);
    float quality_score = 1.0f - std::min(std_deviation / norm_max_std_deviation_, 1.0f);

    float outlier_penalty = 1.0f - outlier_ratio; // 外点比例越高，惩罚越大
    quality_score *= outlier_penalty;

    return quality_score;
}


}  // namespace ld_post

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
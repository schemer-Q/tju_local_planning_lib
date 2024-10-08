/**
 * @file lane.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 车道线数据结构
 * @version 0.1
 * @date 2024-09-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

/**
 * @brief 车道线生成类型
 *
 */
enum GenerateType {
  DETECTED = 0,     ///< 检测生成
  COMPENSATED = 1,  ///< 补偿生成
};

/**
 * @brief LaneLine type, refering to mobileye eq3 definition
 */
enum LaneLineType {
  UNINITED_TYPE = 0,
  SOLID = 1,       ///< 实线
  DASHED = 2,      ///< 虚线
  RESERVED1 = 3,   ///< 保留
  BOTTS_DOTS = 4,  ///< 圆点线
  RESERVED2 = 5,   ///< 保留
  INVALID = 6,     ///< 无效
  UNDECIDED = 7,   ///< 未决定
  DOUBLELANE = 8   ///< 双车道
};

/**
 * @brief Laneline color
 */
enum LaneLineColor { YELLOW = 0, WHITE = 1, OTHER_COLOR = 2, UNINITED_COLOR = 255 };

/**
 * @brief Laneline confidence
 */
enum LaneLineConfidence { BAD = 0, MIDDLE = 1, GOOD = 2, BEST = 3 };

/**
 * @brief LaneLine relative location definition
 * @verbatim
 |    |     |         |      |     |
 |    |     |         |      |     |
 |    |     |         |      |     |
 |    |     |         |      |     |
 |    |     |         |      |     |
 |    |     | ego_car |      |     |
 -3  -2     -1   0    1      2     3 
 @endverbatim
*/
enum LaneLinePositionIndex {
  LEFT_6 = -6,
  LEFT_5 = -5,
  LEFT_4 = -4,
  LEFT_3 = -3,
  LEFT_2 = -2,
  LEFT_1 = -1,
  CHANGING_LINE = 0,  ///< 变道的时候中间的线
  RIGHT_1 = 1,
  RIGHT_2 = 2,
  RIGHT_3 = 3,
  RIGHT_4 = 4,
  RIGHT_5 = 5,
  RIGHT_6 = 6,
  UNINITED_INDEX = 99
};

/**
 * @brief 车道线点实例
 * 
 */
struct LanePointInstance {
  int u, v;
  float depth;
};

/**
 * @brief 车道线实例
 * 
 */
struct LaneInstance {
  int lane_type;
  std::vector<LanePointInstance> points;
};

/**
 * @brief 视觉车道线
 *
 */
struct LaneLineVision {
  LaneLineVision(const float& a_0, const float& a_1, const float& a_2, const float& a_3)
      : a0(a_0), a1(a_1), a2(a_2), a3(a_3){};
  LaneLineVision() = default;
  void PrintInfo() const {
    std::cout << "Laneline: Id: " << track_id << " age: " << age << " LaneLinePositionIndex:" << lane_line_position
              << " LaneLineType: " << lane_line_type << " params: " << a0 << " " << a1 << " " << a2 << " " << a3
              << std::endl;
  };

  void clear() {
    DistortPointsImg.clear();
    OriginPointsImg.clear();
    OriginPointsWorld.clear();

    lane_generate_type = GenerateType::DETECTED;
    lane_line_type = LaneLineType::UNINITED_TYPE;
    lane_line_color = LaneLineColor::UNINITED_COLOR;
    lane_line_conf = LaneLineConfidence::BAD;
    lane_line_position = LaneLinePositionIndex::UNINITED_INDEX;

    lane_conf = 0.;
    dis_to_line_avg = 0.;
    dis_to_line_stddev = 0.;
    straight_slope = 0.;
    straight_intercept = 0.;

    far_x = 0.f;
    age = 0;
    track_id = 0;

    a0_cov = 0.f;
    a70_cov = 0.f;
  }

  /// from detecting module
  // origin distort points
  std::vector<cv::Point2f> DistortPointsImg;
  // points used for image curve fitting
  std::vector<cv::Point2f> OriginPointsImg;
  // points used for world curve fitting
  std::vector<cv::Point3f> OriginPointsWorld;
  GenerateType lane_generate_type = GenerateType::DETECTED;
  LaneLineType lane_line_type = LaneLineType::UNINITED_TYPE;
  LaneLineColor lane_line_color = LaneLineColor::UNINITED_COLOR;
  LaneLineConfidence lane_line_conf = LaneLineConfidence::BAD;
  float lane_conf = 0.;  ///< 置信度 0~1
  LaneLinePositionIndex lane_line_position = LaneLinePositionIndex::UNINITED_INDEX;
  int a = 0;
  LaneInstance laneInstance;

  /// update from tracking module
  /* x = a0 + a1*y + a2*y^2 + a3*a3^3 */
  float a0 = 0.0f;
  float a1 = 0.0f;
  float a2 = 0.0f;
  float a3 = 0.0f;
  float bottom_x = 0.0;  // 图像中心线对应的x坐标
  float ave_x = 0.0f;
  float ave_y = 0.0f;
  float near_x = 0.0f;
  float far_x = 0.f;
  int age = 0;
  uint track_id = 0;
  cv::Scalar lane_line_show_color;

  // 当前帧bev追踪关联得分最大值
  float associate_bev_score;

  // 图像点与车道线拟合线的平均距离
  float dis_to_line_avg = 0.;
  // 图像点与车道线拟合线的距离variance
  float dis_to_line_stddev = 0.;
  // 车道线拟合直线的斜率
  float straight_slope = 0.;
  // 车道线拟合直线的截距
  float straight_intercept = 0.;

  float a0_cov = 0;
  float a70_cov = 0;
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END

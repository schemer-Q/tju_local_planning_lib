/**
 * @file MathUtil.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 数学工具类, 从SDK中移植
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/lane.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_algo {

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

struct PolynomialCoefficients {
  // 默认构造函数
  PolynomialCoefficients() : a0(0), a1(0), a2(0), a3(0) {}

  // 带参数的构造函数
  PolynomialCoefficients(float a0, float a1, float a2, float a3) : a0(a0), a1(a1), a2(a2), a3(a3) {}

  // a0 + a1x + a2x^2 + a3x^3
  float a0, a1, a2, a3;
};

/**
 * @brief 多项式拟合
 *
 * @param in_point [in] 输入点集, 3D, cv::Point3f
 * @param n [in] 多项式指数
 * @param out [out] 输出系数矩阵
 * @return true 成功
 * @return false 失败
 */
bool PolyFitting(const std::vector<cv::Point3f>& in_point, const int& n, cv::Mat& out);

/**
 * @brief 多项式拟合
 *
 * @param in_point [in] 输入点集, 2D, cv::Point2f
 * @param n [in] 多项式指数
 * @param out [out] 输出系数矩阵
 * @return true 成功
 * @return false 失败
 */
bool PolyFitting(const std::vector<cv::Point2f>& in_point, const int& n, cv::Mat& out);

/**
 * @brief 多项式拟合
 *
 * @param xs [in] x坐标集
 * @param ys [in] y坐标集
 * @param n [in] 多项式指数
 * @param out [out] 输出系数矩阵
 * @return true 成功
 * @return false 失败
 */
bool PolyFitting(const std::vector<float>& xs, const std::vector<float>& ys, const int& n, cv::Mat& out);

/**
 * @brief 根据多项式系数计算x值
 *
 * @param a0 一次项系数
 * @param a1 二次项系数
 * @param a2 三次项系数
 * @param a3 四次项系数
 * @param y 输入的y值
 * @return float
 */
static inline float CalculateXValue(float a0, float a1, float a2, float a3, float y) {
  return float(a0 + a1 * y + a2 * std::pow(y, 2) + a3 * pow(y, 3));
}

/**
 * @brief 根据多项式系数计算x值
 *
 * @param lane_fit_param 多项式系数
 * @param y 输入的y值
 * @return float
 */
static inline float CalculateXValue(const std::vector<float>& lane_fit_param, float y) {
  assert(lane_fit_param.size() == 4);
  return CalculateXValue(lane_fit_param[0], lane_fit_param[1], lane_fit_param[2], lane_fit_param[3], y);
}

/**
 * @brief 根据多项式系数计算x值
 *
 * @param laneline
 * @param y
 * @return float
 */
static inline float CalculateXValue(const LaneLineVision& laneline, float y) {
  return CalculateXValue(laneline.a0, laneline.a1, laneline.a2, laneline.a3, y);
}

/**
 * @brief 拟合平行车道线
 *
 * @param points 输入点集
 * @param weighted 是否加权
 * @param poly_num 多项式阶数
 * @return std::vector<PolynomialCoefficients> 多项式系数
 */
std::vector<PolynomialCoefficients> FitParallelLines(const std::vector<std::vector<std::pair<float, float>>>& points,
                                                     bool weighted, int poly_num);

}  // namespace ld_algo

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END

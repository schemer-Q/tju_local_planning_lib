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

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_algo {

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
}  // namespace ld_algo

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END

/**
 * @file camera_info.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 相机信息
 * @version 0.1
 * @date 2024-09-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <memory>

#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

struct CameraInfo {
 public:
  int IMG_W;
  int IMG_H;
  double fx;
  double fy;
  double cx;
  double cy;
  cv::Mat K;
  cv::Mat D;

  typedef std::shared_ptr<CameraInfo> Ptr;
  typedef std::shared_ptr<const CameraInfo> ConstPtr;
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END
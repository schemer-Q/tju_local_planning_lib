#pragma once

#include <opencv2/opencv.hpp>
#include <memory>

#include "tju_local_planning/common/macros.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

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
TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END

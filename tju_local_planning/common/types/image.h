#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

#include "tju_local_planning/common/macros.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

/**
 * @brief 图像数据类型
 * @details 图像数据类型，包含时间戳和图像数据
 */
struct Image {
  double timestamp;  ///< 时间戳, 单位为秒
  cv::Mat image;     ///< 图像数据

  /**
   * @brief 构造函数
   * @param _timestamp 时间戳, 单位为秒
   * @param _image 图像数据
   * @param deep_clone 是否深拷贝图像数据, 默认为true
   */
  Image(const double& _timestamp, const cv::Mat& _image, const bool& deep_clone = true) : timestamp(_timestamp) {
    if (deep_clone) {
      this->image = _image.clone();
    } else {
      this->image = _image;
    }
  }

  /**
   * @brief 默认构造函数
   */
  Image() = default;

  /**
   * @brief 析构函数
   */
  ~Image() = default;

  /**
   * @brief 赋值操作, 浅拷贝
   * @param other 其他图像数据
   * @return Image&
   */
  Image& operator=(const Image& other) {
    timestamp = other.timestamp;
    image = other.image;
    return *this;
  }

  typedef std::shared_ptr<Image> Ptr;
  typedef std::shared_ptr<const Image> ConstPtr;
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END

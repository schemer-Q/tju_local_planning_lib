#pragma once

#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/camera_info.h"
#include "tju_local_planning/tools/log/t_log.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

class CameraUndistort {
 public:
  enum class DistortionModel {
    PINHOLE_RADTAN_5,    // 针孔相机5参数模型 (k1,k2,p1,p2,k3)
    PINHOLE_RADTAN_8,    // 针孔相机8参数模型 (k1,k2,p1,p2,k3,k4,k5,k6)
    FISHEYE_EQUI_4       // 鱼眼相机4参数模型 (k1,k2,k3,k4)
  };

  CameraUndistort(const CameraInfo& camera_info, 
                 const bool& use_new_camera_matrix = true) {
    camera_matrix_ = camera_info.K;
    camera_distro_coeff_ = camera_info.D;
    if (camera_distro_coeff_.rows == 5) {
      distortion_model_ = DistortionModel::PINHOLE_RADTAN_5;
    } else if (camera_distro_coeff_.rows == 8) {
      distortion_model_ = DistortionModel::PINHOLE_RADTAN_8;
    } else if (camera_distro_coeff_.rows == 4) {
      distortion_model_ = DistortionModel::FISHEYE_EQUI_4;
    } else {
      NTERROR << "CameraUndistort::CameraUndistort failed, camera_info.D.rows is not 4, 5 or 8";
      distortion_model_ = DistortionModel::PINHOLE_RADTAN_5;
      camera_distro_coeff_ = cv::Mat_<double>::zeros(5, 1);
    }
    
    cv::Size camera_size = cv::Size(camera_info.IMG_W, camera_info.IMG_H);

    if (use_new_camera_matrix) {
      new_camera_matrix_ =
          cv::getOptimalNewCameraMatrix(camera_matrix_, camera_distro_coeff_, camera_size, 0, camera_size, 0);
    } else {
      new_camera_matrix_ = camera_matrix_;
    }

    cv::initUndistortRectifyMap(camera_matrix_, camera_distro_coeff_, cv::Mat(), new_camera_matrix_, camera_size,
                                CV_32FC1, map_x_, map_y_);
  }
  ~CameraUndistort() = default;

  void UndistortImg(const cv::Mat& src, cv::Mat& dst) { cv::remap(src, dst, map_x_, map_y_, cv::INTER_LINEAR); }

  cv::Point2f DistortPoint(const cv::Point2f& undist_point) {
    std::vector<cv::Point2f> input_points{undist_point};
    std::vector<cv::Point2f> output_points = DistortPoints(input_points);
    return output_points[0];
  }

  std::vector<cv::Point2f> DistortPoints(const std::vector<cv::Point2f>& undist_points) {
    switch (distortion_model_) {
      case DistortionModel::PINHOLE_RADTAN_5:
        return DistortPointsPinholeRadTan5(undist_points);
      case DistortionModel::PINHOLE_RADTAN_8:
        return DistortPointsPinholeRadTan8(undist_points);
      case DistortionModel::FISHEYE_EQUI_4:
        return DistortPointsFisheyeEqui4(undist_points);
      default:
        return std::vector<cv::Point2f>();
    }
  }

 private:
  DistortionModel distortion_model_;

  cv::Mat camera_matrix_;        ///< 相机内参
  cv::Mat camera_distro_coeff_;  ///< 相机畸变系数
  cv::Mat new_camera_matrix_;    ///< 新的相机内参
  cv::Mat map_x_;                ///< 去畸变映射
  cv::Mat map_y_;                ///< 去畸变映射

  std::vector<cv::Point2f> DistortPointsPinholeRadTan5(const std::vector<cv::Point2f>& undist_points) {
    std::vector<cv::Point2f> dist_points;
    if (undist_points.empty()) {
      return dist_points;
    }
    const float& fx = new_camera_matrix_.at<float>(0, 0);
    const float& fy = new_camera_matrix_.at<float>(1, 1);
    const float& cx = new_camera_matrix_.at<float>(0, 2);
    const float& cy = new_camera_matrix_.at<float>(1, 2);
    const float& k1 = camera_distro_coeff_.at<float>(0);
    const float& k2 = camera_distro_coeff_.at<float>(1);
    const float& p1 = camera_distro_coeff_.at<float>(2);
    const float& p2 = camera_distro_coeff_.at<float>(3);
    const float& k3 = camera_distro_coeff_.at<float>(4);

    for (const auto& undist_point : undist_points) {
      double x = (undist_point.x - cx) / fx;
      double y = (undist_point.y - cy) / fy;

      double r2 = x * x + y * y;

      // Radial distorsion
      double xDistort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
      double yDistort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

      // Tangential distorsion
      xDistort = xDistort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
      yDistort = yDistort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

      // Back to absolute coordinates.
      xDistort = xDistort * fx + cx;
      yDistort = yDistort * fy + cy;

      dist_points.emplace_back((float)xDistort, (float)yDistort);
    }

    return dist_points;
  }

  std::vector<cv::Point2f> DistortPointsPinholeRadTan8(const std::vector<cv::Point2f>& undist_points) {
    std::vector<cv::Point2f> dist_points;
    if (undist_points.empty()) return dist_points;
    
    const float& fx = new_camera_matrix_.at<float>(0, 0);
    const float& fy = new_camera_matrix_.at<float>(1, 1);
    const float& cx = new_camera_matrix_.at<float>(0, 2);
    const float& cy = new_camera_matrix_.at<float>(1, 2);
    
    // 8参数模型 (k1,k2,p1,p2,k3,k4,k5,k6)
    const float& k1 = camera_distro_coeff_.at<float>(0);
    const float& k2 = camera_distro_coeff_.at<float>(1);
    const float& p1 = camera_distro_coeff_.at<float>(2);
    const float& p2 = camera_distro_coeff_.at<float>(3);
    const float& k3 = camera_distro_coeff_.at<float>(4);
    const float& k4 = camera_distro_coeff_.at<float>(5);
    const float& k5 = camera_distro_coeff_.at<float>(6);
    const float& k6 = camera_distro_coeff_.at<float>(7);

    for (const auto& undist_point : undist_points) {
      double x = (undist_point.x - cx) / fx;
      double y = (undist_point.y - cy) / fy;
      double r2 = x * x + y * y;
      double r4 = r2 * r2;
      double r6 = r4 * r2;

      // 径向畸变（包含高阶项）
      double radial = (1 + k1 * r2 + k2 * r4 + k3 * r6) / (1 + k4 * r2 + k5 * r4 + k6 * r6);
      double xDistort = x * radial;
      double yDistort = y * radial;

      // 切向畸变
      xDistort += (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
      yDistort += (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

      xDistort = xDistort * fx + cx;
      yDistort = yDistort * fy + cy;

      dist_points.emplace_back((float)xDistort, (float)yDistort);
    }
    return dist_points;
  }

  std::vector<cv::Point2f> DistortPointsFisheyeEqui4(const std::vector<cv::Point2f>& undist_points) {
    std::vector<cv::Point2f> dist_points;
    if (undist_points.empty()) return dist_points;
    
    const float& fx = new_camera_matrix_.at<float>(0, 0);
    const float& fy = new_camera_matrix_.at<float>(1, 1);
    const float& cx = new_camera_matrix_.at<float>(0, 2);
    const float& cy = new_camera_matrix_.at<float>(1, 2);
    
    // 鱼眼4参数 (k1,k2,k3,k4)
    const float& k1 = camera_distro_coeff_.at<float>(0);
    const float& k2 = camera_distro_coeff_.at<float>(1);
    const float& k3 = camera_distro_coeff_.at<float>(2);
    const float& k4 = camera_distro_coeff_.at<float>(3);

    for (const auto& undist_point : undist_points) {
      double x = (undist_point.x - cx) / fx;
      double y = (undist_point.y - cy) / fy;
      double r = std::sqrt(x * x + y * y);
      
      // 鱼眼畸变模型
      double theta = std::atan(r);
      double theta2 = theta * theta;
      double theta4 = theta2 * theta2;
      double theta6 = theta4 * theta2;
      double theta8 = theta4 * theta4;
      double theta_d = theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);

      double scale = (r > 1e-8) ? theta_d / r : 1.0;
      double xDistort = x * scale;
      double yDistort = y * scale;

      xDistort = xDistort * fx + cx;
      yDistort = yDistort * fy + cy;

      dist_points.emplace_back((float)xDistort, (float)yDistort);
    }
    return dist_points;
  }
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END

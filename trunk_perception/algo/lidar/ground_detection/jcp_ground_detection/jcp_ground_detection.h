/**
 * @file jcp_ground_detection.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief 
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "trunk_perception/algo/lidar/ground_detection/ground_detection_base.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

constexpr float RAD2DEG = 180.0F / M_PI;
constexpr float DEG2RAD = M_PI / 180.0F;

struct JCPGroundDetectionParams {
  float min_z = -1.0F;
  float max_z = 3.0F;
  float min_range = 0.0F;
  float max_range = 130.0F;
  float delta_R = 2.0F;
  float delta_R_inv = 0.5F;
  float horizon_fov = 130.0F;
  float horizon_fov_half = 65.0F;
  float horizon_resolution = 0.2F;
  float horizon_resolution_inv = 5.0F;
  int vertical_ring_number = 125;
  int azimuth_number = 650;
  int radial_num = 65;
  float threshold_height = 0.3F;
  float threshold_slope = 10.0F;
  float threshold_d = 2.0F;
  float threshold_s = 5.0F;
  float sensor_height = 3.27F;
  float sensor_pitch = 10.0F;

  float roi_min_y = -20.0F;
  float roi_max_y = 20.0F;
  std::vector<float> roi_x = {0.0F, 25.0F, 80.0F};
  float threshold_dis = 0.1F;
  float threshold_direction = 0.98F;
  float roi_range = 10.0F;
};

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> DynamicMatrix;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> DynamicMatrixI;

enum Label : int { UNKNOWN = -1, NO_GROUND = 0, GROUND = 1, LOW_CONFIDENCE = 2 };

template <class T>
class JCPGroundDetection : virtual public GroundDetectionBase<T> {
 public:
  JCPGroundDetection() = default;
  ~JCPGroundDetection() override = default;

  /**
   * @brief init param
   *
   * @param config yaml node
   * @return int
   */
  int init(const YAML::Node &config) override;

  /**
   * @brief set point cloud transform matrix
   *
   * @param tf transform matrix
   */
  void setTF(const Eigen::Isometry3f &tf) override;

  /**
   * @brief process pipeline
   *
   * @param cloud_in input point cloud
   * @return int
   */
  int process(const typename std::shared_ptr<const pcl::PointCloud<T>> &cloud_in) override;

  /**
   * @brief get no ground cloud
   *
   * @return std::shared_ptr<pcl::PointCloud<T>>
   */
  virtual typename std::shared_ptr<pcl::PointCloud<T>> getNoGroundCloud() override;

  /**
   * @brief get ground cloud
   *
   * @return std::shared_ptr<pcl::PointCloud<T>>
   */
  virtual typename std::shared_ptr<pcl::PointCloud<T>> getGroundCloud() override;

  /**
   * @brief get ground params object
   *
   * @return std::vector<float>
   */
  virtual std::vector<float> getGroundParams() override;

 private:
  void reset();
  void preProcessing(const typename std::shared_ptr<const pcl::PointCloud<T>> &cloud_in);
  void rangeProjection(const typename std::shared_ptr<const pcl::PointCloud<T>> &cloud_in);
  void RECM(const typename std::shared_ptr<const pcl::PointCloud<T>> &cloud_in);
  void JCP(const typename std::shared_ptr<const pcl::PointCloud<T>> &cloud_in);
  void labelPoints(const typename std::shared_ptr<const pcl::PointCloud<T>> &cloud_in,
                   typename std::shared_ptr<pcl::PointCloud<T>> &cloud_ground,
                   typename std::shared_ptr<pcl::PointCloud<T>> &cloud_no_ground);
  void doMorphologyDilate(DynamicMatrixI &image, DynamicMatrixI &image_dilated);
  void postProcessing(typename std::shared_ptr<pcl::PointCloud<T>> &cloud_ground,
                      typename std::shared_ptr<pcl::PointCloud<T>> &cloud_no_ground);
  int estimatePlaneSVD(const typename std::shared_ptr<pcl::PointCloud<T>> &cloud, Eigen::Vector4f &normal_vector);
  int estimatePlaneRANSAC(const typename std::shared_ptr<pcl::PointCloud<T>> &cloud, Eigen::Vector4f &normal_vector);
  float getPointToPlaneDistance(const T &p, const Eigen::Vector4f &vec);

 private:
  Eigen::VectorXf ground_height_default_;
  std::vector<std::pair<int, int>> neighbor_;
  std::vector<std::tuple<int, int, int>> masked_points_;

  DynamicMatrix region_minz_map_;
  DynamicMatrixI cloud_index_map_;
  DynamicMatrixI region_map_;
  DynamicMatrixI label_map_;

  typename std::shared_ptr<const pcl::PointCloud<T>> cloud_in_ptr_ = nullptr;
  typename std::shared_ptr<const pcl::PointCloud<T>> cloud_process_ptr_ = nullptr;
  typename std::shared_ptr<pcl::PointCloud<T>> cloud_ground_ptr_ = nullptr;
  typename std::shared_ptr<pcl::PointCloud<T>> cloud_no_ground_ptr_ = nullptr;

  Eigen::Isometry3f tf_ = Eigen::Isometry3f::Identity();
  JCPGroundDetectionParams params_;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END
/**
 * @file jcp_ground_detection.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-11-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <random>

#include "trunk_perception/algo/lidar/ground_detection/jcp_ground_detection/jcp_ground_detection.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

[[maybe_unused]] void cvShow(const Eigen::MatrixXi &label_map, const std::string &name) {
  const int rows = label_map.rows();
  const int cols = label_map.cols();

  cv::Mat show_image = cv::Mat::zeros(rows, cols, CV_8UC3);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (label_map(r, c) == Label::NO_GROUND) {
        show_image.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 255);  /// bgr
      } else if (label_map(r, c) == Label::GROUND) {
        show_image.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 255, 0);  /// bgr
      } else if (label_map(r, c) == Label::LOW_CONFIDENCE) {
        show_image.at<cv::Vec3b>(r, c) = cv::Vec3b(255, 0, 0);  /// bgr
      } else if (label_map(r, c) != Label::UNKNOWN) {
        show_image.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 255, 255);  /// bgr
      }
    }
  }
  cv::flip(show_image, show_image, 0);
  cv::imshow(name, show_image);
  cv::waitKey(1);
}

int JCPGroundDetection::init(const YAML::Node &config) {
  try {
    params_.min_z = config["min_z"].as<float>();
    params_.max_z = config["max_z"].as<float>();
    params_.min_range = config["min_range"].as<float>();
    params_.max_range = config["max_range"].as<float>();
    params_.delta_R = config["delta_R"].as<float>();
    params_.horizon_fov = config["horizon_fov"].as<float>();
    params_.horizon_resolution = config["horizon_resolution"].as<float>();
    params_.vertical_ring_number = config["vertical_ring_number"].as<int>();
    params_.threshold_height = config["threshold_height"].as<float>();
    params_.threshold_slope = config["threshold_slope"].as<float>();
    params_.threshold_d = config["threshold_d"].as<float>();
    params_.threshold_s = config["threshold_s"].as<float>();
    params_.sensor_height = config["sensor_height"].as<float>();
    params_.sensor_pitch = config["sensor_pitch"].as<float>();
    params_.roi_min_y = config["roi_min_y"].as<float>();
    params_.roi_max_y = config["roi_max_y"].as<float>();
    params_.roi_x = config["roi_x"].as<std::vector<float>>();
    params_.threshold_dis = config["threshold_dis"].as<float>();
    params_.threshold_direction = config["threshold_direction"].as<float>();
    params_.roi_range = config["roi_range"].as<float>();

    params_.delta_R_inv = 1.0F / params_.delta_R;
    params_.horizon_fov_half = params_.horizon_fov * 0.5F;
    params_.horizon_resolution_inv = 1.0F / params_.horizon_resolution;
    params_.azimuth_number = static_cast<int>(params_.horizon_fov * params_.horizon_resolution_inv);
    params_.radial_num = static_cast<int>((params_.max_range - params_.min_range) * params_.delta_R_inv);
  } catch (const YAML::Exception &e) {
    TFATAL << "JCPGroundDetection config error.";
    return 1;
  }

  const int num = 2;
  for (int i = -num; i <= num; ++i) {
    for (int j = -num; j <= num; ++j) {
      if (i == 0 && j == 0) continue;
      neighbor_.emplace_back(i, j);
    }
  }

  const float pitch_rad = params_.sensor_pitch * DEG2RAD;
  ground_height_default_.resize(params_.radial_num);
  for (int idx = 0; idx < params_.radial_num; ++idx) {
    const float range = idx * params_.delta_R + params_.min_range;
    ground_height_default_(idx) = -(params_.sensor_height - range * std::tan(pitch_rad)) * std::cos(pitch_rad);
  }

  region_minz_map_.resize(params_.radial_num, params_.azimuth_number);
  cloud_index_map_.resize(params_.vertical_ring_number, params_.azimuth_number);
  region_map_.resize(params_.vertical_ring_number, params_.azimuth_number);
  label_map_.resize(params_.vertical_ring_number, params_.azimuth_number);

  cloud_ground_ptr_.reset(new PointCloudT());
  cloud_no_ground_ptr_.reset(new PointCloudT());
  return 0;
}

void JCPGroundDetection::setTF(const Eigen::Isometry3f &tf) { tf_ = tf; }

int JCPGroundDetection::process(const PointCloudConstPtr &cloud_in) {
  if (!cloud_in) return 1;

  reset();

  preProcessing(cloud_in);

  rangeProjection(cloud_process_ptr_);

  RECM(cloud_process_ptr_);

  JCP(cloud_process_ptr_);

  labelPoints(cloud_in_ptr_, cloud_ground_ptr_, cloud_no_ground_ptr_);

  postProcessing(cloud_ground_ptr_, cloud_no_ground_ptr_);

  return 0;
}

PointCloudPtr JCPGroundDetection::getNoGroundCloud() { return cloud_no_ground_ptr_; }

PointCloudPtr JCPGroundDetection::getGroundCloud() { return cloud_ground_ptr_; }

std::vector<float> JCPGroundDetection::getGroundParams() { return std::vector<float>(); }

void JCPGroundDetection::reset() {
  cloud_ground_ptr_->clear();
  cloud_no_ground_ptr_->clear();
  masked_points_.clear();

  region_minz_map_.setConstant(100.0F);
  cloud_index_map_.setConstant(-1);
  region_map_.setConstant(-1);
  label_map_.setConstant(-1);
}

void JCPGroundDetection::preProcessing(const PointCloudConstPtr &cloud_in) {
  cloud_in_ptr_ = cloud_in;

  PointCloudPtr cloud_temp(new PointCloudT(*cloud_in));
  for (auto &pt : cloud_temp->points) {
    pt.getVector3fMap() = tf_.inverse() * pt.getVector3fMap();
  }
  cloud_process_ptr_ = cloud_temp;
}

void JCPGroundDetection::rangeProjection(const PointCloudConstPtr &cloud_in) {
  const size_t cloud_size = cloud_in->size();
  masked_points_.reserve(cloud_size);
  for (size_t i = 0UL; i < cloud_size; ++i) {
    const auto &pt = cloud_in->points[i];
    const float range = pt.getVector3fMap().head(2).norm();
    const float height = (tf_ * pt.getVector3fMap())(2);
    if (range < params_.min_range || range > params_.max_range || height < params_.min_z || height > params_.max_z) {
      continue;
    }

    const int row_idx = pt.ring;
    const float horizon_angle = std::atan2(pt.y, pt.x) * RAD2DEG;
    const int col_idx = static_cast<int>((params_.horizon_fov_half - horizon_angle) * params_.horizon_resolution_inv);
    if (row_idx < 0 || row_idx >= params_.vertical_ring_number || col_idx < 0 || col_idx >= params_.azimuth_number) {
      continue;
    }

    if (cloud_index_map_(row_idx, col_idx) != -1) {
      masked_points_.emplace_back(cloud_index_map_(row_idx, col_idx), row_idx, col_idx);
    }

    cloud_index_map_(row_idx, col_idx) = i;
    label_map_(row_idx, col_idx) = Label::GROUND;
    const int radial_indexs = static_cast<int>((range - params_.min_range) * params_.delta_R_inv);
    region_map_(row_idx, col_idx) = radial_indexs;
    region_minz_map_(radial_indexs, col_idx) = std::min(region_minz_map_(radial_indexs, col_idx), pt.z);
  }

  // debug
  // cvShow(label_map_, "image_rangeProjection");
}

void JCPGroundDetection::RECM(const PointCloudConstPtr &cloud_in) {
  const float pitch_delta_z = params_.delta_R * std::tan(params_.sensor_pitch * DEG2RAD);
  for (int c = 0; c < params_.azimuth_number; ++c) {
    bool flag = false;
    region_minz_map_(0, c) = std::min(region_minz_map_(0, c), ground_height_default_(0));
    for (int r = 1; r < params_.radial_num; ++r) {
      if (region_minz_map_(r, c) > 99.0F && !flag) {
        region_minz_map_(r, c) = ground_height_default_(r);
        continue;
      }
      if (region_minz_map_(r, c) > 99.0F && flag) {
        region_minz_map_(r, c) = region_minz_map_(r - 1, c) + pitch_delta_z;
      }
      flag = true;
    }
  }

  const float slope_delta_z = static_cast<float>(params_.delta_R * std::tan(params_.threshold_slope * DEG2RAD));
  for (int c = 0; c < params_.azimuth_number; ++c) {
    float pre_minz = std::min(region_minz_map_(0, c), ground_height_default_(0));
    for (int r = 1; r < params_.radial_num; ++r) {
      const float range = r * params_.delta_R + params_.min_range;
      if (range < params_.roi_range) {
        region_minz_map_(r, c) = std::min(region_minz_map_(r, c), ground_height_default_(r));
      }

      region_minz_map_(r, c) = std::min(region_minz_map_(r, c), pre_minz + slope_delta_z);
      pre_minz = region_minz_map_(r, c);
    }
  }

#pragma omp parallel for
  for (int col = 0; col < params_.azimuth_number; ++col) {
    for (int row = 0; row < params_.vertical_ring_number; ++row) {
      const int pt_id = cloud_index_map_(row, col);
      if (pt_id == -1) continue;

      const int region_index = region_map_(row, col);
      const float region_height = region_minz_map_(region_index, col);

      if (cloud_in->points[pt_id].z >= (region_height + params_.threshold_height)) {
        label_map_(row, col) = Label::NO_GROUND;
      }
    }
  }

  // debug
  // cvShow(label_map_, "image_RECM");
}

void JCPGroundDetection::doMorphologyDilate(DynamicMatrixI &image, DynamicMatrixI &image_dilated) {
  constexpr size_t ksize = 2;
  const size_t r_max = image.rows() - ksize;
  const size_t c_max = image.cols() - ksize;

#pragma omp parallel for
  for (size_t c = ksize; c < c_max; ++c) {
    for (size_t r = ksize; r < r_max; ++r) {
      if (image(r, c) == Label::NO_GROUND) {
        image_dilated(r - 1, c - 1) = Label::NO_GROUND;
        image_dilated(r - 1, c) = Label::NO_GROUND;
        image_dilated(r - 1, c + 1) = Label::NO_GROUND;
        image_dilated(r, c + 1) = Label::NO_GROUND;
        image_dilated(r + 1, c + 1) = Label::NO_GROUND;
        image_dilated(r + 1, c) = Label::NO_GROUND;
        image_dilated(r + 1, c - 1) = Label::NO_GROUND;
        image_dilated(r, c - 1) = Label::NO_GROUND;
        image_dilated(r - 2, c - 2) = Label::NO_GROUND;
        image_dilated(r - 2, c - 1) = Label::NO_GROUND;
        image_dilated(r - 2, c) = Label::NO_GROUND;
        image_dilated(r - 2, c + 1) = Label::NO_GROUND;
        image_dilated(r - 2, c + 2) = Label::NO_GROUND;
        image_dilated(r - 1, c + 2) = Label::NO_GROUND;
        image_dilated(r, c + 2) = Label::NO_GROUND;
        image_dilated(r + 1, c + 2) = Label::NO_GROUND;
        image_dilated(r + 2, c + 2) = Label::NO_GROUND;
        image_dilated(r + 2, c + 1) = Label::NO_GROUND;
        image_dilated(r + 2, c) = Label::NO_GROUND;
        image_dilated(r + 2, c - 1) = Label::NO_GROUND;
        image_dilated(r + 2, c - 2) = Label::NO_GROUND;
        image_dilated(r + 1, c - 2) = Label::NO_GROUND;
        image_dilated(r, c - 2) = Label::NO_GROUND;
        image_dilated(r - 1, c - 2) = Label::NO_GROUND;
      }
    }
  }
}

void JCPGroundDetection::JCP(const PointCloudConstPtr &cloud_in) {
  DynamicMatrixI image_dilated = label_map_;
  doMorphologyDilate(label_map_, image_dilated);

  const size_t cloud_size = cloud_in->size();
  std::vector<std::pair<int, int>> points_rejudge;
  points_rejudge.reserve(cloud_size);

  for (int col = 0; col < params_.azimuth_number; ++col) {
    for (int row = 0; row < params_.vertical_ring_number; ++row) {
      if (label_map_(row, col) == Label::GROUND && image_dilated(row, col) == Label::NO_GROUND) {
        if (cloud_index_map_(row, col) != -1) {
          points_rejudge.emplace_back(row, col);
          label_map_(row, col) = Label::LOW_CONFIDENCE;
        } else {
          label_map_(row, col) = Label::NO_GROUND;
        }
      }
    }
  }

  // debug
  // cvShow(image_dilated, "image_dilated");
  // cvShow(label_map_, "image_rejudged");

  const int num_neighbor = static_cast<int>(neighbor_.size());
  const int num_points = static_cast<int>(points_rejudge.size());
  for (int i = 0; i < num_points; ++i) {
    const auto &pt_coor = points_rejudge[i];
    const auto &pt1 = cloud_in->points[cloud_index_map_(pt_coor.first, pt_coor.second)];
    Eigen::VectorXf D = Eigen::VectorXf::Zero(num_neighbor);
    std::vector<int> mask(num_neighbor, -1);

    for (int id = 0; id < num_neighbor; ++id) {
      const int row = neighbor_[id].first + pt_coor.first;
      const int col = neighbor_[id].second + pt_coor.second;
      if (row < 0 || row >= params_.vertical_ring_number || col < 0 || col >= params_.azimuth_number ||
          cloud_index_map_(row, col) == -1) {
        continue;
      }

      const auto &pt2 = cloud_in->points[cloud_index_map_(row, col)];
      const float diff = (pt1.getVector3fMap() - pt2.getVector3fMap()).norm();
      if (diff < params_.threshold_d) {
        D(id) = std::exp(-params_.threshold_s * diff);
      }
      mask[id] = label_map_(row, col);
    }

    const Eigen::VectorXf W = D / (D.sum() + 1E-6);
    float score_r = 0.0F;
    float score_g = 0.0F;
    for (int i = 0; i < num_neighbor; ++i) {
      if (mask[i] == 0) {
        score_r += W(i);
      } else if (mask[i] == 1) {
        score_g += W(i);
      }
    }
    label_map_(pt_coor.first, pt_coor.second) = score_r > score_g ? Label::NO_GROUND : Label::GROUND;
  }

  // debug
  // cvShow(label_map_, "image_jcp");
}

void JCPGroundDetection::labelPoints(const PointCloudConstPtr &cloud_in, PointCloudPtr &cloud_ground,
                                     PointCloudPtr &cloud_no_ground) {
  const size_t cloud_size = cloud_in->size();
  cloud_ground->reserve(cloud_size);
  cloud_no_ground->reserve(cloud_size);
  for (int col = 0; col < params_.azimuth_number; ++col) {
    for (int row = 0; row < params_.vertical_ring_number; ++row) {
      const auto &pt_id = cloud_index_map_(row, col);
      if (pt_id == -1) continue;

      switch (label_map_(row, col)) {
        case Label::NO_GROUND:
          cloud_no_ground->points.emplace_back(cloud_in->points[pt_id]);
          break;
        case Label::GROUND:
          cloud_ground->points.emplace_back(cloud_in->points[pt_id]);
          break;
        default:
          break;
      }
    }
  }

  for (size_t i = 0UL; i < masked_points_.size(); ++i) {
    const auto &[idx, row, col] = masked_points_[i];
    switch (label_map_(row, col)) {
      case Label::NO_GROUND:
        cloud_no_ground->points.emplace_back(cloud_in->points[idx]);
        break;
      case Label::GROUND:
        cloud_ground->points.emplace_back(cloud_in->points[idx]);
        break;
      default:
        break;
    }
  }

  cloud_ground->width = static_cast<uint32_t>(cloud_ground->size());
  cloud_ground->height = 1;
  cloud_ground->is_dense = true;

  cloud_no_ground->width = static_cast<uint32_t>(cloud_no_ground->size());
  cloud_no_ground->height = 1;
  cloud_no_ground->is_dense = true;
}

void JCPGroundDetection::postProcessing(PointCloudPtr &cloud_ground, PointCloudPtr &cloud_no_ground) {
  const size_t region_size = params_.roi_x.size() - 1UL;
  std::vector<PointCloudPtr> vec_ground_cloud(region_size, nullptr);
  for (auto &cloud : vec_ground_cloud) {
    cloud.reset(new PointCloudT());
    cloud->reserve(cloud_ground->size());
  }

  for (const auto &pt : cloud_ground->points) {
    if (pt.x <= params_.roi_x.front() || pt.x >= params_.roi_x.back()) continue;

    int region_id = 0;
    for (size_t id = 1UL; id <= region_size; ++id) {
      if (pt.x < params_.roi_x[id]) break;
      region_id += 1;
    }

    if (params_.roi_min_y < pt.y && pt.y < params_.roi_max_y) {
      vec_ground_cloud[region_id]->points.emplace_back(pt);
    }
  }

  std::vector<Eigen::Vector4f> vec_normal_vector(region_size, Eigen::Vector4f(0.0F, 0.0F, 1.0F, 0.0F));
  for (size_t i = 0UL; i < region_size; ++i) {
    const int res = estimatePlaneRANSAC(vec_ground_cloud[i], vec_normal_vector[i]);
    if (res != 0 || vec_normal_vector[i](2) < params_.threshold_direction) {
      if (i >= 1UL) {
        vec_normal_vector[i] = vec_normal_vector[i - 1UL];
      } else {
        vec_normal_vector[i] << 0.0F, 0.0F, 1.0F, 0.0F;
      }
    }
  }

  auto condition = [&](const PointT &pt) {
    if (pt.x <= params_.roi_x.front() || pt.x >= params_.roi_x.back()) return true;

    int region_id = 0;
    for (size_t id = 1UL; id <= region_size; ++id) {
      if (pt.x < params_.roi_x[id]) break;
      region_id += 1;
    }

    if (params_.roi_min_y < pt.y && pt.y < params_.roi_max_y) {
      const float distance = getPointToPlaneDistance(pt, vec_normal_vector[region_id]);
      if (distance < params_.threshold_dis) {
        return false;
      } else {
        return true;
      }
    } else {
      return true;
    }
  };
  const auto pos = std::partition(cloud_no_ground->begin(), cloud_no_ground->end(), condition);
  cloud_ground->insert(cloud_ground->end(), pos, cloud_no_ground->end());
  cloud_no_ground->resize(static_cast<size_t>(std::distance(cloud_no_ground->begin(), pos)));

  // 解决fov边界处的障碍物点云分割为地面点的问题
  // PointCloudPtr cloud_roi(new pcl::PointCloud<T>);
  // for (const auto &pt : cloud_ground->points) {
  //   if (std::abs(pt.x) < 20.0F && std::abs(pt.y) < 8.0F) {
  //     cloud_roi->points.emplace_back(pt);
  //   }
  // }
  // Eigen::Vector4f vec(0.0F, 0.0F, 1.0F, 0.0F);
  // const int res = estimatePlaneRANSAC(cloud_roi, vec);
  // if (res == 0 && vec(2) > 0.99F || std::abs(vec(3)) < 0.2F) {
  //   auto cond_ground = [&](const T &pt) {
  //     if (std::abs(pt.x) > 18.0F || std::abs(pt.y) > 10.0F) return true;
  //     const float distance = getPointToPlaneDistance(pt, vec);
  //     if (distance < 0.3F) {
  //       return true;
  //     } else {
  //       return false;
  //     }
  //   };
  //   const auto pos_ground = std::partition(cloud_ground->begin(), cloud_ground->end(), cond_ground);
  //   cloud_no_ground->insert(cloud_no_ground->end(), pos_ground, cloud_ground->end());
  //   cloud_ground->resize(static_cast<size_t>(std::distance(cloud_ground->begin(), pos_ground)));
  // }
}

int JCPGroundDetection::estimatePlaneSVD(const PointCloudPtr &cloud, Eigen::Vector4f &normal_vector) {
  if (!cloud || cloud->empty()) return -1;
  const Eigen::MatrixXf &eigen_points = cloud->getMatrixXfMap(3, 8, 0).transpose();
  const auto pc_mean = eigen_points.colwise().mean();
  const Eigen::MatrixX3f centered = eigen_points.rowwise() - pc_mean;
  const Eigen::MatrixX3f cov = (centered.adjoint() * centered) / double(eigen_points.rows() - 1);
  Eigen::JacobiSVD<Eigen::MatrixX3f> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  const Eigen::VectorXf singular_values = svd.singularValues();
  Eigen::VectorXf normal = svd.matrixU().col(2);
  if (normal(2) < 0) normal *= -1.0F;
  const float d = -(normal.transpose() * pc_mean.head<3>())(0, 0);
  normal_vector << normal(0), normal(1), normal(2), d;
  return 0;
}

float JCPGroundDetection::getPointToPlaneDistance(const PointT &p, const Eigen::Vector4f &vec) {
  return vec(0) * p.x + vec(1) * p.y + vec(2) * p.z + vec(3);
}

int JCPGroundDetection::estimatePlaneRANSAC(const PointCloudPtr &cloud, Eigen::Vector4f &normal_vector) {
  if (!cloud || cloud->empty()) return -1;
  constexpr float threshold = 0.1F;
  const size_t sz = cloud->size();
  const Eigen::MatrixXf &data = cloud->getMatrixXfMap(3, 8, 0);
  Eigen::Matrix3d coefficient_A = Eigen::Matrix3d::Identity();
  Eigen::Vector3d coefficient_b = Eigen::Vector3d::Zero();
  Eigen::Matrix<bool, Eigen::Dynamic, 1> max_inlier_id(sz, 1);
  size_t max_inlier = 0UL;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> distrib(0, sz - 1);
  std::set<int> rand_num;

  for (int i = 0; i < 30; ++i) {
    rand_num.clear();
    for (int j = 0; j < 100; ++j) {
      rand_num.insert(distrib(gen));
      if (rand_num.size() >= 3) break;
    }
    if (rand_num.size() < 3) continue;

    std::set<int>::iterator iter = rand_num.begin();
    for (int k = 0; k < 3 && iter != rand_num.end(); ++k, ++iter) {
      coefficient_A(k, 0) = cloud->points[*iter].x;
      coefficient_A(k, 1) = cloud->points[*iter].y;
      coefficient_A(k, 2) = 1.0;
      coefficient_b(k) = -cloud->points[*iter].z;
    }

    Eigen::Vector3d X = coefficient_A.colPivHouseholderQr().solve(coefficient_b);
    Eigen::Vector4f vec_temp;
    vec_temp << X.cast<float>()(0), X.cast<float>()(1), 1.0F, X.cast<float>()(2);
    vec_temp = vec_temp.array() / vec_temp.topRows(3).norm();
    const auto distance = vec_temp(0) * data.row(0).array() + vec_temp(1) * data.row(1).array() +
                          vec_temp(2) * data.row(2).array() + vec_temp(3);
    const auto inlier_id = distance.array().abs() <= threshold;
    size_t num_inlier = inlier_id.count();
    if (max_inlier < num_inlier) {
      max_inlier = num_inlier;
      max_inlier_id = inlier_id;
    }
  }

  if (max_inlier < 3) return -1;
  Eigen::MatrixXd A(max_inlier, 3);
  Eigen::VectorXd b(max_inlier);
  size_t id = 0;
  for (size_t i = 0; i < sz; ++i) {
    if (max_inlier_id(i, 0)) {
      A(id, 0) = cloud->points[i].x;
      A(id, 1) = cloud->points[i].y;
      A(id, 2) = 1.0;
      b(id) = -cloud->points[i].z;
      ++id;
    }
  }

  Eigen::Vector3d temp_x = A.colPivHouseholderQr().solve(b);
  normal_vector << temp_x.cast<float>()(0), temp_x.cast<float>()(1), 1.0F, temp_x.cast<float>()(2);
  normal_vector = normal_vector.array() / normal_vector.head(3).norm();
  return 0;
}

TRUNK_PERCEPTION_LIB_NAMESPACE_END
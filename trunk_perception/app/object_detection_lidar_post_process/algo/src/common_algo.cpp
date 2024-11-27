/**
 * @file common_algo.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-10-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "trunk_perception/app/object_detection_lidar_post_process/algo/common_algo.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

void downSamplePointCloud2D(const PointCloudT& pc_in, const Eigen::Vector2f& resolution, PointCloudT& pc_out) {
  const size_t pc_size = pc_in.size();
  if (pc_size <= 1UL) {
    pc_out = pc_in;
    return;
  }

  const Eigen::Vector2f min_bound = pc_in.getMatrixXfMap(2, 8, 0).rowwise().minCoeff();
  const Eigen::Vector2f max_bound = pc_in.getMatrixXfMap(2, 8, 0).rowwise().maxCoeff();
  if (resolution.minCoeff() * static_cast<float>(std::numeric_limits<int>::max()) <
      (max_bound - min_bound).maxCoeff()) {
    TERROR << "Voxel resolution is too small.";

    pc_out.reserve(pc_size);
    pc_out.insert(pc_out.end(), pc_in.begin(), pc_in.end());
    return;
  }

  using Vector2UL = Eigen::Matrix<size_t, 2, 1>;
  const Eigen::Vector2f res_inv = 1.0F / resolution.array();
  const Vector2UL size_xy = ((max_bound - min_bound).array() * res_inv.array()).ceil().cast<size_t>();
  std::vector<std::pair<size_t, size_t>> voxel_map;
  voxel_map.reserve(pc_size);

  for (size_t i = 0UL; i < pc_size; ++i) {
    const Eigen::Vector2f point = pc_in[i].getVector3fMap().head(2);
    const Vector2UL xy = ((point - min_bound).array() * res_inv.array() + 0.5F).cast<size_t>();
    const size_t voxel_index = xy(1) * size_xy(0) + xy(0);
    voxel_map.emplace_back(voxel_index, i);
  }

  std::sort(
      voxel_map.begin(), voxel_map.end(),
      [](const std::pair<size_t, size_t>& p1, const std::pair<size_t, size_t>& p2) { return p1.first > p2.first; });

  int count = 1;
  Eigen::Vector2d pt_mean = pc_in[voxel_map.begin()->second].getVector3fMap().head(2).cast<double>();
  size_t index_temp = voxel_map.begin()->first;

  PointT point;
  point.z = 0.0F;
  point.intensity = 0.0F;
  point.ring = 0;
  point.column = 0;
  point.distance = 0.0F;
  point.time = 0.0F;

  size_t sz_out = 0UL;
  pc_out.reserve(pc_size);
  for (size_t i = 1UL; i < pc_size; ++i) {
    const auto& pair = voxel_map[i];
    if (pair.first == index_temp) {
      pt_mean.array() += pc_in[pair.second].getVector3fMap().head(2).array().cast<double>();
      count += 1;
    } else {
      point.getVector3fMap().head(2) = (pt_mean.array() / count).cast<float>();
      pc_out.push_back(point);
      sz_out += 1UL;

      count = 1;
      pt_mean = pc_in[pair.second].getVector3fMap().head(2).cast<double>();
      index_temp = pair.first;
    }

    if (i == pc_size - 1UL) {
      point.getVector3fMap().head(2) = (pt_mean.array() / count).cast<float>();
      pc_out.push_back(point);
      sz_out += 1UL;
    }
  }

  pc_out.resize(sz_out);
}

void downSamplePointCloud2D(const PointCloudT& pc_in, const Eigen::Vector2f& resolution,
                            std::vector<cv::Point2f>& pt_out) {
  const size_t pc_size = pc_in.size();
  if (pc_size <= 1UL) {
    for (const auto& p : pc_in) {
      pt_out.emplace_back(p.x, p.y);
    }

    return;
  }

  const Eigen::Vector2f min_bound = pc_in.getMatrixXfMap(2, 8, 0).rowwise().minCoeff();
  const Eigen::Vector2f max_bound = pc_in.getMatrixXfMap(2, 8, 0).rowwise().maxCoeff();
  if (resolution.minCoeff() * static_cast<float>(std::numeric_limits<int>::max()) <
      (max_bound - min_bound).maxCoeff()) {
    TERROR << "Voxel resolution is too small.";

    pt_out.reserve(pc_size);
    for (const auto& p : pc_in) {
      pt_out.emplace_back(p.x, p.y);
    }

    return;
  }

  using Vector2UL = Eigen::Matrix<size_t, 2, 1>;
  const Eigen::Vector2f res_inv = 1.0F / resolution.array();
  const Vector2UL size_xy = ((max_bound - min_bound).array() * res_inv.array()).ceil().cast<size_t>();
  std::vector<std::pair<size_t, size_t>> voxel_map;
  voxel_map.reserve(pc_size);

  for (size_t i = 0UL; i < pc_size; ++i) {
    const Eigen::Vector2f point = pc_in[i].getVector3fMap().head(2);
    const Vector2UL xy = ((point - min_bound).array() * res_inv.array() + 0.5F).cast<size_t>();
    const size_t voxel_index = xy(1) * size_xy(0) + xy(0);
    voxel_map.emplace_back(voxel_index, i);
  }

  std::sort(
      voxel_map.begin(), voxel_map.end(),
      [](const std::pair<size_t, size_t>& p1, const std::pair<size_t, size_t>& p2) { return p1.first > p2.first; });

  int count = 1;
  Eigen::Vector2d pt_mean = pc_in[voxel_map.begin()->second].getVector3fMap().head(2).cast<double>();
  size_t index_temp = voxel_map.begin()->first;

  size_t sz_out = 0UL;
  pt_out.reserve(pc_size);
  for (size_t i = 1UL; i < pc_size; ++i) {
    const auto& pair = voxel_map[i];
    if (pair.first == index_temp) {
      pt_mean.array() += pc_in[pair.second].getVector3fMap().head(2).array().cast<double>();
      count += 1;
    } else {
      const Eigen::Vector2f pt = (pt_mean.array() / count).cast<float>();
      pt_out.emplace_back(pt(0), pt(1));
      sz_out += 1UL;

      count = 1;
      pt_mean = pc_in[pair.second].getVector3fMap().head(2).cast<double>();
      index_temp = pair.first;
    }

    if (i == pc_size - 1UL) {
      const Eigen::Vector2f pt = (pt_mean.array() / count).cast<float>();
      pt_out.emplace_back(pt(0), pt(1));
      sz_out += 1UL;
    }
  }

  pt_out.resize(sz_out);
}

// 计算polygon
int computeConvexHull(const PointCloudT& pc_in, Eigen::Matrix3Xf& polygon) {
  std::vector<cv::Point2f> contour;
  const Eigen::Vector2f resolution(0.02F, 0.02F);
  downSamplePointCloud2D(pc_in, resolution, contour);
  if (contour.size() < 3) {
    return -1;
  }

  std::vector<cv::Point2f> vec;
  cv::convexHull(contour, vec);
  if (vec.size() < 3) {
    return -1;
  }

  polygon.resize(3, vec.size());
  for (size_t i = 0; i < vec.size(); ++i) {
    polygon.col(i) << vec[i].x, vec[i].y, 0.0F;
  }

  return 0;
}

// 计算包含中心点的polygon
int computeCenterConvexHull(const PointCloudT& pc_in, const Eigen::Vector2f& center_point, Eigen::Matrix3Xf& polygon) {
  std::vector<cv::Point2f> contour;
  const Eigen::Vector2f resolution(0.02F, 0.02F);
  downSamplePointCloud2D(pc_in, resolution, contour);

  // 加入中心点
  contour.emplace_back(cv::Point2f(center_point(0), center_point(1)));
  if (contour.size() < 3) {
    return -1;
  }

  std::vector<cv::Point2f> vec;
  cv::convexHull(contour, vec);
  if (vec.size() < 3) {
    return -1;
  }

  polygon.resize(3, vec.size());
  for (size_t i = 0; i < vec.size(); ++i) {
    polygon.col(i) << vec[i].x, vec[i].y, 0.0F;
  }

  return 0;
}

void computeBoundingBox(const Eigen::Matrix3Xf& polygon, const float theta, BoundingBox& bbox) {
  // 构建旋转矩阵
  Eigen::Matrix2f min_trans;
  min_trans << std::cos(theta), std::sin(theta), -std::sin(theta), std::cos(theta);

  // 坐标转换
  Eigen::MatrixXf cluster_final = min_trans * polygon.topRows(2);

  // 取极值
  const float x_max = cluster_final.row(0).maxCoeff();
  const float x_min = cluster_final.row(0).minCoeff();
  const float y_max = cluster_final.row(1).maxCoeff();
  const float y_min = cluster_final.row(1).minCoeff();

  Eigen::Vector2f center_point;
  center_point << (x_max + x_min) * 0.5F, (y_max + y_min) * 0.5F;
  center_point = min_trans.transpose() * center_point;
  bbox.center << center_point(0), center_point(1), 0.0F;
  bbox.size << std::abs(x_max - x_min), std::abs(y_max - y_min), 0.0F;
  bbox.theta = theta;  // 弧度

  // 按顺序存储四个角点
  bbox.corners2d.col(0) << x_min, y_min;
  bbox.corners2d.col(1) << x_min, y_max;
  bbox.corners2d.col(2) << x_max, y_max;
  bbox.corners2d.col(3) << x_max, y_min;
  bbox.corners2d = min_trans.transpose() * bbox.corners2d;
}

void computeBoundingBox(const PointCloudT& pc_in, const float theta, BoundingBox& bbox) {
  Eigen::Matrix2f R;
  R << std::cos(theta), std::sin(theta), -std::sin(theta), std::cos(theta);
  const auto pc_box = R * pc_in.getMatrixXfMap(2, 8, 0);

  // 按顺序存储四个角点
  const float x_min = pc_box.row(0).minCoeff();
  const float x_max = pc_box.row(0).maxCoeff();
  const float y_min = pc_box.row(1).minCoeff();
  const float y_max = pc_box.row(1).maxCoeff();

  bbox.corners2d.col(0) << x_min, y_min;
  bbox.corners2d.col(1) << x_min, y_max;
  bbox.corners2d.col(2) << x_max, y_max;
  bbox.corners2d.col(3) << x_max, y_min;
  bbox.corners2d = R.transpose() * bbox.corners2d;

  Eigen::Vector2f center_point;
  center_point << (x_max + x_min) * 0.5F, (y_max + y_min) * 0.5F;
  center_point = R.transpose() * center_point;
  bbox.center << center_point(0), center_point(1), 0.0F;
  bbox.size << std::abs(x_max - x_min), std::abs(y_max - y_min), 0.0F;
  bbox.theta = theta;

  // 按距离从小到大重新排列角点
  int nearest_id = 0;
  bbox.corners2d.colwise().squaredNorm().minCoeff(&nearest_id);
  if (nearest_id != 0) {
    const Eigen::Matrix<float, 2, 4> temp = bbox.corners2d;
    const int sz = bbox.corners2d.cols();
    for (int i = 0; i < sz; ++i) {
      bbox.corners2d.col(i) = temp.col((nearest_id + i) % sz);
    }
  }
}

float computeFreespaceArea(const Eigen::Matrix3Xf& polygon, const BoundingBox& bbox) {
  const size_t sz = polygon.cols();
  float area = 0.0F;
  for (size_t i = 0; i < sz; ++i) {
    if (polygon(2, i) > 0.5F) {
      const size_t ii = (i + 1) % sz;
      for (size_t k = 0; k < 4; ++k) {
        const float x1 = polygon(0, ii) - polygon(0, i);
        const float y1 = polygon(1, ii) - polygon(1, i);
        const float x2 = bbox.corners2d(0, k) - polygon(0, i);
        const float y2 = bbox.corners2d(1, k) - polygon(1, i);
        const float signedArea = x2 * y1 - x1 * y2;

        // the corner is at the right of edge
        if (signedArea > 0.0F) {
          area += signedArea;
        }
      }
    }
  }
  return area;
}

void computeVisibleLine(Eigen::Matrix3Xf& polygon) {
  const size_t sz = polygon.cols();

  size_t minIdx = 0UL;
  size_t maxIdx = 0UL;
  float minAngle = 100.0F;
  float maxAngle = -100.0F;
  for (size_t i = 0; i < sz; ++i) {
    const float angle = std::atan2(polygon(1, i), polygon(0, i));
    if (angle < minAngle) {
      minIdx = i;
      minAngle = angle;
    }
    if (angle > maxAngle) {
      maxIdx = i;
      maxAngle = angle;
    }
  }

  // 处理角度在pi和-pi发生变换的情况
  if (M_PI_2 < maxAngle && maxAngle < M_PI && -M_PI < minAngle && minAngle < -M_PI_2) {
    minAngle = 100.0F;
    maxAngle = -100.0F;
    for (size_t i = 0; i < sz; ++i) {
      float angle = std::atan2(polygon(1, i), polygon(0, i));
      angle = std::signbit(angle) ? (angle + 2 * M_PI) : angle;
      if (angle < minAngle) {
        minIdx = i;
        minAngle = angle;
      }
      if (angle > maxAngle) {
        maxIdx = i;
        maxAngle = angle;
      }
    }
  }

  if (maxIdx < minIdx) {
    for (size_t i = maxIdx; i < minIdx; ++i) {
      polygon(2, i) = 1.0F;  // 1 = visible, 0 = invisible
    }
  } else {
    for (size_t i = maxIdx; i < minIdx + sz; ++i) {
      polygon(2, i % sz) = 1.0F;  // 1 = visible, 0 = invisible
    }
  }
}

void polygon2BBox(Eigen::Matrix3Xf& polygon, BoundingBox& bbox) {
  // 计算可视边
  computeVisibleLine(polygon);

  const size_t sz = polygon.cols();
  float min_freespace_area = FLT_MAX;
  float opt_theta = 0.0F;
  for (size_t i = 0; i < sz; ++i) {
    if (polygon(2, i) < 0.5F) {
      continue;
    }
    const size_t ii = (i + 1) % sz;
    const float theta = std::atan2(polygon(1, i) - polygon(1, ii), polygon(0, i) - polygon(0, ii));

    computeBoundingBox(polygon, theta, bbox);
    const float area = computeFreespaceArea(polygon, bbox);
    if (area < min_freespace_area) {
      min_freespace_area = area;
      opt_theta = theta;
    }
  }
  computeBoundingBox(polygon, opt_theta, bbox);

  // 按距离从小到大重新排列角点
  int nearest_id = 0;
  bbox.corners2d.colwise().squaredNorm().minCoeff(&nearest_id);
  if (nearest_id != 0) {
    const Eigen::Matrix<float, 2, 4> temp = bbox.corners2d;
    const int sz = bbox.corners2d.cols();
    for (int i = 0; i < sz; ++i) {
      bbox.corners2d.col(i) = temp.col((nearest_id + i) % sz);
    }
  }
}

double getOverlapRate(const Eigen::MatrixXf& polygon1, const Eigen::MatrixXf& polygon2, const int& scale) {
  if (polygon1.rows() != 2 && polygon1.rows() != 3) {
    TFATAL << "polygon1 input size(rows, cols) error!";
  }

  if (polygon2.rows() != 2 && polygon2.rows() != 3) {
    TFATAL << "polygon2 input size(rows, cols) error!";
  }

  const float min_x_1 = polygon1.row(0).minCoeff();
  const float max_x_1 = polygon1.row(0).maxCoeff();
  const float min_y_1 = polygon1.row(1).minCoeff();
  const float max_y_1 = polygon1.row(1).maxCoeff();
  const float min_x_2 = polygon2.row(0).minCoeff();
  const float max_x_2 = polygon2.row(0).maxCoeff();
  const float min_y_2 = polygon2.row(1).minCoeff();
  const float max_y_2 = polygon2.row(1).maxCoeff();

  if (min_x_1 > max_x_2 || min_y_1 > max_y_2 || min_x_2 > max_x_1 || min_y_2 > max_y_1) {
    return 0.0;
  }

  const float min_x = std::min(min_x_1, min_x_2);
  const float min_y = std::min(min_y_1, min_y_2);
  const float max_x = std::max(max_x_1, max_x_2);
  const float max_y = std::max(max_y_1, max_y_2);

  const int h = static_cast<int>((max_y - min_y + 2.0F) * scale);
  const int w = static_cast<int>((max_x - min_x + 2.0F) * scale);
  cv::Mat im_1 = cv::Mat::zeros(h, w, CV_8UC1);
  cv::Mat im_2 = cv::Mat::zeros(h, w, CV_8UC1);

  const int size_1 = polygon1.cols();
  const int size_2 = polygon2.cols();
  std::vector<cv::Point> pts1(size_1);
  std::vector<cv::Point> pts2(size_2);
  for (int i = 0; i < size_1; ++i) {
    pts1[i].x = static_cast<int>((polygon1(0, i) - min_x + 1.0F) * scale);
    pts1[i].y = static_cast<int>((polygon1(1, i) - min_y + 1.0F) * scale);
  }
  for (int i = 0; i < size_2; ++i) {
    pts2[i].x = static_cast<int>((polygon2(0, i) - min_x + 1.0F) * scale);
    pts2[i].y = static_cast<int>((polygon2(1, i) - min_y + 1.0F) * scale);
  }

  cv::fillConvexPoly(im_1, pts1, cv::Scalar(1));
  cv::fillConvexPoly(im_2, pts2, cv::Scalar(1));
  cv::Mat im_inner = im_1 & im_2;

  const double inner = cv::sum(im_inner).val[0];
  const double v1 = cv::sum(im_1).val[0];
  const double v2 = cv::sum(im_2).val[0];
  return inner / (std::min(v1, v2) + 1E-6);
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
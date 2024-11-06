/**
 * @file cluster_detection.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/app/object_detection_lidar_post_process/algo/cluster_detection.h"
#include "trunk_perception/algo/track/tracker_pipeline/tracker_pipeline_manager.h"
#include "trunk_perception/app/object_detection_lidar_post_process/algo/common_algo.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

int ClusterDetection::Init(const YAML::Node& config) {
  try {
    const std::string cluster_method = config["ClusterMethod"].as<std::string>();
    cluster_ = ClusterManager<PointT>::Create(cluster_method);
    auto res = cluster_->init(config["ClusterParams"]);
    if (res) {
      TFATAL << "ClusterDetection::Init cluster init failed!";
      return 1;
    }

    // remove object points param
    if (config["RemoveObjectPoints"].IsDefined()) {
      remove_object_points_ = config["RemoveObjectPoints"]["Switch"].as<bool>();
      if (remove_object_points_) {
        const auto& rop_config = config["RemoveObjectPoints"]["Params"];
        rop_params_.scale = rop_config["scale"].as<float>();
        rop_params_.x_range = rop_config["x_range"].as<float>();
        rop_params_.y_range = rop_config["y_range"].as<float>();
        rop_params_.x_offset = rop_config["x_offset"].as<float>();
        rop_params_.y_offset = rop_config["y_offset"].as<float>();
        rop_params_.x_dilate = rop_config["x_dilate"].as<float>();
        rop_params_.y_dilate = rop_config["y_dilate"].as<float>();
      }
    }

    // height filter param
    const auto& height_filter_config = config["FilterByRules"]["HeightFilter"];
    params_.height_filter_switch = height_filter_config["Switch"].as<bool>();
    params_.range_x_threshold = height_filter_config["range_x_threshold"].as<float>();
    params_.height_threshold = height_filter_config["height_threshold"].as<float>();

    // intensity filter param
    const auto& intensity_filter_config = config["FilterByRules"]["IntensityFilter"];
    params_.intensity_filter_switch = intensity_filter_config["Switch"].as<bool>();
    params_.low_intensity_threshold = intensity_filter_config["low_intensity_threshold"].as<float>();
    params_.high_intensity_threshold = intensity_filter_config["high_intensity_threshold"].as<float>();
    params_.high_count_threshold = intensity_filter_config["high_count_threshold"].as<int>();
    params_.ratio_threshold = intensity_filter_config["ratio_threshold"].as<float>();

    // merge objects参数
    params_.dilate_x = config["MergeObjects"]["dilate_x"].as<float>();
    params_.dilate_y = config["MergeObjects"]["dilate_y"].as<float>();
    params_.overlap_thresh = config["MergeObjects"]["overlap_thresh"].as<float>();
  } catch (const std::exception& e) {
    TFATAL << "ClusterDetection::Init Failed to load config: " << e.what();
    return 1;
  }
  return 0;
}

int ClusterDetection::Process(std::shared_ptr<OdLidarFrame>& frame) {
  const auto& noground_cloud = frame->noground_cloud;
  if (noground_cloud == nullptr || cluster_ == nullptr) {
    TERROR << "ClusterDetection::Process noground_cloud or cluster is nullptr!";
    return 1;
  }

  // 吸收掉模型检测bbox内的点
  PointCloudConstPtr obstacle_cloud = nullptr;
  if (remove_object_points_) {
    removeObjectPoints(noground_cloud, frame->tracked_objects, obstacle_cloud);
  } else {
    obstacle_cloud = noground_cloud;
  }

  // 点云聚类
  if (cluster_->process(obstacle_cloud)) {
    TERROR << "ClusterDetection::Process cluster process failed!";
    return 2;
  }
  const auto& clusters_cloud = cluster_->getClustersCloud();
  frame->clusters_cloud = clusters_cloud;

  // 聚类点云构建object
  auto& cluster_objects = frame->cluster_objects;
  buildObjects(clusters_cloud, cluster_objects, frame->timestamp);

  // 基于规则滤除
  filterByRules(cluster_objects);

  // nms
  nms(cluster_objects);

  // merge tracked objects and cluster objects
  mergeObjects(cluster_objects, frame->tracked_objects);

  return 0;
}

void ClusterDetection::removeObjectPoints(const PointCloudConstPtr& cloud, const std::vector<Object>& objects,
                                          PointCloudConstPtr& obstacle_cloud) {
  if (objects.empty()) {
    obstacle_cloud = cloud;
    return;
  }

  const auto& scale = rop_params_.scale;
  const auto& rows = rop_params_.x_range * scale;
  const auto& cols = rop_params_.y_range * scale;
  const auto& shift_row = rop_params_.x_offset * scale;
  const auto& shift_col = rop_params_.y_offset * scale;
  cv::Mat im_objects = cv::Mat::zeros(rows, cols, CV_32SC1);  // 图片的行对应点云坐标x、列对应点云坐标y

  cv::Point pt_cv;  // (列，行)
  std::vector<cv::Point> points_cv(4);
  const size_t sz_objs = objects.size();
  for (size_t i = 0UL; i < sz_objs; ++i) {
    // trick: 扩大bbox用于吸收bbox边缘外的点
    BoundingBox bbox = objects[i].bbox;
    bbox.size(0) += rop_params_.x_dilate;
    bbox.size(1) += rop_params_.y_dilate;
    Eigen::Rotation2D<float> R2D(bbox.theta);
    bbox.corners2d.col(0) = R2D * Eigen::Vector2f(bbox.size(0), bbox.size(1)) * 0.5f + bbox.center.head(2);
    bbox.corners2d.col(1) = R2D * Eigen::Vector2f(bbox.size(0), -bbox.size(1)) * 0.5f + bbox.center.head(2);
    bbox.corners2d.col(2) = R2D * Eigen::Vector2f(-bbox.size(0), -bbox.size(1)) * 0.5f + bbox.center.head(2);
    bbox.corners2d.col(3) = R2D * Eigen::Vector2f(-bbox.size(0), bbox.size(1)) * 0.5f + bbox.center.head(2);

    points_cv.clear();
    const auto& corners2d = bbox.corners2d;
    for (size_t j = 0UL; j < corners2d.cols(); ++j) {
      pt_cv.x = static_cast<int>(corners2d(1, j) * scale + shift_col);  // 点云的y值计算出图片中的列坐标
      pt_cv.y = static_cast<int>(corners2d(0, j) * scale + shift_row);  // 点云的x值计算出图片中的行坐标
      points_cv.emplace_back(pt_cv);
    }
    cv::fillConvexPoly(im_objects, points_cv, cv::Scalar(i + 1));
  }

  PointCloudPtr cloud_valid = std::make_shared<PointCloudT>();
  for (const auto& pt : cloud->points) {
    pt_cv.x = static_cast<int>(pt.y * scale + shift_col);  // 点云的y值计算出图片中的列坐标
    pt_cv.y = static_cast<int>(pt.x * scale + shift_row);  // 点云的x值计算出图片中的行坐标
    if (0 <= pt_cv.x && pt_cv.x < im_objects.cols && 0 <= pt_cv.y && pt_cv.y < im_objects.rows &&
        im_objects.at<int>(pt_cv.y, pt_cv.x) == 0) {
      cloud_valid->push_back(pt);
    }
  }

  obstacle_cloud = cloud_valid;
}

void ClusterDetection::buildObjects(const std::vector<PointCloudConstPtr>& clusters_cloud, std::vector<Object>& objects,
                                    const double ts) {
  objects.clear();
  if (clusters_cloud.empty()) return;
  for (const auto& cluster : clusters_cloud) {
    Object object;
    object.timestamp = ts;
    object.detector_type = DetectorType::RuleBased;

    // 计算polygon
    if (computeConvexHull(*cluster, object.convex_polygon)) continue;

    // 拟合bbox
    polygon2BBox(object.convex_polygon, object.bbox);

    object.confidence = 1.0;  // 默认值1.0
    object.type = ObjectType::UNKNOWN;
    object.points_ptr = std::make_shared<PointCloudT>();
    object.points_ptr->points.assign(cluster->begin(), cluster->end());
    object.track_point.setZero();
    object.track_point.head(2) = object.bbox.corners2d.col(0);
    objects.emplace_back(object);
  }
}

void ClusterDetection::filterByRules(std::vector<Object>& objects) {
  auto cond = [&](const Object& object) {
    // 滤除悬空障碍物(如伸入车道的树枝、喷洒的水雾等)
    if (params_.height_filter_switch) {
      const auto& points_ptr = object.points_ptr;
      if (!points_ptr) return true;
      const float min_z = points_ptr->getMatrixXfMap(1, 8, 2).minCoeff();
      if (object.bbox.center.x() > params_.range_x_threshold && min_z > params_.height_threshold) {
        return true;
      }
    }

    // 滤除点云强度较低的障碍物（如水雾等）
    if (params_.intensity_filter_switch) {
      const auto& points_ptr = object.points_ptr;
      if (!points_ptr || points_ptr->empty()) return true;
      const size_t sz = points_ptr->size();
      int count_low = 0;
      int count_high = 0;
      for (const auto& pt : points_ptr->points) {
        if (pt.intensity < params_.low_intensity_threshold) {
          count_low += 1;
        }

        if (pt.intensity > params_.high_intensity_threshold) {
          count_high += 1;
        }
      }

      const float ratio = static_cast<float>(count_low) / sz;
      if ((count_high <= params_.high_count_threshold && ratio > params_.ratio_threshold)) {
        return true;
      }
    }

    return false;
  };
  objects.erase(std::remove_if(objects.begin(), objects.end(), cond), objects.end());
}

void ClusterDetection::nms(std::vector<Object>& objects) {
  if (objects.empty()) return;
  auto cond = [&](const Object& a, const Object& b) { return a.bbox.size.head(2).prod() > b.bbox.size.head(2).prod(); };
  std::sort(objects.begin(), objects.end(), cond);

  auto cond_removed = [&](const Object& obj) -> bool { return std::signbit(obj.confidence); };
  for (size_t i = 0UL; i < objects.size(); ++i) {
    auto& object_i = objects[i];
    if (cond_removed(object_i)) continue;

    bool flag = false;
    for (size_t j = i + 1UL; j < objects.size(); ++j) {
      auto& object_j = objects[j];
      if (cond_removed(object_j)) continue;
      const double iou = getOverlapRate(object_i.convex_polygon, object_j.convex_polygon);
      if (iou > 0.001) {
        flag = true;
        object_j.confidence = -1.0F;  // 将要移除的目标confidence置为-1.0
        object_i.points_ptr->points.insert(object_i.points_ptr->points.end(), object_j.points_ptr->points.begin(),
                                           object_j.points_ptr->points.end());
      }
    }

    if (flag) {
      if (computeConvexHull(*object_i.points_ptr, object_i.convex_polygon)) continue;
      polygon2BBox(object_i.convex_polygon, object_i.bbox);
    }
  }

  objects.erase(std::remove_if(objects.begin(), objects.end(), cond_removed), objects.end());
}

void ClusterDetection::mergeObjects(const std::vector<Object>& cluster_objects, std::vector<Object>& merged_objects) {
  if (cluster_objects.empty()) return;

  std::vector<int> assignment(cluster_objects.size(), -1);
  for (size_t i = 0UL; i < merged_objects.size(); ++i) {
    // trick: 扩大模型检测的bbox吸收聚类的bbox
    BoundingBox bbox = merged_objects[i].bbox;
    bbox.size(0) += params_.dilate_x;
    bbox.size(1) += params_.dilate_y;
    Eigen::Rotation2D<float> R2D(bbox.theta);
    bbox.corners2d.col(0) = R2D * Eigen::Vector2f(bbox.size(0), bbox.size(1)) * 0.5f + bbox.center.head(2);
    bbox.corners2d.col(1) = R2D * Eigen::Vector2f(bbox.size(0), -bbox.size(1)) * 0.5f + bbox.center.head(2);
    bbox.corners2d.col(2) = R2D * Eigen::Vector2f(-bbox.size(0), -bbox.size(1)) * 0.5f + bbox.center.head(2);
    bbox.corners2d.col(3) = R2D * Eigen::Vector2f(-bbox.size(0), bbox.size(1)) * 0.5f + bbox.center.head(2);

    for (size_t j = 0UL; j < cluster_objects.size(); ++j) {
      const double overlap = getOverlapRate(bbox.corners2d, cluster_objects[j].convex_polygon);
      if (overlap > params_.overlap_thresh) {
        assignment[j] = i;
      }
    }
  }

  for (size_t j = 0UL; j < cluster_objects.size(); ++j) {
    if (assignment[j] == -1) {
      merged_objects.emplace_back(cluster_objects[j]);
    }
  }
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
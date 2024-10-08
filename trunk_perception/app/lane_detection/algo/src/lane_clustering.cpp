#include "trunk_perception/app/lane_detection/algo/lane_clustering.h"
#include <algorithm>
#include <cmath>
#include <map>

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_algo {

std::vector<cv::Point3f> LaneClustering::Cluster(const std::vector<cv::Point3f>& points) {
  if (points.empty()) return {};

  std::vector<int> labels = DBSCAN(points);
  return ExtractLargestCluster(points, labels);
}

float LaneClustering::Distance(const cv::Point3f& pt1, const cv::Point3f& pt2) {
  return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2) + std::pow(pt1.z - pt2.z, 2));
}

std::vector<int> LaneClustering::DBSCAN(const std::vector<cv::Point3f>& points) {
  const int UNCLASSIFIED = -1;
  const int NOISE = -2;
  int cluster_id = 0;
  std::vector<int> labels(points.size(), UNCLASSIFIED);

  for (size_t i = 0; i < points.size(); ++i) {
    if (labels[i] == UNCLASSIFIED) {
      std::vector<int> neighbors;

      for (size_t j = 0; j < points.size(); ++j) {
        if (Distance(points[i], points[j]) <= distance_threshold_) {
          neighbors.push_back(j);
        }
      }

      if (neighbors.size() < size_t(min_points_)) {
        labels[i] = NOISE;
      } else {
        for (auto idx : neighbors) {
          labels[idx] = cluster_id;
        }

        size_t k = 0;
        while (k < neighbors.size()) {
          int current_point = neighbors[k];
          std::vector<int> current_neighbors;

          for (size_t j = 0; j < points.size(); ++j) {
            if (Distance(points[current_point], points[j]) <= distance_threshold_) {
              current_neighbors.push_back(j);
            }
          }

          if (current_neighbors.size() >= size_t(min_points_)) {
            for (auto idx : current_neighbors) {
              if (labels[idx] == UNCLASSIFIED || labels[idx] == NOISE) {
                if (labels[idx] == UNCLASSIFIED) {
                  neighbors.push_back(idx);
                }
                labels[idx] = cluster_id;
              }
            }
          }

          ++k;
        }

        ++cluster_id;
      }
    }
  }

  return labels;
}

std::vector<cv::Point3f> LaneClustering::ExtractLargestCluster(const std::vector<cv::Point3f>& points,
                                                               const std::vector<int>& labels) {
  std::map<int, int> cluster_sizes;
  for (const auto& label : labels) {
    if (label >= 0) {
      cluster_sizes[label]++;
    }
  }

  int largest_cluster = -1;
  int max_size = 0;
  for (const auto& cluster_size : cluster_sizes) {
    auto cluster_id = cluster_size.first;
    auto size = cluster_size.second;
    if (size > max_size) {
      largest_cluster = cluster_id;
      max_size = size;
    }
  }

  std::vector<cv::Point3f> largest_cluster_points;
  for (size_t i = 0; i < points.size(); ++i) {
    if (labels[i] == largest_cluster) {
      largest_cluster_points.push_back(points[i]);
    }
  }

  return largest_cluster_points;
}
}  // namespace ld_algo

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END

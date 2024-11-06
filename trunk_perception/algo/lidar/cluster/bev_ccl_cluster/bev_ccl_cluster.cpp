/**
 * @file bev_ccl_cluster.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <stack>

#include "trunk_perception/algo/lidar/cluster/bev_ccl_cluster/bev_ccl_cluster.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

template <class T>
int BEVCCLCluster<T>::init(const YAML::Node& config) {
  try {
    params_.map_range_min_x = config["map_range_min_x"].as<float>();
    params_.map_range_max_x = config["map_range_max_x"].as<float>();
    params_.map_range_min_y = config["map_range_min_y"].as<float>();
    params_.map_range_max_y = config["map_range_max_y"].as<float>();
    params_.map_resolution_x = config["map_resolution_x"].as<float>();
    params_.map_resolution_y = config["map_resolution_y"].as<float>();
    params_.map_range_min_z = config["map_range_min_z"].as<float>();
    params_.map_range_max_z = config["map_range_max_z"].as<float>();
    params_.map_resolution_x_inv = 1.0F / params_.map_resolution_x;
    params_.map_resolution_y_inv = 1.0F / params_.map_resolution_y;
    params_.map_size_x =
        static_cast<int>((params_.map_range_max_x - params_.map_range_min_x) * params_.map_resolution_x_inv);
    params_.map_size_y =
        static_cast<int>((params_.map_range_max_y - params_.map_range_min_y) * params_.map_resolution_y_inv);
    params_.min_points_num = config["min_points_num"].as<int>();
  } catch (const YAML::Exception& e) {
    TFATAL << "BEVCCLCluster config error.";
    return 1;
  }

  bev_map_.resize(params_.map_size_y, params_.map_size_x);
  labeled_map_.resize(params_.map_size_y, params_.map_size_x);

  bev_cell_.resize(params_.map_size_y);
  for (auto& cell : bev_cell_) {
    cell.resize(params_.map_size_x);
  }

  return 0;
}

template <class T>
int BEVCCLCluster<T>::process(const typename std::shared_ptr<const pcl::PointCloud<T>>& cloud_in) {
  if (cloud_in == nullptr) return 1;
  if (cloud_in->empty()) return 0;

  reset();

  makeBev(cloud_in, bev_cell_, bev_map_);

  CCL(bev_map_, labeled_map_);

  labelPoints(cloud_in, bev_cell_, labeled_map_);

  return 0;
}

template <class T>
std::vector<typename std::shared_ptr<const pcl::PointCloud<T>>> BEVCCLCluster<T>::getClustersCloud() {
  return clusters_cloud_;
};

template <class T>
void BEVCCLCluster<T>::reset() {
  for (auto& g : bev_cell_) {
    for (auto& r : g) {
      r.clear();
    }
  }

  labeled_map_.setConstant(-1);
  bev_map_.setConstant(-1);
  clusters_cloud_.clear();
}

template <class T>
void BEVCCLCluster<T>::makeBev(const typename std::shared_ptr<const pcl::PointCloud<T>>& cloud_in, BevCell& bev_cell,
                               Eigen::MatrixXi& bev_map) {
  const size_t pc_size = cloud_in->points.size();
  const Eigen::MatrixXf pc_eigen = cloud_in->getMatrixXfMap(3, 8, 0);  // 3 x n
  const Eigen::VectorXi col =
      ((pc_eigen.row(0).array() - params_.map_range_min_x) * params_.map_resolution_x_inv).floor().cast<int>();
  const Eigen::VectorXi row =
      ((pc_eigen.row(1).array() - params_.map_range_min_y) * params_.map_resolution_y_inv).floor().cast<int>();

  for (size_t i = 0; i < pc_size; ++i) {
    const auto& pt = cloud_in->points[i];
    if (pt.z < params_.map_range_min_z || pt.z > params_.map_range_max_z) continue;
    if (row(i) >= 0 && row(i) < params_.map_size_y && col(i) >= 0 && col(i) < params_.map_size_x) {
      bev_cell[row[i]][col[i]].emplace_back(i);
    }
  }

  for (size_t i = 0; i < params_.map_size_y; ++i) {
    for (size_t j = 0; j < params_.map_size_x; ++j) {
      if (bev_cell[i][j].size() >= 1) {
        bev_map(i, j) = 1;
      }
    }
  }
}

template <class T>
void BEVCCLCluster<T>::CCL(const Eigen::MatrixXi& bev_map, Eigen::MatrixXi& labeled_map) {
  constexpr int kSize = 1;
  const size_t rows = bev_map.rows();
  const size_t cols = bev_map.cols();

  // 边沿越界保护
  Eigen::MatrixXi tmp_map = Eigen::MatrixXi::Constant(rows + kSize * 2, cols + kSize * 2, -1);
  tmp_map.block(kSize, kSize, rows, cols) = bev_map;

  int label_id = 3;
  for (int r = 0; r < tmp_map.rows(); ++r) {
    for (int c = 0; c < tmp_map.cols(); ++c) {
      if (tmp_map(r, c) != 1) {
        continue;
      }

      std::stack<std::pair<size_t, size_t>> neighborPixels;
      neighborPixels.push(std::pair<size_t, size_t>(r, c));
      ++label_id;

      while (!neighborPixels.empty()) {
        std::pair<size_t, size_t> curPixel = neighborPixels.top();
        const size_t curR = curPixel.first;
        const size_t curC = curPixel.second;
        tmp_map(curR, curC) = label_id;
        neighborPixels.pop();

        // push the n - neighbors
        if (tmp_map(curR, curC - 1) == 1) {
          neighborPixels.push(std::pair<size_t, size_t>(curR, curC - 1));
          tmp_map(curR, curC - 1) = label_id;
        }
        if (tmp_map(curR, curC + 1) == 1) {
          neighborPixels.push(std::pair<size_t, size_t>(curR, curC + 1));
          tmp_map(curR, curC + 1) = label_id;
        }
        if (tmp_map(curR - 1, curC) == 1) {
          neighborPixels.push(std::pair<size_t, size_t>(curR - 1, curC));
          tmp_map(curR - 1, curC) = label_id;
        }
        if (tmp_map(curR + 1, curC) == 1) {
          neighborPixels.push(std::pair<size_t, size_t>(curR + 1, curC));
          tmp_map(curR + 1, curC) = label_id;
        }
      }
    }
  }
  labeled_map.swap(tmp_map.block(kSize, kSize, rows, cols));
}

template <class T>
void BEVCCLCluster<T>::labelPoints(const typename std::shared_ptr<const pcl::PointCloud<T>>& cloud_in,
                                   const BevCell& bev_cell, const Eigen::MatrixXi& labeled_map) {
  // 收集目标点的idx
  std::map<int, std::vector<int>> clusters_idx;
  for (size_t r = 0; r < params_.map_size_y; ++r) {
    for (size_t c = 0; c < params_.map_size_x; ++c) {
      int label = labeled_map(r, c);
      if (label > 2) {
        clusters_idx[label].insert(clusters_idx[label].end(), bev_cell_[r][c].begin(), bev_cell_[r][c].end());
      }
    }
  }

  clusters_cloud_.clear();
  clusters_cloud_.reserve(clusters_idx.size());
  for (auto& cluster : clusters_idx) {
    if (cluster.second.size() < static_cast<size_t>(params_.min_points_num)) {
      continue;
    }

    typename std::shared_ptr<pcl::PointCloud<T>> cloud_temp = std::make_shared<pcl::PointCloud<T>>();
    cloud_temp->points.reserve(cluster.second.size());
    for (auto& idx : cluster.second) {
      cloud_temp->points.emplace_back(cloud_in->points[idx]);
    }

    clusters_cloud_.emplace_back(cloud_temp);
  }
}

template class BEVCCLCluster<PointT>;

TRUNK_PERCEPTION_LIB_NAMESPACE_END
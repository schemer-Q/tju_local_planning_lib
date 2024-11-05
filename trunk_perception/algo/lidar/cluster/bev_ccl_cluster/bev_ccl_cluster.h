/**
 * @file bev_ccl_cluster.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <Eigen/Dense>

#include "trunk_perception/algo/lidar/cluster/cluster_base.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

struct BEVCCLClusterParams {
  float map_range_min_x = 0.0F;
  float map_range_max_x = 120.0F;
  float map_range_min_y = -30.0F;
  float map_range_max_y = 30.0F;
  float map_range_min_z = -2.0F;
  float map_range_max_z = 3.0F;
  float map_resolution_x = 2.0F;
  float map_resolution_y = 0.5F;
  float map_resolution_x_inv = 1.0f / map_resolution_x;
  float map_resolution_y_inv = 1.0f / map_resolution_y;
  int map_size_x = static_cast<int>((map_range_max_x - map_range_min_x) * map_resolution_x_inv);
  int map_size_y = static_cast<int>((map_range_max_y - map_range_min_y) * map_resolution_y_inv);
  int min_points_num = 20;
};

template <class T>
class BEVCCLCluster : virtual public ClusterBase<T> {
 private:
  using BevCell = std::vector<std::vector<std::vector<size_t>>>;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BEVCCLCluster() = default;
  ~BEVCCLCluster() = default;

  /**
   * @brief init param
   *
   * @param config yaml node
   * @return int
   */
  int init(const YAML::Node& config) override;

  /**
   * @brief process pipeline
   *
   * @param cloud_in input point cloud
   * @return int
   */
  int process(const typename std::shared_ptr<const pcl::PointCloud<T>>& cloud_in) override;

  /**
   * @brief get the clusters cloud object
   *
   * @return std::vector<typename std::shared_ptr<pcl::PointCloud<T>>>
   */
  std::vector<typename std::shared_ptr<pcl::PointCloud<T>>> getClustersCloud() override;

 private:
  void reset();
  void makeBev(const typename std::shared_ptr<const pcl::PointCloud<T>>& cloud_in, BevCell& bev_cell,
               Eigen::MatrixXi& bev_map);
  void CCL(const Eigen::MatrixXi& bev_map, Eigen::MatrixXi& labeled_map);
  void labelPoints(const typename std::shared_ptr<const pcl::PointCloud<T>>& cloud_in, const BevCell& bev_cell,
                   const Eigen::MatrixXi& labeled_map);

 private:
  BEVCCLClusterParams params_;
  BevCell bev_cell_;
  Eigen::MatrixXi bev_map_;
  Eigen::MatrixXi labeled_map_;
  std::vector<typename std::shared_ptr<pcl::PointCloud<T>>> clusters_cloud_;
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END
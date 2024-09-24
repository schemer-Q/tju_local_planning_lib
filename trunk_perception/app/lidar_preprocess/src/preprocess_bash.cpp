#include <cstddef>
#include <memory>
#include "trunk_perception/app/lidar_preprocess/preprocess_base.h"
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/types/point.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

PreprocessBase::PreprocessBase() = default;

PreprocessBase::~PreprocessBase() = default;

std::uint32_t PreprocessBase::Init(const YAML::Node& config) {
  try {
    lidar_name_ = config["Name"].as<std::string>();
  } catch (const std::exception& e) {
    TFATAL << "PreprocessBase::Init failed, " << e.what();
    return ErrorCode::YAML_CONFIG_ERRPR;
  }
  return ErrorCode::SUCCESS;
}

std::uint32_t PreprocessBase::Run(const double& ts) {
  // init
  tf_cloud_ = nullptr;
  uint32_t ret = ErrorCode::SUCCESS;

  // 获取tf变换
  auto tf = GET_SENSOR_POSE(lidar_name_);
  if (tf == nullptr) {
    TERROR << "PreprocessBase::Run failed, get " << lidar_name_ << " tf failed";
    return ErrorCode::SENSOR_POSE_NOT_FOUND;
  }

  // 获取点云
  std::shared_ptr<PointCloudData> origin_cloud = nullptr;
  ret = GET_SENSOR_DATA_BY_TIME(lidar_name_, "origin", ts, origin_cloud);
  if (ret != ErrorCode::SUCCESS) {
    TERROR << "PreprocessBase::Run failed, get " << lidar_name_ << " cloud failed";
    return ret;
  }
  if (origin_cloud == nullptr) {
    TERROR << "PreprocessBase::Run failed, get " << lidar_name_ << " cloud is nullptr";
    return ErrorCode::POINT_CLOUD_INVALID;
  }

  // tf变换
  tf_cloud_ = TransformPointCloud(origin_cloud->data, *tf);

  // 保存点云
  ret = PUSH_SENSOR_DATA(lidar_name_, "tf", ts, tf_cloud_);
  if (ret != ErrorCode::SUCCESS) {
    TERROR << "PreprocessBase::Run failed, push " << lidar_name_ << " tf cloud failed";
    return ret;
  }

  return ErrorCode::SUCCESS;
}

PointCloudPtr PreprocessBase::TransformPointCloud(const PointCloudConstPtr& cloud, const Eigen::Isometry3f& tf) {
  if (cloud == nullptr) {
    TERROR << "PreprocessBase::TransformPointCloud failed, cloud is nullptr";
    return nullptr;
  }
  PointCloudPtr tf_cloud(new PointCloudT(*cloud));
  auto cloud_xyz = tf_cloud->getMatrixXfMap(4, sizeof(PointT) / sizeof(float), 0);
  cloud_xyz.row(3).setOnes();
  cloud_xyz = (tf.matrix() * cloud_xyz).eval();

  return tf_cloud;
}

std::any PreprocessBase::GetData(const std::string& key) {
  if (key == "tf") {
    return tf_cloud_;
  }
  return nullptr;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END

/**
 * @file data_manager.h
 * @brief 数据管理器，管理当前车型所有传感器数据
 */

#pragma once

#include <sys/types.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "trunk_perception/common/data_manager/data_wrapper/pointcloud_data.h"
#include "trunk_perception/common/data_manager/sensor_wrapper/base.h"
#include "trunk_perception/common/data_manager/sensor_wrapper/camera.hpp"
#include "trunk_perception/common/data_manager/sensor_wrapper/lidar.hpp"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/point.h"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/tools/system/utils.hpp"
#include "trunk_perception/common/data_manager/data_wrapper/od_lidar_frame.h"
TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

enum SensorType {
  UNKNOWN,
  LIDAR,
  CAMERA,
};

class DataManager {
 public:
  static DataManager& instance();

  ~DataManager() = default;

  uint32_t registerSensor(const SensorType& type, const std::string& name, const std::vector<std::string>& types,
                          const uint32_t& buffer_size, const double& max_time_delay);

  uint32_t registerSensorsByConfig(const YAML::Node& config);

  uint32_t registerSensorsByFile(const std::string& file_path);

  std::string getVehicleName() const;

  void setVehicleName(const std::string& vehicle_name);

  uint32_t push(const std::string& sensor_name, const std::string& data_type, const double& timestamp,
                const std::shared_ptr<PointCloudT>& data);

  uint32_t push(const std::string& sensor_name, const std::string& data_type, const double& timestamp,
                const std::shared_ptr<Image>& data);

  uint32_t extractByTime(const std::string& sensor_name, const std::string& data_type, const double& timestamp,
                         std::shared_ptr<PointCloudData>& data);

  uint32_t extractByTime(const std::string& sensor_name, const std::string& data_type, const double& timestamp,
                         std::shared_ptr<ImageData>& data);

  uint32_t setSensorPose(const std::string& sensor_name, const Eigen::Isometry3f& pose);

  std::shared_ptr<const Eigen::Isometry3f> getSensorPose(const std::string& sensor_name) const;

  uint32_t setCameraIntrinsics(const std::string& sensor_name, const std::shared_ptr<CameraInfo>& camera_info);

  std::shared_ptr<CameraInfo> getCameraIntrinsics(const std::string& sensor_name) const;

  void updateOdLidarFrame(const std::shared_ptr<OdLidarFrame>& od_lidar_frame) {
    od_lidar_frame_ = od_lidar_frame;
  }

  std::shared_ptr<OdLidarFrame> getOdLidarFrame() const {
    return od_lidar_frame_;
  }

 private:
  DataManager();

  std::unordered_map<std::string, std::shared_ptr<Lidar>> lidars_;
  std::unordered_map<std::string, std::shared_ptr<Camera>> cameras_;
  std::unordered_map<std::string, SensorType> m_name_to_type_;
  std::string vehicle_name_ = "";

  // 多线程任务共享数据
  std::shared_ptr<OdLidarFrame> od_lidar_frame_ = nullptr;
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END

#define DATA_MANAGER TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE::DataManager::instance()

#define REGISTER_SENSORS_WITH_FILE(file_path) DATA_MANAGER.registerSensorsByFile(file_path)
#define REGISTER_SENSORS_WITH_CONFIG(config) DATA_MANAGER.registerSensorsByConfig(config)

#define PUSH_SENSOR_DATA(sensor_name, type, timestamp, data) DATA_MANAGER.push(sensor_name, type, timestamp, data)

#define GET_SENSOR_DATA_BY_TIME(sensor_name, type, timestamp, data) \
  DATA_MANAGER.extractByTime(sensor_name, type, timestamp, data)

#define SET_SENSOR_POSE(sensor_name, pose) DATA_MANAGER.setSensorPose(sensor_name, pose)
#define GET_SENSOR_POSE(sensor_name) DATA_MANAGER.getSensorPose(sensor_name)

#define SET_CAMERA_INTRINSICS(sensor_name, camera_info) DATA_MANAGER.setCameraIntrinsics(sensor_name, camera_info)
#define GET_CAMERA_INTRINSICS(sensor_name) DATA_MANAGER.getCameraIntrinsics(sensor_name)

#define UPDATE_OD_LIDAR_FRAME(od_lidar_frame) DATA_MANAGER.setOdLidarFrame(od_lidar_frame)
#define GET_OD_LIDAR_FRAME() DATA_MANAGER.getOdLidarFrame()

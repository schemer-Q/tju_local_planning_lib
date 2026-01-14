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
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include "tju_local_planning/common/data_manager/data_wrapper/odometry_data.h"
#include "tju_local_planning/common/data_manager/data_wrapper/pointcloud_data.h"
#include "tju_local_planning/common/data_manager/sensor_wrapper/base.h"
#include "tju_local_planning/common/data_manager/sensor_wrapper/camera.hpp"
#include "tju_local_planning/common/data_manager/sensor_wrapper/lidar.hpp"
#include "tju_local_planning/common/error/code.hpp"
#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/point.h"
#include "tju_local_planning/tools/log/t_log.h"
#include "tju_local_planning/tools/system/utils.hpp"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

enum SensorType {
  UNKNOWN,
  LIDAR,
  CAMERA,
  ODOMETRY,
  //GOALPOINT,
};

/**
 * @brief 数据管理器，管理当前车型所有传感器数据和多线程共享数据
 *
 */
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

  uint32_t push(const double& timestamp, const std::shared_ptr<Odometry>& data);

  uint32_t extractByTime(const std::string& sensor_name, const std::string& data_type, const double& timestamp,
                         std::shared_ptr<PointCloudData>& data);

  uint32_t extractByTime(const std::string& sensor_name, const std::string& data_type, const double& timestamp,
                         std::shared_ptr<ImageData>& data);

  uint32_t extractByTime(const double& timestamp, std::shared_ptr<OdometryData>& data);

  std::shared_ptr<OdometryDataBuffer> getOdometryDataBuffer() { return odometry_data_buffer_; }

  uint32_t setSensorPose(const std::string& sensor_name, const Eigen::Isometry3f& pose);

  std::shared_ptr<const Eigen::Isometry3f> getSensorPose(const std::string& sensor_name) const;

  uint32_t setCameraIntrinsics(const std::string& sensor_name, const std::shared_ptr<CameraInfo>& camera_info);

  std::shared_ptr<CameraInfo> getCameraIntrinsics(const std::string& sensor_name) const;

  uint32_t getMetaInfo(const std::string& sensor_name, std::shared_ptr<CameraMetaInfo>& meta);

  std::shared_ptr<CameraUndistort> getCameraUndistort(const std::string& sensor_name);

  std::shared_ptr<StandardCameraProjection> getCameraProjection(const std::string& sensor_name);

  double getLatestSensorDataTime(const std::string& sensor_name);

 private:
  DataManager();

  std::unordered_map<std::string, std::shared_ptr<Lidar>> lidars_;
  std::unordered_map<std::string, std::shared_ptr<Camera>> cameras_;
  std::unordered_map<std::string, SensorType> m_name_to_type_;
  std::string vehicle_name_ = "";

  std::shared_ptr<OdometryDataBuffer> odometry_data_buffer_ = nullptr;
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END

/**
 * @def DATA_MANAGER
 * @brief 获取DataManager的单例实例
 */
#define DATA_MANAGER TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE::DataManager::instance()

/**
 * @def REGISTER_SENSORS_WITH_FILE
 * @brief 注册传感器，线程不安全
 * @param file_path 传感器配置文件路径
 * @return 错误码
 */
#define REGISTER_SENSORS_WITH_FILE(file_path) DATA_MANAGER.registerSensorsByFile(file_path)

/**
 * @def REGISTER_SENSORS_WITH_CONFIG
 * @brief 注册传感器，线程不安全
 * @param config 传感器配置
 * @return 错误码
 */
#define REGISTER_SENSORS_WITH_CONFIG(config) DATA_MANAGER.registerSensorsByConfig(config)

/**
 * @def PUSH_SENSOR_DATA
 * @brief 向数据管理器中添加数据，线程安全
 * @param sensor_name 传感器名称
 * @param type 数据类型
 * @param timestamp 数据时间戳
 * @param data 数据
 * @return 错误码
 */
#define PUSH_SENSOR_DATA(sensor_name, type, timestamp, data) DATA_MANAGER.push(sensor_name, type, timestamp, data)

/**
 * @def PUSH_ODOMETRY_DATA
 * @brief 向数据管理器中添加数据，线程安全
 * @param timestamp 数据时间戳
 * @param data 数据
 * @return 错误码
 */
#define PUSH_ODOMETRY_DATA(timestamp, data) DATA_MANAGER.push(timestamp, data)

/**
 * @def PUSH_FRONT_RADAR_DATA
 * @brief 向数据管理器中添加前向毫米波雷达数据，线程安全
 * @param timestamp 数据时间戳
 * @param data 数据
 * @return 错误码
 */
#define PUSH_FRONT_RADAR_DATA(timestamp, data) DATA_MANAGER.push(timestamp, data)

/**
 * @def GET_SENSOR_DATA_BY_TIME
 * @brief 从数据管理器中提取数据，线程安全
 * @param sensor_name 传感器名称
 * @param type 数据类型
 * @param timestamp 数据时间戳
 * @param data 数据
 * @return 错误码
 */
#define GET_SENSOR_DATA_BY_TIME(sensor_name, type, timestamp, data) \
  DATA_MANAGER.extractByTime(sensor_name, type, timestamp, data)

/**
 * @def GET_ODOMETRY_DATA_BY_TIME
 * @brief 从数据管理器中提取数据，线程安全
 * @param timestamp 数据时间戳
 * @param data 数据
 * @return 错误码
 */
#define GET_ODOMETRY_DATA_BY_TIME(timestamp, data) DATA_MANAGER.extractByTime(timestamp, data)

/**
 * @def GET_ODOMETRY_DATA_BUFF
 * @brief 从数据管理器中提取数据，线程安全
 * @return Odometry data buff
 */
#define GET_ODOMETRY_DATA_BUFF() DATA_MANAGER.getOdometryDataBuffer()

/**
 * @def GET_FRONT_RADAR_DATA_BY_TIME
 * @brief 从数据管理器中提取前向毫米波雷达数据，线程安全
 * @param timestamp 数据时间戳
 * @param data 数据
 * @return 错误码
 */
#define GET_FRONT_RADAR_DATA_BY_TIME(timestamp, data) DATA_MANAGER.extractByTime(timestamp, data)

/**
 * @def SET_SENSOR_POSE
 * @brief 设置传感器位姿，线程不安全
 * @param sensor_name 传感器名称
 * @param pose 传感器位姿
 * @return 错误码
 */
#define SET_SENSOR_POSE(sensor_name, pose) DATA_MANAGER.setSensorPose(sensor_name, pose)

/**
 * @def GET_SENSOR_POSE
 * @brief 获取传感器位姿，线程不安全
 * @param sensor_name 传感器名称
 * @return 传感器位姿
 */
#define GET_SENSOR_POSE(sensor_name) DATA_MANAGER.getSensorPose(sensor_name)

/**
 * @def SET_CAMERA_INTRINSICS
 * @brief 设置相机内参，线程不安全
 * @param sensor_name 传感器名称
 * @param camera_info 相机内参
 * @return 错误码
 */
#define SET_CAMERA_INTRINSICS(sensor_name, camera_info) DATA_MANAGER.setCameraIntrinsics(sensor_name, camera_info)

/**
 * @def GET_CAMERA_INTRINSICS
 * @brief 获取相机内参，线程不安全
 * @param sensor_name 传感器名称
 * @return 相机内参
 */
#define GET_CAMERA_INTRINSICS(sensor_name) DATA_MANAGER.getCameraIntrinsics(sensor_name)

/**
 * @def GET_META_INFO
 * @brief 获取传感器元数据，线程不安全
 * @param sensor_name 传感器名称
 * @param meta 传感器元数据
 * @return 错误码
 */
#define GET_META_INFO(sensor_name, meta) DATA_MANAGER.getMetaInfo(sensor_name, meta)

/**
 * @def GET_CAMERA_UNDISTORT
 * @brief 获取相机去畸变工具，线程不安全
 * @param sensor_name 传感器名称
 * @return 相机去畸变工具
 */
#define GET_CAMERA_UNDISTORT(sensor_name) DATA_MANAGER.getCameraUndistort(sensor_name)

/**
 * @def GET_CAMERA_PROJECTION
 * @brief 获取相机投影工具，线程不安全
 * @param sensor_name 传感器名称
 * @return 相机投影工具
 */
#define GET_CAMERA_PROJECTION(sensor_name) DATA_MANAGER.getCameraProjection(sensor_name)

/**
 * @def GET_OD_LIDAR_FRAME
 * @brief 获取od lidar帧，线程不安全
 * @return od lidar帧
 */
#define GET_OD_LIDAR_FRAME() DATA_MANAGER.getOdLidarFrame()

/**
 * @def GET_LATEST_SENSOR_DATA_TIME
 * @brief 获取传感器数据最新时间，线程安全
 * @return 时间
 */
#define GET_LATEST_SENSOR_DATA_TIME(sensor_name) DATA_MANAGER.getLatestSensorDataTime(sensor_name)
/**
 * @def UPDATE_TF_FRAME
 * @brief 更新target fusion帧，线程不安全
 * @param tf_frame target fusion帧
 */
#define UPDATE_TF_FRAME(tf_frame) DATA_MANAGER.updateTfFrame(tf_frame)

/**
 * @def GET_TF_FRAME
 * @brief 获取target fusion帧，线程不安全
 * @return target fusion帧
 */
#define GET_TF_FRAME() DATA_MANAGER.getTfFrame()

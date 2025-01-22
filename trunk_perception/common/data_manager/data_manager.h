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
#include <mutex>
#include "trunk_perception/common/data_manager/data_wrapper/fod_vision_frame.h"
#include "trunk_perception/common/data_manager/data_wrapper/ld_frame.h"
#include "trunk_perception/common/data_manager/data_wrapper/od_lidar_frame.h"
#include "trunk_perception/common/data_manager/data_wrapper/side_od_vision_frame.h"
#include "trunk_perception/common/data_manager/data_wrapper/od_front_mm_frame.h"
#include "trunk_perception/common/data_manager/data_wrapper/odometry_data.h"
#include "trunk_perception/common/data_manager/data_wrapper/pointcloud_data.h"
#include "trunk_perception/common/data_manager/data_wrapper/target_fusion_frame.h"
#include "trunk_perception/common/data_manager/sensor_wrapper/base.h"
#include "trunk_perception/common/data_manager/sensor_wrapper/camera.hpp"
#include "trunk_perception/common/data_manager/sensor_wrapper/lidar.hpp"
#include "trunk_perception/common/data_manager/sensor_wrapper/radar_ars430.hpp"
#include "trunk_perception/common/data_manager/sensor_wrapper/radar_crt5p.hpp"
#include "trunk_perception/common/data_manager/sensor_wrapper/radar_cubtektar.hpp"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/point.h"
#include "trunk_perception/common/types/radar_ars430.h"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/tools/system/utils.hpp"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

enum SensorType {
  UNKNOWN,
  LIDAR,
  CAMERA,
  ODOMETRY,
  RADAR_ARS430,
  RADAR_CRT5P,
	RADAR_CUBTEKTAR,
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

  uint32_t push(const double& timestamp, const std::shared_ptr<ars430::RadarObjects>& data);

  uint32_t push(const std::string& sensor_name, const double& timestamp,
                const std::shared_ptr<cr5tp::RadarObjects>& data);

  uint32_t push(const std::string& sensor_name, const double& timestamp,
                const std::shared_ptr<cubtektar::RadarObjects>& data);                      // @author zzg 增加 为彪角雷达

  uint32_t extractByTime(const std::string& sensor_name, const std::string& data_type, const double& timestamp,
                         std::shared_ptr<PointCloudData>& data);

  uint32_t extractByTime(const std::string& sensor_name, const std::string& data_type, const double& timestamp,
                         std::shared_ptr<ImageData>& data);

  uint32_t extractByTime(const double& timestamp, std::shared_ptr<OdometryData>& data);

  uint32_t extractByTime(const double& timestamp, std::shared_ptr<ARS430RadarData>& data);

  uint32_t extractByTime(const std::string& sensor_name, const double& timestamp,
                         std::shared_ptr<CR5TPRadarData>& data);

  uint32_t extractByTime(const std::string& sensor_name, const double& timestamp,
                         std::shared_ptr<CUBTEKTARRadarData>& data);                      // @author zzg 增加 为彪角雷达

  uint32_t setSensorPose(const std::string& sensor_name, const Eigen::Isometry3f& pose);

  std::shared_ptr<const Eigen::Isometry3f> getSensorPose(const std::string& sensor_name) const;

  uint32_t setCameraIntrinsics(const std::string& sensor_name, const std::shared_ptr<CameraInfo>& camera_info);

  std::shared_ptr<CameraInfo> getCameraIntrinsics(const std::string& sensor_name) const;

  uint32_t getMetaInfo(const std::string& sensor_name, std::shared_ptr<CameraMetaInfo>& meta);

  std::shared_ptr<CameraUndistort> getCameraUndistort(const std::string& sensor_name);

  std::shared_ptr<StandardCameraProjection> getCameraProjection(const std::string& sensor_name);

  double getLatestSensorDataTime(const std::string& sensor_name);

  void updateOdLidarFrame(const std::shared_ptr<OdLidarFrame>& od_lidar_frame) { od_lidar_frame_ = od_lidar_frame; }

  std::shared_ptr<OdLidarFrame> getOdLidarFrame() const { return od_lidar_frame_; }

  void updateLdFrame(const std::shared_ptr<LDFrame>& ld_frame) { ld_frame_ = ld_frame; }

  std::shared_ptr<LDFrame> getLdFrame() const { return ld_frame_; }

  void updateTfFrame(const std::shared_ptr<TargetFusionFrame>& tf_frame) { tf_frame_ = tf_frame; }

  std::shared_ptr<TargetFusionFrame> getTfFrame() const { return tf_frame_; }

  void updateFodVisionFrame(const std::shared_ptr<FodVisionFrame>& frame) { fod_vision_frame_ = frame; }

  std::shared_ptr<FodVisionFrame> getFodVisionFrame() const { return fod_vision_frame_; }

  void updateSideOdVisionFrame(const std::shared_ptr<SideOdVisionFrame>& frame) {
    std::lock_guard<std::mutex> lock(side_od_vision_frame_mutex_);
    side_od_vision_frame_ = frame;
  }

  std::shared_ptr<SideOdVisionFrame> getSideOdVisionFrame() const {
    std::lock_guard<std::mutex> lock(side_od_vision_frame_mutex_);
    return side_od_vision_frame_;
  }

  void updateOdFrontMmFrame(const std::shared_ptr<OdFrontMmFrame>& od_front_mm_frame) { od_front_mm_frame_ = od_front_mm_frame; }

  std::shared_ptr<OdFrontMmFrame> getOdFrontMmFrame() const { return od_front_mm_frame_; }

 private:
  DataManager();

  std::unordered_map<std::string, std::shared_ptr<Lidar>> lidars_;
  std::unordered_map<std::string, std::shared_ptr<Camera>> cameras_;
  std::shared_ptr<ARS430Radar> front_radar_;
  std::unordered_map<std::string, std::shared_ptr<CR5TPRadar>> corner_radars_;
	std::unordered_map<std::string, std::shared_ptr<CUBTEKTARRadar>> cubtektar_corner_radars_;      // @author zzg 增加 为彪角雷达
  std::unordered_map<std::string, SensorType> m_name_to_type_;
  std::string vehicle_name_ = "";

  std::shared_ptr<OdometryDataBuffer> odometry_data_buffer_ = nullptr;

  // 多线程任务共享数据
  std::shared_ptr<OdLidarFrame> od_lidar_frame_ = nullptr;  ///< 激光雷达物体检测数据帧
  std::shared_ptr<LDFrame> ld_frame_ = nullptr;              ///< 车道线检测数据帧
  std::shared_ptr<TargetFusionFrame> tf_frame_ = nullptr;     ///< 后融合数据帧
  std::shared_ptr<FodVisionFrame> fod_vision_frame_ = nullptr; ///< 前向视觉目标检测数据帧
  mutable std::mutex side_od_vision_frame_mutex_; ///< 为了主从机数据同步，增加锁保证线程安全
  std::shared_ptr<SideOdVisionFrame> side_od_vision_frame_ = nullptr;  ///< 环视视觉目标检测数据帧
  std::shared_ptr<OdFrontMmFrame> od_front_mm_frame_ = nullptr;  ///< 前向毫米波雷达目标检测数据帧
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END

/**
 * @def DATA_MANAGER
 * @brief 获取DataManager的单例实例
 */
#define DATA_MANAGER TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE::DataManager::instance()

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
 * @def PUSH_CORNER_RADAR_DATA
 * @brief 向数据管理器中添加角雷达数据，线程安全
 * @param sensor_name 传感器名称
 * @param timestamp 数据时间戳
 * @param data 数据
 * @return 错误码
 */
#define PUSH_CORNER_RADAR_DATA(sensor_name, timestamp, data) DATA_MANAGER.push(sensor_name, timestamp, data)

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
 * @def GET_FRONT_RADAR_DATA_BY_TIME
 * @brief 从数据管理器中提取前向毫米波雷达数据，线程安全
 * @param timestamp 数据时间戳
 * @param data 数据
 * @return 错误码
 */
#define GET_FRONT_RADAR_DATA_BY_TIME(timestamp, data) DATA_MANAGER.extractByTime(timestamp, data)

/**
 * @def GET_CORNER_RADAR_DATA_BY_TIME
 * @brief 从数据管理器中提取角雷达数据，线程安全
 * @param sensor_name 传感器名称
 * @param timestamp 数据时间戳
 * @param data 数据
 * @return 错误码
 */
#define GET_CORNER_RADAR_DATA_BY_TIME(sensor_name, timestamp, data) \
  DATA_MANAGER.extractByTime(sensor_name, timestamp, data)

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
 * @def UPDATE_OD_LIDAR_FRAME
 * @brief 更新od lidar帧，线程不安全
 * @param od_lidar_frame od lidar帧
 */
#define UPDATE_OD_LIDAR_FRAME(od_lidar_frame) DATA_MANAGER.setOdLidarFrame(od_lidar_frame)

/**
 * @def GET_OD_LIDAR_FRAME
 * @brief 获取od lidar帧，线程不安全
 * @return od lidar帧
 */
#define GET_OD_LIDAR_FRAME() DATA_MANAGER.getOdLidarFrame()

/**
 * @def UPDATE_LD_FRAME
 * @brief 更新ld帧，线程不安全
 * @param ld_frame ld帧
 */
#define UPDATE_LD_FRAME(ld_frame) DATA_MANAGER.updateLdFrame(ld_frame)

/**
 * @def GET_LD_FRAME
 * @brief 获取ld帧，线程不安全
 * @return ld帧
 */
#define GET_LD_FRAME() DATA_MANAGER.getLdFrame()

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

/**
 * @def UPDATE_FOD_VISION_FRAME
 * @brief 更新fod_vision帧，线程不安全
 * @param fod_vision_frame fod_vision帧
 */
#define UPDATE_FOD_VISION_FRAME(fod_vision_frame) DATA_MANAGER.updateFodVisionFrame(fod_vision_frame)

/**
 * @def GET_FOD_VISION_FRAME
 * @brief 获取fod_vision帧，线程不安全
 * @return fod_vision帧
 */
#define GET_FOD_VISION_FRAME() DATA_MANAGER.getFodVisionFrame()

/**
 * @def UPDATE_SIDE_OD_VISION_FRAME
 * @brief 更新side_od_vision帧，线程安全
 * @param side_od_vision_frame side_od_vision帧
 */
#define UPDATE_SIDE_OD_VISION_FRAME(side_od_vision_frame) DATA_MANAGER.updateSideOdVisionFrame(side_od_vision_frame)

/**
 * @def GET_SIDE_OD_VISION_FRAME
 * @brief 获取side_od_vision帧，线程安全
 * @return side_od_vision帧
 */
#define GET_SIDE_OD_VISION_FRAME() DATA_MANAGER.getSideOdVisionFrame()

/**
 * @def UPDATE_OD_FRONT_MM_FRAME
 * @brief 更新od_front_mm帧，线程不安全
 * @param od_front_mm_frame od_front_mm帧
 */
#define UPDATE_OD_FRONT_MM_FRAME(od_front_mm_frame) DATA_MANAGER.updateOdFrontMmFrame(od_front_mm_frame)

/**
 * @def GET_OD_FRONT_MM_FRAME
 * @brief 获取od_front_mm帧，线程不安全
 * @return od_front_mm帧
 */
#define GET_OD_FRONT_MM_FRAME() DATA_MANAGER.getOdFrontMmFrame()

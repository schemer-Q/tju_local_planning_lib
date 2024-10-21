/**
 * @file data_manager.cpp
 * @brief 数据管理器实现
 */

#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/data_manager/sensor_wrapper/radar_ars430.hpp"
#include "trunk_perception/common/types/point.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

static std::unordered_map<std::string, SensorType> sensor_type_map = {
    {"UNKNOWN", SensorType::UNKNOWN},
    {"LIDAR", SensorType::LIDAR},
    {"CAMERA", SensorType::CAMERA},
    {"ODOMETRY", SensorType::ODOMETRY},
    {"RADAR_ARS430", SensorType::RADAR_ARS430},
    {"RADAR_CRT5P", SensorType::RADAR_CRT5P},
};

static SensorType stringToSensorType(const std::string& type) {
  if (sensor_type_map.find(type) == sensor_type_map.end()) {
    TERROR << "Invalid sensor type: " << type;
    return SensorType::UNKNOWN;
  }
  return sensor_type_map[type];
}

DataManager& DataManager::instance() {
  static DataManager instance;
  return instance;
}

DataManager::DataManager() {}

uint32_t DataManager::registerSensor(const SensorType& type, const std::string& name,
                                     const std::vector<std::string>& types, const uint32_t& buffer_size,
                                     const double& max_time_delay) {
  uint32_t ret = ErrorCode::SUCCESS;
  switch (type) {
    case SensorType::LIDAR:
      if (lidars_.find(name) != lidars_.end()) {
        TERROR << "Lidar " << name << " already exists.";
        return ErrorCode::PARAMETER_ERROR;
      }
      lidars_[name] = std::make_shared<Lidar>();
      ret = lidars_[name]->init(name, types, buffer_size, max_time_delay);
      if (ret != ErrorCode::SUCCESS) {
        TERROR << "Failed to register sensor " << name << " of type LIDAR.";
      }
      m_name_to_type_[name] = SensorType::LIDAR;
      break;
    case SensorType::CAMERA:
      if (cameras_.find(name) != cameras_.end()) {
        TERROR << "Camera " << name << " already exists.";
        return ErrorCode::PARAMETER_ERROR;
      }
      cameras_[name] = std::make_shared<Camera>();
      ret = cameras_[name]->init(name, types, buffer_size, max_time_delay);
      if (ret != ErrorCode::SUCCESS) {
        TERROR << "Failed to register sensor " << name << " of type CAMERA.";
      }
      m_name_to_type_[name] = SensorType::CAMERA;
      break;
    case SensorType::RADAR_ARS430:
      if (front_radar_ != nullptr) {
        TERROR << "Front radar already exists.";
        return ErrorCode::PARAMETER_ERROR;
      }
      front_radar_ = std::make_shared<ARS430Radar>();
      ret = front_radar_->init(name, types, buffer_size, max_time_delay);
      if (ret != ErrorCode::SUCCESS) {
        TERROR << "Failed to register sensor " << name << " of type RADAR_ARS430.";
      }
      m_name_to_type_[name] = SensorType::RADAR_ARS430;
      break;
    case SensorType::RADAR_CRT5P:
      if (corner_radars_.find(name) != corner_radars_.end()) {
        TERROR << "Corner radar " << name << " already exists.";
        return ErrorCode::PARAMETER_ERROR;
      }
      corner_radars_[name] = std::make_shared<CR5TPRadar>();
      ret = corner_radars_[name]->init(name, types, buffer_size, max_time_delay);
      if (ret != ErrorCode::SUCCESS) {
        TERROR << "Failed to register sensor " << name << " of type RADAR_CRT5P.";
      }
      m_name_to_type_[name] = SensorType::RADAR_CRT5P;
      break;
    default:
      TFATAL << "Invalid sensor type: " << type;
      ret = ErrorCode::PARAMETER_ERROR;
      break;
  }
  return ret;
}

uint32_t DataManager::registerOthers(const SensorType& type, const std::string& name, const uint32_t& buffer_size,
                                     const double& max_time_delay) {
  uint32_t ret = ErrorCode::SUCCESS;
  switch (type) {
    case SensorType::ODOMETRY:
      odometry_data_buffer_ = std::make_shared<OdometryDataBuffer>(buffer_size, name, max_time_delay);
      break;
    default:
      TFATAL << "Invalid data type: " << type;
      ret = ErrorCode::PARAMETER_ERROR;
      break;
  }
  return ret;
}

uint32_t DataManager::registerSensorsByConfig(const YAML::Node& config) {
  try {
    if (config["VEHICLE_NAME"]) {
      vehicle_name_ = config["VEHICLE_NAME"].as<std::string>();
    }
    if (config["Sensors"]) {
      for (const auto& sensor : config["Sensors"]) {
        std::string type = sensor["Type"].as<std::string>();
        SensorType sensor_type = stringToSensorType(type);
        std::string name = sensor["Name"].as<std::string>();
        std::vector<std::string> sub_types;
        if (sensor["SubTypes"]) {
          sub_types = sensor["SubTypes"].as<std::vector<std::string>>();
        }
        uint32_t buffer_size = sensor["BufferSize"].as<uint32_t>();
        double max_time_delay = sensor["MaxTimeDelay"].as<double>();
        registerSensor(sensor_type, name, sub_types, buffer_size, max_time_delay);
      }
    } else {
      TERROR << "No Sensors found in config.";
      return ErrorCode::PARAMETER_ERROR;
    }
    if (config["Others"]) {
      for (const auto& param : config["Others"]) {
        const std::string type = param["Type"].as<std::string>();
        SensorType others_type = stringToSensorType(type);
        const std::string name = param["Name"].as<std::string>();
        const uint32_t buffer_size = param["BufferSize"].as<uint32_t>();
        const double max_time_delay = param["MaxTimeDelay"].as<double>();
        registerOthers(others_type, name, buffer_size, max_time_delay);
      }
    }
  } catch (const YAML::Exception& e) {
    TERROR << "Failed to register sensors by config: " << e.what();
  }
  return ErrorCode::SUCCESS;
}

uint32_t DataManager::registerSensorsByFile(const std::string& file_path) {
  if (!is_file_exist(file_path)) {
    TFATAL << "File " << file_path << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  YAML::Node config = YAML::LoadFile(file_path);
  return registerSensorsByConfig(config);
}

std::string DataManager::getVehicleName() const { return vehicle_name_; }

void DataManager::setVehicleName(const std::string& vehicle_name) { vehicle_name_ = vehicle_name; }

uint32_t DataManager::push(const std::string& sensor_name, const std::string& data_type, const double& timestamp,
                           const std::shared_ptr<PointCloudT>& data) {
  if (lidars_.find(sensor_name) == lidars_.end()) {
    TERROR << "Lidar " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return lidars_[sensor_name]->push(data_type, timestamp, data);
}

uint32_t DataManager::push(const std::string& sensor_name, const std::string& data_type, const double& timestamp,
                           const std::shared_ptr<Image>& data) {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    TERROR << "Camera " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return cameras_[sensor_name]->push(data_type, timestamp, data);
}

uint32_t DataManager::push(const double& timestamp, const std::shared_ptr<Odometry>& data) {
  if (!odometry_data_buffer_) {
    TFATAL << "Odometry data buffer is nullptr.";
    return ErrorCode::UNINITIALIZED;
  }

  if (!data) {
    TFATAL << "Odometry push failed for data is nullptr.";
    return ErrorCode::PARAMETER_ERROR;
  }

  return odometry_data_buffer_->push(OdometryData(timestamp, data));
}

uint32_t DataManager::push(const double& timestamp, const std::shared_ptr<ars430::RadarObjects>& data) {
  if (!front_radar_) {
    TERROR << "Front radar is nullptr.";
    return ErrorCode::UNINITIALIZED;
  }

  return front_radar_->push("", timestamp, data);
}

uint32_t DataManager::push(const std::string& sensor_name, const double& timestamp,
                           const std::shared_ptr<cr5tp::RadarObjects>& data) {
  if (corner_radars_.find(sensor_name) == corner_radars_.end()) {
    TERROR << "Corner radar " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return corner_radars_[sensor_name]->push("", timestamp, data);
}

uint32_t DataManager::extractByTime(const std::string& sensor_name, const std::string& data_type,
                                    const double& timestamp, std::shared_ptr<PointCloudData>& data) {
  if (lidars_.find(sensor_name) == lidars_.end()) {
    TERROR << "Lidar " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return lidars_[sensor_name]->extractByTime(data_type, timestamp, data);
}

uint32_t DataManager::extractByTime(const std::string& sensor_name, const std::string& data_type,
                                    const double& timestamp, std::shared_ptr<ImageData>& data) {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    TERROR << "Camera " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return cameras_[sensor_name]->extractByTime(data_type, timestamp, data);
}

uint32_t DataManager::extractByTime(const double& timestamp, std::shared_ptr<OdometryData>& data) {
  if (!odometry_data_buffer_) {
    TFATAL << "Odometry data buffer is nullptr.";
    return ErrorCode::UNINITIALIZED;
  }

  data = std::make_shared<OdometryData>();
  uint32_t result = odometry_data_buffer_->extractByTime(timestamp, *data);
  if (result != ErrorCode::SUCCESS) {
    TERROR << "Odometry extractByTime failed for extract failed.";
    data = nullptr;
  }
  return result;
}

uint32_t DataManager::extractByTime(const double& timestamp, std::shared_ptr<ARS430RadarData>& data) {
  if (!front_radar_) {
    TERROR << "Front radar is nullptr.";
    return ErrorCode::UNINITIALIZED;
  }

  return front_radar_->extractByTime("", timestamp, data);
}

uint32_t DataManager::extractByTime(const std::string& sensor_name, const double& timestamp,
                                   std::shared_ptr<CR5TPRadarData>& data) {
  if (corner_radars_.find(sensor_name) == corner_radars_.end()) {
    TERROR << "Corner radar " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return corner_radars_[sensor_name]->extractByTime("", timestamp, data);
}

uint32_t DataManager::setSensorPose(const std::string& sensor_name, const Eigen::Isometry3f& pose) {
  if (m_name_to_type_.find(sensor_name) == m_name_to_type_.end()) {
    TERROR << "Sensor " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  switch (m_name_to_type_.at(sensor_name)) {
    case SensorType::LIDAR:
      return lidars_[sensor_name]->setPose(pose);
    case SensorType::CAMERA:
      return cameras_[sensor_name]->setPose(pose);
    case SensorType::RADAR_ARS430:
      return front_radar_->setPose(pose);
    case SensorType::RADAR_CRT5P:
      return corner_radars_[sensor_name]->setPose(pose);
    default:
      TFATAL << "Invalid sensor type: " << m_name_to_type_[sensor_name];
      return ErrorCode::PARAMETER_ERROR;
  }
  return ErrorCode::SUCCESS;
}

std::shared_ptr<const Eigen::Isometry3f> DataManager::getSensorPose(const std::string& sensor_name) const {
  if (m_name_to_type_.find(sensor_name) == m_name_to_type_.end()) {
    TERROR << "Sensor " << sensor_name << " does not exist.";
    return nullptr;
  }
  switch (m_name_to_type_.at(sensor_name)) {
    case SensorType::LIDAR:
      return lidars_.at(sensor_name)->pose();
    case SensorType::CAMERA:
      return cameras_.at(sensor_name)->pose();
    case SensorType::RADAR_ARS430:
      return front_radar_->pose();
    case SensorType::RADAR_CRT5P:
      return corner_radars_.at(sensor_name)->pose();
    default:
      TFATAL << "Invalid sensor type: " << m_name_to_type_.at(sensor_name);
      return nullptr;
  }
}

uint32_t DataManager::setCameraIntrinsics(const std::string& sensor_name,
                                          const std::shared_ptr<CameraInfo>& camera_info) {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    TERROR << "Camera " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return cameras_[sensor_name]->updateMeta("camera_info", camera_info);
}

std::shared_ptr<CameraInfo> DataManager::getCameraIntrinsics(const std::string& sensor_name) const {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    TERROR << "Sensor " << sensor_name << " does not exist.";
    return nullptr;
  }
  std::shared_ptr<CameraMetaInfo> meta = nullptr;
  cameras_.at(sensor_name)->getMeta(meta);
  return meta->camera_info_ptr;
}

uint32_t DataManager::getMetaInfo(const std::string& sensor_name, std::shared_ptr<CameraMetaInfo>& meta) {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    TERROR << "Sensor " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return cameras_[sensor_name]->getMeta(meta);
}

std::shared_ptr<CameraUndistort> DataManager::getCameraUndistort(const std::string& sensor_name) {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    TERROR << "Sensor " << sensor_name << " does not exist.";
    return nullptr;
  }
  return cameras_[sensor_name]->getUndistort();
}

std::shared_ptr<StandardCameraProjection> DataManager::getCameraProjection(const std::string& sensor_name) {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    TERROR << "Sensor " << sensor_name << " does not exist.";
    return nullptr;
  }
  return cameras_[sensor_name]->getProjection();
}


TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END
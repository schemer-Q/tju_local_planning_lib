/**
 * @file data_manager.cpp
 * @brief 数据管理器实现
 */

#include "tju_local_planning/common/data_manager/data_manager.h"
#include "tju_local_planning/common/types/point.h"
#include "tju_local_planning/tools/log/t_log.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

static std::unordered_map<std::string, SensorType> sensor_type_map = {
    {"UNKNOWN", SensorType::UNKNOWN},
    {"LIDAR", SensorType::LIDAR},
    {"CAMERA", SensorType::CAMERA},
    {"ODOMETRY", SensorType::ODOMETRY},
};

static SensorType stringToSensorType(const std::string& type) {
  if (sensor_type_map.find(type) == sensor_type_map.end()) {
    NTERROR << "Invalid sensor type: " << type;
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
        NTERROR << "Lidar " << name << " already exists.";
        return ErrorCode::PARAMETER_ERROR;
      }
      lidars_[name] = std::make_shared<Lidar>();
      ret = lidars_[name]->init(name, types, buffer_size, max_time_delay);
      if (ret != ErrorCode::SUCCESS) {
        NTERROR << "Failed to register sensor " << name << " of type LIDAR.";
      }
      m_name_to_type_[name] = SensorType::LIDAR;
      break;
    case SensorType::CAMERA:
      if (cameras_.find(name) != cameras_.end()) {
        NTERROR << "Camera " << name << " already exists.";
        return ErrorCode::PARAMETER_ERROR;
      }
      cameras_[name] = std::make_shared<Camera>();
      ret = cameras_[name]->init(name, types, buffer_size, max_time_delay);
      if (ret != ErrorCode::SUCCESS) {
        NTERROR << "Failed to register sensor " << name << " of type CAMERA.";
      }
      m_name_to_type_[name] = SensorType::CAMERA;
      break;
    case SensorType::ODOMETRY:
      odometry_data_buffer_ = std::make_shared<OdometryDataBuffer>(buffer_size, name, max_time_delay);
      m_name_to_type_[name] = SensorType::ODOMETRY;
      break;
    default:
      NTFATAL << "Invalid sensor type: " << type;
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
      NTERROR << "No Sensors found in config.";
      return ErrorCode::PARAMETER_ERROR;
    }
  } catch (const YAML::Exception& e) {
    NTERROR << "Failed to register sensors by config: " << e.what();
  }
  return ErrorCode::SUCCESS;
}

uint32_t DataManager::registerSensorsByFile(const std::string& file_path) {
  if (!is_file_exist(file_path)) {
    NTFATAL << "File " << file_path << " does not exist.";
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
    NTERROR << "Lidar " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return lidars_[sensor_name]->push(data_type, timestamp, data);
}

uint32_t DataManager::push(const std::string& sensor_name, const std::string& data_type, const double& timestamp,
                           const std::shared_ptr<Image>& data) {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    NTERROR << "Camera " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return cameras_[sensor_name]->push(data_type, timestamp, data);
}

uint32_t DataManager::push(const double& timestamp, const std::shared_ptr<Odometry>& data) {
  if (!odometry_data_buffer_) {
    NTFATAL << "Odometry data buffer is nullptr.";
    return ErrorCode::UNINITIALIZED;
  }

  if (!data) {
    NTFATAL << "Odometry push failed for data is nullptr.";
    return ErrorCode::PARAMETER_ERROR;
  }

  return odometry_data_buffer_->push(OdometryData(timestamp, data));
}

uint32_t DataManager::extractByTime(const std::string& sensor_name, const std::string& data_type,
                                    const double& timestamp, std::shared_ptr<PointCloudData>& data) {
  if (lidars_.find(sensor_name) == lidars_.end()) {
    NTERROR << "Lidar " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return lidars_[sensor_name]->extractByTime(data_type, timestamp, data);
}

uint32_t DataManager::extractByTime(const std::string& sensor_name, const std::string& data_type,
                                    const double& timestamp, std::shared_ptr<ImageData>& data) {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    NTERROR << "Camera " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return cameras_[sensor_name]->extractByTime(data_type, timestamp, data);
}

uint32_t DataManager::extractByTime(const double& timestamp, std::shared_ptr<OdometryData>& data) {
  if (!odometry_data_buffer_) {
    NTFATAL << "Odometry data buffer is nullptr.";
    return ErrorCode::UNINITIALIZED;
  }

  data = std::make_shared<OdometryData>();
  uint32_t result = odometry_data_buffer_->extractByTime(timestamp, *data);
  if (result != ErrorCode::SUCCESS) {
    NTERROR << "Odometry extractByTime failed for extract failed.";
    data = nullptr;
  }
  return result;
}

uint32_t DataManager::setSensorPose(const std::string& sensor_name, const Eigen::Isometry3f& pose) {
  if (m_name_to_type_.find(sensor_name) == m_name_to_type_.end()) {
    NTERROR << "Sensor " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  switch (m_name_to_type_.at(sensor_name)) {
    case SensorType::LIDAR:
      return lidars_[sensor_name]->setPose(pose);
    case SensorType::CAMERA:
      return cameras_[sensor_name]->setPose(pose);
    default:
      NTFATAL << "Invalid sensor type: " << m_name_to_type_[sensor_name];
      return ErrorCode::PARAMETER_ERROR;
  }
  return ErrorCode::SUCCESS;
}

std::shared_ptr<const Eigen::Isometry3f> DataManager::getSensorPose(const std::string& sensor_name) const {
  if (m_name_to_type_.find(sensor_name) == m_name_to_type_.end()) {
    NTERROR << "Sensor " << sensor_name << " does not exist.";
    return nullptr;
  }
  switch (m_name_to_type_.at(sensor_name)) {
    case SensorType::LIDAR:
      return lidars_.at(sensor_name)->pose();
    case SensorType::CAMERA:
      return cameras_.at(sensor_name)->pose();
    default:
      NTFATAL << "Invalid sensor type: " << m_name_to_type_.at(sensor_name);
      return nullptr;
  }
}

uint32_t DataManager::setCameraIntrinsics(const std::string& sensor_name,
                                          const std::shared_ptr<CameraInfo>& camera_info) {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    NTERROR << "Camera " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return cameras_[sensor_name]->updateMeta("camera_info", camera_info);
}

std::shared_ptr<CameraInfo> DataManager::getCameraIntrinsics(const std::string& sensor_name) const {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    NTERROR << "Sensor " << sensor_name << " does not exist.";
    return nullptr;
  }
  std::shared_ptr<CameraMetaInfo> meta = nullptr;
  cameras_.at(sensor_name)->getMeta(meta);
  return meta->camera_info_ptr;
}

uint32_t DataManager::getMetaInfo(const std::string& sensor_name, std::shared_ptr<CameraMetaInfo>& meta) {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    NTERROR << "Sensor " << sensor_name << " does not exist.";
    return ErrorCode::PARAMETER_ERROR;
  }
  return cameras_[sensor_name]->getMeta(meta);
}

std::shared_ptr<CameraUndistort> DataManager::getCameraUndistort(const std::string& sensor_name) {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    NTERROR << "Sensor " << sensor_name << " does not exist.";
    return nullptr;
  }
  return cameras_[sensor_name]->getUndistort();
}

std::shared_ptr<StandardCameraProjection> DataManager::getCameraProjection(const std::string& sensor_name) {
  if (cameras_.find(sensor_name) == cameras_.end()) {
    NTERROR << "Sensor " << sensor_name << " does not exist.";
    return nullptr;
  }
  return cameras_[sensor_name]->getProjection();
}

double DataManager::getLatestSensorDataTime(const std::string& sensor_name) {
  if (m_name_to_type_.find(sensor_name) == m_name_to_type_.end()) {
    NTERROR << "Sensor " << sensor_name << " does not exist.";
    return -1.0;  // 传感器不存在，返回异常值-1.0
  }

  switch (m_name_to_type_.at(sensor_name)) {
    case SensorType::LIDAR:
      return lidars_[sensor_name]->getLatestOriginDataTime();
    case SensorType::CAMERA:
      return cameras_[sensor_name]->getLatestOriginDataTime();
    case SensorType::ODOMETRY:
      return odometry_data_buffer_->getLatestDataTime();
    default:
      NTFATAL << "Invalid sensor type: " << m_name_to_type_[sensor_name];
      return -1.0;  // 无效传感器类型，返回异常值-1.0
  }
}

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END

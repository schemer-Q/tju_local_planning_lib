#include "trunk_perception/app/object_detection_lidar/od_lidar_lidarnetsdk.h"
#include <memory>
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/tools/log/t_log.h"

#include "lidar_net/lidar_net_det.h"
#include "lidar_net/logging.h"
#include "lidar_net/object.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

static std::unordered_map<lidar_net::base::ObjectType, ObjectType> SDKObjectTypeDict = {
    {lidar_net::base::ObjectType::UNKNOWN, ObjectType::UNKNOWN},
    {lidar_net::base::ObjectType::VEHICLE, ObjectType::VEHICLE},
    {lidar_net::base::ObjectType::CYCLIST, ObjectType::BICYCLE},
    {lidar_net::base::ObjectType::PEDESTRIAN, ObjectType::PEDESTRIAN},
    {lidar_net::base::ObjectType::BUS, ObjectType::BUS},
    {lidar_net::base::ObjectType::ART, ObjectType::ART},
    {lidar_net::base::ObjectType::ART_NO_TRAILER, ObjectType::ART_NO_TRAILER},
    {lidar_net::base::ObjectType::ART_SEMI_TRAILER, ObjectType::ART_SEMI_TRAILER},
    {lidar_net::base::ObjectType::ART_FULL_TRAILER, ObjectType::ART_FULL_TRAILER},
    {lidar_net::base::ObjectType::TRUCK, ObjectType::TRUCK},
    {lidar_net::base::ObjectType::TRUCK_HEAD, ObjectType::TRUCK_HEAD},
    {lidar_net::base::ObjectType::TRAILER, ObjectType::TRAILER},
    {lidar_net::base::ObjectType::TRUCK_NO_TRAILER, ObjectType::TRUCK_NO_TRAILER},
    {lidar_net::base::ObjectType::TRUCK_SEMI_TRAILER, ObjectType::TRUCK_SEMI_TRAILER},
    {lidar_net::base::ObjectType::TRUCK_FULL_TRAILER, ObjectType::TRUCK_FULL_TRAILER},
    {lidar_net::base::ObjectType::ALIEN_VEHICLE, ObjectType::ALIEN_VEHICLE},
    {lidar_net::base::ObjectType::FORKLIFT, ObjectType::FORKLIFT},
    {lidar_net::base::ObjectType::ECCENTRIC_TRUNK_HEAD, ObjectType::ECCENTRIC_TRUNK_HEAD},
    {lidar_net::base::ObjectType::MOBILE_CRANE, ObjectType::MOBILE_CRANE},
};

static ObjectType ConvertSDKObjectType(lidar_net::base::ObjectType type) {
  auto it = SDKObjectTypeDict.find(type);
  if (it == SDKObjectTypeDict.end()) {
    return ObjectType::UNKNOWN;
  }
  return it->second;
}

class LidarNetSDKLogger : public lidar_net::LidarNetLogger {
  void log(lidar_net::LidarNetLogger::Severity severity, const std::string_view msg) noexcept override {
    try {
      switch (severity) {
        case lidar_net::LidarNetLogger::Severity::kINTERNAL_ERROR:
          TFATAL << "[lidar_net] " << msg;
          break;
        case lidar_net::LidarNetLogger::Severity::kERROR:
          TERROR << "[lidar_net] " << msg;
          break;
        case lidar_net::LidarNetLogger::Severity::kWARNING:
          TWARNING << "[lidar_net] " << msg;
          break;
        case lidar_net::LidarNetLogger::Severity::kINFO:
          TINFO << "[lidar_net] " << msg;
          break;
        case lidar_net::LidarNetLogger::Severity::kVERBOSE:
          TDEBUG << "[lidar_net] " << msg;
          break;
      }
    } catch (const std::exception& exc) {
      TERROR << exc.what();
    }
  }
};

OdLidarLidarNetSdk::OdLidarLidarNetSdk() {}

OdLidarLidarNetSdk::~OdLidarLidarNetSdk() {}

std::uint32_t OdLidarLidarNetSdk::Init(const YAML::Node& config) {
  try {
    model_config_path_ = config["ModelConfig"].as<std::string>();
    lidar_name_ = config["Lidar"].as<std::string>();
  } catch (const std::exception& e) {
    TFATAL << "OdLidarLidarNetSdk::Init failed: " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }

  static std::shared_ptr<LidarNetSDKLogger> logger = std::make_shared<LidarNetSDKLogger>();

  detector_ = std::make_shared<lidar_net::LidarNetDetector>();
  if (!detector_->Init(model_config_path_, logger, "toml")) {
    TFATAL << "LidarNetDetector::Init failed";
    return ErrorCode::LIDAR_NET_SDK_INIT_FAILED;
  }

  TINFO << "OdLidarLidarNetSdk::Init success";
  return ErrorCode::SUCCESS;
}

std::uint32_t OdLidarLidarNetSdk::Run(const double& ts) {
  if (!detector_) {
    TERROR << "OdLidarLidarNetSdk::Run failed: LidarNetDetector is not initialized";
    return ErrorCode::UNINITIALIZED;
  }

  uint32_t ret = ErrorCode::SUCCESS;

  // 获取 lidar 数据
  std::shared_ptr<PointCloudData> lidar_data = nullptr;
  ret = GET_SENSOR_DATA_BY_TIME(lidar_name_, "tf", ts, lidar_data);
  if (ret != ErrorCode::SUCCESS) {
    TERROR << "OdLidarLidarNetSdk::Run failed: GET_SENSOR_DATA_BY_TIME failed";
    return ret;
  }

  if (lidar_data == nullptr || lidar_data->data == nullptr) {
    TERROR << "OdLidarLidarNetSdk::Run failed: lidar_data is nullptr";
    return ErrorCode::POINT_CLOUD_INVALID;
  }

  // 创建 od_lidar_frame
  std::shared_ptr<OdLidarFrame> od_lidar_frame = std::make_shared<OdLidarFrame>();
  double timestamp = lidar_data->time;
  od_lidar_frame->timestamp = timestamp;
  od_lidar_frame->pointcloud = lidar_data->data;

  // model inference
  std::vector<lidar_net::base::BoundingBox> sdk_bboxes;
  const size_t points_num = lidar_data->data->size();
  Eigen::MatrixXf xyzi = Eigen::MatrixXf::Identity(4, points_num);
  xyzi.topRows(3) = lidar_data->data->getMatrixXfMap(3, 8, 0);
  for (size_t i = 0UL; i < points_num; ++i) {
    xyzi(3, i) = lidar_data->data->points[i].intensity;
  }
  detector_->ProcessingEigenRow(xyzi, sdk_bboxes);

  // convert sdk bbox to bboxes
  od_lidar_frame->detected_objects.reserve(sdk_bboxes.size());
  for (auto& bbox : sdk_bboxes) {
    Object obj;
    obj.timestamp = timestamp;
    obj.bbox.center = bbox.center;
    obj.bbox.size = bbox.size;
    obj.bbox.theta = bbox.theta;
    obj.bbox.corners2d = bbox.corners2d;
    obj.type_probs = bbox.type_probs;
    obj.confidence = bbox.confidence;
    obj.type = ConvertSDKObjectType(bbox.type);
    obj.convex_polygon.resize(3, bbox.corners2d.cols());
    for (auto i = 0; i < bbox.corners2d.cols(); ++i) {
      obj.convex_polygon.col(i) << bbox.corners2d(0, i), bbox.corners2d(1, i), 0.0F;
    }
    od_lidar_frame->detected_objects.emplace_back(obj);
  }

  // 更新 od_lidar_frame
  DATA_MANAGER.updateOdLidarFrame(od_lidar_frame);

  return ErrorCode::SUCCESS;
}

std::any OdLidarLidarNetSdk::GetData(const std::string& key) { return nullptr; }

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
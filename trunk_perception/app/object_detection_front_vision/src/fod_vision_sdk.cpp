/**
 * @file fod_vision_sdk.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-11-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "trunk_perception/app/object_detection_front_vision/fod_vision_sdk.h"

#include "trunk_perception/common/data_manager/data_wrapper/image_data.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

// r3d object type 目前只使用了CAR=0、 TRUCK=1、BUS=2三个类别，港口的VEHICLE对应CAR
// enum class ObjectType {
//   UNKNOW = -1,
//   CAR = 0,         // 小客车
//   TRUCK = 1,       // 卡车
//   BUS = 2,         // 公交车
//   CONE = 3,        // 锥桶
//   PEDESTRIAN = 4,  // 行人
//   CYCLE = 5,       // 两轮车
//   TRICYCLE = 6,    // 三轮车
//   OBJECT_TYPE_CONSTRUCTION_AREA = 100  // construction area
// };

static std::unordered_map<int, ObjectType> SDKVisionObjectTypeMap = {
    {-1, ObjectType::UNKNOWN},
    {0, ObjectType::VEHICLE},
    {1, ObjectType::TRUCK},
    {2, ObjectType::BUS},
};

static ObjectType ConvertSDKVisionObjectType(const int type) {
  auto it = SDKVisionObjectTypeMap.find(type);
  if (it == SDKVisionObjectTypeMap.end()) {
    return ObjectType::UNKNOWN;
  }
  return it->second;
}

std::unordered_map<std::string, r3d::CameraNode> name_camera_node_map = {
    {"CAMERA_0_F60", r3d::CameraNode::F60},
};

class FodVisionSDKLogger : public SDKLogger {
  void Log(trunk::perception::LogLevel level, const std::string& msg) override {
    switch (level) {
      case trunk::perception::LogLevel::TRACE:
        TTRACE << msg;
        break;
      case trunk::perception::LogLevel::DEBUG:
        TDEBUG << msg;
        break;
      case trunk::perception::LogLevel::INFO:
        TINFO << msg;
        break;
      case trunk::perception::LogLevel::WARNING:
        TWARNING << msg;
        break;
      case trunk::perception::LogLevel::ERROR:
        TERROR << msg;
        break;
      case trunk::perception::LogLevel::FATAL:
        TFATAL << msg;
        break;
    }
  }
};

FodVisionSDK::~FodVisionSDK() {
  if (r3d_data_) {
    delete r3d_data_;
    r3d_data_ = nullptr;
  }
  r3d::SDK_VISDETA5R3D_RELEASE(r3d_instance_);
}

std::uint32_t FodVisionSDK::Init(const YAML::Node& config) {
  uint32_t code = ErrorCode::SUCCESS;

  try {
    name_ = config["Name"].as<std::string>();
    type_ = config["Type"].as<std::string>();
    config_file_ = config["ConfigFile"].as<std::string>();
  } catch (const std::exception& e) {
    TFATAL << "FodVisionSDK::Init failed, error: " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }

  // init inference data
  r3d_data_ = new r3d::VisDetR3DDconfFrontData;

  // 初始化相机参数
  code = InitCameraParams();
  if (code != ErrorCode::SUCCESS) {
    TWARNING << "FodVisionSDK::Init failed, error: " << code;
    TINFO << "FodVisionSDK::Init, Lazy init SDK is turned on";
    lazy_init_ = true;
    return ErrorCode::SUCCESS;
  } else {
    // 初始化SDK
    std::shared_ptr<FodVisionSDKLogger> logger = std::make_shared<FodVisionSDKLogger>();
    const float crop_ratio = 0.0F;
    const r3d::CameraNode camera_node = name_camera_node_map[name_];
    const std::vector<r3d::CameraParam> camera_cfgs{*camera_param_};
    int ret = r3d::SDK_VISDETA5R3D_INIT(&r3d_instance_, config_file_, camera_node, camera_cfgs, crop_ratio, logger);
    if (ret != 0) {
      TFATAL << "FodVisionSDK::Init SDK_VISDETA5R3D_INIT failed, error: " << ret;
      return ErrorCode::FRONT_VISION_DETECTOR_SDK_INIT_FAILED;
    }
    is_init_ = true;
  }

  TINFO << "FodVisionSDK::Init success, name: " << name_ << ", type: " << type_;
  return ErrorCode::SUCCESS;
}

std::uint32_t FodVisionSDK::Run(const double& ts) {
  uint32_t code = ErrorCode::SUCCESS;

  // 懒初始化
  if (lazy_init_ && !is_init_) {
    code = InitCameraParams();
    if (code != ErrorCode::SUCCESS) {
      TERROR << "FodVisionSDK::Run lazy init failed, error: " << code;
      return code;
    }
    // 初始化SDK
    std::shared_ptr<FodVisionSDKLogger> logger = std::make_shared<FodVisionSDKLogger>();
    const float crop_ratio = 0.0F;
    const r3d::CameraNode camera_node = name_camera_node_map[name_];
    const std::vector<r3d::CameraParam> camera_cfgs{*camera_param_};
    int ret = r3d::SDK_VISDETA5R3D_INIT(&r3d_instance_, config_file_, camera_node, camera_cfgs, crop_ratio, logger);
    if (ret != 0) {
      TFATAL << "FodVisionSDK::Init SDK_VISDETA5R3D_INIT failed, error: " << ret;
      return ErrorCode::FRONT_VISION_DETECTOR_SDK_INIT_FAILED;
    }
    is_init_ = true;
  }

  // 获取图像数据
  std::shared_ptr<ImageData> image_data = nullptr;
  code = GET_SENSOR_DATA_BY_TIME(name_, type_, ts, image_data);
  if (code != ErrorCode::SUCCESS) {
    TERROR << "FodVisionSDK::Run get image data failed, error: " << code;
    return code;
  }
  if (!image_data) {
    TERROR << "FodVisionSDK::Run image data is nullptr";
    return ErrorCode::IMAGE_DATA_INVALID;
  }
  if (!image_data->data || image_data->data->image.empty()) {
    TERROR << "FodVisionSDK::Run image data is nullptr or empty";
    return ErrorCode::IMAGE_DATA_INVALID;
  }

  // init fod vision frame
  std::shared_ptr<FodVisionFrame> fod_vision_frame = std::make_shared<FodVisionFrame>();
  fod_vision_frame->timestamp = image_data->time;

  // SDK infer
  r3d_data_->images = {image_data->data->image};
  int ret = r3d::SDK_VISDETA5R3D_INFER(r3d_instance_, *r3d_data_);
  if (ret) {
    TERROR << "FodVisionSDK::Run SDK_VISDETA5R3D_INFER failed, error: " << ret;
    return ErrorCode::FRONT_VISION_DETECTOR_SDK_INFER_FAILED;
  }

  postProcess(fod_vision_frame);

  UPDATE_FOD_VISION_FRAME(fod_vision_frame);

  return code;
}

void FodVisionSDK::postProcess(const std::shared_ptr<FodVisionFrame>& frame) {
  const auto& sdk_objects = r3d_data_->batchDetectedR3D[0];
  frame->detected_objects.reserve(sdk_objects.size());

  cv::Mat img = r3d_data_->images[0];
  for (auto& vision_obj : sdk_objects) {
    Object obj;
    obj.timestamp = frame->timestamp;
    obj.detector_type = DetectorType::VisionModel;
    obj.bbox.center << vision_obj.location[0], vision_obj.location[1], vision_obj.location[2];
    obj.bbox.size << vision_obj.dimension[0], vision_obj.dimension[1], vision_obj.dimension[2];
    obj.bbox.theta = vision_obj.heading;
    obj.bbox.direction << std::cos(obj.bbox.theta), std::sin(obj.bbox.theta), 0.0f;

    Eigen::Rotation2D<float> R2D(obj.bbox.theta);
    const auto& center_xy = obj.bbox.center.head(2);
    obj.bbox.corners2d.col(0) = R2D * Eigen::Vector2f(obj.bbox.size(0), obj.bbox.size(1)) * 0.5F + center_xy;
    obj.bbox.corners2d.col(1) = R2D * Eigen::Vector2f(obj.bbox.size(0), -obj.bbox.size(1)) * 0.5F + center_xy;
    obj.bbox.corners2d.col(2) = R2D * Eigen::Vector2f(-obj.bbox.size(0), -obj.bbox.size(1)) * 0.5F + center_xy;
    obj.bbox.corners2d.col(3) = R2D * Eigen::Vector2f(-obj.bbox.size(0), obj.bbox.size(1)) * 0.5F + center_xy;

    obj.confidence = vision_obj.det2d.score;
    obj.type = ConvertSDKVisionObjectType(vision_obj.det2d.cl);
    obj.convex_polygon.resize(3, obj.bbox.corners2d.cols());
    for (auto i = 0; i < obj.bbox.corners2d.cols(); ++i) {
      obj.convex_polygon.col(i) << obj.bbox.corners2d(0, i), obj.bbox.corners2d(1, i), 0.0F;
    }
    frame->detected_objects.emplace_back(obj);
  }
}

std::any FodVisionSDK::GetData(const std::string& key) { return std::any(); }

uint32_t FodVisionSDK::InitCameraParams() {
  uint32_t code = ErrorCode::SUCCESS;
  std::shared_ptr<CameraMetaInfo> meta = nullptr;
  code = GET_META_INFO(name_, meta);

  if (code != ErrorCode::SUCCESS) {
    TERROR << "FodVisionSDK::Init failed, error: " << code;
    return code;
  }

  if (!meta) {
    TERROR << "FodVisionSDK::Init failed, meta is nullptr";
    return ErrorCode::FRONT_VISION_DETECTOR_INIT_CAMERA_PARAMS_FAILED;
  }
  if (!meta->camera_info_ptr || !meta->pose_ptr) {
    TERROR << "FodVisionSDK::Init failed, camera_info_ptr or pose_ptr is nullptr";
    return ErrorCode::FRONT_VISION_DETECTOR_INIT_CAMERA_PARAMS_FAILED;
  }

  camera_param_ = std::make_shared<r3d::CameraParam>();
  camera_param_->Intrinsic[0] = meta->camera_info_ptr->fx;
  camera_param_->Intrinsic[1] = 0.0;
  camera_param_->Intrinsic[2] = meta->camera_info_ptr->cx;
  camera_param_->Intrinsic[3] = 0.0;
  camera_param_->Intrinsic[4] = meta->camera_info_ptr->fy;
  camera_param_->Intrinsic[5] = meta->camera_info_ptr->cy;
  camera_param_->Intrinsic[6] = 0.0;
  camera_param_->Intrinsic[7] = 0.0;
  camera_param_->Intrinsic[8] = 1.0;
  // 畸变系数, 使用5参数，其他格式需要修改SDK
  if (meta->camera_info_ptr->D.rows == 5) {
    camera_param_->DistortionFactor[0] = meta->camera_info_ptr->D.at<double>(0, 0);
    camera_param_->DistortionFactor[1] = meta->camera_info_ptr->D.at<double>(1, 0);
    camera_param_->DistortionFactor[2] = meta->camera_info_ptr->D.at<double>(2, 0);
    camera_param_->DistortionFactor[3] = meta->camera_info_ptr->D.at<double>(3, 0);
    camera_param_->DistortionFactor[4] = meta->camera_info_ptr->D.at<double>(4, 0);
  } else {
    // TFATAL << "FodVisionSDK::Init failed, camera_info_ptr->D.rows is not 5, FodVisionSDK not support";
    // return ErrorCode::FRONT_VISION_DETECTOR_INIT_CAMERA_PARAMS_FAILED;

    // 先伪装一下避免程序中断
    TERROR << "FodVisionSDK::Init failed, camera_info_ptr->D.rows is not 5, FodVisionSDK not support";
    camera_param_->DistortionFactor[0] = 0.0;
    camera_param_->DistortionFactor[1] = 0.0;
    camera_param_->DistortionFactor[2] = 0.0;
    camera_param_->DistortionFactor[3] = 0.0;
    camera_param_->DistortionFactor[4] = 0.0;
  }

  constexpr float RAD2DEG = 180.0F / M_PI;
  const Eigen::Vector3f euler_angles = meta->pose_ptr->rotation().eulerAngles(2, 1, 0);
  camera_param_->roll = euler_angles[2] * RAD2DEG;
  camera_param_->pitch = euler_angles[1] * RAD2DEG;
  camera_param_->yaw = euler_angles[0] * RAD2DEG;
  camera_param_->x = meta->pose_ptr->translation().x();
  camera_param_->y = meta->pose_ptr->translation().y();
  camera_param_->z = meta->pose_ptr->translation().z();
  camera_param_->height = meta->camera_info_ptr->IMG_H;
  camera_param_->width = meta->camera_info_ptr->IMG_W;

  camera_undistort_ = GET_CAMERA_UNDISTORT(name_);
  if (!camera_undistort_) {
    TERROR << "FodVisionSDK::Init failed, camera_undistort_ is nullptr: " << ErrorCode::UNINITIALIZED;
    return ErrorCode::UNINITIALIZED;
  }
  camera_projection_ = GET_CAMERA_PROJECTION(name_);
  if (!camera_projection_) {
    TERROR << "FodVisionSDK::Init failed, camera_projection_ is nullptr: " << ErrorCode::UNINITIALIZED;
    return ErrorCode::UNINITIALIZED;
  }

  TINFO << "FodVisionSDK::InitCameraParams success";

  return code;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
#include "trunk_perception/app/camera_preprocess/preprocess_base.h"
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

CameraPreprocessBase::CameraPreprocessBase() = default;

CameraPreprocessBase::~CameraPreprocessBase() = default;

std::uint32_t CameraPreprocessBase::Init(const YAML::Node& config) {
  try {
    camera_name_ = config["Name"].as<std::string>();
  } catch (const std::exception& e) {
    TFATAL << "CameraPreprocessBase::Init failed, " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }
  return ErrorCode::SUCCESS;
}

std::uint32_t CameraPreprocessBase::Run(const double& ts) {
  // init
  uint32_t code = ErrorCode::SUCCESS;
  undistort_image_ = nullptr;

  // 获取undistort类
  auto undistort = GET_CAMERA_UNDISTORT(camera_name_);
  if (!undistort) {
    TERROR << "CameraPreprocessBase::Run failed, get " << camera_name_ << " undistort failed";
    return ErrorCode::UNINITIALIZED;
  }

  // 获取image
  std::shared_ptr<ImageData> image = nullptr;
  code = GET_SENSOR_DATA_BY_TIME(camera_name_, "origin", ts, image);
  if (code != ErrorCode::SUCCESS) {
    TERROR << "CameraPreprocessBase::Run failed, get " << camera_name_ << " image failed, code: " << code;
    return code;
  }
  if (!image || !image->data || image->data->image.empty()) {
    TERROR << "CameraPreprocessBase::Run failed, get " << camera_name_ << " image is nullptr or empty";
    return ErrorCode::IMAGE_DATA_INVALID;
  }

  // 进行undistort
  undistort_image_ = std::make_shared<Image>();
  double image_ts = image->time;
  undistort_image_->timestamp = image_ts;
  undistort->UndistortImg(image->data->image, undistort_image_->image);

  // push data
  code = PUSH_SENSOR_DATA(camera_name_, "undistorted", image_ts, undistort_image_);
  if (code != ErrorCode::SUCCESS) {
    TERROR << "CameraPreprocessBase::Run failed, push " << camera_name_ << " undistort image failed, code: " << code;
    return code;
  }
  return code;
}

std::any CameraPreprocessBase::GetData(const std::string& key) {
  if (key == "undistorted") {
    return undistort_image_;
  }
  return nullptr;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
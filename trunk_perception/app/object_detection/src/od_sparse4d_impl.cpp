#include "trunk_perception/app/object_detection/od_sparse4d_impl.h"
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/tools/gen_mapxy.h"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/app/object_detection/detection_det_sdk_utils.h"

#include "detection_net_sdk/data.h"
#include "detection_net_sdk/object_detector.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

OdSparse4DImpl::OdSparse4DImpl() = default;

OdSparse4DImpl::~OdSparse4DImpl() {
  if (input_) {
    delete input_;
    input_ = nullptr;
  }
}

std::uint32_t OdSparse4DImpl::Init(const YAML::Node& config) {
  try {
    dist_min_x_ = config["DistortionMap"]["MinX"].as<int>();
    dist_min_y = config["DistortionMap"]["MinY"].as<int>();
    dist_max_x_ = config["DistortionMap"]["MaxX"].as<int>();
    dist_max_y_ = config["DistortionMap"]["MaxY"].as<int>();
    model_config_path_ = config["ModelConfig"].as<std::string>();
    cameras_ = config["Cameras"].as<std::vector<std::string>>();
  } catch (const std::exception& e) {
    TFATAL << "OdSparse4DImpl::Init failed: " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }
  if (cameras_.empty()) {
    TFATAL << "OdSparse4DImpl::Init failed: cameras is empty";
    return ErrorCode::PARAMETER_ERROR;
  }

  static std::shared_ptr<DetectionNetSDKLogger> logger = std::make_shared<DetectionNetSDKLogger>();

  detector_ = std::make_shared<net::ObjectDetector>();
  if (!detector_->Init(model_config_path_, logger)) {
    TFATAL << "OdSparse4DImpl::Init failed: init detector failed";
    return ErrorCode::OD_SPARSE4D_INIT_FAILED;
  }

  input_ = new net::Sparse4dWithDistortionInput();
  input_->images.resize(cameras_.size());
  input_->camera_params.resize(cameras_.size());
  distortion_maps_.resize(cameras_.size(), nullptr);
  TINFO << "OdSparse4DImpl::Init success";

  return ErrorCode::SUCCESS;
}

std::uint32_t OdSparse4DImpl::Run(const double& ts) {
  std::shared_ptr<SideOdVisionFrame> side_od_vision_frame = std::make_shared<SideOdVisionFrame>();

  uint32_t code = ErrorCode::SUCCESS;

  if (!detector_) {
    TERROR << "OdSparse4DImpl::Run failed: detector is not initialized";
    return ErrorCode::UNINITIALIZED;
  }

  // 依次获取每个相机数据
  std::vector<double> timestamps(6);
  for (size_t i = 0; i < cameras_.size(); ++i) {
    std::shared_ptr<ImageData> image_data = nullptr;
    code = GET_SENSOR_DATA_BY_TIME(cameras_[i], "origin", ts, image_data);
    if (code != ErrorCode::SUCCESS) {
      TERROR << "OdSparse4DImpl::Run failed: get " << cameras_[i] << " image failed, code: " << code;
      return code;
    }

    if (!image_data || !image_data->data || image_data->data->image.empty()) {
      TERROR << "OdSparse4DImpl::Run failed: get " << cameras_[i] << " image is nullptr or empty";
      return ErrorCode::IMAGE_DATA_INVALID;
    }

    timestamps[i] = image_data->time;
    input_->images[i] = image_data->data->image;
  }

  // 相机参数
  if (!camera_params_initialized_) {
    for (size_t i = 0; i < cameras_.size(); ++i) {
      std::shared_ptr<CameraMetaInfo> meta = nullptr;
      code = GET_META_INFO(cameras_[i], meta);
      if (code != ErrorCode::SUCCESS) {
        TERROR << "OdSparse4DImpl::Run failed: get " << cameras_[i] << " meta info failed, code: " << code;
        return code;
      }
      if (meta == nullptr || meta->pose_ptr == nullptr || meta->camera_info_ptr == nullptr) {
        TERROR << "OdSparse4DImpl::Init failed: get " << cameras_[i] << " meta info is nullptr";
        return ErrorCode::SENSOR_POSE_NOT_FOUND;
      }

      input_->camera_params[i].intrinsic = Eigen::Matrix3f::Identity();
      input_->camera_params[i].intrinsic(0, 0) = meta->camera_info_ptr->fx;
      input_->camera_params[i].intrinsic(1, 1) = meta->camera_info_ptr->fy;
      input_->camera_params[i].intrinsic(0, 2) = meta->camera_info_ptr->cx;
      input_->camera_params[i].intrinsic(1, 2) = meta->camera_info_ptr->cy;

      input_->camera_params[i].extrinsic = meta->pose_ptr->matrix();

      // 畸变映射表
      if (distortion_maps_[i] == nullptr) {
        distortion_maps_[i] = std::make_shared<net::DistortionMap>();
        DistortionMapResult ret = DistortionUtils::calculateDistortionMap(
            meta->camera_info_ptr->IMG_W, meta->camera_info_ptr->IMG_H, meta->camera_info_ptr->K,
            meta->camera_info_ptr->D, dist_min_x_, dist_min_y, dist_max_x_, dist_max_y_);
        distortion_maps_[i]->map_x = ret.map_x;
        distortion_maps_[i]->map_y = ret.map_y;
        distortion_maps_[i]->min_x = ret.min_x;
        distortion_maps_[i]->min_y = ret.min_y;
        distortion_maps_[i]->max_x = ret.max_x;
        distortion_maps_[i]->max_y = ret.max_y;
      }
      input_->camera_params[i].distortion_map_ptr = distortion_maps_[i];
      input_->camera_params[i].distortion_map_set = true;
    }
    camera_params_initialized_ = true;
  }

  // timestamp
  input_->timestamp = timestamps[0];

  // ego_pose
  std::shared_ptr<OdometryData> odometry_data_ptr = nullptr;
  code = GET_ODOMETRY_DATA_BY_TIME(timestamps[0], odometry_data_ptr);
  if (code != ErrorCode::SUCCESS || odometry_data_ptr == nullptr || odometry_data_ptr->data == nullptr) {
    TERROR << "OdSparse4DImpl::Run failed: get odometry data failed, code: " << code;
    return code;
  }
  input_->ego_pose = odometry_data_ptr->data->Matrix().cast<float>();

  // detect
  std::vector<net::BoundingBox> sdk_bboxes;
  if (!detector_->Processing(input_, sdk_bboxes)) {
    TERROR << "OdSparse4DImpl::Run failed: detect failed";
    return ErrorCode::OD_SPARSE4D_DETECT_FAILED;
  }

  side_od_vision_frame->timestamp = timestamps[0];
  // 将检测结果转换为TrunkPerception的检测结果
  for (const auto& bbox : sdk_bboxes) {
    Object obj;
    obj.timestamp = timestamps[0];
    obj.type = DetectionNetSDKObjectTypeDict[bbox.type];
    obj.bbox.center = bbox.center;
    obj.bbox.size = bbox.size;
    obj.velocity.x() = bbox.velocity.x();
    obj.velocity.y() = bbox.velocity.y();
    obj.bbox.theta = bbox.theta;
    obj.confidence = bbox.confidence;
    side_od_vision_frame->detected_objects.emplace_back(obj);
  }

  UPDATE_SIDE_OD_VISION_FRAME(side_od_vision_frame);

  return ErrorCode::SUCCESS;
}

std::any OdSparse4DImpl::GetData(const std::string& key) { return std::any(); }

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
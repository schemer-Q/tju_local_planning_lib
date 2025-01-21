#include "trunk_perception/app/object_detection/od_mm_sparse4d_impl.h"

#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/data_manager/data_wrapper/od_front_mm_frame.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/app/object_detection/detection_det_sdk_utils.h"

#include "detection_net_sdk/data.h"
#include "detection_net_sdk/object_detector.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;


OdMMSparse4DImpl::OdMMSparse4DImpl() {}

OdMMSparse4DImpl::~OdMMSparse4DImpl() {
  if (input_) {
    delete input_;
    input_ = nullptr;
  }
}

std::uint32_t OdMMSparse4DImpl::Init(const YAML::Node& config) {
  try {
    model_config_path_ = config["ModelConfig"].as<std::string>();
  } catch (const std::exception& e) {
    TFATAL << "OdMMSparse4DImpl::Init failed: " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }

  static std::shared_ptr<DetectionNetSDKLogger> logger = std::make_shared<DetectionNetSDKLogger>();
  detector_ = std::make_shared<net::ObjectDetector>();
  if (!detector_->Init(model_config_path_, logger)) {
    TFATAL << "OdMMSparse4DImpl::Init failed: init detector failed";
    return ErrorCode::OD_MMSPARSE4D_INIT_FAILED;
  }

  input_ = new net::PointPillarSparse4dInput();

  TINFO << "OdMMSparse4DImpl::Init success";
  return ErrorCode::SUCCESS;
}

std::uint32_t OdMMSparse4DImpl::Run(const double& ts) {
  std::shared_ptr<OdFrontMmFrame> od_front_mm_frame = std::make_shared<OdFrontMmFrame>();

  if (!detector_ || !input_) {
    TFATAL << "OdMMSparse4DImpl::Run failed: detector or input is not initialized";
    return ErrorCode::UNINITIALIZED;
  }

  uint32_t code = ErrorCode::SUCCESS;

  // 第一版本直接获取1L1V多模态模型所需的数据，没有使用配置文件管理
  // camera_info
  if (!camera_params_initialized_) {
    std::shared_ptr<CameraMetaInfo> meta = nullptr;
    code = GET_META_INFO("CAMERA_0_F60", meta);
    if (code != ErrorCode::SUCCESS) {
      TERROR << "OdMMSparse4DImpl::Run failed: get CAMERA_0_F60 meta info failed, code: " << code;
      return code;
    }
    net::CameraParameters camera_params;
    camera_params.intrinsic = Eigen::Matrix3f::Identity();
    camera_params.intrinsic(0, 0) = meta->camera_info_ptr->fx;
    camera_params.intrinsic(1, 1) = meta->camera_info_ptr->fy;
    camera_params.intrinsic(0, 2) = meta->camera_info_ptr->cx;
    camera_params.intrinsic(1, 2) = meta->camera_info_ptr->cy;
    camera_params.extrinsic = meta->pose_ptr->matrix();
    input_->camera_params.emplace_back(camera_params);
    camera_params_initialized_ = true;
  }

  // points
  double refer_ts = ts;
  std::shared_ptr<PointCloudData> points_data = nullptr;
  code = GET_SENSOR_DATA_BY_TIME("LIDAR_0", "tf", refer_ts, points_data);
  if (code != ErrorCode::SUCCESS) {
    TERROR << "OdMMSparse4DImpl::Run failed: get LIDAR_0 tf points data failed, code: " << code;
    return code;
  }
  if (points_data == nullptr || points_data->data == nullptr) {
    TERROR << "OdMMSparse4DImpl::Run failed: get LIDAR_0 tf points data is nullptr";
    return ErrorCode::POINT_CLOUD_INVALID;
  }
  auto xyzi = points_data->data->getMatrixXfMap(4, 8, 0);
  input_->points = xyzi;
  refer_ts = points_data->time;

  // image
  std::shared_ptr<ImageData> image_data = nullptr;
  code = GET_SENSOR_DATA_BY_TIME("CAMERA_0_F60", "undistorted", refer_ts, image_data);
  if (code != ErrorCode::SUCCESS) {
    TERROR << "OdMMSparse4DImpl::Run failed: get CAMERA_0_F60 undistorted image data failed, code: " << code;
    return code;
  }
  if (image_data == nullptr || image_data->data == nullptr || image_data->data->image.empty()) {
    TERROR << "OdMMSparse4DImpl::Run failed: get CAMERA_0_F60 undistorted image data is nullptr";
    return ErrorCode::IMAGE_DATA_INVALID;
  }
  input_->images.emplace_back(image_data->data->image);

  // timestamp
  input_->timestamp = refer_ts;

  // ego_pose
  std::shared_ptr<OdometryData> odometry_data_ptr = nullptr;
  code = GET_ODOMETRY_DATA_BY_TIME(refer_ts, odometry_data_ptr);
  if (code != ErrorCode::SUCCESS || odometry_data_ptr == nullptr || odometry_data_ptr->data == nullptr) {
    TERROR << "OdMMSparse4DImpl::Run failed: get odometry data failed, code: " << code;
    return code;
  }
  input_->ego_pose = odometry_data_ptr->data->Matrix().cast<float>();

  // detect
  std::vector<net::BoundingBox> sdk_bboxes;
  if (!detector_->Processing(input_, sdk_bboxes)) {
    TERROR << "OdMMSparse4DImpl::Run failed: detect failed";
    return ErrorCode::OD_MMSPARSE4D_DETECT_FAILED;
  }

  // 将检测结果转换为TrunkPerception的检测结果
  for (const auto& bbox : sdk_bboxes) {
    Object obj;
    obj.timestamp = refer_ts;
    obj.type = DetectionNetSDKObjectTypeDict[bbox.type];
    obj.bbox.center = bbox.center;
    obj.bbox.size = bbox.size;
    obj.velocity.x() = bbox.velocity.x();
    obj.velocity.y() = bbox.velocity.y();
    obj.bbox.theta = bbox.theta;
    obj.confidence = bbox.confidence;
    od_front_mm_frame->detected_objects.emplace_back(obj);
  }

  UPDATE_OD_FRONT_MM_FRAME(od_front_mm_frame);

  return ErrorCode::SUCCESS;
}

std::any OdMMSparse4DImpl::GetData(const std::string& key) { return std::any(); }

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END

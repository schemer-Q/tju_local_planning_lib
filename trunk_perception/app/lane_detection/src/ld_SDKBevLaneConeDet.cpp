#include "trunk_perception/app/lane_detection/ld_SDKBevLaneConeDet.h"
#include <cstdint>
#include <memory>
#include "SDKBevLaneConeDetHighway/SDKBevLaneConeDet.h"
#include "trunk_perception/app/lane_detection/algo/MathUtil.h"
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/data_manager/data_wrapper/image_data.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/lane.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

class VisionSDKLogger : public SDKLogger {
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

LaneDetectorSDKBevLaneConeDet::~LaneDetectorSDKBevLaneConeDet() {
  if (bevlane_data_) {
    delete bevlane_data_;
    bevlane_data_ = nullptr;
  }
  bevlaneconedet::SDK_BEVLANECONEDET_RELEASE(bevlane_instance_);
}

std::uint32_t LaneDetectorSDKBevLaneConeDet::Init(const YAML::Node& config) {
  uint32_t code = ErrorCode::SUCCESS;

  try {
    name_ = config["Name"].as<std::string>();
    type_ = config["Type"].as<std::string>();
    config_file_ = config["ConfigFile"].as<std::string>();
    ransac_max_iterations_ = config["PostProcess"]["Ransac"]["MaxIterations"].as<int>();
    ransac_inlier_pts_rate_ = config["PostProcess"]["Ransac"]["InlierPtsRate"].as<float>();
    ransac_degree_ = config["PostProcess"]["Ransac"]["Degree"].as<int>();
    min_cone_margin_pts_threshold_ = config["PostProcess"]["MinConeMarginPtsThreshold"].as<size_t>();
    bev_lane_pt_min_interval_ = config["PostProcess"]["BEVLanePtMinInterval"].as<float>();
    detected_lane_min_pts_ = config["PostProcess"]["DetectedLaneMinPts"].as<size_t>();
    detected_lane_min_length_x_ = config["PostProcess"]["DetectedLaneMinLengthX"].as<float>();
    detected_lane_min_range_x_ = config["PostProcess"]["DetectedLaneMinRangeX"].as<float>();
    lane_bev_fitting_exponent_ = config["PostProcess"]["LaneBevFittingExponent"].as<size_t>();
  } catch (const std::exception& e) {
    TFATAL << "LaneDetectorSDKBevLaneConeDet::Init failed, error: " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }

  // 初始化Ransac
  ransac_fitter_ = std::make_shared<ld_algo::RansacPolynomialFitter>(ransac_max_iterations_, ransac_inlier_pts_rate_,
                                                                     ransac_degree_);
  lane_cluster_ = ld_algo::LaneClustering(3, 5.0f);

  bevlane_data_ = new bevlaneconedet::BevLaneConeDetData();

  // 初始化相机参数
  code = InitCameraParams();
  if (code != ErrorCode::SUCCESS) {
    TWARNING << "LaneDetectorSDKBevLaneConeDet::Init failed, error: " << code;
    TINFO << "LaneDetectorSDKBevLaneConeDet::Init, Lazy init SDK is turned on";
    lazy_init_ = true;
    return ErrorCode::SUCCESS;
  } else {
    // 初始化SDK
    std::shared_ptr<VisionSDKLogger> bevlane_logger = std::make_shared<VisionSDKLogger>();
    bool use_vpi = true;
    int ret = bevlaneconedet::SDK_BEVLANECONEDET_INIT(&bevlane_instance_, config_file_, name_, *camera_param_,
                                                      bevlane_logger, use_vpi);
    if (ret != 0) {
      TFATAL << "LaneDetectorSDKBevLaneConeDet::Init SDK_BEVLANECONEDET_INIT failed, error: " << ret;
      return ErrorCode::LANE_DETECTOR_SDK_INIT_FAILED;
    }
    is_init_ = true;
  }

  TINFO << "LaneDetectorSDKBevLaneConeDet::Init success, name: " << name_ << ", type: " << type_;
  return ErrorCode::SUCCESS;
}

std::uint32_t LaneDetectorSDKBevLaneConeDet::Run(const double& ts) {
  uint32_t code = ErrorCode::SUCCESS;

  // 懒初始化
  if (lazy_init_ && !is_init_) {
    code = InitCameraParams();
    if (code != ErrorCode::SUCCESS) {
      TERROR << "LaneDetectorSDKBevLaneConeDet::Run lazy init failed, error: " << code;
      return code;
    }
    // 初始化SDK
    std::shared_ptr<VisionSDKLogger> bevlane_logger = std::make_shared<VisionSDKLogger>();
    bool use_vpi = true;
    int ret = bevlaneconedet::SDK_BEVLANECONEDET_INIT(&bevlane_instance_, config_file_, name_, *camera_param_,
                                                      bevlane_logger, use_vpi);
    if (ret != 0) {
      TFATAL << "LaneDetectorSDKBevLaneConeDet::Init SDK_BEVLANECONEDET_INIT failed, error: " << ret;
      return ErrorCode::LANE_DETECTOR_SDK_INIT_FAILED;
    }
    is_init_ = true;
  }

  // 获取图像数据
  std::shared_ptr<ImageData> image_data = nullptr;
  code = GET_SENSOR_DATA_BY_TIME(name_, type_, ts, image_data);
  if (code != ErrorCode::SUCCESS) {
    TERROR << "LaneDetectorSDKBevLaneConeDet::Run get image data failed, error: " << code;
    return code;
  }
  if (!image_data) {
    TERROR << "LaneDetectorSDKBevLaneConeDet::Run image data is nullptr";
    return ErrorCode::IMAGE_DATA_INVALID;
  }
  if (!image_data->data || image_data->data->image.empty()) {
    TERROR << "LaneDetectorSDKBevLaneConeDet::Run image data is nullptr or empty";
    return ErrorCode::IMAGE_DATA_INVALID;
  }

  // init ld frame
  std::shared_ptr<LDFrame> ld_frame = std::make_shared<LDFrame>();
  ld_frame->timestamp = image_data->time;

  // SDK infer
  bevlane_data_->images = {{image_data->data->image}};
  int ret = bevlaneconedet::SDK_BEVLANECONEDET_INFER(bevlane_instance_, *bevlane_data_);
  if (ret) {
    TERROR << "LaneDetectorSDKBevLaneConeDet::Run SDK_BEVLANECONEDET_INFER failed, error: " << ret;
    return ErrorCode::LANE_DETECTOR_SDK_INFER_FAILED;
  }

  LanePost(ld_frame);

  UPDATE_LD_FRAME(ld_frame);

  return code;
}

void LaneDetectorSDKBevLaneConeDet::LanePost(const std::shared_ptr<LDFrame>& ld_frame) {
  auto& lane_types = bevlane_data_->detected3DLaneTypes;
  for (size_t i = 0; i < bevlane_data_->detected3DLane.size(); ++i) {
    int lane_type = lane_types[i];
    LaneLineVision lane;

    // 进行线的外点滤除
    std::vector<Eigen::Vector2d> laneline_pts_ransac;
    for (auto& pt : bevlane_data_->detected3DLane[i]) {
      laneline_pts_ransac.push_back({pt.x, pt.y});
    }

    // use ransac fitter to filter out the unormal points
    ld_algo::RansacResult ransac_result;
    if (!ransac_fitter_->FitWithInliers(laneline_pts_ransac, ransac_result)) {
      continue;
    }

    std::vector<cv::Point3f> laneline_pts;
    for (int idx : ransac_result.inlier_idxs) {
      laneline_pts.push_back(cv::Point3f(bevlane_data_->detected3DLane[i][idx].x,
                                         bevlane_data_->detected3DLane[i][idx].y,
                                         bevlane_data_->detected3DLane[i][idx].z));
    }

    auto lane_3d = lane_cluster_.Cluster(laneline_pts);
    if (lane_3d.size() < min_cone_margin_pts_threshold_) {
      continue;
    }

    // 纵向距离由近至远排序
    std::sort(lane_3d.begin(), lane_3d.end(), [](const cv::Point3f& p1, const cv::Point3f& p2) { return p1.x < p2.x; });

    float last_pt_x = -10.f;
    for (const auto& world_pt : lane_3d) {
      // todo bevlanedet输出的是x是相机z, y是相机x，z是相机y
      // cv::Point3f world_pt = camera_proj_ptr_->CameraToWorld({camera_pt.y, camera_pt.z, camera_pt.x});
      // 每米最多保留一个点
      if (world_pt.x - last_pt_x <= bev_lane_pt_min_interval_) {
        continue;
      }
      last_pt_x = world_pt.x;
      lane.OriginPointsWorld.emplace_back(world_pt.x, world_pt.y, world_pt.z);
      auto img_pt = camera_projection_->WorldToImg(cv::Point3f(world_pt.x, world_pt.y, world_pt.z));
      // 可视化图像为原图，对img_pt添加畸变
      img_pt = camera_undistort_->DistortPoint(img_pt);
      lane.OriginPointsImg.emplace_back(img_pt);
      // img_fit_pts.emplace_back(img_pt.y, img_pt.x);
    }

    // 车道线检测点数最小限制
    if (lane.OriginPointsWorld.size() < detected_lane_min_pts_ ||
        // 车道线远端点的最小距离限制
        lane.OriginPointsWorld.back().x < detected_lane_min_length_x_ ||
        // 车道线检测长度最短限制
        lane.OriginPointsWorld.back().x < detected_lane_min_range_x_) {
      continue;
    }

    // 图像纵坐标由上至下排序
    std::sort(lane.OriginPointsImg.begin(), lane.OriginPointsImg.end(),
              [](const cv::Point& p1, const cv::Point& p2) { return p1.y < p2.y; });

    // 拟合bev多项式曲线
    cv::Mat polyline_bev;
    if (!ld_algo::PolyFitting(lane.OriginPointsWorld, lane_bev_fitting_exponent_, polyline_bev)) {
      continue;
    }
    lane.a0 = polyline_bev.at<double>(0, 0);
    lane.a1 = polyline_bev.at<double>(1, 0);
    lane.a2 = polyline_bev.at<double>(2, 0);
    lane.a3 = polyline_bev.at<double>(3, 0);

    if (LaneLineType(lane_type) == LaneLineType::SOLID) {
      lane.lane_line_type = LaneLineType::SOLID;  // solid
    } else {
      lane.lane_line_type = LaneLineType::DASHED;  // dashed
    }
    lane.lane_line_color = LaneLineColor::WHITE;

    ld_frame->lanes_detected.push_back(lane);
  }
}

std::any LaneDetectorSDKBevLaneConeDet::GetData(const std::string& key) { return std::any(); }

uint32_t LaneDetectorSDKBevLaneConeDet::InitCameraParams() {
  uint32_t code = ErrorCode::SUCCESS;
  std::shared_ptr<CameraMetaInfo> meta = nullptr;
  code = GET_META_INFO(name_, meta);

  if (code != ErrorCode::SUCCESS) {
    TERROR << "LaneDetectorSDKBevLaneConeDet::Init failed, error: " << code;
    return code;
  }

  if (!meta) {
    TERROR << "LaneDetectorSDKBevLaneConeDet::Init failed, meta is nullptr";
    return ErrorCode::LANE_DETECTOR_INIT_CAMERA_PARAMS_FAILED;
  }
  if (!meta->camera_info_ptr || !meta->pose_ptr) {
    TERROR << "LaneDetectorSDKBevLaneConeDet::Init failed, camera_info_ptr or pose_ptr is nullptr";
    return ErrorCode::LANE_DETECTOR_INIT_CAMERA_PARAMS_FAILED;
  }

  camera_param_ = std::make_shared<bevlaneconedet::CameraParam>();
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
  // if (meta->camera_info_ptr->D.rows == 5) {
  //   camera_param_->DistortionFactor[0] = meta->camera_info_ptr->D.at<double>(0, 0);
  //   camera_param_->DistortionFactor[1] = meta->camera_info_ptr->D.at<double>(1, 0);
  //   camera_param_->DistortionFactor[2] = meta->camera_info_ptr->D.at<double>(2, 0);
  //   camera_param_->DistortionFactor[3] = meta->camera_info_ptr->D.at<double>(3, 0);
  //   camera_param_->DistortionFactor[4] = meta->camera_info_ptr->D.at<double>(4, 0);
  // } else {
  //   TFATAL << "LaneDetectorSDKBevLaneConeDet::Init failed, camera_info_ptr->D.rows is not 5, SDKBevLaneConeDetHighway
  //   "
  //             "not support";
  //   return ErrorCode::LANE_DETECTOR_INIT_CAMERA_PARAMS_FAILED;
  // }
  // 使用5参数模型，伪装一下,SDK内并没有使用
  camera_param_->DistortionFactor[0] = 0.0;
  camera_param_->DistortionFactor[1] = 0.0;
  camera_param_->DistortionFactor[2] = 0.0;
  camera_param_->DistortionFactor[3] = 0.0;
  camera_param_->DistortionFactor[4] = 0.0;

  Eigen::Quaternionf quat(meta->pose_ptr->rotation());
  camera_param_->Quaternion[0] = quat.x();
  camera_param_->Quaternion[1] = quat.y();
  camera_param_->Quaternion[2] = quat.z();
  camera_param_->Quaternion[3] = quat.w();
  camera_param_->x = meta->pose_ptr->translation().x();
  camera_param_->y = meta->pose_ptr->translation().y();
  camera_param_->z = meta->pose_ptr->translation().z();
  camera_param_->image_height = meta->camera_info_ptr->IMG_H;
  camera_param_->image_width = meta->camera_info_ptr->IMG_W;

  camera_undistort_ = GET_CAMERA_UNDISTORT(name_);
  if (!camera_undistort_) {
    TERROR << "LaneDetectorSDKBevLaneConeDet::Init failed, camera_undistort_ is nullptr: " << ErrorCode::UNINITIALIZED;
    return ErrorCode::UNINITIALIZED;
  }
  camera_projection_ = GET_CAMERA_PROJECTION(name_);
  if (!camera_projection_) {
    TERROR << "LaneDetectorSDKBevLaneConeDet::Init failed, camera_projection_ is nullptr: " << ErrorCode::UNINITIALIZED;
    return ErrorCode::UNINITIALIZED;
  }

  TINFO << "LaneDetectorSDKBevLaneConeDet::InitCameraParams success";

  return code;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END

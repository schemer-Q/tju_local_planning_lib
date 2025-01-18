#include "trunk_perception/app/lane_detection_post/bev_lane_post_impl.h"
#include <cstddef>
#include <memory>
#include "trunk_perception/app/lane_detection/algo/MathUtil.h"
#include "trunk_perception/common/data_manager/data_manager.h"
#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/lane.h"
#include "trunk_perception/tools/log/t_log.h"
#include "trunk_perception/app/lane_detection_post/algo/tracker/lane_point_tracker.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

BevLanePostImpl::~BevLanePostImpl() = default;

std::uint32_t BevLanePostImpl::Init(const YAML::Node& config) {
  try {
    // tracker 
    lane_tracker_ptr_ = std::make_shared<ld_post::LanePointTracker>();
    lane_tracker_ptr_->Init(config["Tracker"]);

  } catch (const std::exception& e) {
    TFATAL << "BevLanePostImpl::Init() failed, " << e.what();
    return ErrorCode::YAML_CONFIG_ERROR;
  }
  return ErrorCode::SUCCESS;
}

std::uint32_t BevLanePostImpl::Run(const double& ts) {
  // 获取当前帧数据
  auto ld_frame = GET_LD_FRAME();
  if (ld_frame == nullptr) {
    TWARNING << "BevLanePostImpl::Run() ld_frame is nullptr";
    return ErrorCode::LANE_TRACKER_LD_FRAME_NOT_FOUND;
  }

  // NOTE: 这里跟原SDK有区别
  // 原SDK: 转移矩阵失效时，标记tracklet为lost，会被删除
  // 这里: Odom获取失败，放弃当前帧的跟踪，返回错误码，已有tacklet不会被删除
  std::shared_ptr<OdometryData> odometry;
  if (GET_ODOMETRY_DATA_BY_TIME(ld_frame->timestamp, odometry) != ErrorCode::SUCCESS) {
    return ErrorCode::LANE_TRACKER_ODOMETRY_NOT_FOUND;
  }
  if (odometry == nullptr || odometry->data == nullptr) {
    return ErrorCode::LANE_TRACKER_ODOMETRY_NOT_FOUND;
  }
  auto pose = odometry->data->Matrix();
  TDEBUG<<"odometry: "<<odometry->data;  /// todo cjs 
  auto res = lane_tracker_ptr_->Run(ld_frame->lanes_detected, pose, ts, ld_frame->lanes_tracked);

  return res;
}

std::any BevLanePostImpl::GetData(const std::string& key) { return std::any(); }

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END

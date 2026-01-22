#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>
#include <vector>
#include <cmath>

#include "tju_local_planning/algo/cost_map/occupancy_grid/occupancy_grid.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "occupancy_grid_test_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/test/occupancy_grid", 1, true);

  using namespace tju_local_planning::cost_map;
  GridConfig cfg;
  cfg.resolution = 0.2;
  cfg.width = 200;
  cfg.height = 200;
  cfg.origin_x = -20.0;
  cfg.origin_y = -20.0;

  OccupancyGrid grid(cfg);

  // 构造一些示例占据点（一个方块和一条直线）
  PointCloud pc;
  for (double x = -5.0; x <= 5.0; x += 0.1) {
    for (double y = -5.0; y <= 5.0; y += 0.1) {
      if (std::hypot(x, y) < 4.5) {
        PointXYZ p; p.x = x; p.y = y; p.z = 0.0; pc.push_back(p);
      }
    }
  }
  // 直线障碍
  for (double x = -10.0; x <= 10.0; x += 0.1) {
    PointXYZ p; p.x = x; p.y = 8.0; p.z = 0.0; pc.push_back(p);
  }

  Pose sensor_pose; sensor_pose.x = 0; sensor_pose.y = 0; sensor_pose.yaw = 0;
  grid.update(pc, sensor_pose);

  nav_msgs::OccupancyGrid msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.info.resolution = cfg.resolution;
  msg.info.width = cfg.width;
  msg.info.height = cfg.height;
  msg.info.origin.position.x = cfg.origin_x;
  msg.info.origin.position.y = cfg.origin_y;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(cfg.width * cfg.height);
  for (int iy = 0; iy < cfg.height; ++iy) {
    for (int ix = 0; ix < cfg.width; ++ix) {
      double p = grid.queryCellProbability(ix, iy);
      int idx = iy * cfg.width + ix;
      if (p > 0.65) msg.data[idx] = 100;
      else if (p < 0.35) msg.data[idx] = 0;
      else msg.data[idx] = -1;
    }
  }

  ros::Rate rate(1.0);
  while (ros::ok()) {
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

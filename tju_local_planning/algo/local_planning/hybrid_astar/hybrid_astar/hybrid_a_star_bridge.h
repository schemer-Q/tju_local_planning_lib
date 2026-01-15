#ifndef HYBRID_A_STAR_BRIDGE_H
#define HYBRID_A_STAR_BRIDGE_H
#include "hybrid_a_star.h"
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <chrono>

#include "tju_local_planning/algo/local_planning/local_trajectory_planner_base.h"
#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/pose_point.h"
#include "tju_local_planning/common/types/target_pose.h"
#include "tju_local_planning/common/types/odometry.h"
#include "tju_local_planning/common/types/local_trajectory_points.h"
#include "tju_local_planning/tools/log/t_log.h"

TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN
using namespace TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE;

class HybridAStarBridge: public LocalTrajectoryPlannerBase{
public:
    // 构造函数
    HybridAStarBridge() ;
    
    // 析构函数
    ~HybridAStarBridge() = default;
    
    // INIT接口
    int init(const YAML::Node& config) override;

    // Process接口
    int process(const PosePoint& current_pose,const PosePoint& target_pose) override;
    int process() override;

    // Getdata接口
    std::any GetData(const std::string& key) override;

private:
    // 地图数据结构
    struct PGMMap {
        std::vector<uint8_t> data;   // 地图数据
        int width;                   // 地图宽度 (像素)
        int height;                  // 地图高度 (像素)
        double resolution;           // 分辨率 (米/像素)
        double origin_x;             // 原点x坐标 (米)
        double origin_y;             // 原点y坐标 (米)

        int get(int map_x, int map_y) const {
            if (map_x < 0 || map_x >= width || map_y < 0 || map_y >= height) {
                return 0; // 超出地图范围，视为不可行
            }
            return static_cast<int>(data[(height - map_y - 1) * width + map_x]);
        }
    };
    // 车辆参数
    struct VehicleParams {
        double length;               // 车辆长度 (米)
        double width;                // 车辆宽度 (米)
        double wheelbase;            // 轴距 (米)
        double max_steering_angle;   // 最大转向角 (角度)
        double a_lat_max;            // 最大横向加速度（米每秒方）
        double v_max_vehicle;        // 车辆最大速度（米每米）
        double a_max;                // 最大加速度（米每秒方）
    };
    
    // 规划参数
    struct PlanningParams {
        double segment_length; // 每次扩展的轨迹段长度（总长度）
        int max_iterations;          // 最大迭代次数
        double collision_check_step; // 碰撞检测步长 (米)
        int steering_angle_partitions; // 新增：转向角度切分数
        int segment_length_discrete_num; // 一个轨迹段被离散为多少步
        double steering_penalty; // 转向惩罚系数（惩罚非直行）
        double reversing_penalty; // 倒车惩罚系数
        double steering_change_penalty; // 转向变化惩罚系数（惩罚频繁变舵）
        double shot_distance; // 尝试RS路径（解析扩展）的距离阈值
    };
    
private:
    std::unique_ptr<PGMMap> map_;            // PGM地图
    VehicleParams vehicle_params_;           // 车辆参数
    PlanningParams planning_params_;         // 规划参数 
    std::vector<PosePoint> trajectory_;      // 局部路径输出容器
    std::shared_ptr<HybridAStar> astar_searcher_ptr_; // HybridAStar搜索器指针
    bool debug = false;                       // 测试模式

private:
    /**
     * @brief 加载PGM地图
     * @param pgm_file PGM文件路径
     * @param resolution 地图分辨率 (米/像素)
     * @param origin_x 地图原点x坐标 (米)
     * @param origin_y 地图原点y坐标 (米)
     * @return 是否加载成功
     */
    bool loadMap(const std::string& pgm_file, double resolution, double origin_x, double origin_y);
    
    /**
     * @brief 入参类型转换
     * @param current_pose 当前位姿
     * @param target_pose 目标位姿（车身坐标系）
     * @return std::make_pair(Vec3d::start, Vec3d::goal) start 规划起点;goal 规划终点
     */
    std::pair<Vec3d,Vec3d> inputTransform(const PosePoint& current_pose, const PosePoint& target_pose) const;

    /**
     * @brief 出参类型转换
     * @param path 规划路径
     * @return 局部路径
     */
    std::vector<PosePoint> outputTransform(const VectorVec4d&path) const;

    /**
     * @brief 弧度归一化
     */
    double normalize_angle(double angle) const{
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;

        return angle;
    }

    /**
     * @brief 计算曲率
     */
    double calcCurvature(const VectorVec4d& path, size_t i) const{
        if (i <= 0 || i >= path.size() - 1) return 0.0; // 边界点曲率设为0

        const auto& p0 = path[i - 1];
        const auto& p1 = path[i];
        const auto& p2 = path[i + 1];

        double x0 = p0.x(), y0 = p0.y();
        double x1 = p1.x(), y1 = p1.y();
        double x2 = p2.x(), y2 = p2.y();

        // double a = std::hypot(x1 - x0, y1 - y0);
        // double b = std::hypot(x2 - x1, y2 - y1);
        // double c = std::hypot(x2 - x0, y2 - y0);
        // double dot = std::abs(x1 - x0)*(x2 - x1) + (y1 - y0)*(y2 - y1)

        double area = std::abs((x1 - x0)*(y2 - y1) - (y1 - y0)*(x2 - x1));
        double curvature = 0.0;

        if (std::abs(area) < 1e-6 ) return 0.0;
        else {
            double dist1 = (std::hypot((x1 - x0), (y1 - y0)) > 0.0) ? std::hypot((x1 - x0), (y1 - y0)): 0.001 ; 
            double dist2 = (std::hypot((x2 - x1), (y2 - y1)) > 0.0) ? std::hypot((x2 - x1), (y2 - y1)): 0.001 ; 
            curvature = 2*area/(dist1*dist2*(dist1+dist2));
        }
        // if (a * b * c < 1e-6) return 0.0; // 防止除零

        // double curvature = 4 * area / (a * b * c);

        return curvature;
    }
    
};

TJU_LOCAL_PLANNING_LIB_NAMESPACE_END
#endif // HYBRID_A_STAR_BRIDGE_H

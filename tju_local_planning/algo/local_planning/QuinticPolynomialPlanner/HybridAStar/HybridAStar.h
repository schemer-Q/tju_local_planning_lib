#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H
#define M_PI		3.14159265358979323846
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <utility>

#include "tju_local_planning/algo/local_planning/QuinticPolynomialPlanner/local_trajectory_planner_base.h"
#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/pose_point.h"
#include "tju_local_planning/common/types/target_pose.h"
#include "tju_local_planning/common/types/odometry.h"
#include "tju_local_planning/common/types/local_trajectory_points.h"
#include "tju_local_planning/tools/log/t_log.h"

TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN
using namespace TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE;

// 车辆位姿结构体
struct Pose {
    double x;       // 世界坐标系x坐标 (米)
    double y;       // 世界坐标系y坐标 (米)
    double theta;   // 朝向角度 (弧度)
    int gear;       // 档位(1前进 -1倒退)

    Pose(double x_ = 0, double y_ = 0, double theta_ = 0, int gear_ = 1) 
        : x(x_), y(y_), theta(theta_), gear(gear_) {}
    
    bool operator==(const Pose& other) const {
        return std::abs(x - other.x) < 1e-5 && 
               std::abs(y - other.y) < 1e-5 &&
               std::abs(theta - other.theta) < 1e-5 &&
               gear == other.gear;
    }
};

// 节点结构体，用于A*搜索
struct Node {
    Pose pose;              // 车辆位姿
    double g_cost;          // 从起点到当前节点的实际代价
    double h_cost;          // 到目标点的启发式代价
    double f_cost() const { return g_cost + h_cost; } // 总代价
    std::shared_ptr<Node> parent; // 父节点指针
    
    Node(const Pose& p, double g, double h, std::shared_ptr<Node> parent_ = nullptr)
        : pose(p), g_cost(g), h_cost(h), parent(parent_) {}
    
    // 用于优先队列比较
    bool operator>(const Node& other) const {
        return f_cost() > other.f_cost();
    }
};

// 混合A*路径规划类
class HybridAStar : public LocalTrajectoryPlannerBase {
public:
    // 构造函数
    HybridAStar() ;
    
    // 析构函数
    ~HybridAStar() = default;
    
    //INIT接口
    int init(const YAML::Node& config) override;
    //Process接口
    int process(const PosePoint& current_pose,const PosePoint& target_pose) override;
    int process() override;
    //Getdata接口
    std::any GetData(const std::string& key) override;
    
    
private:
    // PGM地图数据结构
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
        double max_steering_angle;   // 最大转向角 (弧度)
        double a_lat_max;            // 最大横向加速度（米每秒方）
        double v_max_vehicle;        // 车辆最大速度（米每米）
        double a_max;                // 最大加速度（米每秒方）
    };
    
    // 规划参数
    struct PlanningParams {
        double step_size;            // 模拟步长 (米)
        int max_iterations;          // 最大迭代次数
        double collision_check_step; // 碰撞检测步长 (米)
        int steering_angle_partitions; // 新增：转向角度切分数
    };
    
    std::unique_ptr<PGMMap> map_;            // PGM地图
    VehicleParams vehicle_params_;           // 车辆参数
    PlanningParams planning_params_;         // 规划参数
    std::vector<Pose> path;                  // 规划路径容器
    std::vector<PosePoint> trajectory_;      // 局部路径输出容器
    
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
     * @brief 设置车辆参数
     * @param length 车辆长度 (米)
     * @param width 车辆宽度 (米)
     * @param wheelbase 轴距 (米)
     * @param max_steering_angle 最大转向角 (弧度)
     */
    void setVehicleParams(double length, double width, double wheelbase, double max_steering_angle);
    
    /**
     * @brief 设置规划参数
     * @param step_size 模拟步长 (米)
     * @param max_iterations 最大迭代次数
     * @param collision_check_step 碰撞检测步长 (米)
     */
    void setPlanningParams(double step_size, int max_iterations, double collision_check_step);
    
    /**
     * @brief 规划路径
     * @param start 起始位姿(世界坐标)
     * @param goal 目标位姿（世界坐标）
     * @return 是否规划成功
     */
    bool plan(const Pose& start, const Pose& goal);

    /**
     * @brief 检查位姿是否在障碍物中
     * @param pose 要检查的位姿
     * @return 是否碰撞
     */
    bool isInCollision(const Pose& pose) const;
    
    /**
     * @brief 检查路径段是否碰撞
     * @param start 起点位姿
     * @param end 终点位姿
     * @return 是否碰撞
     */
    bool isPathInCollision(const Pose& start, const Pose& end) const;
    
    /**
     * @brief 计算启发式代价
     * @param pose 当前位姿
     * @param goal 目标位姿
     * @return 启发式代价
     */
    double calculateHeuristic(const Pose& pose, const Pose& goal) const;
    
    /**
     * @brief 离散化位姿，用于哈希
     * @param pose 输入位姿
     * @return 离散化后的位姿
     */
    Pose discretizePose(const Pose& pose) const;
    
    /**
     * @brief 生成后继节点
     * @param current 当前节点
     * @param goal 目标位姿
     * @return 后继节点列表
     */
    std::vector<std::shared_ptr<Node>> generateSuccessors(
        const std::shared_ptr<Node>& current, const Pose& goal) const;
    
    /**
     * @brief 从节点回溯路径
     * @param node 终点节点
     * @param path 输出的路径
     */
    void backtrackPath(const std::shared_ptr<Node>& node, std::vector<Pose>& path) const;
    
    /**
     * @brief 世界坐标转地图坐标
     * @param x 世界x坐标
     * @param y 世界y坐标
     * @param map_x 输出的地图x坐标
     * @param map_y 输出的地图y坐标
     * @return 是否在有效范围内
     */
    bool worldToMap(double x, double y, int& map_x, int& map_y) const;
    
    /**
     * @brief 地图坐标转世界坐标
     * @param map_x 地图x坐标
     * @param map_y 地图y坐标
     * @param x 输出的世界x坐标
     * @param y 输出的世界y坐标
     */
    void mapToWorld(int map_x, int map_y, double& x, double& y) const;

    /**
     * @brief 入参类型转换
     * @param current_pose 当前位姿
     * @param target_pose 目标位姿（车身坐标系）
     * @return std::make_pair(start, goal) start 规划起点;goal 规划终点
     */
    std::pair<Pose, Pose> dataTransform(const PosePoint& current_pose, const PosePoint& target_pose) const;

    /**
     * @brief 出参类型转换
     * @param path 规划器路径
     * @return trajectory 输出局部路径
     */
    std::vector<PosePoint> dataTransform(const std::vector<Pose>& path) const;

    double normalize_angle(double angle) const{
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;

        return angle;
    }

    double calcCurvature(const std::vector<Pose>& path, size_t i) const{
        if (i <= 0 || i >= path.size() - 1) return 0.0; // 边界点曲率设为0

        const auto& p0 = path[i - 1];
        const auto& p1 = path[i];
        const auto& p2 = path[i + 1];

        double x0 = p0.x, y0 = p0.y;
        double x1 = p1.x, y1 = p1.y;
        double x2 = p2.x, y2 = p2.y;

        double a = std::hypot(x1 - x0, y1 - y0);
        double b = std::hypot(x2 - x1, y2 - y1);
        double c = std::hypot(x2 - x0, y2 - y0);

        double area = std::abs((x1 - x0)*(y2 - y0) - (y1 - y0)*(x2 - x0)) / 2.0;

        if (a * b * c < 1e-6) return 0.0; // 防止除零

        double curvature = 4 * area / (a * b * c);

        return curvature;
    }
};

TJU_LOCAL_PLANNING_LIB_NAMESPACE_END
#endif // HYBRID_A_STAR_H

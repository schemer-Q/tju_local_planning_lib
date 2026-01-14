#include "HybridAStar.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <chrono>

// 哈希函数用于Pose
namespace std {
    template<>
    struct hash<tju::local_planning::Pose> {
        size_t operator()(const tju::local_planning::Pose& p) const {
            size_t h1 = hash<double>()(p.x);
            size_t h2 = hash<double>()(p.y);
            size_t h3 = hash<double>()(p.theta);
            size_t h4 = hash<int>()(p.gear);
            return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
        }
    };
}
//保证std命名空间在最外层

TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN
int HybridAStar::init(const YAML::Node& config) {
  std::string map_file;
  YAML::Node map_yaml;
  try {
    vehicle_params_.length = config["length"].as<double>();
    vehicle_params_.width = config["width"].as<double>();
    vehicle_params_.wheelbase = config["wheelbase"].as<double>();
    vehicle_params_.max_steering_angle = config["max_steering_angle"].as<double>();
    vehicle_params_.a_lat_max = config["a_lat_max"].as<double>();
    vehicle_params_.v_max_vehicle = config["v_max_vehicle"].as<double>();
    vehicle_params_.a_max = config["a_max"].as<double>();
    planning_params_.step_size = config["step_size"].as<double>();
    planning_params_.max_iterations = config["max_iterations"].as<double>();
    planning_params_.collision_check_step = config["collision_check_step"].as<double>();
    planning_params_.steering_angle_partitions = config["steering_angle_partitions"].as<int>();
    map_file = config["map_file"].as<std::string>();
    map_yaml = YAML::LoadFile(config["yaml_file"].as<std::string>());
  }
  catch (YAML::InvalidNode e) {
    NTWARNING << "load config failed and use default value" ;
    std::cerr<<"load config failed and use default value"<<std::endl;
    //using default value
  }
  // load map.yaml
  double origin_x;
  double origin_y;
  double resolution;

  if (map_yaml["origin"] && map_yaml["origin"].IsSequence()) {
    origin_x = map_yaml["origin"][0].as<double>();
    origin_y = map_yaml["origin"][1].as<double>();
    resolution = map_yaml["resolution"].as<double>();
  }
  else {NTWARNING << "map_yaml incorrect" ;}
  NTINFO<<"load map from: "<<map_file;
  if (!loadMap(map_file, resolution, origin_x, origin_y)) {
    NTWARNING << "!!!can not load map!!!" ;
    std::cerr<<"!!!can not load map!!!"<<std::endl;
    return 1;
  }

return 0;
}

int HybridAStar::process() {
    return 1;
}

int HybridAStar::process(const PosePoint& current_pose,const PosePoint& target_pose) {
    // 获取开始时间点
    auto start_time = std::chrono::high_resolution_clock::now();

    auto [start, goal] = dataTransform(current_pose, target_pose);
    if (!plan(start, goal)) {
        NTERROR<<"!!!plan failed!!!";
        std::cerr<<"hybridastar.cpp:: "<<"!!!plan failed!!!"<<std::endl;
        return 1;
    }
    else {
        trajectory_ = dataTransform(path);
        std::cerr<<"hybridastar.cpp:: "<<"路径容器trajecctory_.size:"<<trajectory_.size()<<std::endl;
        // 获取结束时间点
        auto end_time = std::chrono::high_resolution_clock::now();
        // 计算持续时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::cerr<<"hybridastar.cpp:: "<<"plan time: "<<duration.count()<<"ms"<<std::endl;

        return 0;
    }
}

std::pair<Pose, Pose> HybridAStar::dataTransform(const PosePoint& current_pose, const PosePoint& target_pose) const{
// 将target_pose（局部坐标）转换到世界坐标,路径规划在世界坐标进行，碰撞检测中路径点世界坐标转换到pgm坐标，用于查询占据情况。
    double target_x_global = current_pose.x + target_pose.x * cos(current_pose.yaw) - target_pose.y * sin(current_pose.yaw);
    double target_y_global = current_pose.y + target_pose.x * sin(current_pose.yaw) + target_pose.y * cos(current_pose.yaw);
    double target_yaw_global = normalize_angle(current_pose.yaw + target_pose.yaw);

    Pose start;
    Pose goal;

    start.x = current_pose.x;
    start.y = current_pose.y;
    // test
    // std::cerr<<"start.x: "<<start.x<<" start.y: "<<start.y<<std::endl;

    start.theta = current_pose.yaw;
    start.gear = 1;
    // 起点终点默认前进档，对规划不影响
    // goal.x = target_x_global;
    // goal.y = target_y_global;
    // goal.theta = target_yaw_global;

    //test 无需局部转全局
    goal.x = target_pose.x;
    goal.y = target_pose.y;
    goal.theta = target_pose.yaw;
    goal.gear = 1;
    //test
    // std::cerr<<"goal.x: "<<goal.x<<" goal.y: "<<goal.y<<std::endl;
    std::cerr<<"hybridastar.cpp:: "<<"全局定位"<<" start.x: "<<start.x<<" start.y: "<<start.y<<" start.theta: "<<start.theta<<std::endl;
    // std::cerr<<"hybridastar.cpp:: "<<"hy接受局部"<<" target.x: "<<target_pose.x<<" target.y: "<<target_pose.y<<" target.theta: "<<target_pose.yaw<<std::endl;
    std::cerr<<"hybridastar.cpp:: "<<"hy全局"<<" goal.x: "<<goal.x<<" goal.y: "<<goal.y<<" goal.theta: "<<goal.theta<<std::endl;
    return std::make_pair(start, goal);
}

std::vector<PosePoint> HybridAStar::dataTransform(const std::vector<Pose>& path) const{
    PosePoint posepoint = PosePoint{};
    std::vector<PosePoint> trajectory;
    double v_max;
    for (size_t i = 0; i < path.size(); i++) {
        posepoint.x = path[i].x;
        posepoint.y = path[i].y;
        posepoint.yaw = path[i].theta;
        posepoint.gear = (path[i].gear>0) ? 3 : 1;
        posepoint.curve = calcCurvature(path, i);
        trajectory.push_back(posepoint);
    }

    // 速度规划
    size_t N = trajectory.size(); 
    // 声明一个定长数组,用于储存速度
    std::vector<double> v(N, vehicle_params_.v_max_vehicle);

    // 1. 曲率限速
    for (size_t i = 0; i < N; ++i) {
        double v_curve;
        if (i == 0 || i == N-1) {v_curve = 0.0;} // 起点终点速度置零
        else {v_curve = sqrt(vehicle_params_.a_lat_max / (std::abs(trajectory[i].curve) + 1e-6));}
        v[i] = std::min(v[i], v_curve);
    }

    // 2. 前向加速度约束
    for (size_t i = 1; i < N; ++i) {
        double ds = std::hypot(trajectory[i].x - trajectory[i-1].x, trajectory[i].y - trajectory[i-1].y);
        v[i] = std::min(v[i], sqrt(v[i-1]*v[i-1] + 2*vehicle_params_.a_max*ds));
    }

    // 3. 后向减速度约束
    for (int i = N-2; i >= 0; --i) {
        double ds = std::hypot(trajectory[i+1].x - trajectory[i].x, trajectory[i+1].y - trajectory[i].y);
        v[i] = std::min(v[i], sqrt(v[i+1]*v[i+1] + 2*vehicle_params_.a_max*ds));
    }

    // 4. 赋值回路径点
    for (size_t i = 0; i < N; ++i) {
        trajectory[i].speed = v[i];
    }
    
    std::cerr<<"hybridastar.cpp:: "<<"trajectory size: "<<trajectory.size()<<std::endl;

    return trajectory;
}

std::any HybridAStar::GetData(const std::string& key) {return trajectory_;}


HybridAStar::HybridAStar() : map_(nullptr) {
    // 默认车辆参数 (小型车)
    vehicle_params_.length = 4.0;
    vehicle_params_.width = 1.8;
    vehicle_params_.wheelbase = 2.5;
    vehicle_params_.max_steering_angle = M_PI / 4.0; // 45度
    vehicle_params_.a_lat_max = 2.0;
    vehicle_params_.v_max_vehicle = 5.0;
    vehicle_params_.a_max = 3.0;
    // 默认规划参数
    planning_params_.step_size = 0.1;
    planning_params_.max_iterations = 5000;
    planning_params_.collision_check_step = 0.1;
    planning_params_.steering_angle_partitions = 10;
}

bool HybridAStar::loadMap(const std::string& pgm_file, double resolution, 
                         double origin_x, double origin_y) {
    std::ifstream file(pgm_file, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open PGM file: " << pgm_file << std::endl;
        return false;
    }
    
    // 读取PGM头信息
    std::string format;
    std::getline(file, format); // 第一行: P5或P2
    if (format != "P5" && format != "P2") {
        std::cerr << "Invalid PGM file format" << std::endl;
        return false;
    }
    
    // 跳过注释
    std::string line;
    while (std::getline(file, line)) {
        if (line[0] != '#') break;
    }
    
    // 读取宽度和高度
    std::istringstream iss(line);
    int width, height;
    iss >> width >> height;
    

    // 读取最大灰度值
    std::getline(file, line);
    int max_val;
    std::istringstream iss_max(line);
    iss_max >> max_val;
    
    // 分配内存
    map_ = std::make_unique<PGMMap>();
    map_->width = width;
    map_->height = height;
    map_->resolution = resolution;
    map_->origin_x = origin_x;
    map_->origin_y = origin_y;
    map_->data.resize(width * height);
    
    // 读取图像数据
    if (format == "P5") {
        // 二进制格式
        file.read(reinterpret_cast<char*>(map_->data.data()), width * height);
    } else {
        // ASCII格式
        for (int i = 0; i < width * height; ++i) {
            int val;
            file >> val;
            map_->data[i] = static_cast<uint8_t>(val);
        }
    }

    // test
    // for (auto i:map_->data) { 
    //     i = static_cast<int>(i);
    //     if (i!= 0) {std::cout<<"_______________i: "<<i<<" __________________";}
    // }

    return true;
}

void HybridAStar::setVehicleParams(double length, double width, 
                                 double wheelbase, double max_steering_angle) {
    vehicle_params_.length = length;
    vehicle_params_.width = width;
    vehicle_params_.wheelbase = wheelbase;
    vehicle_params_.max_steering_angle = max_steering_angle;
}

void HybridAStar::setPlanningParams(double step_size, int max_iterations, 
                                  double collision_check_step) {
    planning_params_.step_size = step_size;
    planning_params_.max_iterations = max_iterations;
    planning_params_.collision_check_step = collision_check_step;
}

bool HybridAStar::plan(const Pose& start, const Pose& goal) {
    if (!map_) {
        std::cerr<<"hybridastar.cpp:: " << "Map not loaded!" << std::endl;
        return false;
    }
    
    // 检查起点和终点是否在障碍物中
    if (isInCollision(start)) {
        std::cerr<<"hybridastar.cpp:: " << "Start pose is in collision!" << std::endl;
        return false;
    }
    
    if (isInCollision(goal)) {
        std::cerr<<"hybridastar.cpp:: " << "Goal pose is in collision!" << std::endl;
        return false;
    }
    
    // 优先队列，按f_cost排序
    auto cmp = [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
        return a->f_cost() > b->f_cost();
    };
    std::priority_queue<std::shared_ptr<Node>, 
                        std::vector<std::shared_ptr<Node>>, 
                        decltype(cmp)> open_set(cmp);
    
    // 已访问节点集合
    std::unordered_set<Pose> visited;
    
    // 起始节点
    double h_cost = calculateHeuristic(start, goal);
    auto start_node = std::make_shared<Node>(start, 0.0, h_cost);
    open_set.push(start_node);
    visited.insert(discretizePose(start));
    
    bool found = false;
    std::shared_ptr<Node> final_node = nullptr;
    int iterations = 0;
    
    while (!open_set.empty() && iterations < planning_params_.max_iterations) {
        iterations++;
        
        auto current = open_set.top();
        open_set.pop();
        
        // 检查是否到达目标
        if (std::abs(current->pose.x - goal.x) < 0.5 && 
            std::abs(current->pose.y - goal.y) < 0.5 &&
            std::abs(current->pose.theta - goal.theta) < 0.2) {
            final_node = current;
            found = true;
            break;
        }
        
        // 生成后继节点
        auto successors = generateSuccessors(current, goal);
        for (const auto& successor : successors) {
            auto disc_pose = discretizePose(successor->pose);
            if (visited.find(disc_pose) == visited.end()) {
                visited.insert(disc_pose);
                open_set.push(successor);
            }
        }
    }
    
    if (found && final_node) {
        backtrackPath(final_node, path);
        return true;
    }
    
    std::cerr<<"hybridastar.cpp:: " << "Path not found after " << iterations << " iterations." << std::endl;
    NTERROR<<"Path not found after " << iterations << " iterations.";
    return false;
}

bool HybridAStar::isInCollision(const Pose& pose) const {
    // 检查车辆轮廓是否碰撞
    // 简化模型: 将车辆视为矩形，检查四个角点
    // return false;
    // 计算车辆四个角点的世界坐标
    double half_length = vehicle_params_.length / 2.0;
    double half_width = vehicle_params_.width / 2.0;
    
    // 四个角点相对于车辆中心的偏移
    std::vector<std::pair<double, double>> corners = {
        {half_length, half_width},
        {half_length, -half_width},
        {-half_length, -half_width},
        {-half_length, half_width}
    };
    
    // 旋转并平移每个角点
    for (const auto& corner : corners) {
        double x = pose.x + corner.first * std::cos(pose.theta) - corner.second * std::sin(pose.theta);
        double y = pose.y + corner.first * std::sin(pose.theta) + corner.second * std::cos(pose.theta);
        int map_x, map_y;
        if (worldToMap(x, y, map_x, map_y)) {
            if (map_->get(map_x, map_y) < 250) {
                std::cerr<<"hybridastar.cpp:: "<<"!!碰撞!!"<<"map_x: "<<map_x<<" map_y: "<<map_y<<" val: "<<map_->get(map_x, map_y)<<std::endl; 
                // std::cerr<<"____________________map_x: "<<map_x<<" map_y: "<<map_y<<" val: "<<map_->get(map_x, map_y)<<"_____________________"<<std::endl;
                return true;
            }
        } else {
            // 超出地图边界视为碰撞
            std::cerr<<"hybridastar.cpp:: "<<"！！超出边界！！"<<"map_x: "<<map_x<<" map_y: "<<map_y<<std::endl;
            // std::cerr<<"____________________map_x: "<<map_x<<" map_y: "<<map_y<<std::endl;
            return true;
        }
    }

    return false;
}

bool HybridAStar::isPathInCollision(const Pose& start, const Pose& end) const {
    // 线性插值检查路径段
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double dtheta = end.theta - start.theta;
    double dist = std::hypot(dx, dy);
    
    int steps = static_cast<int>(dist / planning_params_.collision_check_step) + 1;
    
    for (int i = 0; i <= steps; ++i) {
        double ratio = static_cast<double>(i) / steps;
        Pose intermediate;
        intermediate.x = start.x + ratio * dx;
        intermediate.y = start.y + ratio * dy;
        intermediate.theta = start.theta + ratio * dtheta;
        intermediate.gear = start.gear; 
        // 保持与起点一致，但实际上gear对碰撞检测没有影响。
        if (isInCollision(intermediate)) {
            return true;
        }
    }
    
    return false;
}

double HybridAStar::calculateHeuristic(const Pose& pose, const Pose& goal) const {
    // 欧几里得距离
    double dx = goal.x - pose.x;
    double dy = goal.y - pose.y;
    double distance = std::hypot(dx, dy);
    
    // 角度差
    double angle_diff = std::abs(pose.theta - goal.theta);
    angle_diff = std::min(angle_diff, 2 * M_PI - angle_diff);
    
    // 综合考虑距离和角度
    return distance + 0.1 * angle_diff;
}

Pose HybridAStar::discretizePose(const Pose& pose) const {
    // 将位姿离散化为网格和角度区间
    const double xy_resolution = 0.05;  // 位置分辨率 (米)
    const double theta_resolution = M_PI / 36.0; // 角度分辨率 (5度)
    
    Pose disc_pose;
    disc_pose.x = std::round(pose.x / xy_resolution) * xy_resolution;
    disc_pose.y = std::round(pose.y / xy_resolution) * xy_resolution;
    disc_pose.theta = std::round(pose.theta / theta_resolution) * theta_resolution;
    disc_pose.gear = pose.gear;

    // 规范化角度到[0, 2π)
    while (disc_pose.theta < 0) disc_pose.theta += 2 * M_PI;
    while (disc_pose.theta >= 2 * M_PI) disc_pose.theta -= 2 * M_PI;
    
    return disc_pose;
}

std::vector<std::shared_ptr<Node>> HybridAStar::generateSuccessors(
    const std::shared_ptr<Node>& current, const Pose& goal) const {
    std::vector<std::shared_ptr<Node>> successors;
    
    // 计算转向角度步长
    double angle_step = 2 * vehicle_params_.max_steering_angle / 
                       (planning_params_.steering_angle_partitions - 1);
    
    // 生成转向角度列表
    std::vector<double> steering_angles;
    for (int i = 0; i < planning_params_.steering_angle_partitions; ++i) {
        double angle = -vehicle_params_.max_steering_angle + i * angle_step;
        steering_angles.push_back(angle);
    }
    // // 可能的转向角度 (左转、直行、右转)
    // std::vector<double> steering_angles = {
    //     -vehicle_params_.max_steering_angle, // 左转
    //     0.0,                                 // 直行
    //     vehicle_params_.max_steering_angle   // 右转
    // };
    
    // 可能的行驶方向 (前进、后退)
    std::vector<double> directions = {1.0, -1.0};
    
    for (double steer : steering_angles) {
        for (double dir : directions) {
            // 车辆运动模型 (简化的自行车模型)
            Pose new_pose;
            double step = planning_params_.step_size * dir;
            
            new_pose.theta = current->pose.theta + (step / vehicle_params_.wheelbase) * std::tan(steer);
            new_pose.x = current->pose.x + step * std::cos(new_pose.theta);
            new_pose.y = current->pose.y + step * std::sin(new_pose.theta);
            new_pose.gear = (dir > 0) ? 1 : -1;
            // 规范化角度
            while (new_pose.theta < 0) new_pose.theta += 2 * M_PI;
            while (new_pose.theta >= 2 * M_PI) new_pose.theta -= 2 * M_PI;
            
            // 检查路径是否碰撞
            if (!isPathInCollision(current->pose, new_pose)) {
                double move_cost = std::abs(step); // 移动代价
                double h_cost = calculateHeuristic(new_pose, goal);
                auto new_node = std::make_shared<Node>(new_pose, 
                                                     current->g_cost + move_cost, 
                                                     h_cost, 
                                                     current);
                successors.push_back(new_node);
            }
        }
    }
    
    return successors;
}

void HybridAStar::backtrackPath(const std::shared_ptr<Node>& node, std::vector<Pose>& path) const {
    path.clear();
    std::shared_ptr<Node> current = node;
    
    while (current) {
        path.push_back(current->pose);
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());
}

bool HybridAStar::worldToMap(double x, double y, int& map_x, int& map_y) const {
    map_x = static_cast<int>((x - map_->origin_x) / map_->resolution);
    map_y = static_cast<int>((y - map_->origin_y) / map_->resolution);
    // std::cerr<<"origin_x: "<<map_->origin_x<<" origin_y: "<<map_->origin_y<<" resolution: "<<map_->resolution<<std::endl;
    // std::cerr<<"map_x: "<<map_x<<" map_y: "<<map_y<<std::endl;
    // std::cerr<<"x: "<<x<<" y: "<<y<<std::endl;
    return map_x >= 0 && map_x < map_->width && map_y >= 0 && map_y < map_->height;
}

void HybridAStar::mapToWorld(int map_x, int map_y, double& x, double& y) const {
    x = map_->origin_x + (map_x + 0.5) * map_->resolution;
    y = map_->origin_y + (map_y + 0.5) * map_->resolution;
}


TJU_LOCAL_PLANNING_LIB_NAMESPACE_END
#include "hybrid_a_star_bridge.h"
#include <fstream>
#include <sstream>
#include <iostream>

TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN

HybridAStarBridge::HybridAStarBridge() : map_(nullptr), astar_searcher_ptr_(nullptr) {
    // 默认车辆参数 (小型车)
    vehicle_params_.length = 4.0;
    vehicle_params_.width = 1.8;
    vehicle_params_.wheelbase = 2.5;
    vehicle_params_.max_steering_angle = 45.0; // 45度
    vehicle_params_.a_lat_max = 2.0;
    vehicle_params_.v_max_vehicle = 5.0;
    vehicle_params_.a_max = 3.0;
    // 默认规划参数
    planning_params_.segment_length = 1.6;
    planning_params_.collision_check_step = 0.1;
    planning_params_.steering_angle_partitions = 10;
    planning_params_.segment_length_discrete_num = 8;
    planning_params_.steering_penalty = 1.5;
    planning_params_.reversing_penalty = 3.0;
    planning_params_.steering_change_penalty = 2.0;
    planning_params_.shot_distance = 4.0;
}

int HybridAStarBridge::init(const YAML::Node& config) {
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
    planning_params_.segment_length = config["segment_length"].as<double>();
    planning_params_.collision_check_step = config["collision_check_step"].as<double>();
    planning_params_.steering_angle_partitions = config["steering_angle_partitions"].as<int>();
    planning_params_.segment_length_discrete_num = config["segment_length_discrete_num"].as<int>();
    planning_params_.steering_penalty = config["steering_penalty"].as<double>();
    planning_params_.reversing_penalty = config["reversing_penalty"].as<double>();
    planning_params_.steering_change_penalty = config["steering_change_penalty"].as<double>();
    planning_params_.shot_distance = config["shot_distance"].as<double>();
    int debug_flag = config["debug"].as<int>();
    debug = true;
    /*算法内部将坐标系写死，可以在app中根据配置修改坐标系*/
    // if (debug_flag == 1) {
    //     debug = true;
    // }
    // else {
    //     debug = false;
    // }
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

  astar_searcher_ptr_ = std::make_shared<HybridAStar>(
            vehicle_params_.max_steering_angle,
            planning_params_.steering_angle_partitions,
            planning_params_.segment_length,
            planning_params_.segment_length_discrete_num,
            vehicle_params_.wheelbase,
            planning_params_.steering_penalty,
            planning_params_.reversing_penalty,
            planning_params_.steering_change_penalty,
            planning_params_.shot_distance
    );
    
  astar_searcher_ptr_->SetVehicleShape(vehicle_params_.length, vehicle_params_.width, vehicle_params_.wheelbase);

  astar_searcher_ptr_->Init(
                map_->origin_x,
                1.0*map_->width * map_->resolution,
                map_->origin_y,
                1.0*map_->height * map_->resolution,
                1.0,
                map_->resolution
        );

  unsigned int map_w = std::floor(map_->width);
  unsigned int map_h = std::floor(map_->height);
  for (unsigned int w = 0; w < map_w; ++w) {
      for (unsigned int h = 0; h < map_h; ++h) {
          if (map_->data[h*map_w + w] < 250) {
              astar_searcher_ptr_->SetObstacle(w, map_h-1-h);
              // printf("map_data[%d] = %d\n", h*map_w + w, map_->data[h*map_w + w]);
          }
      }
  }

return 0;
}

int HybridAStarBridge::process() {
    return 1;
}

int HybridAStarBridge::process(const PosePoint& current_pose,const PosePoint& target_pose) {
  // 获取开始时间点
  auto start_time = std::chrono::high_resolution_clock::now();

  std::pair<Vec3d, Vec3d> result = inputTransform(current_pose, target_pose);

  Vec3d start_state = result.first;
  Vec3d goal_state = result.second;
  
    if (astar_searcher_ptr_->Search(start_state, goal_state)) {
      // 输出路径
        auto path = astar_searcher_ptr_->GetPath();

        trajectory_ = outputTransform(path);
        std::cerr<<"HybridAStarBridge.cpp:: "<<"路径容器trajecctory_.size:"<<trajectory_.size()<<std::endl;
        // 获取结束时间点
        auto end_time = std::chrono::high_resolution_clock::now();
        // 计算持续时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::cerr<<"HybridAStarBridge.cpp:: "<<"plan time: "<<duration.count()<<"ms"<<std::endl;
    }
    astar_searcher_ptr_->Reset();
    return 0;
}

std::any HybridAStarBridge::GetData(const std::string& key) {return trajectory_;}

std::pair<Vec3d,Vec3d> HybridAStarBridge::inputTransform(const PosePoint& current_pose,const PosePoint& target_pose) const {
  if (debug == true) {
    // debug模式，直接使用全局坐标
  Vec3d start_state(current_pose.x, current_pose.y, current_pose.yaw);
  Vec3d goal_state(target_pose.x, target_pose.y, target_pose.yaw);
  return std::make_pair(start_state, goal_state);
  }
  // 将target_pose（局部坐标）转换到世界坐标,路径规划在世界坐标进行，碰撞检测中路径点世界坐标转换到pgm坐标，用于查询占据情况。
  double target_x_global = current_pose.x + target_pose.x * cos(current_pose.yaw) - target_pose.y * sin(current_pose.yaw);
  double target_y_global = current_pose.y + target_pose.x * sin(current_pose.yaw) + target_pose.y * cos(current_pose.yaw);
  double target_yaw_global = normalize_angle(current_pose.yaw + target_pose.yaw);

  Vec3d start_state(current_pose.x, current_pose.y, current_pose.yaw);

  Vec3d goal_state(target_x_global, target_y_global, target_yaw_global);

  return std::make_pair(start_state, goal_state);
}

std::vector<PosePoint> HybridAStarBridge::outputTransform(const VectorVec4d& path) const {
  PosePoint posepoint = PosePoint{};
    std::vector<PosePoint> trajectory;
    double v_max;
    for (size_t i = 0; i < path.size(); i++) {
        posepoint.x = path[i][0];
        posepoint.y = path[i][1];
        posepoint.yaw = path[i][2];
        if (path[i][3] == 1) {
          posepoint.gear = 3; //D
        }
        else {
          posepoint.gear = 1; //R
        }
        // posepoint.gear = path[i][3]; // 1前进 -1倒退
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
        if (i == N-1) {v_curve = 0.0;} // 终点速度置零
        else if (i == 0 ) {v_curve=0.1;} // 起点速度置0.3
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
    
    std::cerr<<"HybridAStarBridge.cpp:: "<<"trajectory size: "<<trajectory.size()<<std::endl;

    return trajectory;
}

bool HybridAStarBridge::loadMap(const std::string& pgm_file, double resolution, 
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

TJU_LOCAL_PLANNING_LIB_NAMESPACE_END
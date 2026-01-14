#pragma once
#include <vector>
#include <string>
#include <cstdint>

namespace tju_local_planning {
namespace cost_map {

struct PointXYZ { double x; double y; double z; };
using PointCloud = std::vector<PointXYZ>;

struct Pose { double x; double y; double z; double yaw; };

struct GridConfig {
    double resolution = 0.1;
    int width = 200;
    int height = 200;
    double origin_x = -10.0;
    double origin_y = -10.0;
    int log_odds_min = -128;
    int log_odds_max = 127;
    int log_odds_free = -1;
    int log_odds_occ = 3;
};

class OccupancyGrid {
public:
    OccupancyGrid();
    explicit OccupancyGrid(const GridConfig &cfg);
    bool init(const GridConfig &cfg);
    void update(const PointCloud &pc, const Pose &sensor_pose);
    void setFreeRaycast(bool enable);
    double queryCellProbability(int ix, int iy) const;
    bool isOccupiedWorld(double x, double y, double threshold) const;
    bool save(const std::string &path) const;
    bool load(const std::string &path);

private:
    int index(int ix, int iy) const;
    GridConfig cfg_;
    std::vector<int8_t> log_odds_;
    bool use_raycast_ = true;
};

} // namespace cost_map
} // namespace tju_local_planning
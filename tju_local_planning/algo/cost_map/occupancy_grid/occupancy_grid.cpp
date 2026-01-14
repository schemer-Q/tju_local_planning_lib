#include "occupancy_grid.h"
#include <fstream>
#include <algorithm>

namespace tju_local_planning {
namespace cost_map {

OccupancyGrid::OccupancyGrid() = default;

OccupancyGrid::OccupancyGrid(const GridConfig &cfg) {
    init(cfg);
}

bool OccupancyGrid::init(const GridConfig &cfg) {
    cfg_ = cfg;
    log_odds_.assign(static_cast<size_t>(cfg_.width) * cfg_.height, 0);
    return true;
}

void OccupancyGrid::update(const PointCloud &pc, const Pose &sensor_pose) {
    (void)pc; (void)sensor_pose;
}

void OccupancyGrid::setFreeRaycast(bool enable) {
    use_raycast_ = enable;
}

double OccupancyGrid::queryCellProbability(int ix, int iy) const {
    if (ix < 0 || iy < 0 || ix >= cfg_.width || iy >= cfg_.height) return 0.5;
    int idx = index(ix, iy);
    int8_t lo = log_odds_[idx];
    double odds = std::pow(2.0, static_cast<double>(lo) / 4.0);
    return odds / (1.0 + odds);
}

bool OccupancyGrid::isOccupiedWorld(double x, double y, double threshold) const {
    int ix = static_cast<int>((x - cfg_.origin_x) / cfg_.resolution);
    int iy = static_cast<int>((y - cfg_.origin_y) / cfg_.resolution);
    return queryCellProbability(ix, iy) >= threshold;
}

bool OccupancyGrid::save(const std::string &path) const {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) return false;
    ofs.write(reinterpret_cast<const char*>(&cfg_), sizeof(cfg_));
    ofs.write(reinterpret_cast<const char*>(log_odds_.data()), log_odds_.size());
    return ofs.good();
}

bool OccupancyGrid::load(const std::string &path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return false;
    GridConfig cfg;
    ifs.read(reinterpret_cast<char*>(&cfg), sizeof(cfg));
    if (!ifs) return false;
    cfg_ = cfg;
    size_t n = static_cast<size_t>(cfg_.width) * cfg_.height;
    log_odds_.resize(n);
    ifs.read(reinterpret_cast<char*>(log_odds_.data()), n);
    return ifs.good();
}

int OccupancyGrid::index(int ix, int iy) const {
    return iy * cfg_.width + ix;
}

} // namespace cost_map
} // namespace tju_local_planning
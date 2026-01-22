# occupancy_grid 单元测试（ROS 节点）

说明：该测试是一个简单的 ROS 节点，使用本仓库中的 `OccupancyGrid` 实现将合成点云转换为占据栅格并发布为 `nav_msgs/OccupancyGrid`，可在 RViz 中查看。

构建与运行：

1. 确保你有 ROS（ROS1）环境并在 catkin 工作空间中。将本仓库的顶层 `tju_local_planning` 目录作为 catkin 工作空间的 `src` 子目录，或把 `test_occupancy_grid` 包复制到你的 ROS workspace 的 `src`。

2. 在工作空间根目录运行：
```bash
catkin_make
source devel/setup.bash
rosrun test_occupancy_grid occupancy_grid_test_node
```

3. 在 RViz 中添加 `Map` 或 `OccupancyGrid` 话题 `/test/occupancy_grid`，选择 `map` 作为 Fixed Frame。

注意：该测试节点通过直接包含仓库内头文件来复用 `OccupancyGrid`，CMakeLists 已将仓库根目录加入 include 路径。

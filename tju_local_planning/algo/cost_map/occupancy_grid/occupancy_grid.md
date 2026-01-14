# 栅格占据图（Occupancy Grid）实现笔记

概述
- 基于二维栅格（x,y 平面）记录每个格子被占据的概率，使用 log‑odds 表示并用逆传感器模型或射线投射（raycast）更新。

优先级：高
实现难度：低

关键点
- 数据预处理：里程计/IMU 去畸变、点云下采样（voxel filter）、地面分割（可选）。
- 传感器模型：
  - 使用 log‑odds 增量更新：l_t = l_{t-1} + inverse_sensor(z) - l_0。
  - 对于激光/雷达，建议采用射线投射更新空闲/占据；对稀疏点云可只做占据更新。
- 分辨率与范围：格子大小（0.05–0.5 m）由车辆最小避障精度和计算预算决定。
- 存储与窗口策略：支持全局栅格（内存足够时）或固定大小局部窗口（随机器人位姿移动）。

接口建议
- `void init(GridConfig cfg)` — 分辨率、尺寸、坐标系、log‑odds 初值
- `void update(const PointCloud &pc, const Pose &sensor_pose)` — 增量更新（支持按帧或按扫描线）
- `void setFreeRaycast(bool enable)` — 是否启用射线清扫空闲区域
- `Probability queryCell(int ix, int iy)`、`bool isOccupied(x,y,threshold)`
- 支持序列化：`save(file)` / `load(file)` 用于调试与回放

性能提示
- 使用整型 log‑odds（量化）加速更新；对局部窗口采用循环缓冲以避免全域移动成本。
- 在多线程环境中，将点云分块并批量更新格子以减少锁竞争。

测试建议
- 单元测试：输入简单点云（单点、直线、矩形障碍），验证占据/空闲更新与阈值判断。
- 集成测试：与上游仿真或回放数据（bag/pcd）验证实时性与误报率。

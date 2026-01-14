# 高度图 / 高度栅格（Elevation Grid）实现笔记

概述
- 每个平面格子记录该单元格内点云的高度统计（min/mean/max）、点数及粗糙度，用于地面可遍历性评估。

优先级：中
实现难度：中

关键点
- 数据预处理：先做地面分割（RANSAC 或基于法线），或采用传感器高度参考去除离群点；下采样以减少计算。
- 格子统计：每格维护 `count, z_min, z_max, z_mean, roughness`；roughness 可用高度方差或法线方差衡量。
- 可遍历性判定：基于 `z_max - z_min`（台阶阈值）、坡度（局部法线）和粗糙度生成代价分数。

接口建议
- `void update(const PointCloud &pc, const Pose &sensor_pose)` — 增量统计更新
- `Traversability getTraversability(x,y)` — 返回可通过/不可通过/谨慎与分数
- `void setCellSize(double meters)`、`void setSlopeThreshold(double)`

性能提示
- 对于大范围场景使用局部窗口策略；统计累加用原子/批量合并减少锁。
- 可把不可遍历区域作为额外占据层与 `OccupancyGrid` 融合。

测试建议
- 使用仿真地形（台阶、坡道、碎石）验证可遍历性分数与阈值。

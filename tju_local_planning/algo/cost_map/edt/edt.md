# 欧式距离变换（EDT）实现笔记

概述
- 在二值占据图或阈值化的栅格上计算至最近障碍的欧式距离，为规划器生成安全/代价场。

优先级：中
实现难度：低

关键点
- 前置依赖：需要可靠的 `OccupancyGrid` 作为输入（占据/空闲二值化）。
- 算法选择：可用经典的二维 EDT（O(n) 的 Danielsson 或 Felzenszwalb 算法）或快速近似（局部卷积/距离传播）。
- 时变策略：对高频场景仅在局部窗口或在障碍发生变化时重算 EDT，避免每帧全域重算。

接口建议
- `void computeFrom(const OccupancyGrid &grid)` — 全图/局部计算
- `double queryDistance(x,y)`、`double getCost(x,y)` — 距离和代价（结合膨胀半径和惩罚函数）
- 支持参数：`inflation_radius`, `max_cost`, `distance_weight`。

代价生成示例
- 常用代价函数：cost = exp(-alpha * distance) 或 piecewise linear（距离小于阈值为极高代价）。

性能提示
- 使用整型/平方距离避免开方开销；对整数栅格可用查表映射到代价值。
- 在并行更新中把影响区域限制到障碍变化的 bounding box。

测试建议
- 在已知占据图上验证距离值（例如单点障碍），并验证代价函数随距离单调变化。

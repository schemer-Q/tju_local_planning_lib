# TrunkPerception

[![PIPELINE](http://git-rd.trunk.tech/wangxu/trunk_perception_lib/badges/master/pipeline.svg)](http://git-rd.trunk.tech/wangxu/trunk_perception_lib)  [![代码覆盖率](https://img.shields.io/codecov/c/github/username/TrunkPerception.svg)](http://git-rd.trunk.tech/wangxu/trunk_perception_lib)

`TrunkPerception`负责为`L2` `L4`项目及`港口`(计划)自动驾驶系统提供感知算法实现，包括物体检测跟踪功能、车道线检测跟踪功能等。

## 文档

[TrunkPerception 说明文档](http://10.11.3.9:8888/pplib/html/index.html)

## 使用示例

- [高速L2/L4工程TAD](http://git-rd.trunk.tech/trunkpilot/tad_soc_release/-/tree/feat-wangxu-refact_per/src/per)

## 开发进度

### V0.1.0版本

- [x] 传感器抽象/数据管理/功能任务抽象/故障码管理
- [x] 图像预处理功能
- [x] 激光预处理功能
- [x] 激光物体检测功能[DSVT]
- [x] 激光物体跟踪功能
- [x] 视觉车道线检测功能[BEVLaneDet]
- [x] 视觉车道线跟踪功能
- [x] CI[编译&发版]

### V0.2.0版本

- [ ] 前向视觉物体检测[DETR3D]
- [ ] 前向物体检测后融合[Lidar+Camera+Radar]

### V0.3.0版本

- [ ] 环视视觉物体检测[Sparse4D]

### BackLog

- [ ] ...

## 使用

### 依赖

|          依赖库          |             版本             |
| :----------------------: | :--------------------------: |
|          Eigen           |              3               |
|          gtest           |              -               |
|          gflags          |              -               |
|          opencv          |     4.5@Orin / 3@Others      |
|           pcl            |             1.8+             |
|           yaml           |             0.6+             |
|           jian           | [config.json](./config.json) |
|       lidarnetsdk        | [config.json](./config.json) |
| sdkbevlaneconedethighway | [config.json](./config.json) |

### 编译

```shell
mkdir build && cd build
# sudo dpkg -r trunk_perception # 如果本地安装了，可以先卸载，防止编译时引用错误的头文件
cmake -DCMAKE_INSTALL_PREFIX=/opt/trunk_master -DCMAKE_BUILD_TYPE=Release ..
make package
sudo dpkg -i packages/{xxx.deb}
```

### 引用

添加环境变量

```shell
export LD_LIBRARY_PATH=/opt/trunk_master/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/opt/trunk_master:$CMAKE_PREFIX_PATH
```


```cmake
find_package(trunk_perception 0.0.1 REQUIRED trunk_perception)
include_directories(${TRUNK_PERCEPTION_INCLUDE_DIRS})
link_directories(${TRUNK_PERCEPTION_LIBRARY_DIRS})
message(STATUS "TRUNK_PERCEPTION_INCLUDE_DIRS ${TRUNK_PERCEPTION_INCLUDE_DIRS}")
message(STATUS "TRUNK_PERCEPTION_LIBRARY_DIRS ${TRUNK_PERCEPTION_LIBRARY_DIRS}")
message(STATUS "TRUNK_PERCEPTION_INSTALL_PATH ${TRUNK_PERCEPTION_INSTALL_PATH}")
message(STATUS "TRUNK_PERCEPTION_LIBRARIES ${TRUNK_PERCEPTION_LIBRARIES}")
```


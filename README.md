# SuntaePerception

[![PIPELINE](http://git-rd.suntae.tech/txs/tju_local_planning_lib/badges/master/pipeline.svg)](http://git-rd.suntae.tech/txs/tju_local_planning_lib)  [![代码覆盖率](https://img.shields.io/codecov/c/github/username/SuntaePerception.svg)](http://git-rd.suntae.tech/txs/tju_local_planning_lib)

`SuntaePerception`负责为自动驾驶系统提供感知算法实现，包括物体检测跟踪功能、车道线检测跟踪功能等。

## 文档

[SuntaePerception 说明文档](http://10.11.3.9:8888/pplib/html/index.html)

## 使用示例

- [工程TAD](http://git-rd.SUNTAE.tech/suntaepilot/tad_soc_release/-/tree/release/src/per)

## 开发进度

### V0.1.0版本

- [x] 传感器抽象/数据管理/功能任务抽象/故障码管理
- [x] 图像预处理功能
- [x] 激光预处理功能
- [x] 视觉车道线跟踪功能
- [x] CI[编译&发版]

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
|           jian           | [config.json](./config.json) ||

### 编译

```shell
mkdir build && cd build
# sudo dpkg -r tju_local_planning # 如果本地安装了，可以先卸载，防止编译时引用错误的头文件
cmake -DCMAKE_INSTALL_PREFIX=/opt/tju_master -DCMAKE_BUILD_TYPE=Release ..
make package
sudo dpkg -i packages/{xxx.deb}
```

### 引用

添加环境变量

```shell
export LD_LIBRARY_PATH=/opt/tju_master/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/opt/tju_master:$CMAKE_PREFIX_PATH
```


```cmake
find_package(tju_local_planning 0.0.1 REQUIRED tju_local_planning)
include_directories(${TJU_LOCAL_PLANNING_INCLUDE_DIRS})
link_directories(${TJU_LOCAL_PLANNING_LIBRARY_DIRS})
message(STATUS "TJU_LOCAL_PLANNING_INCLUDE_DIRS ${TJU_LOCAL_PLANNING_INCLUDE_DIRS}")
message(STATUS "TJU_LOCAL_PLANNING_LIBRARY_DIRS ${TJU_LOCAL_PLANNING_LIBRARY_DIRS}")
message(STATUS "TJU_LOCAL_PLANNING_INSTALL_PATH ${TJU_LOCAL_PLANNING_INSTALL_PATH}")
message(STATUS "TJU_LOCAL_PLANNING_LIBRARIES ${TJU_LOCAL_PLANNING_LIBRARIES}")
```

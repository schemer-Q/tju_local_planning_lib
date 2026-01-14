# 夹抱车局部路径规划库

[![PIPELINE](http://git-rd.suntae.tech/txs/tju_local_planning_lib/badges/master/pipeline.svg)](http://git-rd.suntae.tech/txs/tju_local_planning_lib)  [![代码覆盖率](https://img.shields.io/codecov/c/github/username/SuntaePerception.svg)](http://git-rd.suntae.tech/txs/tju_local_planning_lib)

`tju_local_planning` 是一个面向夹抱车（夹抱/叉装车）局部路径规划的算法包，旨在为狭窄场景下的运输车辆提供可靠的局部路径规划与避障能力。主要功能包括：

1. 可完成狭窄通道安全路径规划。
2. 可处理决策层与执行层的中间通信。
3. 可根据感知算法结果绘制动态地图。
4. 可根据车辆模型和外形规划避障路径。


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

# 基于 ROS2 的自动驾驶决策规划算法实战开发

这是一个基于 **ROS2 Humble** 的自动驾驶决策规划学习项目，代码组织为标准 ROS2 工作空间结构。项目围绕 PNC 地图、全局路径、参考线、局部路径、速度规划、轨迹合成、车辆模型展示和数据绘图等模块展开，适合用于学习 ROS2 下自动驾驶规划系统的工程组织方式。

当前仓库主要包含：

- 自定义消息与服务接口：`base_msgs`
- C++ 决策规划主模块：`planning`
- Python 数据绘图节点：`data_plot`
- URDF/Xacro 车辆模型与 RViz 可视化配置
- 一键依赖安装脚本和 apt 软件源修复脚本

## 项目结构

```text
.
├── src
│   ├── base_msgs              # 自定义 msg/srv 接口
│   ├── data_plot              # Python 绘图节点
│   └── planning               # 决策规划核心包
│       ├── config             # 规划场景与参数配置
│       ├── launch             # ROS2 启动文件
│       ├── rviz               # RViz 配置
│       ├── urdf               # 主车和障碍车模型
│       └── src
│           ├── common         # 配置读取、数学工具
│           ├── decision_center
│           ├── global_planner
│           ├── local_planner
│           ├── move_cmd
│           ├── planning_process
│           ├── pnc_map_creator
│           ├── reference_line
│           └── vehicle_info
├── install_autocar_deps.sh    # Ubuntu 22.04 + ROS2 Humble 依赖安装脚本
└── fix_apt_sources.sh         # apt 软件源修复脚本
```

## 环境要求

推荐环境：

- Ubuntu 22.04
- ROS2 Humble
- CMake 3.22 或更高
- GCC/G++、Make、Git、curl
- colcon
- Eigen 3.4.0
- yaml-cpp
- OSQP v1.0.0
- osqp-eigen v0.10.0
- Python3、matplotlib、transforms3d
- RViz2、xacro、robot_state_publisher、joint_state_publisher

## 安装依赖

仓库提供了依赖安装脚本：

```bash
cd /home/mason/AutoCar
./install_autocar_deps.sh
```

脚本会安装 ROS2 Humble、CMake、colcon、Eigen、yaml-cpp、OSQP、osqp-eigen、RViz 相关工具和 Python 绘图库。执行时需要输入本机 sudo 密码。

如果 apt 软件源较慢或异常，可以先执行：

```bash
sudo bash /home/mason/AutoCar/fix_apt_sources.sh
```

## 编译

```bash
cd /home/mason/AutoCar
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

建议把 ROS2 环境加入 `~/.bashrc`：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

如果已经编译过工作空间，也可以加入：

```bash
echo "source /home/mason/AutoCar/install/setup.bash" >> ~/.bashrc
```

## 运行

启动规划演示：

```bash
source /opt/ros/humble/setup.bash
source /home/mason/AutoCar/install/setup.bash
ros2 launch planning planning_launch.py
```

该 launch 文件会启动：

- 主车 URDF/Xacro 模型
- `robot_state_publisher`
- `joint_state_publisher`
- RViz2
- PNC 地图服务节点
- 全局路径服务节点
- 规划主流程节点

## 核心节点

| 节点 | 可执行文件 | 说明 |
|---|---|---|
| PNC 地图服务 | `pnc_map_server` | 根据配置生成 PNC 地图，并发布 RViz 可视化数据 |
| 全局路径服务 | `global_path_server` | 接收地图请求，生成全局路径 |
| 规划主流程 | `planning_process` | 串联地图请求、全局路径请求和后续规划流程 |
| 主车运动命令 | `car_move_cmd` | 主车运动控制命令节点 |
| 障碍车运动命令 | `obs_move_cmd` | 障碍车运动控制命令节点 |
| 数据绘图 | `data_plot` | Python 绘图节点 |

## 话题与服务

主要服务：

| 服务 | 类型 | 说明 |
|---|---|---|
| `pnc_map_server` | `base_msgs/srv/PNCMapService` | 请求指定类型的 PNC 地图 |
| `global_path_server` | `base_msgs/srv/GlobalPathService` | 请求指定类型的全局路径 |

主要话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `pnc_map` | `base_msgs/msg/PNCMap` | PNC 地图数据 |
| `pnc_map_markerarray` | `visualization_msgs/msg/MarkerArray` | 地图 RViz 可视化 |
| `global_path` | `nav_msgs/msg/Path` | 全局路径 |
| `global_path_rviz` | `visualization_msgs/msg/Marker` | 全局路径 RViz 可视化 |

## 自定义接口

`base_msgs` 中定义了规划相关的消息和服务：

- `PNCMap.msg`：道路边界、中心线、道路长度、车道宽度
- `Referline.msg` / `ReferlinePoint.msg`：参考线
- `LocalPath.msg` / `LocalPathPoint.msg`：局部路径
- `LocalSpeeds.msg` / `LocalSpeedsPoint.msg`：速度规划结果
- `LocalTrajectory.msg` / `LocalTrajectoryPoint.msg`：轨迹结果
- `ObsInfo.msg`：障碍物信息
- `PlotInfo.msg`：绘图信息
- `PNCMapService.srv`：地图请求服务
- `GlobalPathService.srv`：全局路径请求服务

查看接口：

```bash
ros2 interface show base_msgs/msg/PNCMap
ros2 interface show base_msgs/srv/PNCMapService
```

## 配置文件

规划参数位于：

```text
src/planning/config/planning_static_obs_config.yaml
src/planning/config/planning_dynamic_obs_config.yaml
```

主要配置项包括：

- 主车和障碍车尺寸、初始位姿、初速度
- PNC 地图类型、道路长度、车道半宽、分段长度、限速
- 全局路径规划类型
- 局部路径曲线类型和路径点数量
- 速度规划点数量
- 参考线前后采样范围
- 决策安全距离
- 障碍物检测范围

地图类型：

```text
0: straight
1: sturn
```

局部路径曲线类型：

```text
0: 一次曲线
1: 三次曲线
2: 五次曲线
```

## 开发状态

当前版本偏向学习和工程框架搭建，已经具备 ROS2 包结构、自定义接口、节点拆分、launch 启动、配置读取、URDF/RViz 展示和基础服务调用流程。

部分规划算法仍处于开发阶段，例如：

- 全局路径搜索逻辑
- 参考线生成与平滑
- 局部路径规划与平滑
- 速度规划与平滑
- 决策中心
- 轨迹合成
- 数据绘图订阅与可视化

## 常用命令

查看节点：

```bash
ros2 node list
```

查看话题：

```bash
ros2 topic list
```

查看服务：

```bash
ros2 service list
```

查看 TF：

```bash
ros2 run tf2_tools view_frames
```

单独运行 turtlesim 测试 ROS2 环境：

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

## 参考资料

本项目 README 内容参考了仓库代码结构、安装脚本，以及本地资料《【保姆级】基于ROS2的自动驾驶决策规划算法实战开发.pdf》的环境安装和项目组织思路。

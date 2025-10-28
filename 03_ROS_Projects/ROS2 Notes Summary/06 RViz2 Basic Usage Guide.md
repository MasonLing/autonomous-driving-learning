# RViz2 基础使用指南

## 基本概念

### 什么是RViz2？
```bash
- ROS2中的3D可视化工具
- 用于显示传感器数据、机器人模型、路径规划等
- 支持插件系统，可扩展功能
- 基于Qt和OpenGL开发
```

### 主要功能
```
1. 机器人模型显示
2. 传感器数据可视化
3. 地图显示
4. 路径规划和导航可视化
5. 坐标变换(TF)显示
6. 标记(Marker)显示
```

## 安装和启动

### 安装RViz2
```bash
# 在Ubuntu中安装
sudo apt install ros-humble-rviz2

# 或安装桌面完整版（包含RViz2）
sudo apt install ros-humble-desktop
```

### 启动RViz2
```bash
# 基本启动
ros2 run rviz2 rviz2

# 启动时加载特定配置
ros2 run rviz2 rviz2 -d <config_file>

# 禁用OpenGL（如果遇到图形问题）
ros2 run rviz2 rviz2 --disable-gl
```

## 界面概览

### 主界面布局
```
┌─────────────────────────────────────────┐
│ 菜单栏 (File, View, Panels, Help)        │
├─────────────┬───────────────────────────┤
│             │                           │
│  左侧面板   │        3D视图区             │
│  (Displays) │      (主要显示区域)         │
│             │                           │
├─────────────┼───────────────────────────┤
│             │      右侧面板              │
│  时间面板   │     (Views, Tools)         │
│             │                           │
└─────────────┴───────────────────────────┘
```

## 基本配置和显示

### 1. 设置固定坐标系(Fixed Frame)
```bash
# 重要：必须设置正确的固定坐标系
- 通常在 "Global Options" → "Fixed Frame" 中设置
- 常见坐标系: "map", "odom", "base_link"
- 必须与TF树中的坐标系匹配
```

### 2. 添加显示类型(Displays)

#### 添加机器人模型
```
1. 点击左侧面板底部的 "Add" 按钮
2. 选择 "RobotModel"
3. 在 "Robot Description" 参数中填写:
   - 对于ROS2: "robot_description"
4. 确保URDF文件正确加载
```

#### 添加TF显示
```
1. 添加 "TF" 显示类型
2. 查看坐标变换树
3. 调试TF变换问题
```

#### 添加激光雷达数据
```
1. 添加 "LaserScan" 显示类型
2. 设置Topic: "/scan"
3. 调整颜色和大小
```

#### 添加点云数据
```
1. 添加 "PointCloud2" 显示类型
2. 设置Topic: "/points"
3. 配置样式和衰减时间
```

#### 添加地图
```
1. 添加 "Map" 显示类型
2. 设置Topic: "/map"
3. 设置颜色方案
```

#### 添加路径
```
1. 添加 "Path" 显示类型
2. 设置Topic: "/path"
3. 配置线条颜色和宽度
```

## 实际示例：可视化机器人

### 创建示例发布者
```cpp
// 在 ~/ros2_ws/src 创建新包
ros2 pkg create rviz_demo --build-type ament_cmake --dependencies rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs

// 创建示例节点：src/robot_publisher.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class RobotPublisher : public rclcpp::Node
{
public:
    RobotPublisher() : Node("robot_publisher"), count_(0)
    {
        // 创建激光雷达数据发布者
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        
        // 创建里程计发布者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // 创建TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // 创建定时器
        timer_ = this->create_wall_timer(100ms, std::bind(&RobotPublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "机器人数据发布器已启动");
    }

private:
    void timer_callback()
    {
        publish_laser_scan();
        publish_odometry();
        publish_tf();
        count_++;
    }
    
    void publish_laser_scan()
    {
        auto scan = sensor_msgs::msg::LaserScan();
        scan.header.stamp = this->now();
        scan.header.frame_id = "laser_frame";
        
        scan.angle_min = -M_PI/2;
        scan.angle_max = M_PI/2;
        scan.angle_increment = M_PI/180;
        scan.time_increment = 0.0;
        scan.scan_time = 0.1;
        scan.range_min = 0.1;
        scan.range_max = 10.0;
        
        // 生成模拟激光数据
        int num_readings = (scan.angle_max - scan.angle_min) / scan.angle_increment;
        scan.ranges.resize(num_readings);
        
        for(int i = 0; i < num_readings; ++i) {
            // 模拟墙壁在2米处
            double angle = scan.angle_min + i * scan.angle_increment;
            scan.ranges[i] = 2.0 / cos(angle);
        }
        
        laser_pub_->publish(scan);
    }
    
    void publish_odometry()
    {
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        
        // 模拟机器人运动
        double t = count_ * 0.1;
        odom.pose.pose.position.x = sin(t * 0.1);
        odom.pose.pose.position.y = cos(t * 0.1);
        odom.pose.pose.orientation.z = sin(t * 0.05);
        odom.pose.pose.orientation.w = cos(t * 0.05);
        
        odom_pub_->publish(odom);
    }
    
    void publish_tf()
    {
        // 发布 base_link 到 laser_frame 的变换
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "laser_frame";
        
        transform.transform.translation.x = 0.1;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.2;
        
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        tf_broadcaster_->sendTransform(transform);
        
        // 发布 odom 到 base_link 的变换
        geometry_msgs::msg::TransformStamped odom_transform;
        odom_transform.header.stamp = this->now();
        odom_transform.header.frame_id = "odom";
        odom_transform.child_frame_id = "base_link";
        
        double t = count_ * 0.1;
        odom_transform.transform.translation.x = sin(t * 0.1);
        odom_transform.transform.translation.y = cos(t * 0.1);
        
        odom_transform.transform.rotation.z = sin(t * 0.05);
        odom_transform.transform.rotation.w = cos(t * 0.05);
        
        tf_broadcaster_->sendTransform(odom_transform);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 编译和运行示例
```bash
# 编译包
cd ~/ros2_ws
colcon build --packages-select rviz_demo
source install/setup.bash

# 运行示例节点
ros2 run rviz_demo robot_publisher
```

## RViz2 详细配置步骤

### 步骤1：启动RViz2并配置固定坐标系
```bash
# 新终端
ros2 run rviz2 rviz2
```

1. 在左侧 "Global Options" 中设置:
   - Fixed Frame: "odom"

### 步骤2：添加TF显示
1. 点击 "Add" 按钮
2. 选择 "TF"
3. 展开TF显示，查看坐标框架树

### 步骤3：添加激光雷达显示
1. 点击 "Add" 按钮
2. 选择 "LaserScan"
3. 配置参数:
   - Topic: "/scan"
   - Size: 0.05
   - Style: "Points"

### 步骤4：添加机器人模型（可选）
1. 点击 "Add" 按钮
2. 选择 "RobotModel"
3. 设置 Robot Description: "robot_description"

### 步骤5：添加里程计显示
1. 点击 "Add" 按钮
2. 选择 "Odometry"
3. 配置参数:
   - Topic: "/odom"
   - Color: 选择喜欢的颜色
   - Shape: "Axes" 或 "Arrow"

## 常用显示类型详解

### 1. Marker 显示
```
用途: 显示自定义几何形状
常见应用:
  - 显示检测框
  - 显示文本标签
  - 显示箭头和线条
配置:
  - Topic: "/visualization_marker"
  - Namespace: 标记命名空间
```

### 2. PointCloud2 显示
```
用途: 显示3D点云数据
配置:
  - Topic: "/points"
  - Style: Points, Squares, Flat Squares
  - Size: 点的大小
  - Decay Time: 点云衰减时间
```

### 3. Map 显示
```
用途: 显示2D占据栅格地图
配置:
  - Topic: "/map"
  - Color Scheme: map, costmap, raw
  - Alpha: 透明度
```

### 4. Path 显示
```
用途: 显示路径轨迹
配置:
  - Topic: "/path"
  - Color: 路径颜色
  - Line Style: Lines, Points
```

## 视图控制

### 视图类型
```bash
# 正交视图 (Orbit)
- 围绕目标点旋转
- 适合观察机器人整体

# 第一人称视图 (FPS)
- 类似游戏中的第一人称
- 适合沉浸式观察

# 俯视图 (Top-Down)
- 从上往下看
- 适合2D导航调试

# XY模式 (XY Orbit)
- 在XY平面内旋转
```

### 视图控制技巧
```
鼠标操作:
  - 左键拖拽: 旋转视图
  - 中键拖拽: 平移视图
  - 右键拖拽/滚轮: 缩放视图

快捷键:
  - Shift: 配合鼠标进行精细控制
  - F: 聚焦到选定目标
  - Z: 重置视图
```

## 工具使用

### 内置工具
```
1. 移动相机 (Move Camera)
2. 选择 (Select)
3. 2D位姿估计 (2D Pose Estimate)
4. 2D导航目标 (2D Nav Goal)
5. 发布点 (Publish Point)
6. 测量 (Measure)
```

### 2D位姿估计工具
```
用途: 设置机器人初始位置
步骤:
  1. 点击 "2D Pose Estimate" 工具
  2. 在地图上点击并拖拽设置位置和方向
  3. 机器人会更新到该位置
```

### 2D导航目标工具
```
用途: 设置导航目标点
步骤:
  1. 点击 "2D Nav Goal" 工具
  2. 在地图上点击并拖拽设置目标位置和方向
  3. 导航系统会规划路径
```

## 配置文件管理

### 保存配置
```
1. 点击 File → Save Config As
2. 选择保存位置，通常为 ~/.rviz2/default.rviz
3. 配置会保存所有显示设置和视图设置
```

### 加载配置
```
1. 点击 File → Open Config
2. 选择之前保存的 .rviz 文件
3. 或使用命令行: rviz2 -d config_file.rviz
```

### 常用配置文件位置
```bash
# 默认配置位置
~/.rviz2/default.rviz

# 包内配置示例
ros2 pkg prefix rviz2
# 然后查看: share/rviz2/default.rviz
```

## 调试技巧

### 常见问题解决

#### 1. 看不到任何数据
```bash
# 检查话题是否发布
ros2 topic list
ros2 topic echo /scan

# 检查TF树
ros2 run tf2_tools view_frames.py
```

#### 2. 坐标系错误
```bash
# 检查TF变换
ros2 run tf2_ros tf2_echo base_link laser_link

# 查看所有坐标系
ros2 run tf2_tools view_frames.py
```

#### 3. 显示异常
```bash
# 重置RViz2配置
rm ~/.rviz2/default.rviz

# 使用软件渲染（如果OpenGL有问题）
ros2 run rviz2 rviz2 --disable-gl
```

### 性能优化
```
1. 减少显示数量
2. 增加衰减时间减少渲染负载
3. 使用较低的点云分辨率
4. 关闭不需要的显示类型
```

## 实用工作流程

### 导航调试工作流
```bash
# 终端1: 启动机器人
ros2 launch my_robot launch_file.launch.py

# 终端2: 启动导航
ros2 launch nav2_bringup navigation_launch.py

# 终端3: 启动RViz2
ros2 run rviz2 rviz2 -d `ros2 pkg prefix nav2_bringup`/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### SLAM调试工作流
```bash
# 终端1: 启动机器人
ros2 launch my_robot slam_launch.py

# 终端2: 启动SLAM工具箱
ros2 launch slam_toolbox online_async_launch.py

# 终端3: 启动RViz2
ros2 run rviz2 rviz2 -d `ros2 pkg prefix slam_toolbox`/share/slam_toolbox/online_sync.rviz
```

## 插件开发（高级）

### 创建自定义RViz2显示插件
```cpp
// 基本插件结构
#include <rviz_common/display_context.hpp>
#include <rviz_common/display.hpp>

class MyDisplay : public rviz_common::Display
{
public:
    MyDisplay();
    ~MyDisplay() override;
    
    void onInitialize() override;
    void update(float dt, float ros_dt) override;
    
private:
    // 插件实现
};
```

### 插件编译配置
```cmake
# 在CMakeLists.txt中添加
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

# 创建插件库
rviz_common_plugin_library(
    my_rviz_plugins
    src/my_display.cpp
)
```

## 常用快捷键汇总

```bash
# 视图控制
G: 切换网格显示
R: 重置视图
F: 聚焦到选定对象
Ctrl+R: 录制视频

# 工具选择
M: 移动相机工具
S: 选择工具
P: 发布点工具

# 显示控制
Space: 暂停/继续数据
Delete: 删除选定显示
```

RViz2是ROS2生态中最重要的可视化工具，熟练掌握其使用可以大大提高机器人开发和调试的效率。建议在实际项目中多加练习，熟悉各种显示类型的配置和调试方法。
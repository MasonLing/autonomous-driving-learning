# ROS2 自定义消息类型知识整理

## 基本概念

### 什么是自定义消息？
```cpp
- ROS2中用户根据特定需求定义的数据结构
- 扩展标准消息类型，适应具体应用场景
- 支持复杂的数据结构和嵌套类型
- 需要编译生成对应的C++/Python代码
```

### 自定义消息的优势
```
1. 领域特定: 针对具体应用设计数据结构
2. 类型安全: 编译时检查数据类型
3. 代码生成: 自动生成序列化/反序列化代码
4. 工具集成: 与ROS2工具链完美集成
```

## 创建自定义消息包

### 1. 创建专用包
```bash
# 在工作空间src目录下
cd ~/ros2_ws/src

# 创建消息包，注意命名通常以_msgs结尾
ros2 pkg create custom_msgs --build-type ament_cmake
```

### 2. 目录结构
```
custom_msgs/
├── CMakeLists.txt
├── include/
├── msg/           # 存放自定义消息定义文件
├── package.xml
└── src/
```

## 自定义消息定义

### 基本消息类型

#### 创建简单消息 `msg/SimpleMessage.msg`
```
# 简单自定义消息示例
string name
int32 id
float64 value
bool status
```

#### 创建复杂消息 `msg/ComplexMessage.msg`
```
# 复杂自定义消息示例
string header_frame_id
time timestamp
int32[] data_array
float64[3] position
SimpleMessage basic_info  # 嵌套其他自定义消息
```

#### 创建专业消息 `msg/RobotStatus.msg`
```
# 机器人状态消息
string robot_name
uint8 mode              # 0:空闲, 1:运行, 2:错误
float32 battery_level
float32[6] joint_angles
geometry_msgs/Point current_position  # 引用标准消息
```

## 包配置

### package.xml 配置
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypelocation="http://www.ros.org/schema/package_format3.xsd"?>
<package format="3">
  <name>custom_msgs</name>
  <version>0.0.0</version>
  <description>Custom message definitions</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 消息生成依赖 -->
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  
  <!-- 如果使用了其他消息类型 -->
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt 配置
```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SimpleMessage.msg"
  "msg/ComplexMessage.msg"
  "msg/RobotStatus.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# 安装消息文件
install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(std_msgs)

ament_package()
```

## 在发布者中使用自定义消息

### 修改发布者代码
```cpp
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/robot_status.hpp"  // 包含自定义消息头文件
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class CustomPublisher : public rclcpp::Node
{
public:
    CustomPublisher() : Node("custom_publisher"), battery_level_(100.0)
    {
        // 创建使用自定义消息的发布者
        publisher_ = this->create_publisher<custom_msgs::msg::RobotStatus>(
            "robot_status", 
            10
        );
        
        timer_ = this->create_wall_timer(
            1000ms, 
            std::bind(&CustomPublisher::timer_callback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "自定义消息发布者已启动");
    }

private:
    void timer_callback()
    {
        // 创建自定义消息对象
        auto message = custom_msgs::msg::RobotStatus();
        
        // 设置消息字段
        message.robot_name = "Robot_001";
        message.mode = 1;  // 运行模式
        message.battery_level = battery_level_;
        
        // 设置数组字段
        message.joint_angles = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
        
        // 设置嵌套的标准消息字段
        message.current_position.x = 1.0;
        message.current_position.y = 2.0;
        message.current_position.z = 0.5;
        
        // 发布消息
        publisher_->publish(message);
        
        // 模拟电池消耗
        battery_level_ -= 0.5;
        if (battery_level_ < 0) battery_level_ = 100.0;
        
        RCLCPP_INFO(this->get_logger(), 
                   "发布机器人状态: %s, 电池: %.1f%%, 位置: (%.1f, %.1f, %.1f)",
                   message.robot_name.c_str(),
                   message.battery_level,
                   message.current_position.x,
                   message.current_position.y,
                   message.current_position.z);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_msgs::msg::RobotStatus>::SharedPtr publisher_;
    float battery_level_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## 在订阅者中使用自定义消息

### 修改订阅者代码
```cpp
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/robot_status.hpp"

using std::placeholders::_1;

class CustomSubscriber : public rclcpp::Node
{
public:
    CustomSubscriber() : Node("custom_subscriber")
    {
        // 创建使用自定义消息的订阅者
        subscriber_ = this->create_subscription<custom_msgs::msg::RobotStatus>(
            "robot_status", 
            10, 
            std::bind(&CustomSubscriber::robot_status_callback, this, _1)
        );
        
        RCLCPP_INFO(this->get_logger(), "自定义消息订阅者已启动");
    }

private:
    void robot_status_callback(const custom_msgs::msg::RobotStatus::SharedPtr msg)
    {
        // 处理自定义消息
        std::string mode_str;
        switch(msg->mode) {
            case 0: mode_str = "空闲"; break;
            case 1: mode_str = "运行"; break;
            case 2: mode_str = "错误"; break;
            default: mode_str = "未知";
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "收到机器人状态:\n"
                   "  名称: %s\n"
                   "  模式: %s\n"
                   "  电池: %.1f%%\n"
                   "  关节角度: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n"
                   "  当前位置: (%.1f, %.1f, %.1f)",
                   msg->robot_name.c_str(),
                   mode_str.c_str(),
                   msg->battery_level,
                   msg->joint_angles[0], msg->joint_angles[1], msg->joint_angles[2],
                   msg->joint_angles[3], msg->joint_angles[4], msg->joint_angles[5],
                   msg->current_position.x, msg->current_position.y, msg->current_position.z);
    }
    
    rclcpp::Subscription<custom_msgs::msg::RobotStatus>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## 编译和运行

### 1. 编译自定义消息包
```bash
cd ~/ros2_ws
colcon build --packages-select custom_msgs
source install/setup.bash
```

### 2. 修改使用包的配置

在使用自定义消息的包中，修改`package.xml`：
```xml
<depend>custom_msgs</depend>
```

修改`CMakeLists.txt`：
```cmake
find_package(custom_msgs REQUIRED)
ament_target_dependencies(your_node custom_msgs)
```

### 3. 编译和运行
```bash
# 编译所有包
cd ~/ros2_ws
colcon build

# 运行示例
source install/setup.bash

# 终端1 - 发布者
ros2 run your_package custom_publisher_node

# 终端2 - 订阅者
ros2 run your_package custom_subscriber_node
```

## 消息字段类型详解

### 基本数据类型
```
bool
int8, int16, int32, int64
uint8, uint16, uint32, uint64
float32, float64
string
time, duration
```

### 数组类型
```
int32[]        # 变长整数数组
float64[3]     # 固定长度浮点数数组
string[]       # 字符串数组
```

### 嵌套类型
```
geometry_msgs/Point     # 标准消息类型
custom_msgs/SimpleMessage  # 其他自定义消息
```

## 高级特性

### 常量定义
```
# 在.msg文件中定义常量
int32 SUCCESS=0
int32 FAILURE=1
string DEFAULT_NAME="unknown"
```

### 默认值
```
# ROS2不支持在.msg中直接设置默认值
# 需要在代码中设置默认值
```

## 最佳实践

### 命名规范
1. **包名**: 使用蛇形命名，以`_msgs`结尾
2. **消息文件**: 使用帕斯卡命名，如`RobotStatus.msg`
3. **字段名**: 使用蛇形命名，如`battery_level`

### 设计原则
1. **单一职责**: 每个消息类型专注一个功能
2. **向后兼容**: 避免删除或修改现有字段
3. **文档完善**: 在.msg文件中添加注释说明

### 版本管理
1. **语义版本**: 遵循语义化版本规范
2. **兼容性**: 新字段添加到消息末尾
3. **弃用策略**: 标记弃用字段而非立即删除

## 调试工具

### 查看自定义消息
```bash
# 查看消息定义
ros2 interface show custom_msgs/msg/RobotStatus

# 手动发布测试消息
ros2 topic pub /robot_status custom_msgs/msg/RobotStatus "
robot_name: 'TestRobot'
mode: 1
battery_level: 85.0
joint_angles: [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
current_position:
  x: 1.0
  y: 2.0
  z: 0.0
"

# 查看消息类型
ros2 topic type /robot_status
ros2 topic info /robot_status
```

## 常见问题解决

### 1. 编译错误：找不到消息头文件
```bash
# 确保自定义消息包已编译
# 确保正确设置了依赖
# 重新source工作空间
```

### 2. 消息字段不匹配
```bash
# 检查.msg文件是否正确
# 重新编译消息包
# 清理并重新编译所有包
```

### 3. 运行时消息类型错误
```bash
# 检查发布者和订阅者使用的消息类型是否一致
# 确认所有节点都使用了更新后的消息定义
```

自定义消息类型是ROS2中构建复杂应用的基础，合理设计消息结构可以大大提高系统的可维护性和扩展性。
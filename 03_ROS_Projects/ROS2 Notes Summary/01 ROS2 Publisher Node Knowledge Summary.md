# ROS2 发布者节点知识整理

## 基本概念

### 什么是发布者(Publisher)?
```cpp
- ROS2中负责向特定主题(Topic)发送消息的节点
- 采用发布-订阅模式，实现节点间的异步通信
- 一个节点可以包含多个发布者，向不同主题发布消息
```

### ROS2通信模型
```
发布者(Publisher) → 主题(Topic) → 订阅者(Subscriber)
     ↓
   消息(Message)
```

## 核心组件详解

### 1. 节点(Node)
```cpp
class SimplePublisher : public rclcpp::Node
```
- **作用**: ROS2中的基本执行单元
- **功能**: 封装ROS2功能，提供通信接口
- **命名**: 每个节点应有唯一名称

### 2. 发布者(Publisher)
```cpp
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

```
- **作用**: 消息发送接口
- **消息类型**: 使用模板指定(`std_msgs::msg::String`)
- **队列大小**: 指定消息缓冲区大小

### 3. 定时器(Timer)
```cpp
rclcpp::TimerBase::SharedPtr timer_;
```
- **作用**: 定期触发消息发布
- **类型**: 壁钟定时器(wall timer)
- **精度**: 毫秒级定时

### 4. 消息(Message)
```cpp
std_msgs::msg::String
```
- **作用**: 节点间传递的数据结构
- **类型**: 标准消息类型或自定义类型
- **访问**: 通过成员变量访问数据

## 完整代码实现

```cpp
// 包含必要的头文件
#include "rclcpp/rclcpp.hpp"           // ROS2 C++客户端库
#include "std_msgs/msg/string.hpp"     // 标准字符串消息类型

// 使用命名空间，避免重复写rclcpp::等前缀
using namespace std::chrono_literals;

// 创建发布者节点类，继承自rclcpp::Node
class SimplePublisher : public rclcpp::Node
{
public:
    // 构造函数：节点初始化
    SimplePublisher() : Node("simple_publisher"), count_(0)
    {
        // 创建发布者，参数：
        // 1. 主题名称: "topic"
        // 2. 消息队列大小: 10
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        
        // 创建定时器，每500ms触发一次
        // std::bind用于绑定成员函数和this指针
        timer_ = this->create_wall_timer(
            500ms, 
            std::bind(&SimplePublisher::timer_callback, this)
        );
        
        // 打印启动信息
        RCLCPP_INFO(this->get_logger(), "发布者节点已启动，正在发布消息...");
    }

private:
    // 定时器回调函数
    void timer_callback()
    {
        // 创建消息对象
        auto message = std_msgs::msg::String();
        
        // 设置消息内容
        message.data = "Hello, ROS2! 计数: " + std::to_string(count_++);
        
        // 发布消息
        publisher_->publish(message);
        
        // 打印日志信息
        RCLCPP_INFO(this->get_logger(), "发布消息: '%s'", message.data.c_str());
    }
    
    // 成员变量
    rclcpp::TimerBase::SharedPtr timer_;           // 定时器指针
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // 发布者指针
    size_t count_;                                  // 计数器
};

// 主函数
int main(int argc, char * argv[])
{
    // 初始化ROS2 C++客户端库
    rclcpp::init(argc, argv);
    
    // 创建节点并进入循环
    auto node = std::make_shared<SimplePublisher>();
    
    // 保持节点运行，直到接收到终止信号
    rclcpp::spin(node);
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return 0;
}
```

## 关键函数详解

### 1. 创建发布者
```cpp
publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
```
- **模板参数**: 消息类型
- **参数1**: 主题名称(字符串)
- **参数2**: 队列大小(整数)

### 2. 创建定时器
```cpp
timer_ = this->create_wall_timer(500ms, std::bind(&SimplePublisher::timer_callback, this));
```
- **参数1**: 时间间隔(500毫秒)
- **参数2**: 回调函数绑定

### 3. 消息发布
```cpp
publisher_->publish(message);
```
- **作用**: 将消息发送到主题
- **参数**: 消息对象

### 4. 节点初始化
```cpp
rclcpp::init(argc, argv);
```
- **作用**: 初始化ROS2通信层
- **参数**: main函数的参数

### 5. 节点运行循环
```cpp
rclcpp::spin(node);
```
- **作用**: 保持节点运行，处理回调
- **阻塞**: 直到接收到终止信号

## 消息类型说明

### 标准消息类型
```cpp
std_msgs::msg::String    // 字符串消息
std_msgs::msg::Int32     // 32位整数
std_msgs::msg::Float64   // 64位浮点数
geometry_msgs::msg::Twist // 速度指令
```

### 消息结构
```cpp
std_msgs::msg::String {
    std::string data;  // 实际数据内容
}
```

## 编译配置

### CMakeLists.txt关键部分
```cmake
# 创建可执行文件
add_executable(publisher_node src/publisher.cpp)

# 设置依赖
ament_target_dependencies(publisher_node rclcpp std_msgs)

# 安装到指定目录
install(TARGETS publisher_node DESTINATION lib/${PROJECT_NAME})
```

### package.xml依赖
```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

## 运行和调试

### 编译命令
```bash
colcon build --packages-select cpp_pubsub
```

### 运行节点
```bash
ros2 run cpp_pubsub publisher_node
```

### 调试命令
```bash
# 查看活跃主题
ros2 topic list

# 查看主题消息
ros2 topic echo /topic

# 查看节点信息
ros2 node list
ros2 node info /simple_publisher
```

## 常见问题及解决

### 1. 节点命名冲突
- **问题**: 相同名称的节点不能同时运行
- **解决**: 使用不同的节点名称

### 2. 主题不匹配
- **问题**: 发布者和订阅者主题名称不一致
- **解决**: 检查并统一主题名称

### 3. 消息类型不匹配
- **问题**: 发布和订阅的消息类型不同
- **解决**: 使用相同的消息类型

### 4. 队列溢出
- **问题**: 消息发布速度大于处理速度
- **解决**: 增加队列大小或优化处理逻辑

## 最佳实践

### 代码组织
1. 将节点类声明和实现分离
2. 使用有意义的节点和主题名称
3. 添加充分的日志输出

### 性能考虑
1. 合理设置发布频率
2. 选择合适的队列大小
3. 使用合适的数据类型

### 错误处理
1. 检查资源创建是否成功
2. 添加异常处理机制
3. 实现优雅的关闭逻辑

## 扩展功能

### 参数化配置
```cpp
// 可从命令行参数读取配置
declare_parameter("publish_frequency", 2.0);
```

### 服务质量(QoS)
```cpp
// 设置服务质量策略
rclcpp::QoS qos_profile(10);
```

### 自定义消息
```cpp
// 创建自定义消息类型
// 需要在package.xml和CMakeLists.txt中配置
```

这个发布者节点是ROS2编程的基础，理解其原理和实现细节对后续学习至关重要。
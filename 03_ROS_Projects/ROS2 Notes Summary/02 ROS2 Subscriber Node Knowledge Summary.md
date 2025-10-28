# ROS2 订阅者节点知识整理

## 基本概念

### 什么是订阅者(Subscriber)?
```cpp
- ROS2中负责从特定主题(Topic)接收消息的节点
- 采用发布-订阅模式，实现节点间的异步通信
- 一个节点可以包含多个订阅者，从不同主题接收消息
```

### ROS2通信模型
```
发布者(Publisher) → 主题(Topic) → 订阅者(Subscriber)
     ↓                       ↓
   消息(Message)          回调函数(Callback)
```

## 核心组件详解

### 1. 节点(Node)
```cpp
class SimpleSubscriber : public rclcpp::Node
```
- **作用**: ROS2中的基本执行单元
- **功能**: 封装ROS2功能，提供通信接口
- **命名**: 每个节点应有唯一名称

### 2. 订阅者(Subscriber)
```cpp
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
```
- **作用**: 消息接收接口
- **消息类型**: 使用模板指定(`std_msgs::msg::String`)
- **队列大小**: 指定消息缓冲区大小
- **回调函数**: 收到消息时自动执行的函数

### 3. 回调函数(Callback Function)
```cpp
void topic_callback(const std_msgs::msg::String::SharedPtr msg)
```
- **作用**: 异步处理接收到的消息
- **触发条件**: 每当有新消息到达时自动调用
- **参数**: 接收到的消息指针

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

// 使用std::placeholders，用于回调函数参数绑定
using std::placeholders::_1;

// 创建订阅者节点类，继承自rclcpp::Node
class SimpleSubscriber : public rclcpp::Node
{
public:
    // 构造函数：节点初始化
    SimpleSubscriber() : Node("simple_subscriber")
    {
        // 创建订阅者，参数：
        // 1. 主题名称: "topic" (必须与发布者相同)
        // 2. 消息队列大小: 10
        // 3. 回调函数: 收到消息时自动调用
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 
            10, 
            std::bind(&SimpleSubscriber::topic_callback, this, _1)
        );
        
        // 打印启动信息
        RCLCPP_INFO(this->get_logger(), "订阅者节点已启动，正在等待消息...");
    }

private:
    // 主题回调函数，当收到消息时自动执行
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 打印接收到的消息
        RCLCPP_INFO(this->get_logger(), "收到消息: '%s'", msg->data.c_str());
    }
    
    // 成员变量
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;  // 订阅者指针
};

// 主函数
int main(int argc, char * argv[])
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<SimpleSubscriber>();
    
    // 保持节点运行，处理接收到的消息
    rclcpp::spin(node);
    
    // 关闭ROS2
    rclcpp::shutdown();
    
    return 0;
}
```

## 关键函数详解

### 1. 创建订阅者
```cpp
subscriber_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 
    10, 
    std::bind(&SimpleSubscriber::topic_callback, this, _1)
);
```
- **模板参数**: 消息类型
- **参数1**: 主题名称(字符串)
- **参数2**: 队列大小(整数)
- **参数3**: 回调函数绑定

### 2. 回调函数定义
```cpp
void topic_callback(const std_msgs::msg::String::SharedPtr msg)
```
- **参数类型**: 消息的共享指针
- **const修饰**: 保证不会修改消息内容
- **命名约定**: 通常以`_callback`结尾

### 3. 消息处理
```cpp
RCLCPP_INFO(this->get_logger(), "收到消息: '%s'", msg->data.c_str());
```
- **访问消息数据**: 通过`msg->data`访问
- **日志输出**: 使用RCLCPP_INFO记录接收信息

### 4. 节点运行循环
```cpp
rclcpp::spin(node);
```
- **作用**: 保持节点运行，等待并处理消息
- **阻塞**: 直到接收到终止信号
- **内部机制**: 不断检查消息队列并触发回调

## 消息类型说明

### 标准消息类型
```cpp
std_msgs::msg::String    // 字符串消息
std_msgs::msg::Int32     // 32位整数
std_msgs::msg::Float64   // 64位浮点数
sensor_msgs::msg::Image  // 图像数据
```

### 消息指针类型
```cpp
const std_msgs::msg::String::SharedPtr msg
// SharedPtr是ROS2的智能指针，自动管理内存
```

## 回调机制详解

### 回调函数绑定
```cpp
std::bind(&SimpleSubscriber::topic_callback, this, _1)
```
- **std::bind**: 用于绑定成员函数和对象实例
- **&类名::函数名**: 获取成员函数地址
- **this**: 当前对象指针
- **_1**: 参数占位符，表示第一个参数

### 异步处理特性
- **非阻塞**: 主线程继续运行，不等待消息
- **事件驱动**: 消息到达时自动触发回调
- **并发安全**: ROS2内部处理线程安全

## 编译配置

### CMakeLists.txt关键部分
```cmake
# 创建可执行文件
add_executable(subscriber_node src/subscriber.cpp)

# 设置依赖
ament_target_dependencies(subscriber_node rclcpp std_msgs)

# 安装到指定目录
install(TARGETS subscriber_node DESTINATION lib/${PROJECT_NAME})
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
# 终端1 - 运行发布者
ros2 run cpp_pubsub publisher_node

# 终端2 - 运行订阅者
ros2 run cpp_pubsub subscriber_node
```

### 调试命令
```bash
# 查看活跃主题
ros2 topic list

# 查看主题消息
ros2 topic echo /topic

# 查看节点信息
ros2 node list
ros2 node info /simple_subscriber

# 手动发布测试消息
ros2 topic pub /topic std_msgs/msg/String "{data: '测试消息'}"
```

## 常见问题及解决

### 1. 收不到消息
- **问题**: 订阅者没有收到发布者的消息
- **检查**:
  - 主题名称是否一致
  - 消息类型是否匹配
  - 节点是否正常运行
  - 网络配置是否正确

### 2. 回调函数不执行
- **问题**: 消息到达但回调函数没有被调用
- **检查**:
  - 回调函数绑定是否正确
  - `rclcpp::spin()`是否被调用
  - 节点是否在运行状态

### 3. 消息处理延迟
- **问题**: 消息处理速度跟不上接收速度
- **解决**:
  - 优化回调函数处理逻辑
  - 增加队列大小
  - 使用多线程处理

### 4. 内存泄漏
- **问题**: 长时间运行后内存占用不断增加
- **解决**:
  - 检查回调函数中的内存分配
  - 使用智能指针管理资源
  - 避免在回调中创建大对象

## 高级特性

### 服务质量(QoS)配置
```cpp
// 设置可靠性为RELIABLE，持久性为VOLATILE
rclcpp::QoS qos_profile(10);
qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
subscriber_ = this->create_subscription<std_msgs::msg::String>(
    "topic", qos_profile, callback);
```

### 消息过滤
```cpp
// 在回调函数中实现消息过滤
void topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data.find("重要") != std::string::npos) {
        // 只处理包含"重要"的消息
        process_important_message(msg);
    }
}
```

### 多主题订阅
```cpp
// 一个节点订阅多个主题
subscriber1_ = create_subscription<MsgType1>("topic1", 10, callback1);
subscriber2_ = create_subscription<MsgType2>("topic2", 10, callback2);
```

## 最佳实践

### 代码组织
1. 保持回调函数简洁高效
2. 使用有意义的节点和主题名称
3. 添加充分的日志输出，但避免过度日志

### 性能优化
1. 在回调函数中避免耗时操作
2. 使用合适的数据结构和算法
3. 考虑使用多线程处理复杂任务

### 错误处理
1. 在回调函数中添加异常处理
2. 检查消息数据的有效性
3. 实现优雅的关闭逻辑

### 资源管理
1. 使用RAII原则管理资源
2. 及时释放不需要的资源
3. 监控内存和CPU使用情况

## 扩展应用

### 数据转换和处理
```cpp
// 在回调函数中进行数据转换
void sensor_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 将ROS图像消息转换为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    // 进行图像处理...
}
```

### 状态机集成
```cpp
// 根据接收到的消息改变节点状态
void command_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "start") {
        current_state_ = State::RUNNING;
    } else if (msg->data == "stop") {
        current_state_ = State::IDLE;
    }
}
```

### 与其他节点协作
```cpp
// 接收消息后发布其他消息
void input_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // 处理输入消息
    auto output_msg = process_input(msg);
    // 发布处理结果
    output_publisher_->publish(output_msg);
}
```

订阅者节点是ROS2系统中重要的信息消费者，理解其工作原理和最佳实践对于构建可靠的ROS2应用至关重要。通过合理设计回调函数和消息处理逻辑，可以创建高效、稳定的订阅者节点。
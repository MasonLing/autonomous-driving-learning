# ROS2 服务通信实现知识整理

## 基本概念

### 什么是服务(Service)?
```cpp
- ROS2中基于请求-响应模式的同步通信机制
- 客户端(Client)发送请求，服务端(Server)处理并返回响应
- 一对多关系：一个服务端可服务多个客户端
- 适用于需要确认结果的远程过程调用(RPC)
```

### 服务通信模型
```
客户端(Client) → 服务(Service) → 服务端(Server)
    请求(Request)                  处理请求
    响应(Response) ←────────────── 返回结果
```

## 创建自定义服务

### 1. 创建服务定义包
```bash
cd ~/ros2_ws/src
ros2 pkg create custom_srvs --build-type ament_cmake
```

### 2. 目录结构
```
custom_srvs/
├── CMakeLists.txt
├── include/
├── srv/           # 存放自定义服务定义文件
├── package.xml
└── src/
```

### 3. 服务定义文件

#### 简单计算服务 `srv/AddTwoInts.srv`
```
# 请求部分
int64 a
int64 b
---
# 响应部分
int64 sum
```

#### 机器人控制服务 `srv/RobotCommand.srv`
```
# 请求部分：命令类型和参数
string command_type     # move, stop, get_status
float64 target_x
float64 target_y
float64 speed
---
# 响应部分：执行结果
bool success
string message
float64 current_x
float64 current_y
```

## 包配置

### package.xml 配置
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypelocation="http://www.ros.org/schema/package_format3.xsd"?>
<package format="3">
  <name>custom_srvs</name>
  <version>0.0.0</version>
  <description>Custom service definitions</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 服务生成依赖 -->
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt 配置
```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_srvs)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成服务接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
  "srv/RobotCommand.srv"
)

# 安装服务文件
install(
  DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

## 服务端(Server)实现

### 完整服务端代码
```cpp
#include "rclcpp/rclcpp.hpp"
#include "custom_srvs/srv/add_two_ints.hpp"
#include "custom_srvs/srv/robot_command.hpp"

using namespace std::chrono_literals;

class MathServiceServer : public rclcpp::Node
{
public:
    MathServiceServer() : Node("math_service_server")
    {
        // 创建加法服务
        add_service_ = this->create_service<custom_srvs::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&MathServiceServer::handle_add_two_ints, this,
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        // 创建机器人控制服务
        robot_service_ = this->create_service<custom_srvs::srv::RobotCommand>(
            "robot_command",
            std::bind(&MathServiceServer::handle_robot_command, this,
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        // 模拟机器人状态
        current_x_ = 0.0;
        current_y_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "数学服务端已启动");
        RCLCPP_INFO(this->get_logger(), "服务列表:");
        RCLCPP_INFO(this->get_logger(), "  - /add_two_ints");
        RCLCPP_INFO(this->get_logger(), "  - /robot_command");
    }

private:
    // 处理加法请求
    void handle_add_two_ints(
        const std::shared_ptr<custom_srvs::srv::AddTwoInts::Request> request,
        std::shared_ptr<custom_srvs::srv::AddTwoInts::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), 
                   "收到加法请求: %ld + %ld", request->a, request->b);
        
        // 执行计算
        response->sum = request->a + request->b;
        
        RCLCPP_INFO(this->get_logger(), 
                   "返回结果: %ld", response->sum);
    }
    
    // 处理机器人命令
    void handle_robot_command(
        const std::shared_ptr<custom_srvs::srv::RobotCommand::Request> request,
        std::shared_ptr<custom_srvs::srv::RobotCommand::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), 
                   "收到机器人命令: %s", request->command_type.c_str());
        
        if (request->command_type == "move") {
            // 处理移动命令
            response->success = move_robot(request->target_x, request->target_y, request->speed);
            response->message = response->success ? "移动成功" : "移动失败";
            response->current_x = current_x_;
            response->current_y = current_y_;
            
        } else if (request->command_type == "stop") {
            // 处理停止命令
            response->success = true;
            response->message = "机器人已停止";
            response->current_x = current_x_;
            response->current_y = current_y_;
            
        } else if (request->command_type == "get_status") {
            // 处理状态查询
            response->success = true;
            response->message = "状态查询成功";
            response->current_x = current_x_;
            response->current_y = current_y_;
            
        } else {
            // 未知命令
            response->success = false;
            response->message = "未知命令类型: " + request->command_type;
            response->current_x = current_x_;
            response->current_y = current_y_;
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "命令执行结果: %s - %s", 
                   response->success ? "成功" : "失败", 
                   response->message.c_str());
    }
    
    // 模拟机器人移动
    bool move_robot(double target_x, double target_y, double speed)
    {
        RCLCPP_INFO(this->get_logger(), 
                   "移动机器人到位置: (%.2f, %.2f), 速度: %.2f", 
                   target_x, target_y, speed);
        
        // 模拟移动过程
        current_x_ = target_x;
        current_y_ = target_y;
        
        // 这里可以添加实际的移动逻辑
        // 返回移动是否成功
        return true;
    }
    
    // 服务声明
    rclcpp::Service<custom_srvs::srv::AddTwoInts>::SharedPtr add_service_;
    rclcpp::Service<custom_srvs::srv::RobotCommand>::SharedPtr robot_service_;
    
    // 机器人状态
    double current_x_;
    double current_y_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto server_node = std::make_shared<MathServiceServer>();
    
    RCLCPP_INFO(server_node->get_logger(), "服务端运行中...");
    
    rclcpp::spin(server_node);
    
    rclcpp::shutdown();
    return 0;
}
```

## 客户端(Client)实现

### 完整客户端代码
```cpp
#include "rclcpp/rclcpp.hpp"
#include "custom_srvs/srv/add_two_ints.hpp"
#include "custom_srvs/srv/robot_command.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ServiceClient : public rclcpp::Node
{
public:
    ServiceClient() : Node("service_client")
    {
        // 创建加法服务客户端
        add_client_ = this->create_client<custom_srvs::srv::AddTwoInts>("add_two_ints");
        
        // 创建机器人服务客户端
        robot_client_ = this->create_client<custom_srvs::srv::RobotCommand>("robot_command");
        
        // 等待服务可用
        while (!add_client_->wait_for_service(1s) || !robot_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "客户端被中断");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务上线...");
        }
        
        RCLCPP_INFO(this->get_logger(), "服务客户端已启动");
        
        // 启动测试请求
        timer_ = this->create_wall_timer(
            2s,
            std::bind(&ServiceClient::send_test_requests, this)
        );
    }

private:
    void send_test_requests()
    {
        static int request_count = 0;
        request_count++;
        
        if (request_count == 1) {
            // 第一次请求：加法服务
            send_add_request(5, 3);
        } else if (request_count == 2) {
            // 第二次请求：机器人移动
            send_robot_command("move", 10.0, 5.0, 1.0);
        } else if (request_count == 3) {
            // 第三次请求：状态查询
            send_robot_command("get_status", 0, 0, 0);
        } else if (request_count == 4) {
            // 第四次请求：停止
            send_robot_command("stop", 0, 0, 0);
        } else {
            // 停止定时器
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "测试完成");
        }
    }
    
    // 发送加法请求
    void send_add_request(int a, int b)
    {
        auto request = std::make_shared<custom_srvs::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        
        RCLCPP_INFO(this->get_logger(), "发送加法请求: %d + %d", a, b);
        
        // 异步发送请求
        auto future_result = add_client_->async_send_request(request);
        
        // 等待响应（同步方式）
        if (rclcpp::spin_until_future_complete(
            shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future_result.get();
            RCLCPP_INFO(this->get_logger(), 
                       "收到加法响应: %ld + %ld = %ld", 
                       request->a, request->b, response->sum);
        } else {
            RCLCPP_ERROR(this->get_logger(), "加法服务调用失败");
        }
    }
    
    // 发送机器人命令
    void send_robot_command(const std::string& command, 
                           double x, double y, double speed)
    {
        auto request = std::make_shared<custom_srvs::srv::RobotCommand::Request>();
        request->command_type = command;
        request->target_x = x;
        request->target_y = y;
        request->speed = speed;
        
        RCLCPP_INFO(this->get_logger(), 
                   "发送机器人命令: %s, 目标: (%.1f, %.1f), 速度: %.1f", 
                   command.c_str(), x, y, speed);
        
        // 异步发送请求
        auto future_result = robot_client_->async_send_request(request);
        
        // 等待响应（同步方式）
        if (rclcpp::spin_until_future_complete(
            shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future_result.get();
            RCLCPP_INFO(this->get_logger(), 
                       "机器人命令响应: %s", 
                       response->success ? "成功" : "失败");
            RCLCPP_INFO(this->get_logger(), 
                       "消息: %s", response->message.c_str());
            RCLCPP_INFO(this->get_logger(), 
                       "当前位置: (%.1f, %.1f)", 
                       response->current_x, response->current_y);
        } else {
            RCLCPP_ERROR(this->get_logger(), "机器人服务调用失败");
        }
    }
    
    // 客户端声明
    rclcpp::Client<custom_srvs::srv::AddTwoInts>::SharedPtr add_client_;
    rclcpp::Client<custom_srvs::srv::RobotCommand>::SharedPtr robot_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto client_node = std::make_shared<ServiceClient>();
    
    // 由于使用了定时器，直接spin即可
    rclcpp::spin(client_node);
    
    rclcpp::shutdown();
    return 0;
}
```

## 编译配置

### 使用服务的包的package.xml
```xml
<depend>custom_srvs</depend>
<depend>rclcpp</depend>
```

### 使用服务的包的CMakeLists.txt
```cmake
find_package(custom_srvs REQUIRED)
find_package(rclcpp REQUIRED)

# 服务端可执行文件
add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server custom_srvs rclcpp)

# 客户端可执行文件
add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client custom_srvs rclcpp)

install(TARGETS
  service_server
  service_client
  DESTINATION lib/${PROJECT_NAME}
)
```

## 编译和运行

### 1. 编译服务定义包
```bash
cd ~/ros2_ws
colcon build --packages-select custom_srvs
source install/setup.bash
```

### 2. 编译服务节点包
```bash
colcon build --packages-select your_service_package
source install/setup.bash
```

### 3. 运行服务
```bash
# 终端1 - 启动服务端
ros2 run your_service_package service_server

# 终端2 - 启动客户端
ros2 run your_service_package service_client
```

## 服务通信详解

### 服务端关键函数

#### 创建服务
```cpp
add_service_ = this->create_service<custom_srvs::srv::AddTwoInts>(
    "add_two_ints",
    std::bind(&MathServiceServer::handle_add_two_ints, this,
             std::placeholders::_1, std::placeholders::_2)
);
```

#### 回调函数签名
```cpp
void handle_add_two_ints(
    const std::shared_ptr<custom_srvs::srv::AddTwoInts::Request> request,
    std::shared_ptr<custom_srvs::srv::AddTwoInts::Response> response)
```

### 客户端关键函数

#### 创建客户端
```cpp
add_client_ = this->create_client<custom_srvs::srv::AddTwoInts>("add_two_ints");
```

#### 等待服务可用
```cpp
while (!add_client_->wait_for_service(1s)) {
    RCLCPP_INFO(this->get_logger(), "等待服务上线...");
}
```

#### 发送请求（异步）
```cpp
auto future_result = add_client_->async_send_request(request);
```

#### 等待响应
```cpp
if (rclcpp::spin_until_future_complete(
    shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS)
{
    auto response = future_result.get();
    // 处理响应
}
```

## 调试工具

### 服务相关命令
```bash
# 查看服务列表
ros2 service list

# 查看服务类型
ros2 service type /add_two_ints

# 查看服务定义
ros2 interface show custom_srvs/srv/AddTwoInts

# 手动调用服务
ros2 service call /add_two_ints custom_srvs/srv/AddTwoInts "{a: 10, b: 20}"

# 查找特定服务
ros2 service find custom_srvs/srv/AddTwoInts
```

## 高级特性

### 异步客户端模式
```cpp
// 使用回调处理异步响应
auto future = client_->async_send_request(request,
    [this](rclcpp::Client<ServiceT>::SharedFuture future) {
        auto response = future.get();
        // 处理响应
        RCLCPP_INFO(this->get_logger(), "异步收到响应");
    });
```

### 服务质量(QoS)配置
```cpp
// 配置服务可靠性
auto qos = rclcpp::ServicesQoS().reliable();
add_service_ = this->create_service<custom_srvs::srv::AddTwoInts>(
    "add_two_ints", 
    std::bind(&MathServiceServer::handle_add_two_ints, this,
             std::placeholders::_1, std::placeholders::_2),
    qos);
```

### 超时处理
```cpp
// 设置请求超时
auto future = client_->async_send_request(request);
auto status = future.wait_for(std::chrono::seconds(5));
if (status == std::future_status::ready) {
    auto response = future.get();
    // 处理响应
} else {
    RCLCPP_ERROR(this->get_logger(), "服务请求超时");
}
```

## 最佳实践

### 服务设计原则
1. **单一职责**: 每个服务专注一个明确的功能
2. **合理的超时**: 设置适当的服务调用超时
3. **错误处理**: 在服务端和客户端都实现完善的错误处理

### 性能优化
1. **避免阻塞**: 服务端回调函数应快速返回
2. **合理超时**: 根据业务需求设置合适的客户端超时
3. **连接复用**: 客户端可重复使用连接

### 错误处理
```cpp
// 服务端错误处理
void handle_request(...) {
    try {
        // 业务逻辑
        response->success = true;
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("处理失败: ") + e.what();
    }
}

// 客户端错误处理
if (!client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(), "服务不可用");
    return;
}
```

## 常见问题解决

### 1. 服务不可用
```bash
# 检查服务端是否运行
ros2 service list

# 检查服务类型匹配
ros2 service type /service_name

# 检查网络连接
```

### 2. 超时问题
```cpp
// 增加等待时间
client_->wait_for_service(10s);

// 或使用循环等待
while (!client_->wait_for_service(1s)) {
    // 处理等待逻辑
}
```

### 3. 类型不匹配
```bash
# 确认服务端和客户端使用相同的服务定义
ros2 interface show custom_srvs/srv/AddTwoInts
```

服务通信是ROS2中实现同步请求-响应模式的核心机制，适用于需要确认结果的远程调用场景。合理设计服务接口和实现可靠的错误处理是构建稳定ROS2系统的关键。
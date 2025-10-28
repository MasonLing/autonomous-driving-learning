# ROS2 命令行工具全面指南

## 基本概念

### 什么是ROS2 CLI工具？
```bash
- ROS2提供的一套命令行界面工具
- 用于监控、调试和管理ROS2系统
- 基于`ros2`命令，通过不同子命令实现功能
- 支持节点、主题、服务、参数等所有ROS2核心概念
```

### 核心命令结构
```
ros2 <command> <subcommand> [options] [arguments]
```

## 节点管理命令

### 查看节点信息
```bash
# 列出所有活跃节点
ros2 node list

# 查看特定节点信息
ros2 node info /node_name

# 查看节点详细信息（包括发布者、订阅者、服务等）
ros2 node info /simple_publisher
```

### 节点生命周期管理
```bash
# 运行节点
ros2 run <package_name> <executable_name>

# 示例：运行发布者节点
ros2 run cpp_pubsub publisher_node

# 通过启动文件运行节点
ros2 launch <package_name> <launch_file.launch.py>
```

## 主题(Topic)管理命令

### 查看主题信息
```bash
# 列出所有活跃主题
ros2 topic list

# 列出包含类型的主题信息
ros2 topic list -t

# 查看特定主题信息
ros2 topic info /topic_name

# 查看主题类型
ros2 topic type /topic_name

# 查看主题带宽使用情况
ros2 topic bw /topic_name

# 查看主题频率统计
ros2 topic hz /topic_name

# 查看主题延迟统计
ros2 topic delay /topic_name
```

### 主题数据操作
```bash
# 实时显示主题消息
ros2 topic echo /topic_name

# 显示特定字段
ros2 topic echo /topic_name --field data

# 发布消息到主题
ros2 topic pub /topic_name <message_type> "<message_data>"

# 示例：发布字符串消息
ros2 topic pub /topic std_msgs/msg/String "{data: 'Hello ROS2'}"

# 一次性发布消息
ros2 topic pub -1 /topic std_msgs/msg/String "{data: 'Hello'}"

# 周期性发布消息
ros2 topic pub -r 1 /topic std_msgs/msg/String "{data: 'Hello'}"

# 从文件发布消息
ros2 topic pub /topic std_msgs/msg/String -f message_file.yaml
```

## 服务(Service)管理命令

### 查看服务信息
```bash
# 列出所有可用服务
ros2 service list

# 列出包含类型的服务信息
ros2 service list -t

# 查看服务类型
ros2 service type /service_name

# 查找提供特定类型服务的节点
ros2 service find <service_type>
```

### 服务调用和监控
```bash
# 调用服务
ros2 service call /service_name <service_type> "<request_data>"

# 示例：调用加法服务
ros2 service call /add_two_ints custom_srvs/srv/AddTwoInts "{a: 5, b: 3}"

# 查看服务调用结果详情
ros2 service call /add_two_ints custom_srvs/srv/AddTwoInts "{a: 5, b: 3}" --verbose
```

## 参数(Parameter)管理命令

### 参数操作
```bash
# 列出节点参数
ros2 param list /node_name

# 获取参数值
ros2 param get /node_name parameter_name

# 设置参数值
ros2 param set /node_name parameter_name value

# 示例：设置发布频率
ros2 param set /simple_publisher publish_frequency 2.0

# 导出参数到文件
ros2 param dump /node_name

# 从文件加载参数
ros2 param load /node_name parameter_file.yaml

# 查看参数描述
ros2 param describe /node_name parameter_name
```

### 参数文件示例
```yaml
# parameters.yaml
simple_publisher:
  ros__parameters:
    publish_frequency: 2.0
    message_prefix: "Custom"
    enable_logging: true
```

## 消息接口管理

### 消息类型操作
```bash
# 查看消息定义
ros2 interface show <message_type>

# 示例：查看字符串消息
ros2 interface show std_msgs/msg/String

# 查看服务定义
ros2 interface show custom_srvs/srv/AddTwoInts

# 查看动作定义
ros2 interface show <action_type>

# 列出所有可用接口
ros2 interface list

# 列出包提供的接口
ros2 interface package <package_name>
```

### 接口包管理
```bash
# 查找提供特定接口的包
ros2 interface packages

# 查看包的所有接口
ros2 interface package std_msgs
```

## 包(Package)管理命令

### 包信息查询
```bash
# 列出工作空间中所有包
ros2 pkg list

# 查看包信息
ros2 pkg prefix <package_name>

# 查找包所在路径
ros2 pkg executables <package_name>

# 列出包的可执行文件
ros2 pkg executables cpp_pubsub
```

### 包创建和管理
```bash
# 创建新的CMake包
ros2 pkg create <package_name> --build-type ament_cmake

# 创建Python包
ros2 pkg create <package_name> --build-type ament_python

# 设置包依赖
ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp std_msgs

# 查看包XML信息
ros2 pkg xml <package_name>
```

## 动作(Action)管理命令

### 动作操作
```bash
# 列出所有动作
ros2 action list

# 列出包含类型的动作
ros2 action list -t

# 查看动作信息
ros2 action info /action_name

# 查看动作类型
ros2 action type /action_name

# 发送动作目标
ros2 action send_goal /action_name <action_type> "<goal_data>"

# 示例：发送斐波那契动作目标
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

## 组件管理命令

### 组件容器
```bash
# 运行组件容器
ros2 run rclcpp_components component_container

# 列出已加载组件
ros2 component list

# 加载组件到容器
ros2 component load /ComponentManager <package_name> <component_class>

# 卸载组件
ros2 component unload <component_id>
```

## 系统监控和调试

### 系统状态监控
```bash
# 查看系统守护进程状态
ros2 daemon status

# 启动守护进程
ros2 daemon start

# 停止守护进程
ros2 daemon stop

# 查看进程信息
ros2 run system_monitor process_monitor
```

### 实时监控工具
```bash
# 监控节点图
rqt_graph

# 监控主题
rqt_topic

# 监控节点
rqt_node

# 监控服务
rqt_service

# 监控参数
rqt_parameter

# 数据绘图
rqt_plot
```

## 常用组合命令和技巧

### 实用命令组合
```bash
# 查看节点及其发布/订阅关系
ros2 node list | xargs -I {} ros2 node info {}

# 监控特定类型的所有主题
ros2 topic list -t | grep std_msgs/msg/String

# 批量设置参数
ros2 param load /node_name parameters.yaml

# 查找所有使用特定消息类型的主题
ros2 topic list -t | grep "std_msgs/msg/String"
```

### 调试技巧
```bash
# 详细模式运行节点
ros2 run <package> <node> --ros-args --log-level debug

# 重映射主题名称
ros2 run <package> <node> --ros-args -r <old_topic>:=<new_topic>

# 设置节点命名空间
ros2 run <package> <node> --ros-args -r __ns:=/my_namespace

# 设置节点名称
ros2 run <package> <node> --ros-args -r __node:=new_node_name
```

## 工作空间和构建命令

### 工作空间管理
```bash
# 编译工作空间
colcon build

# 编译特定包
colcon build --packages-select <package_name>

# 编译并运行测试
colcon test

# 清理构建文件
rm -rf build install log

# 查看包依赖
rosdep check <package_name>

# 安装包依赖
rosdep install -i --from-path src --rosdistro humble -y
```

### 环境设置
```bash
# 设置工作空间环境
source install/setup.bash

# 检查环境变量
echo $ROS_DISTRO
echo $ROS_VERSION

# 设置ROS域ID（多机器人系统）
export ROS_DOMAIN_ID=10
```

## 高级调试命令

### 性能分析
```bash
# 监控系统资源使用
ros2 run system_monitor cpu_monitor
ros2 run system_monitor memory_monitor

# 查看主题统计
ros2 topic hz /topic_name --window 10

# 查看消息延迟
ros2 topic delay /topic_name
```

### 记录和回放
```bash
# 录制所有主题
ros2 bag record -a

# 录制特定主题
ros2 bag record -o my_bag /topic1 /topic2

# 查看录制的包信息
ros2 bag info my_bag

# 回放录制的数据
ros2 bag play my_bag

# 查看包内容
ros2 bag play my_bag --loop
```

## 故障排除命令

### 常见问题诊断
```bash
# 检查ROS2安装
ros2 doctor

# 详细系统检查
ros2 doctor --report

# 检查网络配置
export ROS_LOCALHOST_ONLY=0  # 允许网络通信
export ROS_DOMAIN_ID=0       # 设置域ID

# 查看日志级别
ros2 run <package> <node> --ros-args --log-level INFO
```

### 连接问题调试
```bash
# 检查发现服务
ros2 daemon status

# 重启发现服务
ros2 daemon stop
ros2 daemon start

# 检查网络发现
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 node list
```

## 实用脚本和别名

### 常用别名设置
```bash
# 添加到 ~/.bashrc
alias rl='ros2 node list'
alias rt='ros2 topic list'
alias rtl='ros2 topic list -t'
alias rs='ros2 service list'
alias rsl='ros2 service list -t'
alias rn='ros2 node info'
alias rte='ros2 topic echo'
alias rth='ros2 topic hz'
alias rtc='ros2 topic pub --once'
```

### 实用函数
```bash
# 查找节点完整名称
ros2_find_node() {
    ros2 node list | grep $1
}

# 快速查看主题信息
ros2_topic_info() {
    ros2 topic info $1
    echo "--- Message Type ---"
    ros2 topic type $1
    echo "--- Message Definition ---"
    ros2 interface show $(ros2 topic type $1)
}
```

## 版本和配置信息

### 系统信息查询
```bash
# 查看ROS2版本
ros2 version

# 查看所有已安装的包
ros2 pkg list

# 查看RMW实现
echo $RMW_IMPLEMENTATION

# 查看环境配置
env | grep ROS
```

### 配置检查
```bash
# 检查默认RMW
ros2 run rclcpp_components component_container --ros-args --log-level debug

# 验证消息类型
ros2 interface show std_msgs/msg/String

# 检查服务质量设置
ros2 topic info /topic_name --verbose
```

## 实际应用场景

### 场景1：调试发布者-订阅者通信
```bash
# 终端1：运行发布者
ros2 run cpp_pubsub publisher_node

# 终端2：查看主题
ros2 topic list
ros2 topic echo /topic

# 终端3：监控频率
ros2 topic hz /topic

# 终端4：查看节点图
rqt_graph
```

### 场景2：调试服务通信
```bash
# 终端1：运行服务端
ros2 run my_package service_server

# 终端2：查看服务
ros2 service list
ros2 service type /add_two_ints

# 终端3：手动测试服务
ros2 service call /add_two_ints custom_srvs/srv/AddTwoInts "{a: 10, b: 20}"

# 终端4：运行客户端
ros2 run my_package service_client
```

### 场景3：参数调试
```bash
# 终端1：运行带参数的节点
ros2 run my_package node_with_parameters

# 终端2：查看和修改参数
ros2 param list /node_name
ros2 param get /node_name parameter_name
ros2 param set /node_name parameter_name new_value

# 终端3：导出参数
ros2 param dump /node_name > params.yaml
```

这些ROS2命令行工具涵盖了系统监控、调试、管理和维护的所有方面。熟练掌握这些命令可以大大提高ROS2开发和调试的效率。建议在实际项目中多加练习，形成自己的工作流程。
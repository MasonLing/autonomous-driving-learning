#include"planning_process.h"

namespace Planning
{
    PlanningProcess::PlanningProcess() : Node("planning_process")//规划总流程
    {
        RCLCPP_INFO(this->get_logger(),"planning_process created");

        //读取配置文件
        process_config_ = std::make_unique<ConfigReader>();
        process_config_->read_planning_process_config();
        obs_dis_ = process_config_->process().obs_dis_;

        //创建地图服务器和全局路径客户端
        map_client_ = this->create_client<PNCMapService>("pnc_map_server");
        global_path_client_ = this->create_client<GlobalPathService>("pnc_map_server");
    }

    bool PlanningProcess::process()//总流程
    {
        //阻塞1s，等待rviz2和xacro模型启动
        rclcpp::Rate rate(1.0);
        rate.sleep();


        //初始化
        if(!planning_init())
        {
            RCLCPP_ERROR(this->get_logger(),"planning init failed!");  
            return false;  
        }
        //进入规划主流程

        return true;
    }
    bool PlanningProcess::planning_init()//流程初始化
    {
        //生成车辆

        //连接地图服务器
        if (!connect_server(map_client_))
        {
            RCLCPP_ERROR(this->get_logger(), "Map server connect failed!");
            return false;
        }
        

        //获取地图

        if(!map_request())
        {
            RCLCPP_ERROR(this->get_logger(), "Map request and response failed!");
            return false;
        }
        
        //连接全局路径服务器
        if (!connect_server(global_path_client_))
        {
            RCLCPP_ERROR(this->get_logger(), "Global_path server connect failed!");
            return false;
        }

        //获取全局路径
        if(!global_path_request())
        {
            RCLCPP_ERROR(this->get_logger(), "Global_path request and response failed!");
            return false;
        }

        return true;
    }

    template <typename T>
    bool connect_server(const T &client)//连接服务器
    {
        //判断客户类型
        std::string server_name;
        if constexpr (std::is_same_v<T, rclcpp::Client<PNCMapService>::SharedPtr>)//C++17引入的
        {
            server_name = "pnc_map";
        }
        else if constexpr (std::is_same_v<T, rclcpp::l<GlobalPathService>::SharedPtr>)//C++17引入的
        {
            server_name = "global_path";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "wrong client type!");
            return false; 
        }
        
        //等待服务器
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) //对ctrl+c操作处理 防止进入死循环
            {
                RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the %s server.", server_name.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "%s server not available, waiting again...",server_name.c_str());
        }

        return true;
    }
    

    bool PlanningProcess::map_request()//发送地图请求
    {
        return false;
    }

    bool PlanningProcess::global_path_request()//发送全局路径请求
    {
        return false;
    }



} // namespace Planning
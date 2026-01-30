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
        global_path_client_ = this->create_client<GlobalPathService>("global_path_server");
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

    bool PlanningProcess::map_request()//发送地图请求
    {
        RCLCPP_INFO(this->get_logger(),"Sending map request/");

        //生成请求
        auto request = std::make_shared<PNCMapService::Request>();
        request->map_type = process_config_->pnc_map().type_;

        //获取响应
        auto result_future = map_client_->async_send_request(request);

        // 判断响应是否成功
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(),"Map response sucess");
            pnc_map_ = result_future.get()->pnc_map;//获取响应中的pnc_map
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),"Map response failed!");
            return false;
        }
    }

    bool PlanningProcess::global_path_request()//发送全局路径请求
    {
        RCLCPP_INFO(this->get_logger(),"Sending global_path request/");

        //生成请求
        auto request = std::make_shared<GlobalPathService::Request>();
        request->pnc_map = pnc_map_;
        request->global_planner_type =process_config_->global_path().type_;

        //获取响应
        auto result_future = global_path_client_->async_send_request(request);

        // 判断响应是否成功
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(),"global_path response sucess");
            global_path_ = result_future.get()->global_path;//获取响应中的pnc_map
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),"Global path response failed!");
            return false;
        }
    }




} // namespace Planning
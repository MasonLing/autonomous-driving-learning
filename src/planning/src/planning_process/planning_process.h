#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "nav_msgs/msg/path.hpp"

#include "config_reader.h"
#include "main_car_info.h"
#include "obs_car_info.h"
#include "reference_line_creator.h"
#include "decision_center.h"
#include "local_path_planner.h"
#include "local_speeds_planner.h"
#include "local_trajectory_combiner.h"

// 基于他是C++还需要包含以下
#include <vector>
#include <cmath>
#include <algorithm>

#include <type_traits>


namespace Planning
{
  using namespace std::chrono_literals;
  using base_msgs::msg::PNCMap;
  using base_msgs::srv::GlobalPathService;
  using base_msgs::srv::PNCMapService;
  using nav_msgs::msg::Path;
  
  class PlanningProcess : public rclcpp::Node//规划总流程
  {
    public:
      PlanningProcess();
      bool process();//规划总流程

    private:
      bool planning_init();//流程初始化

      template <typename T>
      bool connect_server(const T &client);//连接服务器
      bool map_request();//发送地图的请求
      bool global_path_request();//发送全局路径请求

    public:
      inline PNCMap pnc_map() const { return pnc_map_;}//获取地图
      inline Path global_path() const { return global_path_;}//获取全局路径


    private:
      std::unique_ptr<ConfigReader> process_config_; //配置
      double obs_dis_ = 0.0;//考虑障碍物的距离

      PNCMap pnc_map_;//地图
      Path global_path_;//全局路径
      rclcpp::Client<PNCMapService>::SharedPtr map_client_;//地图请求客户端
      rclcpp::Client<GlobalPathService>::SharedPtr global_path_client_;//全局路径请求客户端
  };
  template <typename T>
  inline bool PlanningProcess::connect_server(const T &client)
  {
      //判断客户类型
      std::string server_name;
      if constexpr (std::is_same_v<T, rclcpp::Client<PNCMapService>::SharedPtr>)
      {
          server_name = "pnc_map";
      }
      else if constexpr (std::is_same_v<T, rclcpp::Client<GlobalPathService>::SharedPtr>)
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
          RCLCPP_INFO(this->get_logger(), "%s server not available, waiting again...", server_name.c_str());
      }

      return true;
  }
} // namespace
#endif  //PLANNING_PROCESS_H_
#include "pnc_map_creator_straight.h"

namespace Planning
{
    PNCMapCreatorStraight::PNCMapCreatorStraight() //直道地图
    {
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"),"pnc_map_creator_node created");
    }

    PNCMap PNCMapCreatorStraight::creat_pnc_map()//生成地图
    {
        return pnc_map_;  // 移除括号，直接返回对象
    }
} // namespace Planning

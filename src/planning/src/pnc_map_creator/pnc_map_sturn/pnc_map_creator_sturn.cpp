#include "pnc_map_creator_sturn.h"

namespace Planning
{
    PNCMapCreatorSTurn::PNCMapCreatorSTurn() //S弯道地图
    {
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"),"pnc_map_creator_sturn_node created");
    }
    
    PNCMap PNCMapCreatorSTurn::creat_pnc_map()//生成地图
    {
        return pnc_map_;
    }
} // namespace Planning

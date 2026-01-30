#include "reference_line_smoother.h"

namespace Planning
{
    ReferencelineSmoother::ReferencelineSmoother()//创建参考线
    {
        RCLCPP_INFO(rclcpp::get_logger("reference_line"),"reference_line_smoother created");
    }

}//namespace Planning
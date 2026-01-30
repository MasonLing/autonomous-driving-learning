#include "local_path_smoother.h"

namespace Planning
{
    LocalPathSmoother::LocalPathSmoother()//速度平滑器
    {
        RCLCPP_INFO(rclcpp::get_logger("local_path"),"local_path_smoother created");
    }
}//namespace Planning
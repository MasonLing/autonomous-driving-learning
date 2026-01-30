#ifndef PNC_MAP_CREATE_STURN_H_
#define PNC_MAP_CREATE_STURN_H_

#include "pnc_map_creator_base.h"
#include "base_msgs/msg/pnc_map.hpp"

namespace Planning
{
class PNCMapCreatorSTurn : public PNCMapCreatorBase //S弯道地图
    {
    public:
       PNCMapCreatorSTurn();
       PNCMap creat_pnc_map() override;//生成地图同样意思：[base_msgs::msg::PNCMap creat_pnc_map() override;]

    };
}  // namespace Planning
#endif  // PNC_MAP_CREATE_STURN_H_

#ifndef REFERENCE_LINE_SMOOTHER_H_
#define REFERENCE_LINE_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include <Eigen/Dense> //eigen
#include <OsqpEigen/OsqpEigen.h>//osqpeigen
#include <cmath>

namespace Planning
{
    class ReferencelineSmoother //参考线平滑
    {
    public:
        ReferencelineSmoother();
    private:

    };
}  // namespace Planning
#endif  // REFERENCE_LINE_SMOOTHER_H_

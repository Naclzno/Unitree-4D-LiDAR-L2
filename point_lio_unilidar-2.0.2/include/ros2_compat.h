#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

inline rclcpp::Time timeFromSec(double sec)
{
    const int32_t sec_part = static_cast<int32_t>(std::floor(sec));
    const uint32_t nsec_part = static_cast<uint32_t>((sec - static_cast<double>(sec_part)) * 1e9);
    return rclcpp::Time(sec_part, nsec_part, RCL_SYSTEM_TIME);
}

inline double stampSec(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

inline rclcpp::Logger pointlioLogger()
{
    return rclcpp::get_logger("pointlio_mapping");
}

#ifndef ROS_INFO
#define ROS_INFO(...) RCLCPP_INFO(pointlioLogger(), __VA_ARGS__)
#endif

#ifndef ROS_WARN
#define ROS_WARN(...) RCLCPP_WARN(pointlioLogger(), __VA_ARGS__)
#endif

#ifndef ROS_ERROR
#define ROS_ERROR(...) RCLCPP_ERROR(pointlioLogger(), __VA_ARGS__)
#endif

#ifndef ROS_ASSERT
#define ROS_ASSERT(cond)                                                                          \
    do                                                                                            \
    {                                                                                             \
        if (!(cond))                                                                              \
        {                                                                                         \
            RCLCPP_FATAL(pointlioLogger(), "assertion failed: %s", #cond);                        \
            throw std::runtime_error("ROS_ASSERT failed: " #cond);                                \
        }                                                                                         \
    } while (0)
#endif

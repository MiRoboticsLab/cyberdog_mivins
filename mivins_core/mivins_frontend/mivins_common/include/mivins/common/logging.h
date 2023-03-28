// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef LUCY_LOGGING
#include <oal/logfile/baselogger.hpp>
#define PREFIX "tracking"
#define LOG_TAG "svo"
#include <oal/logfile/enable_logging_macros.hpp>
#endif

#ifdef LOG_USE_ROS
#include <ros/console.h>
#define LOG_DEBUG_STREAM(x) RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), x)
#define LOG_INFO_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), x)
#define LOG_WARN_STREAM(x) RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), x)
#define LOG_WARN_STREAM_THROTTLE(rate, x) ROS_WARN_STREAM_THROTTLE(rate, x)
#define LOG_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), x)
#else
#ifdef LUCY_LOGGING
#define LOG_INFO_STREAM(x)            \
    {                                 \
        std::stringstream ss;         \
        ss << x;                      \
        LOGI("%s", ss.str().c_str()); \
    }
#define LOG_WARN_STREAM(x)            \
    {                                 \
        std::stringstream ss;         \
        ss << x;                      \
        LOGW("%s", ss.str().c_str()); \
    }
#define LOG_DEBUG_STREAM(x)           \
    {                                 \
        std::stringstream ss;         \
        ss << x;                      \
        LOGD("%s", ss.str().c_str()); \
    }
#define LOG_ERROR_STREAM(x)           \
    {                                 \
        std::stringstream ss;         \
        ss << x;                      \
        LOGE("%s", ss.str().c_str()); \
    }
#else
#ifdef LOG_DEBUG
#define LOG_INFO_STREAM(x)                                                     \
    {                                                                          \
        std::cerr << "\033[0;0m[INFO] SVO: " << x << "\033[0;0m" << std::endl; \
    }
#define LOG_DEBUG_STREAM(x)                                                     \
    {                                                                           \
        std::cerr << "\033[0;0m[DEBUG] SVO: " << x << "\033[0;0m" << std::endl; \
    }
#define LOG_WARN_STREAM(x)                                                      \
    {                                                                           \
        std::cerr << "\033[0;33m[WARN] SVO: " << x << "\033[0;0m" << std::endl; \
    }
#define LOG_ERROR_STREAM(x)                                                      \
    {                                                                            \
        std::cerr << "\033[1;31m[ERROR] SVO: " << x << "\033[0;0m" << std::endl; \
    }
#else
#define LOG_INFO_STREAM(x)                                                     \
    {                                                                          \
    }
#define LOG_DEBUG_STREAM(x)                                                     \
    {                                                                           \
    }
#define LOG_WARN_STREAM(x)                                                      \
    {                                                                           \
        std::cerr << "\033[0;33m[WARN] SVO: " << x << "\033[0;0m" << std::endl; \
    }
#define LOG_ERROR_STREAM(x)                                                      \
    {                                                                            \
        std::cerr << "\033[1;31m[ERROR] SVO: " << x << "\033[0;0m" << std::endl; \
    }
#endif
#endif
#include <chrono> // Adapted from rosconsole. Copyright (c) 2008, Willow Garage, Inc.
#define LOG_WARN_STREAM_THROTTLE(rate, x)                                                 \
    do                                                                                    \
    {                                                                                     \
        static double __log_stream_throttle__last_hit__ = 0.0;                            \
        std::chrono::time_point<std::chrono::system_clock> __log_stream_throttle__now__ = \
            std::chrono::system_clock::now();                                             \
        if (__log_stream_throttle__last_hit__ + rate <=                                   \
            std::chrono::duration_cast<std::chrono::seconds>(                             \
                __log_stream_throttle__now__.time_since_epoch())                          \
                .count())                                                                 \
        {                                                                                 \
            __log_stream_throttle__last_hit__ =                                           \
                std::chrono::duration_cast<std::chrono::seconds>(                         \
                    __log_stream_throttle__now__.time_since_epoch())                      \
                    .count();                                                             \
            LOG_WARN_STREAM(x);                                                           \
        }                                                                                 \
    } while (0)
#endif

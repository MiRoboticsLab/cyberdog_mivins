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

#include <chrono>
#include <string>
#include <sstream>
#include <iomanip> // std::setw
#include <ctime>   // std::localtime,

namespace rpg_common
{
    class Timer
    {
    public:
        typedef std::chrono::high_resolution_clock Clock;
        typedef std::chrono::time_point<Clock> TimePoint;
        typedef std::chrono::nanoseconds Nanoseconds;
        typedef std::chrono::seconds Seconds;

        /// The constructor directly starts the timer.
        Timer()
            : m_start_time(Clock::now()), m_duration(Nanoseconds::zero()), m_accumulated(Nanoseconds::zero())
        {
        }

        /// Starts the timer
        inline void Start()
        {
            m_start_time = Clock::now();
        }

        /// Resumes the timer. Total time can be obtained with getAccumulated().
        inline void Resume()
        {
            m_start_time = Clock::now();
        }

        /// Returns duration in seconds
        inline double Stop()
        {
            const TimePoint end_time(Clock::now());
            m_duration = std::chrono::duration_cast<Nanoseconds>(end_time - m_start_time);
            m_accumulated += m_duration;
            return static_cast<double>(m_duration.count()) * 1e-9;
        }

        /// Returns duration of last measurement in seconds
        inline double GetTime() const
        {
            return static_cast<double>(m_duration.count()) * 1e-9;
        }

        /// Returns duration of last measurement in milliseconds
        inline double GetMilliseconds() const
        {
            return static_cast<double>(m_duration.count()) * 1e-6;
        }

        /// Returns duration since the last reset or construction of the timer
        inline double GetAccumulated() const
        {
            return static_cast<double>(m_accumulated.count()) * 1e-9;
        }

        /// Reset the current timer and the accumulated
        inline void Reset()
        {
            m_start_time = TimePoint();
            m_duration = Nanoseconds::zero();
            m_accumulated = Nanoseconds::zero();
        }

        /// Get seconds since 1.1.1970
        static double GetCurrentTime()
        {
            return static_cast<double>(
                       std::chrono::duration_cast<Nanoseconds>(Clock::now() - TimePoint())
                           .count()) *
                   1e-9;
        }

        /// Get a formated string of the current time, hour, minute and second
        static std::string GetCurrentTimeStr()
        {
            std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::tm *t = std::localtime(&now);
            if (t == NULL)
                return std::string("ERROR");
            std::ostringstream ss;
            ss << t->tm_year - 100 << "-"
               << std::setw(2) << std::setfill('0') << t->tm_mon + 1 << "-"
               << std::setw(2) << std::setfill('0') << t->tm_mday << "_"
               << std::setw(2) << std::setfill('0') << t->tm_hour << "-"
               << std::setw(2) << std::setfill('0') << t->tm_min << "-"
               << std::setw(2) << std::setfill('0') << t->tm_sec;
            return ss.str();
        }

    private:
        TimePoint m_start_time;
        Nanoseconds m_duration;
        Nanoseconds m_accumulated;
    };

} // namespace rpg_common
namespace rpg = rpg_common;

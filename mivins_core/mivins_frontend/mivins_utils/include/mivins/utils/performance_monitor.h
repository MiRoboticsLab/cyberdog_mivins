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

#ifndef VIKIT_PERFORMANCE_MONITOR_H
#define VIKIT_PERFORMANCE_MONITOR_H

#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <mivins/utils/timer.h>

namespace vk
{

    struct LogItem
    {
        double data;
        bool set;
    };

    class PerformanceMonitor
    {
    public:
        PerformanceMonitor();
        ~PerformanceMonitor();
        void init(const std::string &trace_name, const std::string &trace_dir);
        void addTimer(const std::string &name);
        void addLog(const std::string &name);
        void writeToFile();
        void startTimer(const std::string &name);
        void stopTimer(const std::string &name);
        double getTime(const std::string &name) const;
        void log(const std::string &name, double data);

    private:
        std::map<std::string, Timer> timers_;
        std::map<std::string, LogItem> logs_;
        std::string trace_name_; //<! name of the thread that started the performance monitor
        std::string trace_dir_;  //<! directory where the logfiles are saved
        std::ofstream ofs_;

        void trace();
        void traceHeader();
    };

} // namespace vk

#endif // VIKIT_PERFORMANCE_MONITOR_H

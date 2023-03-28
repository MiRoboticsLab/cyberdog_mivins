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

#include <vector>
#include <string>
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "fast/fast.h"

typedef std::vector<fast::Fastxy> Corners;

namespace utils
{

    std::string getFileDir()
    {
        std::string filename(__FILE__);
        for (auto s = filename.rbegin(); s < filename.rend(); ++s)
            if (*s == '/')
                return std::string(filename.begin(), (s + 1).base());
        std::cout << "ERROR getFileDir(): could not decompose string" << std::endl;
        return std::string("/");
    }

    void showFeatures(
        const cv::Mat &img,
        const Corners &corners)
    {
        cv::Mat img_rgb(img.size(), CV_8UC3);
        cv::cvtColor(img, img_rgb, cv::COLOR_GRAY2RGB);
        for (auto c : corners)
        {
            cv::circle(img_rgb, cv::Point2i(c.x, c.y), 2, cv::Scalar(0, 255, 0));
        }
        cv::imshow("corners", img_rgb);
        cv::waitKey(0);
    }

} // namespace utils

int main(int /*argc*/, char ** /*argv*/)
{
    const int NUM_TRIALS = 100;
    const int threshold = 20;
    Corners corners;
    std::string filename = utils::getFileDir() + "/data/test.jpg";
    std::cout << "Test FAST detector on image: " << filename << std::endl;
    cv::Mat img = cv::imread(filename, 0);
    assert(!img.empty());

    // cv::imshow("input_image", img);
    // cv::waitKey(50);

    std::cout << "Testing PLAIN version" << std::endl;
    double time_accumulator = 0;
    for (int i = 0; i < NUM_TRIALS; ++i)
    {
        corners.clear();
        double t = (double)cv::getTickCount();
        fast::FastCornerDetect9((fast::fast_byte *)(img.data), img.cols, img.rows, img.cols, threshold, corners);
        time_accumulator += ((cv::getTickCount() - t) / cv::getTickFrequency());
    }
    printf("PLAIN version took %f seconds (average over %d trials).\n", time_accumulator / ((double)NUM_TRIALS), NUM_TRIALS);
    std::cout << std::endl
              << "Fast feature detector test: " << corners.size() << " features detected." << std::endl;

#if __ARM_NEON__
    std::cout << "Testing NEON version" << std::endl;
    time_accumulator = 0;
    for (int i = 0; i < NUM_TRIALS; ++i)
    {
        corners.clear();
        double t = (double)cv::getTickCount();
        fast::FastCornerDetect9Neon((fast::fast_byte *)(img.data), img.cols, img.rows, img.cols, threshold, corners);
        time_accumulator += ((cv::getTickCount() - t) / cv::getTickFrequency());
    }
    printf("NEON version took %f seconds (average over %d trials).\n", time_accumulator / ((double)NUM_TRIALS), NUM_TRIALS);
    std::cout << std::endl
              << "Fast feature detector test: " << corners.size() << " features detected." << std::endl;
#endif

#if __SSE2__
    std::cout << "Testing SSE2 version" << std::endl;
    time_accumulator = 0;
    for (int i = 0; i < NUM_TRIALS; ++i)
    {
        corners.clear();
        double t = (double)cv::getTickCount();
        fast::FastCornerDetect10Sse2((fast::fast_byte *)(img.data), img.cols, img.rows, img.cols, threshold, corners);
        time_accumulator += ((cv::getTickCount() - t) / cv::getTickFrequency());
    }
    printf("SSE2 version took %f seconds (average over %d trials).\n", time_accumulator / ((double)NUM_TRIALS), NUM_TRIALS);
    std::cout << std::endl
              << "Fast feature detector test: " << corners.size() << " features detected." << std::endl;
#endif

    // display results
    utils::showFeatures(img, corners);

    return 0;
}

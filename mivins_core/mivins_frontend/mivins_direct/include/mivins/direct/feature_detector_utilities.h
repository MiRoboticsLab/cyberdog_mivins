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

#include <memory>
#include <array>
#include <mivins/common/types.h>
#include <mivins/common/camera_fwd.h>
#include <mivins/common/occupancy_grid_2d.h>
#include <mivins/direct/feature_detector_types.h>

namespace mivins
{
  
    class ExtractorNode;
    class AbstractDetector;
    using AbstractDetectorPtr = std::shared_ptr<AbstractDetector>;

    namespace feature_detector_utils
    {

        /// Factory returns a pointer to a detector that is specified with in the options.
        AbstractDetectorPtr MakeDetector(
            const DetectorOptions &options,
            const CameraPtr &cam);

        void FillFeatures(const Corners &corners,
                          const FeatureType &type,
                          const cv::Mat &mask,
                          const double &threshold,
                          const size_t max_n_features,
                          Keypoints &keypoints,
                          Scores &scores,
                          Levels &levels,
                          Gradients &gradients,
                          FeatureTypes &types,
                          OccupandyGrid2D &grid);

        void FastDetect(
            const ImgPyramid &img_pyr,
            const int threshold,
            const int border,
            const size_t min_level,
            const size_t max_level,
            Corners &corners,
            OccupandyGrid2D &grid);

        void ShiTomasiDetect(
            const ImgPyramid &img_pyr,
            const int threshold,
            const int border,
            const size_t min_level,
            const size_t max_level,
            Corners &corners,
            OccupandyGrid2D &grid,
            OccupandyGrid2D &closeness_check_grid);

        void EdgeletDetector_V1(
            const ImgPyramid &img_pyr,
            const int threshold,
            const int border,
            const int min_level,
            const int max_level,
            Corners &corners,
            OccupandyGrid2D &grid);

        void EdgeletDetector_V2(
            const ImgPyramid &img_pyr,
            const int threshold,
            const int border,
            const int min_level,
            const int max_level,
            Corners &corners,
            OccupandyGrid2D &grid);

        bool GetCornerAngle(
            const ImgPyramid &img_pyr,
            const Eigen::Ref<const Keypoint> &level_0,
            const size_t level,
            double *angle);

        /// Get patch minimum eigenvalue.
        bool GetShiTomasiScore(
            const cv::Mat &img,
            const Eigen::Vector2i &px,
            double *score);

        void SetCornerAngles(
            const ImgPyramid &img_pyr,
            Corners *corners);

        void setCornerLevel(
            const ImgPyramid &mag_pyr,
            Corners *corners);

        void ComputeDerivMaxMagnitude(
            const cv::Mat &img_8u,
            cv::Mat &mag_8u);

        void ComputeDerivHuangMumford(
            const cv::Mat &img_8u,
            cv::Mat &mag_32f,
            float alpha = 10.0f,
            float q = 0.98f);

        void Nonmax(
            const cv::Mat &img_32f,
            const float thresh,
            Corners *corners);

        void DisplayGrid(
            const OccupandyGrid2D &old_grid,
            const Keypoints &keypoints,
            const int img_width,
            const int img_height);

        void NonlinearDiffusion(
            const cv::Mat &img_8u,
            cv::Mat &img_8u_diffused,
            const double timestep = 0.25, // absolute max is 0.5 for stability
            const double final_time = 2.5);

        void DetectGuillermoEdges(
            const cv::Mat &src_gray,
            cv::Mat &dest,
            int low_threshold = 20,
            int ratio = 3,
            int kernel_size = 3);

        void DetectCannyEdges(
            const cv::Mat &src_gray,
            cv::Mat &dest,
            int low_threshold = 20,
            int ratio = 3,
            int kernel_size = 3);

        void DetectSobelEdges(
            const cv::Mat &src_gray,
            cv::Mat &dest,
            int low_threshold = 30,
            int kernel_size = 2);

        void DrawFeatures(
            const Frame &frame,
            const size_t level,
            const bool only_matched_features,
            cv::Mat *img_rgb);

        double GetAngleAtPixelUsingHistogram(
            const cv::Mat &img,
            const Eigen::Vector2i &px,
            const size_t halfpatch_size);

        void Nonmax_3x3(
            const std::vector<Eigen::Vector2i> &corners,
            const std::vector<int> &scores,
            std::vector<int> &nonmax_corners);

        void MergeGrids(const OccupandyGrid2D &grid1, OccupandyGrid2D *grid2);
        
        bool CompareNodes(std::pair<int,ExtractorNode*>& e1, std::pair<int,ExtractorNode*>& e2);

        Corners DistributeOctTree(
            const Corners& corners, const int &min_x, const int &max_x, 
            const int &min_y, const int &max_y, const int &N);

        // Compute an angle histogram.
        namespace angle_hist
        {

            constexpr size_t n_bins = 36;
            using AngleHistogram = std::array<double, n_bins>;

            void GetAngleHistogram(
                const cv::Mat &img, int x, int y, int halfpatch_size, AngleHistogram &hist);

            bool GradientAndMagnitudeAtPixel(
                const cv::Mat &img, int x, int y, double *mag, double *angle);

            void SmoothOrientationHistogram(
                AngleHistogram &hist);

            double GetDominantAngle(
                const AngleHistogram &hist);

        } // angle_hist
    }     // feature_detector_utils
} // namespace mivins

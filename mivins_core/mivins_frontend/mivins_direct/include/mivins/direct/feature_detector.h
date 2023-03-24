// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#include <list>
#include <vector>
#include <mivins/common/types.h>
#include <mivins/common/camera_fwd.h>
#include <mivins/common/occupancy_grid_2d.h>
#include <mivins/direct/feature_detector_types.h>

namespace mivins
{

    class ExtractorNode
    {
    public:
        ExtractorNode():no_more(false){}

        void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

        std::vector<Corner> corners;
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNode>::iterator lit;
        bool no_more;
    };
    
    //------------------------------------------------------------------------------
    /// All detectors should derive from this abstract class.
    class AbstractDetector
    {
    public:
        typedef std::shared_ptr<AbstractDetector> Ptr;

        DetectorOptions m_options;

        /// Default constructor.
        AbstractDetector(
            const DetectorOptions &options,
            const CameraPtr &cam);

        /// Default destructor.
        virtual ~AbstractDetector() = default;

        // no copy
        AbstractDetector &operator=(const AbstractDetector &) = delete;
        AbstractDetector(const AbstractDetector &) = delete;

        void Detect(const FramePtr &frame);

        virtual void Detect(
            const ImgPyramid &img_pyr,
            const cv::Mat &mask,
            const size_t max_n_features,
            Keypoints &px_vec,
            Scores &score_vec,
            Levels &level_vec,
            Gradients &grad_vec,
            FeatureTypes &types_vec) = 0;

        inline void ResetGrid()
        {
            grid_.reset();
            closeness_check_grid_.reset();
        }

        // occupancy for the current feature type
        OccupandyGrid2D grid_;
        // this is to additionally check whether detected features are near some exiting ones
        // useful for having mutliple detectors
        OccupandyGrid2D closeness_check_grid_;
    };

    //------------------------------------------------------------------------------
    /// FAST detector by Edward Rosten.
    class FastDetector : public AbstractDetector
    {
    public:
        using AbstractDetector::AbstractDetector; // default constructor
        virtual ~FastDetector() = default;

        virtual void Detect(
            const ImgPyramid &img_pyr,
            const cv::Mat &mask,
            const size_t max_n_features,
            Keypoints &px_vec,
            Scores &score_vec,
            Levels &level_vec,
            Gradients &grad_vec,
            FeatureTypes &types_vec) override;
    };

    //------------------------------------------------------------------------------
    /// Detect pixels that have a high gradient magnitude over multiple pyramid levels.
    /// These gradient pixels are good for camera tracking.
    class GradientDetector : public AbstractDetector
    {
    public:
        using AbstractDetector::AbstractDetector; // default constructor
        virtual ~GradientDetector() = default;

        virtual void Detect(
            const ImgPyramid &img_pyr,
            const cv::Mat &mask,
            const size_t max_n_features,
            Keypoints &px_vec,
            Scores &score_vec,
            Levels &level_vec,
            Gradients &grad_vec,
            FeatureTypes &types_vec) override;
    };

    //------------------------------------------------------------------------------
    /// Detect pixels that have a high gradient magnitude over multiple pyramid levels.
    /// These gradient pixels are good for camera tracking.
    class GradientDetectorGrid : public AbstractDetector
    {
    public:
        using AbstractDetector::AbstractDetector; // default constructor
        virtual ~GradientDetectorGrid() = default;

        virtual void Detect(
            const ImgPyramid &img_pyr,
            const cv::Mat &mask,
            const size_t max_n_features,
            Keypoints &px_vec,
            Scores &score_vec,
            Levels &level_vec,
            Gradients &grad_vec,
            FeatureTypes &types_vec) override;
    };

    //------------------------------------------------------------------------------
    /// @todo
    class FastGradDetector : public AbstractDetector
    {
    public:
        using AbstractDetector::AbstractDetector; // default constructor
        virtual ~FastGradDetector() = default;

        virtual void Detect(
            const ImgPyramid &img_pyr,
            const cv::Mat &mask,
            const size_t max_n_features,
            Keypoints &px_vec,
            Scores &score_vec,
            Levels &level_vec,
            Gradients &grad_vec,
            FeatureTypes &types_vec) override;
    };

    //------------------------------------------------------------------------------
    /// shitomasi detector with gradient detector
    class ShiTomasiGradDetector : public AbstractDetector
    {
    public:
        using AbstractDetector::AbstractDetector; // default constructor
        virtual ~ShiTomasiGradDetector() = default;

        virtual void Detect(
            const ImgPyramid &img_pyr,
            const cv::Mat &mask,
            const size_t max_n_features,
            Keypoints &px_vec,
            Scores &score_vec,
            Levels &level_vec,
            Gradients &grad_vec,
            FeatureTypes &types_vec) override;
    };

    //------------------------------------------------------------------------------
    /// shitomasi detector: This should be called after other detectors. The intention
    /// of this detector is to get some extra features for loop closing. It detects shitomasi
    /// features and keeps the ones which are a minimum distance away from existing corners.
    class ShiTomasiDetector : public AbstractDetector
    {
    public:
        using AbstractDetector::AbstractDetector; // default constructor
        virtual ~ShiTomasiDetector() = default;

        virtual void Detect(
            const ImgPyramid &img_pyr,
            const cv::Mat &mask,
            const size_t max_n_features,
            Keypoints &px_vec,
            Scores &score_vec,
            Levels &level_vec,
            Gradients &grad_vec,
            FeatureTypes &types_vec) override;
    };

    //------------------------------------------------------------------------------
    /// Dummy detector that selects all pixels
    class AllPixelsDetector : public AbstractDetector
    {
    public:
        using AbstractDetector::AbstractDetector; // default constructor
        virtual ~AllPixelsDetector() = default;

        virtual void Detect(
            const ImgPyramid &img_pyr,
            const cv::Mat &mask,
            const size_t max_n_features,
            Keypoints &px_vec,
            Scores &score_vec,
            Levels &level_vec,
            Gradients &grad_vec,
            FeatureTypes &types_vec) override;
    };

    //------------------------------------------------------------------------------
    /// Detect pixels that have strong gradients according to the paper
    /// Huang, J. and Mumford, D. (1999). Statistics of natural images and models. (CVPR)
    class GradientHuangMumfordDetector : public AbstractDetector
    {
    public:
        using AbstractDetector::AbstractDetector; // default constructor
        virtual ~GradientHuangMumfordDetector() = default;

        virtual void Detect(
            const ImgPyramid &img_pyr,
            const cv::Mat &mask,
            const size_t max_n_features,
            Keypoints &px_vec,
            Scores &score_vec,
            Levels &level_vec,
            Gradients &grad_vec,
            FeatureTypes &types_vec) override;
    };

    class CannyDetector : public AbstractDetector
    {
    public:
        using AbstractDetector::AbstractDetector; // default constructor
        virtual ~CannyDetector() = default;

        virtual void Detect(
            const ImgPyramid &img_pyr,
            const cv::Mat &mask,
            const size_t max_n_features,
            Keypoints &px_vec,
            Scores &score_vec,
            Levels &level_vec,
            Gradients &grad_vec,
            FeatureTypes &types_vec) override;
    };

    class SobelDetector : public AbstractDetector
    {
    public:
        using AbstractDetector::AbstractDetector; // default constructor
        virtual ~SobelDetector() = default;

        virtual void Detect(
            const ImgPyramid &img_pyr,
            const cv::Mat &mask,
            const size_t max_n_features,
            Keypoints &px_vec,
            Scores &score_vec,
            Levels &level_vec,
            Gradients &grad_vec,
            FeatureTypes &types_vec) override;
    };
    
    class SobelGradDetector : public AbstractDetector
    {
    public:
        using AbstractDetector::AbstractDetector; // default constructor
        virtual ~SobelGradDetector() = default;

        virtual void Detect(
            const ImgPyramid& img_pyr,
            const cv::Mat& mask,
            const size_t max_n_features,
            Keypoints& px_vec,
            Scores& score_vec,
            Levels& level_vec,
            Gradients& grad_vec,
            FeatureTypes& types_vec) override;
    };

} // namespace mivins

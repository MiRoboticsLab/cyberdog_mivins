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

#include <list>
#include <vector>
#include <string>
//#include <cmath>      // sin, cos
#include <memory>    // shared_ptr
#include <stdexcept> // assert, runtime_error

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <mivins/utils/performance_monitor.h>

#include <mivins/common/logging.h>
#include <mivins/common/types.h>
#include <mivins/common/camera.h>
#include <mivins/common/transformation.h>
//#include <mivins/common/frame.h>

namespace mivins
{
    //using namespace Eigen;
    using Eigen::Matrix;
    using Eigen::Matrix2d;
    using Eigen::Matrix2f;
    using Eigen::Matrix3d;
    using Eigen::Vector2d;
    using Eigen::Vector2f;
    using Eigen::Vector2i;
    using Eigen::Vector3d;

    typedef std::shared_ptr<vk::PerformanceMonitor> PerformanceMonitorPtr;

    extern PerformanceMonitorPtr g_permon;
#define SVO_LOG(name, value) g_permon->log(std::string(name), (value))
#define SVO_START_TIMER(name) g_permon->startTimer((name))
#define SVO_STOP_TIMER(name) g_permon->stopTimer((name))

    // forward declaration of modules
    class Frame;
    typedef std::shared_ptr<Frame> FramePtr;
    typedef std::weak_ptr<Frame> FrameWeakPtr;
    class FrameBundle;
    typedef std::shared_ptr<FrameBundle> FrameBundlePtr;
    struct Feature;
    typedef std::shared_ptr<Feature> FeaturePtr;
    typedef std::weak_ptr<Feature> FeatureWeakPtr;
    class Point;
    typedef std::shared_ptr<Point> PointPtr;
    class ChannelImu;
    typedef std::shared_ptr<ChannelImu> ImuHandlerPtr;
    class ChannelOdom;
    typedef std::shared_ptr<ChannelOdom> OdomHandlerPtr;
    class SparseImgAlignBase;
    typedef std::shared_ptr<SparseImgAlignBase> SparseImgAlignBasePtr;
    class FrontendLocalMap;
    typedef std::shared_ptr<FrontendLocalMap> FrontendMapPtr;
    class PatchMatcher;
    typedef std::shared_ptr<PatchMatcher> PatchMatcherPtr;
    class SeedInverse;
    typedef SeedInverse SeedImplementation;
    typedef std::shared_ptr<SeedImplementation> SeedPtr;
    typedef std::vector<SeedPtr> Seeds;
    class AbstractDetector;
    typedef std::shared_ptr<AbstractDetector> DetectorPtr;
    typedef std::vector<cv::Mat> ImgPyramid;
    typedef std::vector<FeaturePtr> Features;
    class BundleAdjustment;
    typedef std::shared_ptr<BundleAdjustment> BundleAdjustmentPtr;

    enum class BundleAdjustmentType
    {
        kNone,
        kGtsam,
        kCeres
    };

    struct DetectorOptions;
    struct DepthOptimizationOptions;
    struct ReprojectorOptions;
    struct InitializationOptions;
    struct FeatureTrackerOptions;
    struct LoopClosureOptions;
    class AbstractInitialization;
    typedef std::unique_ptr<AbstractInitialization> InitializerPtr;
    class PoseOptimizer;
    typedef std::unique_ptr<PoseOptimizer> PoseOptimizerPtr;
    class SparseImgAlign;
    typedef std::unique_ptr<SparseImgAlign> SparseImgAlignPtr;
    class DepthOptimization;
    typedef std::unique_ptr<DepthOptimization> DepthOptimizationPtr;
    class Reprojector;
    typedef std::unique_ptr<Reprojector> ReprojectorPtr;
    class GlobalMap;
    typedef std::shared_ptr<GlobalMap> GlobalMapPtr;

} // namespace mivins

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

#include <queue>
#include <memory> // std::shared_ptr
#include <mutex>
#include <thread>
#include <condition_variable>
#include <mivins/utils/performance_monitor.h>
#include <mivins/direct/patch_matcher.h>

namespace mivins
{

    // forward declarations
    class AbstractDetector;
    typedef std::shared_ptr<AbstractDetector> DetectorPtr;
    struct DetectorOptions;

    /// Depth-filter config parameters
    struct DepthOptimizationOptions
    {
        /// Threshold for the uncertainty of the seed. If seed's sigma2 is thresh
        /// smaller than the inital sigma, it is considered as converged.
        /// Default value is 200. If seeds should converge quicker, set it to 50 or
        /// if you want very precise 3d points, set it higher.
        double seed_convergence_sigma2_thresh = 200.0;

        /// Threshold for map point seeds convergence. Should be higher to make sure
        /// we have an accurate map (for loop closing).
        double mappoint_convergence_sigma2_thresh = 500.0;

        /// Use inverse-depth parametrization for seeds.
        /// Default is true. Set to false if you are using the depth-filter to
        /// reconstruct small objects with fixed depth range.
        bool use_inverse_depth = true;

        /// Specify the max pyramid level for the matcher.
        /// Normally, you don't need to change this parameters.
        size_t max_search_level = 2;

        /// Show additional debug output
        bool verbose = false;

        /// Start separate thread for seed updates
        bool use_threaded_depthfilter = true;

        /// Update the 3D point linked by the feature in the seed (false for REMODE-CPU)
        bool update_3d_point = true;

        /// Do epipolar search on unit sphere
        bool scan_epi_unit_sphere = false;

        /// Restrict number of features per frame.
        size_t max_n_seeds_per_frame = 200;

        size_t max_map_seeds_per_frame = 200;

        /// use affine model to compensate for brightness change
        bool affine_est_offset = true;
        bool affine_est_gain = false;
        bool save_time_consumption = false;
        bool trace_statistics = false;
        std::string trace_dir = "/tmp";
        ///
        bool extra_map_points = false;
        bool use_init_depth = false;
    };

    /// Depth filter implements the Bayesian Update proposed in:
    /// "Video-based, Real-Time Multi View Stereo" by G. Vogiatzis and C. Hern??ndez.
    /// In Image and Vision Computing, 29(7):434-441, 2011.
    class DepthOptimization
    {
    protected:
        /// FilterJob for multi-threading. can either be to update a seed with the frame
        /// or to initialize new seeds in the frame
        struct FilterJob
        {
            enum Type
            {
                UPDATE,
                SEED_INIT
            } type;
            FramePtr cur_frame;
            FramePtr ref_frame;
            size_t ref_frame_seed_index;
            double min_depth, max_depth, mean_depth;

            /// Default constructor
            FilterJob()
                : cur_frame(nullptr), ref_frame(nullptr)
            {
            }

            /// Constructor for seed update
            FilterJob(const FramePtr &_cur_frame, const FramePtr &_ref_frame, const size_t _ref_index)
                : type(UPDATE), cur_frame(_cur_frame), ref_frame(_ref_frame), ref_frame_seed_index(_ref_index)
            {
            }

            /// Constructor for seed initialization
            FilterJob(const FramePtr &f, double min_d, double max_d, double mean_d)
                : type(SEED_INIT), cur_frame(f), ref_frame(nullptr), min_depth(min_d), max_depth(max_d), mean_depth(mean_d)
            {
            }
        };

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef std::shared_ptr<DepthOptimization> Ptr;
        typedef std::mutex MutexT;
        typedef std::unique_lock<MutexT> ULockT;
        typedef std::queue<FilterJob> FilterJobQueue;

        DepthOptimizationOptions m_opt_options;
        

        /// Default Constructor
        DepthOptimization(
            const DepthOptimizationOptions &options,
            const DetectorOptions &detector,
            const std::shared_ptr<CameraBundle> &cams);

        /// Constructor for REMODE-CPU
        DepthOptimization(
            const DepthOptimizationOptions &options);

        /// Destructor stops thread if necessary.
        virtual ~DepthOptimization();

        /// Start this thread when seed updating should be in a parallel thread.
        void StartFilterThread();

        /// Stop the parallel thread that is running.
        void StopFilterThread();

        /// Add new keyframe to the queue, depth_max is only required for direct-depth
        void AddKeyframe(
            const FramePtr &frame,
            const double mean_depth,
            const double min_depth,
            const double max_depth);

        /// Resets all jobs of the parallel thread
        void Reset();

        /// test
        PatchMatcher &GetMatcher() { return *m_matcher; }

        /// Update seeds
        /// \param frames_with_seeds List of frames which contain seeds that should be
        /// updated with new_frame
        /// \param new_frame The new frame that is used as observation to update all
        /// seeds in frames_with_seeds
        size_t SeedsUpdate(
            const std::vector<FramePtr> &frames_with_seeds,
            const FramePtr &new_frame);

        // need public access to set grid occupancy
        MutexT m_feature_detector_mut;
        DetectorPtr m_feature_detector;
        DetectorPtr m_sec_feature_detector; // for extra points used for loop closing

    protected:
        MutexT m_jobs_mut;
        FilterJobQueue m_jobs;
        std::condition_variable m_jobs_condvar;
        std::unique_ptr<std::thread> m_thread;
        bool m_quit_thread = false;
        PatchMatcher::Ptr m_matcher;

        /// A thread that is continuously updating the seeds.
        void SeedsUpdateLoop();
    };

    namespace depth_optimization_utils
    {

        /// Initialize new seeds from a frame.
        void SeedsInitialization(
            const FramePtr &frame,
            const DetectorPtr &feature_detector,
            const size_t max_n_seeds,
            const float min_depth,
            const float max_depth,
            const float mean_depth,
            const bool use_depth_prior);

        /// Update Seed
        bool SingleSeedUpdate(
            const Frame &cur_frame,
            Frame &ref_frame,
            const size_t &seed_index,
            PatchMatcher &matcher,
            const FloatType sigma2_convergence_threshold,
            const bool check_visibility = true,
            const bool check_convergence = false,
            const bool use_vogiatzis_update = true);

        bool VogiatzisModelUpdate(
            const FloatType z,
            const FloatType tau2,
            const FloatType z_range,
            Eigen::Ref<SeedState> &seed);

        bool GaussianModelUpdate(
            const FloatType z,
            const FloatType tau2,
            Eigen::Ref<SeedState> &seed);

        /// Compute the uncertainty of the measurement.
        double UncertaintyComputation(
            const Transformation &T_ref_cur,
            const BearingVector &f,
            const FloatType z,
            const FloatType px_error_angle);

        double EpiGradAngleComputation(
            const Transformation &T_cur_ref,
            const BearingVector &f_ref,
            const GradientVector &grad,
            const FloatType depth_estimate);

#ifdef SVO_USE_PHOTOMETRIC_DISPARITY_ERROR
        bool SeedCovarianceSet(
            const int halfpatch_size,
            const double image_noise2,
            SeedImplementation::Ptr seed);

        double GetSeedDisparityUncertainty(
            const SeedImplementation::Ptr &seed,
            const Transformation &T_cur_ref);
#endif

    } // namespace depth_optimization_utils

} // namespace mivins

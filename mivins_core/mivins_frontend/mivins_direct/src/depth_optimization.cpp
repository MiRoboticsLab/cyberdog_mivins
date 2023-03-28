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

#include <mivins/direct/depth_optimization.h>

#include <algorithm>
#include <mivins/utils/math_utils.h>
#include <mivins/utils/cv_utils.h>
#include <mivins/common/camera.h>
#include <mivins/common/frame.h>
#include <mivins/common/point.h>
#include <mivins/common/logging.h>
#include <mivins/common/seed.h>
#include <mivins/direct/patch_matcher.h>
#include <mivins/direct/feature_detector.h>
#include <mivins/direct/feature_detector_utilities.h>
#include <mivins/mivins_global_types.h>

namespace mivins
{
    PerformanceMonitorPtr extract_feature_permon;
    PerformanceMonitorPtr update_filter_permon;

    DepthOptimization::DepthOptimization(
        const DepthOptimizationOptions &options,
        const DetectorOptions &detector_options,
        const CameraBundle::Ptr &cams)
        : DepthOptimization(options)
    {
        // TODO: make a detector for every camera!
        m_feature_detector =
            feature_detector_utils::MakeDetector(detector_options, cams->getCameraShared(0));
        m_sec_feature_detector.reset();
        if (m_opt_options.extra_map_points)
        {
            DetectorOptions sec_detector_options = detector_options;
            sec_detector_options.detector_type = DetectorType::kShiTomasi;
            m_sec_feature_detector =
                feature_detector_utils::MakeDetector(
                    sec_detector_options, cams->getCameraShared(0));
        }
       
        if (m_opt_options.trace_statistics)
        {
            std::string path = getenv("HOME") + m_opt_options.trace_dir;
            extract_feature_permon.reset(new vk::PerformanceMonitor());
            extract_feature_permon->addLog("extract_feature");
            extract_feature_permon->addLog("extract_feature_timestamp");
            extract_feature_permon->init("trace_extract_feature", path);

            update_filter_permon.reset(new vk::PerformanceMonitor());
            update_filter_permon->addLog("update_filter");
            update_filter_permon->addLog("update_filter_timestamp");
            update_filter_permon->init("trace_update_filter", path);
        }
    }

    DepthOptimization::DepthOptimization(
        const DepthOptimizationOptions &options)
        : m_opt_options(options), m_matcher(new PatchMatcher())
    {
        LOG_INFO_STREAM("DepthOptimization: created.");
        m_matcher->m_patch_matcher_options.scan_on_unit_sphere = options.scan_epi_unit_sphere;
        m_matcher->m_patch_matcher_options.affine_est_offset_ = options.affine_est_offset;
        m_matcher->m_patch_matcher_options.affine_est_gain_ = options.affine_est_gain;
        if (m_opt_options.use_threaded_depthfilter)
            StartFilterThread();
    }

    DepthOptimization::~DepthOptimization()
    {
        StopFilterThread();
        LOG_INFO_STREAM("DepthOptimization: destructed.");
    }

    void DepthOptimization::StartFilterThread()
    {
        if (m_thread)
        {
            LOG_ERROR_STREAM("DepthOptimization: Thread already started!");
            return;
        }
        LOG_INFO_STREAM("DepthOptimization: Start thread.");
        m_thread.reset(new std::thread(&DepthOptimization::SeedsUpdateLoop, this));
    }

    void DepthOptimization::StopFilterThread()
    {
        LOG_DEBUG_STREAM("DepthOptimization: stop thread invoked.");
        if (m_thread != nullptr)
        {
            LOG_DEBUG_STREAM("DepthOptimization: interrupt and join thread... ");
            m_quit_thread = true;
            m_jobs_condvar.notify_all();
            m_thread->join();
            m_thread.reset();
        }
    }

    void DepthOptimization::AddKeyframe(
        const FramePtr &frame,
        const double mean_depth,
        const double min_depth,
        const double max_depth)
    {
        // allocate memory for new features.
        frame->ResizeFeatureStorage(
            frame->num_features_ + m_feature_detector->grid_.size() +
            (m_sec_feature_detector ? m_sec_feature_detector->closeness_check_grid_.size() : 0u));

        if (m_thread == nullptr)
        {
            ULockT lock(m_feature_detector_mut);
            // TicToc t_extract_feature;
            vk::Timer timer;
            if (m_opt_options.trace_statistics)
            {
                timer.start();
            }
            depth_optimization_utils::SeedsInitialization(
                frame, m_feature_detector, m_opt_options.max_n_seeds_per_frame,
                min_depth, max_depth, mean_depth, m_opt_options.use_init_depth);
            
            if (m_opt_options.trace_statistics)
            {
                extract_feature_permon->log("extract_feature", timer.stop());
                extract_feature_permon->log("extract_feature_timestamp", frame->GetTimestampNSec());
                extract_feature_permon->writeToFile();
            }
            if (m_opt_options.extra_map_points)
            {
                for (size_t idx = 0; idx < frame->NumFeatures(); idx++)
                {
                    const FeatureType &type = frame->type_vec_[idx];
                    if (!isMapPoint(type) && type != FeatureType::kOutlier)
                    {
                        m_sec_feature_detector->closeness_check_grid_.fillWithKeypoints(
                            frame->px_vec_.col(static_cast<int>(idx)));
                    }
                }
                depth_optimization_utils::SeedsInitialization(
                    frame, m_sec_feature_detector,
                    m_opt_options.max_n_seeds_per_frame + m_opt_options.max_map_seeds_per_frame,
                    min_depth, max_depth, mean_depth, m_opt_options.use_init_depth);
            }
        }
        else
        {
            ULockT lock(m_jobs_mut);

            // clear all other jobs, this one has priority
            while (!m_jobs.empty())
                m_jobs.pop();
            m_jobs.push(FilterJob(frame, min_depth, max_depth, mean_depth));
            m_jobs_condvar.notify_all();
        }
    }

    void DepthOptimization::Reset()
    {
        ULockT lock(m_jobs_mut);
        while (!m_jobs.empty())
            m_jobs.pop();
        LOG_INFO_STREAM("DepthOptimization: RESET.");
    }

    void DepthOptimization::SeedsUpdateLoop()
    {
        while (true)
        {
            // wait for new jobs
            FilterJob job;
            {
                ULockT lock(m_jobs_mut);
                while (m_jobs.empty() && !m_quit_thread)
                    m_jobs_condvar.wait(lock);

                if (m_quit_thread)
                    return;

                job = m_jobs.front();
                m_jobs.pop();
            } // release lock

            // process jobs
            if (job.type == FilterJob::SEED_INIT)
            {
                ULockT lock(m_feature_detector_mut);
                depth_optimization_utils::SeedsInitialization(
                    job.cur_frame, m_feature_detector,
                    m_opt_options.max_n_seeds_per_frame,
                    job.min_depth, job.max_depth, job.mean_depth, m_opt_options.use_init_depth);
                if (m_opt_options.extra_map_points)
                {
                    for (size_t idx = 0; idx < job.cur_frame->NumFeatures(); idx++)
                    {
                        const FeatureType &type = job.cur_frame->type_vec_[idx];
                        if (!isMapPoint(type) && type != FeatureType::kOutlier)
                        {
                            m_sec_feature_detector->closeness_check_grid_.fillWithKeypoints(
                                job.cur_frame->px_vec_.col(static_cast<int>(idx)));
                        }
                    }
                    depth_optimization_utils::SeedsInitialization(
                        job.cur_frame, m_sec_feature_detector,
                        m_opt_options.max_n_seeds_per_frame + m_opt_options.max_map_seeds_per_frame,
                        job.min_depth, job.max_depth, job.mean_depth, m_opt_options.use_init_depth);
                }
            }
            else if (job.type == FilterJob::UPDATE)
            {
                // We get higher precision (10x in the synthetic blender dataset)
                // when we keep updating seeds even though they are converged until
                // the frame handler selects a new keyframe.
                depth_optimization_utils::SingleSeedUpdate(
                    *job.cur_frame, *job.ref_frame, job.ref_frame_seed_index, *m_matcher,
                    m_opt_options.seed_convergence_sigma2_thresh, true, false);
            }
        }
    }

    size_t DepthOptimization::SeedsUpdate(
        const std::vector<FramePtr> &ref_frames_with_seeds,
        const FramePtr &cur_frame)
    {
        size_t n_success = 0;
        if (m_thread == nullptr)
        {
            for (const FramePtr &ref_frame : ref_frames_with_seeds)
            {
                // TicToc t_update_filter;
                vk::Timer timer;
                if (m_opt_options.trace_statistics)
                {
                    timer.start();
                }
                for (size_t i = 0; i < ref_frame->num_features_; ++i)
                {
                    const FeatureType &type = ref_frame->type_vec_[i];
                    if (isSeed(type))
                    {
                        double cur_thresh = m_opt_options.seed_convergence_sigma2_thresh;
                        // we use a different threshold for map points to get better accuracy
                        if (type == FeatureType::kMapPointSeed ||
                            type == FeatureType::kMapPointSeedConverged)
                        {
                            cur_thresh = m_opt_options.mappoint_convergence_sigma2_thresh;
                        }
                        // We get higher precision (10x in the synthetic blender dataset)
                        // when we keep updating seeds even though they are converged until
                        // the frame handler selects a new keyframe.
                        if (depth_optimization_utils::SingleSeedUpdate(
                                *cur_frame, *ref_frame, i, *m_matcher, cur_thresh, true, false))
                        {
                            ++n_success;
                        }
                    }
                }
                if (m_opt_options.trace_statistics)
                {
                    update_filter_permon->log("update_filter", timer.stop());
                    update_filter_permon->log("update_filter_timestamp", cur_frame->GetTimestampNSec());
                    update_filter_permon->writeToFile();
                }
            }
            LOG_DEBUG_STREAM("DepthOptimization: " << cur_frame->cam()->getLabel() << " updated "
                                                   << n_success << " Seeds successfully.");
        }
        else
        {
            ULockT lock(m_jobs_mut);
            for (const FramePtr &ref_frame : ref_frames_with_seeds)
            {
                for (size_t i = 0; i < ref_frame->num_features_; ++i)
                {
                    if (isSeed(ref_frame->type_vec_[i]))
                    {
                        m_jobs.push(FilterJob(cur_frame, ref_frame, i));
                    }
                }
            }
            m_jobs_condvar.notify_all();
        }
        return n_success;
    }

    namespace depth_optimization_utils
    {

        void SeedsInitialization(
            const FramePtr &frame,
            const AbstractDetector::Ptr &feature_detector,
            const size_t max_n_seeds,
            const float min_depth,
            const float max_depth,
            const float mean_depth,
            const bool use_depth_prior)
        {
            // Detect new features.
            Keypoints new_px;
            Scores new_scores;
            Levels new_levels;
            Gradients new_grads;
            FeatureTypes new_types;
            Bearings new_f;

            //! @todo (MWE) FIXME - When we (ab)use the init seeds to initialize the first
            //! seeds the check px_vec_.cols()<(n_new+n_old) will fail but in case we don't
            //! have any old features we don't have concurrency issues and we just do it;
            //! still we should resolve this at some point and have a clean init / update procedure
            //!
            //! maybe the detector should take over more of these things as the detectors
            //! know what's going on -> but take care to be thread-safe
            //!
            bool no_features_in_frame = (frame->NumFeatures() == 0);

            const int max_n_features = max_n_seeds - frame->NumFeatures();
            if (max_n_features <= 0)
            {
                VLOG(3) << "Skip seed initialization. Have already enough features.";
                return;
            }
            if (no_features_in_frame)
            {
                ///TODO remove
                frame->ClearFeatureStorage();
                CHECK_EQ(frame->px_vec_.size(), 0);

                feature_detector->Detect(
                    frame->img_pyr_, frame->GetMask(), max_n_features, frame->px_vec_,
                    frame->score_vec_, frame->level_vec_, frame->grad_vec_, frame->type_vec_);

                frame->num_features_ = frame->px_vec_.cols();
                frame->invmu_sigma2_a_b_vec_.resize(Eigen::NoChange, frame->NumFeatures());
                frame->landmark_vec_.resize(frame->px_vec_.cols(), nullptr);
                frame->seed_ref_vec_.resize(frame->px_vec_.cols());

                // compute and normalize bearing vectors
                frame_utils::ComputeNormalizedBearingVectors(frame->px_vec_,
                                                             *frame->cam(), &frame->f_vec_);
                for (size_t i = 0; i < frame->num_features_; ++i)
                {
                    if (frame->type_vec_[i] == FeatureType::kCorner)
                        frame->type_vec_[i] = FeatureType::kCornerSeed;
                    else if (frame->type_vec_[i] == FeatureType::kEdgelet)
                        frame->type_vec_[i] = FeatureType::kEdgeletSeed;
                    else if (frame->type_vec_[i] == FeatureType::kMapPoint)
                        frame->type_vec_[i] = FeatureType::kMapPointSeed;
                    else
                        LOG(FATAL) << "Unknown feature types.";
                }
            }
            else
            {
                feature_detector->Detect(
                    frame->img_pyr_, frame->GetMask(), max_n_features, new_px, new_scores,
                    new_levels, new_grads, new_types);
                frame_utils::ComputeNormalizedBearingVectors(new_px, *frame->cam(), &new_f);
            }

            // Add features to frame.
            const size_t n_old = frame->num_features_;
            const size_t n_new = new_px.cols();

            CHECK_GE(frame->px_vec_.cols(), static_cast<int>(n_new + n_old));
            frame->px_vec_.middleCols(n_old, n_new) = new_px;
            frame->f_vec_.middleCols(n_old, n_new) = new_f;
            frame->grad_vec_.middleCols(n_old, n_new) = new_grads;
            frame->score_vec_.segment(n_old, n_new) = new_scores;
            frame->level_vec_.segment(n_old, n_new) = new_levels;
            for (size_t i = 0, j = n_old; i < n_new; ++i, ++j)
            {
                if (new_types[i] == FeatureType::kCorner)
                    frame->type_vec_[j] = FeatureType::kCornerSeed;
                else if (new_types[i] == FeatureType::kEdgelet)
                    frame->type_vec_[j] = FeatureType::kEdgeletSeed;
                else if (new_types[i] == FeatureType::kMapPoint)
                    frame->type_vec_[j] = FeatureType::kMapPointSeed;
                else
                    LOG(FATAL) << "Unknown feature types.";
            }
            frame->num_features_ = n_old + n_new;

            // initialize seeds
            frame->seed_mu_range_ = seed::getMeanRangeFromDepthMinMax(min_depth, max_depth);
            //use_depth_prior    放置外面参数
            int old_total = n_old;
            int new_total = n_new;
            if (no_features_in_frame)
            {
                if (!frame->depth_image_.empty() && use_depth_prior && old_total != 0)
                { //DEPTHTODO
                    const float depth_img_min = frame->cam_->getDepthMin();
                    const float depth_img_max = frame->cam_->getDepthMax();
                    const float depth_img_scale = frame->cam_->getDepthScale();

                    if (depth_img_min == 0.0f || depth_img_max == 0.0f || depth_img_scale == 0.0f)
                    {
                        std::cout << "[IN DepthOptimization]depth min: " << depth_img_min << "; "
                                  << "[IN DepthOptimization]depth max: " << depth_img_max << "; "
                                  << "[IN DepthOptimization]depth scale: " << depth_img_scale << "\n";
                        std::cout << "[IN DepthOptimization]depth_img_min, depth_img_max and depth_img_scale shouldn't be 0.0f\n";
                        exit(-1);
                    }
                    for (int n = 0; n < old_total; n++)
                    {
                        double p_u, p_v;
                        p_u = frame->px_vec_(0, n);
                        p_v = frame->px_vec_(1, n);
                        float depth = frame->GetValidDepthFromImage(round(p_v), round(p_u));
                        if (depth >= depth_img_min && depth <= depth_img_max)
                        {                                                                                                 // TODO param
                            FloatType seed_mu_range_temp = seed::getMeanRangeFromDepthMinMax(0.95 * depth, 1.05 * depth); // TODO param
                            frame->invmu_sigma2_a_b_vec_.block(0, n, 1, 1).setConstant(seed::getMeanFromDepth(depth));
                            frame->invmu_sigma2_a_b_vec_.block(1, n, 1, 1).setConstant(seed::getInitSigma2FromMuRange(seed_mu_range_temp));
                            frame->invmu_sigma2_a_b_vec_.block(2, n, 2, 1).setConstant(10.0);
                        }
                        else
                        {
                            frame->invmu_sigma2_a_b_vec_.block(0, n, 1, 1).setConstant(seed::getMeanFromDepth(mean_depth));
                            frame->invmu_sigma2_a_b_vec_.block(1, n, 1, 1).setConstant(seed::getInitSigma2FromMuRange(frame->seed_mu_range_));
                            frame->invmu_sigma2_a_b_vec_.block(2, n, 2, 1).setConstant(10.0);
                        }
                    }
                }
                else
                {
                    frame->invmu_sigma2_a_b_vec_.block(0, 0, 1, old_total).setConstant(seed::getMeanFromDepth(mean_depth));
                    frame->invmu_sigma2_a_b_vec_.block(1, 0, 1, old_total).setConstant(seed::getInitSigma2FromMuRange(frame->seed_mu_range_));
                    frame->invmu_sigma2_a_b_vec_.block(2, 0, 2, old_total).setConstant(10.0);
                }
            }
            else
            {
                if (!frame->depth_image_.empty() && use_depth_prior && new_total != 0)
                { //DEPTHTODO
                    const float depth_img_min = frame->cam_->getDepthMin();
                    const float depth_img_max = frame->cam_->getDepthMax();
                    const float depth_img_scale = frame->cam_->getDepthScale();

                    if (depth_img_min == 0.0f || depth_img_max == 0.0f || depth_img_scale == 0.0f)
                    {
                        std::cout << "[IN DepthOptimization]depth min: " << depth_img_min << "; "
                                  << "[IN DepthOptimization]depth max: " << depth_img_max << "; "
                                  << "[IN DepthOptimization]depth scale: " << depth_img_scale << "\n";
                        std::cout << "[IN DepthOptimization]depth_img_min, depth_img_max and depth_img_scale shouldn't be 0.0f\n";
                        exit(-1);
                    }
                    for (int n = 0; n < new_total; n++)
                    {
                        double p_u, p_v;
                        p_u = frame->px_vec_(0, old_total + n);
                        p_v = frame->px_vec_(1, old_total + n);
                        float depth = frame->GetValidDepthFromImage(round(p_v), round(p_u));
                        if (depth >= depth_img_min && depth <= depth_img_max)
                        {                                                                                                 // TODO param
                            FloatType seed_mu_range_temp = seed::getMeanRangeFromDepthMinMax(0.95 * depth, 1.05 * depth); // TODO param
                            frame->invmu_sigma2_a_b_vec_.block(0, old_total + n, 1, 1).setConstant(seed::getMeanFromDepth(depth));
                            frame->invmu_sigma2_a_b_vec_.block(1, old_total + n, 1, 1).setConstant(seed::getInitSigma2FromMuRange(seed_mu_range_temp));
                            frame->invmu_sigma2_a_b_vec_.block(2, old_total + n, 2, 1).setConstant(10.0);
                        }
                        else
                        {
                            frame->invmu_sigma2_a_b_vec_.block(0, old_total + n, 1, 1).setConstant(seed::getMeanFromDepth(mean_depth));
                            frame->invmu_sigma2_a_b_vec_.block(1, old_total + n, 1, 1).setConstant(seed::getInitSigma2FromMuRange(frame->seed_mu_range_));
                            frame->invmu_sigma2_a_b_vec_.block(2, old_total + n, 2, 1).setConstant(10.0);
                        }
                    }
                }
                else
                {
                    frame->invmu_sigma2_a_b_vec_.block(0, old_total, 1, new_total).setConstant(seed::getMeanFromDepth(mean_depth));
                    frame->invmu_sigma2_a_b_vec_.block(1, old_total, 1, new_total).setConstant(seed::getInitSigma2FromMuRange(frame->seed_mu_range_));
                    frame->invmu_sigma2_a_b_vec_.block(2, old_total, 2, new_total).setConstant(10.0);
                }
            }

            LOG_DEBUG_STREAM("DepthOptimization: " << frame->cam()->getLabel() << " Initialized " << new_total << " new seeds");
        }

        bool SingleSeedUpdate(
            const Frame &cur_frame,
            Frame &ref_frame,
            const size_t &seed_index,
            PatchMatcher &matcher,
            const FloatType sigma2_convergence_threshold,
            const bool check_visibility,
            const bool check_convergence,
            const bool use_vogiatzis_update)
        {
            if (cur_frame.GetFrameId() == ref_frame.GetFrameId())
            {
                LOG_WARN_STREAM_THROTTLE(1.0, "update seed with ref frame");
                return false;
            }

            constexpr double px_noise = 1.0;
            static double px_error_angle = cur_frame.GetAngleError(px_noise);

            // check if seed is diverged
            const FeatureType type = ref_frame.type_vec_[seed_index];
            if (type == FeatureType::kOutlier)
            {
                return false;
            }

            // check if already converged
            if ((type == FeatureType::kCornerSeedConverged ||
                 type == FeatureType::kEdgeletSeedConverged ||
                 type == FeatureType::kMapPointSeedConverged) &&
                check_convergence)
            {
                return false;
            }

            // Create wrappers
            FeatureWrapper ref_ftr = ref_frame.GetFeatureWrapper(seed_index);
            Eigen::Ref<SeedState> state = ref_frame.invmu_sigma2_a_b_vec_.col(seed_index);

            // check if point is visible in the current image
            Transformation T_cur_ref = cur_frame.T_f_w_ * ref_frame.T_f_w_.Inverse();
            if (check_visibility)
            {
                const Eigen::Vector3d xyz_f(T_cur_ref * (seed::getDepth(state) * ref_ftr.f));
                Eigen::Vector2d px;
                if (!cur_frame.cam()->project3(xyz_f, &px).isKeypointVisible())
                    return false;

                // check margin
                const Eigen::Vector2i pxi = px.cast<int>();
                const int boundary = 9;

                if (!cur_frame.cam()->isKeypointVisibleWithMargin(pxi, boundary))
                    return false;
            }

            // set matcher options
            if (ref_ftr.type == FeatureType::kEdgeletSeed || ref_ftr.type == FeatureType::kEdgeletSeedConverged)
                matcher.m_patch_matcher_options.align_1d = true;
            else
                matcher.m_patch_matcher_options.align_1d = false;

            // sanity checks
            if (std::isnan(seed::mu(state)))
                LOG_ERROR_STREAM("seed is nan!");

            if (std::isnan(std::sqrt(seed::sigma2(state))))
                LOG(WARNING) << "seed sigma is nan!" << seed::sigma2(state) << ", sq" << std::sqrt(seed::sigma2(state)) << ", check-convergence = " << check_convergence;

            // search epipolar line, find match, and triangulate to find new depth z
            double depth;
            PatchMatcher::MatchResult res =
                matcher.FindEpipolarMatchDirect(
                    ref_frame, cur_frame, T_cur_ref, ref_ftr, seed::getInvDepth(state),
                    seed::getInvMinDepth(state), seed::getInvMaxDepth(state), depth);

            if (res != PatchMatcher::MatchResult::kSuccess)
            {
                if (!matcher.m_reject)
                {
                    seed::increaseOutlierProbability(state);
                }
                if (matcher.m_patch_matcher_options.verbose)
                {
                    std::cout << "filter fail = " << PatchMatcher::GetResultString(res) << std::endl;
                }
                return false;
            }

            // compute tau
            const FloatType depth_sigma = UncertaintyComputation(T_cur_ref.Inverse(), ref_ftr.f, depth, px_error_angle);

            // update the estimate
            if (use_vogiatzis_update)
            {
                if (!VogiatzisModelUpdate(
                        seed::getMeanFromDepth(depth),
                        seed::getSigma2FromDepthSigma(depth, depth_sigma),
                        ref_frame.seed_mu_range_,
                        state))
                {
                    ref_ftr.type = FeatureType::kOutlier;
                    return false;
                }
            }
            else
            {
                if (!GaussianModelUpdate(
                        seed::getMeanFromDepth(depth),
                        seed::getSigma2FromDepthSigma(depth, depth_sigma),
                        state))
                {
                    ref_ftr.type = FeatureType::kOutlier;
                    return false;
                }
            }

            // check if converged
            if (seed::isConverged(state,
                                  ref_frame.seed_mu_range_,
                                  sigma2_convergence_threshold))
            {
                if (ref_ftr.type == FeatureType::kCornerSeed)
                    ref_ftr.type = FeatureType::kCornerSeedConverged;
                else if (ref_ftr.type == FeatureType::kEdgeletSeed)
                    ref_ftr.type = FeatureType::kEdgeletSeedConverged;
                else if (ref_ftr.type == FeatureType::kMapPointSeed)
                    ref_ftr.type = FeatureType::kMapPointSeedConverged;
            }
            return true;
        }

        bool VogiatzisModelUpdate(
            const FloatType z, // Measurement
            const FloatType tau2,
            const FloatType mu_range,
            Eigen::Ref<SeedState> &mu_sigma2_a_b)
        {
            FloatType &mu = mu_sigma2_a_b(0);
            FloatType &sigma2 = mu_sigma2_a_b(1);
            FloatType &a = mu_sigma2_a_b(2);
            FloatType &b = mu_sigma2_a_b(3);

            const FloatType norm_scale = std::sqrt(sigma2 + tau2);
            if (std::isnan(norm_scale))
            {
                LOG(WARNING) << "Update Seed: Sigma2+Tau2 is NaN";
                return false;
            }

            const FloatType oldsigma2 = sigma2;
            const FloatType s2 = 1.0 / (1.0 / sigma2 + 1.0 / tau2);
            const FloatType m = s2 * (mu / sigma2 + z / tau2);
            const FloatType uniform_x = 1.0 / mu_range;
            FloatType C1 = a / (a + b) * vk::normPdf<FloatType>(z, mu, norm_scale);
            FloatType C2 = b / (a + b) * uniform_x;
            const FloatType normalization_constant = C1 + C2;
            C1 /= normalization_constant;
            C2 /= normalization_constant;
            const FloatType f = C1 * (a + 1.0) / (a + b + 1.0) + C2 * a / (a + b + 1.0);
            const FloatType e = C1 * (a + 1.0) * (a + 2.0) / ((a + b + 1.0) * (a + b + 2.0)) + C2 * a * (a + 1.0) / ((a + b + 1.0) * (a + b + 2.0));

            // update parameters
            const FloatType mu_new = C1 * m + C2 * mu;
            sigma2 = C1 * (s2 + m * m) + C2 * (sigma2 + mu * mu) - mu_new * mu_new;
            mu = mu_new;
            a = (e - f) / (f - e / f);
            b = a * (1.0 - f) / f;

            // TODO: This happens sometimes.
            if (sigma2 < 0.0)
            {
                LOG(WARNING) << "Seed sigma2 is negative!";
                sigma2 = oldsigma2;
            }
            if (mu < 0.0)
            {
                LOG(WARNING) << "Seed diverged! mu is negative!!";
                mu = 1.0;
                return false;
            }
            return true;
        }

        bool GaussianModelUpdate(
            const FloatType z, // Measurement
            const FloatType tau2,
            Eigen::Ref<SeedState> &mu_sigma2_a_b)
        {
            FloatType &mu = mu_sigma2_a_b(0);
            FloatType &sigma2 = mu_sigma2_a_b(1);
            FloatType &a = mu_sigma2_a_b(2);
            FloatType &b = mu_sigma2_a_b(3);

            const FloatType norm_scale = std::sqrt(sigma2 + tau2);
            if (std::isnan(norm_scale))
            {
                LOG(WARNING) << "Update Seed: Sigma2+Tau2 is NaN";
                return false;
            }

            const FloatType denom = (sigma2 + tau2);
            mu = (sigma2 * z + tau2 * mu) / denom;
            sigma2 = sigma2 * tau2 / denom;

            CHECK_GE(sigma2, 0.0);
            CHECK_GE(mu, 0.0);
            return true;
        }

        double UncertaintyComputation(
            const Transformation &T_ref_cur,
            const BearingVector &f,
            const FloatType z,
            const FloatType px_error_angle)
        {
            const BearingVector &t = T_ref_cur.GetPosition();
            const BearingVector a = f * z - t;
            FloatType t_norm = t.norm();
            FloatType a_norm = a.norm();
            FloatType alpha = std::acos(f.dot(t) / t_norm);            // dot product
            FloatType beta = std::acos(a.dot(-t) / (t_norm * a_norm)); // dot product
            FloatType beta_plus = beta + px_error_angle;
            FloatType gamma_plus = M_PI - alpha - beta_plus;                        // triangle angles sum to PI
            FloatType z_plus = t_norm * std::sin(beta_plus) / std::sin(gamma_plus); // law of sines
            return (z_plus - z);                                                    // tau
        }

        double EpiGradAngleComputation(
            const Transformation &T_cur_ref,
            const BearingVector &f_ref,
            const GradientVector &grad_ref,
            const FloatType depth_estimate)
        {
            // compute epipolar line in current image
            const BearingVector &e_hom = T_cur_ref.GetPosition();
            const BearingVector u_infty_hom(T_cur_ref.GetRotation().Rotate(f_ref));
            const BearingVector l(e_hom.cross(u_infty_hom)); // epipolar line in homogeneous coordinates

            const BearingVector f_ref_plus(f_ref + BearingVector(0.1 * grad_ref[0], 0.1 * grad_ref[1], f_ref[2]));

            const BearingVector f_cur = T_cur_ref * (f_ref * depth_estimate);
            const BearingVector f_cur_plus = T_cur_ref * (f_ref_plus * depth_estimate);

            GradientVector grad_cur(vk::project2(f_cur_plus) - vk::project2(f_cur));
            GradientVector l_dir(l[1], -l[0]);

            grad_cur.normalize();
            l_dir.normalize();

            return std::fabs(l_dir.dot(grad_cur));
        }

#ifdef SVO_USE_PHOTOMETRIC_DISPARITY_ERROR
        bool SeedCovarianceSet(
            const int halfpatch_size,
            const double image_noise2,
            SeedImplementation::Ptr seed)
        {
            FramePtr frame = seed->ftr_->frame.lock();
            if (!frame)
            {
                LOG_ERROR_STREAM("Could not lock weak_ptr<Frame> in DepthOptimization::SeedCovarianceSet");
                return false;
            }
            Eigen::Matrix2d H;
            H.setZero();
            const int patch_size = 2 * halfpatch_size;
            const int L = seed->ftr_->level;
            const cv::Mat &img = frame->img_pyr_[L];
            const int step = img.step;
            const int u = seed->ftr_->px[0] / (1 << L);
            const int v = seed->ftr_->px[1] / (1 << L);

            if (u - halfpatch_size < 0 || u + halfpatch_size >= img.cols || v - halfpatch_size < 0 || v + halfpatch_size >= img.rows)
                return false;

            for (int y = 0; y < patch_size; ++y)
            {
                uint8_t *p = img.data + (v - halfpatch_size + y) * step + (u - halfpatch_size);
                for (int x = 0; x < patch_size; ++x, ++p)
                {
                    const Eigen::Vector2d J((p[1] - p[-1]), (p[step] - p[-step]));
                    H += J * J.transpose();
                }
            }
            H /= 260100.0 * patch_size * patch_size; // 260100 = 2^2 * 255^2 for the missing 0.5 of the derivative and the 255 of uint8_t image
            seed->patch_cov_ = 2.0 * image_noise2 * H.inverse();

            if ((bool)std::isnan((double)seed->patch_cov_(0, 0)))
            {
                LOG_WARN_STREAM("Seed Patch Covariance is NaN");
                return false;
            }
            return true;
        }

        double GetSeedDisparityUncertainty(
            const SeedImplementation::Ptr &seed,
            const Transformation &T_cur_ref)
        {
            // compute epipolar line in current image
            // we use the essential matrix since we will only need the angle of the line
            // which should not be affected by the K-matrix. (distortion yes, but hopefully
            // neglectible).
            Eigen::Matrix3d E_cur_ref(vk::skew(T_cur_ref.GetPosition()) * T_cur_ref.GetRotationMatrix());
            Eigen::Vector3d l_cur(E_cur_ref * seed->ftr_->f);

            // TODO apply rotation to l_cur to get l_ref

            // find uncertainty in direction of epipolar line, marginalize bivariate distribution
            double epi_angle_cur = atan(-l_cur[0] / l_cur[1]);
            double s = sin(-epi_angle_cur);
            double c = cos(-epi_angle_cur);
            double disp_sigma2 =
                seed->patch_cov_.determinant() / (seed->patch_cov_(1, 1) * c * c + 2 * seed->patch_cov_(0, 1) * s * c + seed->patch_cov_(0, 0) * s * s);
            return fmax(disp_sigma2, 1.0); // we don't want less than 1.0px uncertainty. because e.g. sampling errors
        }
#endif

    } // namespace depth_optimization_utils
} // namespace mivins

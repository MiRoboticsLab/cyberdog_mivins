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

#include <mutex>
#include <queue>
#include <functional>
#include <unordered_map>

#include <mivins/utils/timer.h>
#include <mivins/camera_models/ncamera.h>

#include "mivins/common/frame.h"
#include "mivins/frontend_local_map.h"
#include "mivins/mivins_global_types.h"

namespace vk
{
    class PerformanceMonitor;
}

namespace mivins
{
    /// Keyframe Selection Criterion
    enum class KeyframeCriterion
    {
        DOWNLOOKING,
        FORWARD
    };

    /// Options for base frame handler module. Sets tracing and quality options.
    struct BaseOptions
    {
        /// The VO only keeps a list of N keyframes in the map. If a new keyframe
        /// is selected, then one furthest away from the current position is removed
        /// Default is 10. Set to 0 if all keyframes should remain in the map.
        /// More keyframes may reduce drift but since no loop-closures are actively
        /// detected and closed it is not beneficial to accumulate all keyframes.
        size_t max_n_kfs = 10;
        bool use_joint_init = true;
        //bool use_backend_init = false;

        /// Keyframe selection criterion: If we have a downlooking camera (e.g. on
        /// quadrotor), select DOWNLOOKING. Otherwise, select FORWARD.
        KeyframeCriterion kfselect_criterion = KeyframeCriterion::DOWNLOOKING;

        /// (!) Parameter for DOWNLOOKING keyframe criterion. We select a new KF
        /// whenever we move kfselect_min_dist of the average depth away from the
        /// closest keyframe.
        double kfselect_min_dist = 0.12;

        /// Keyframe selection for FORWARD: If we are tracking more than this amount
        /// of features, then we don't take a new keyframe.
        size_t kfselect_numkfs_upper_thresh = 110;

        /// Keyframe selection for FORWARD : If we have less than this amount of
        /// features we always select a new keyframe.
        size_t kfselect_numkfs_lower_thresh = 80;

        /// Keyframe selection for FORWARD : Minimum distance in meters (set initial
        /// scale!) before a new keyframe is selected.
        double kfselect_min_dist_metric = 0.5;

        /// Keyframe selection for FORWARD: Minimum angle in degrees to closest KF
        double kfselect_min_angle = 5.0;

        int kfselect_min_num_frames_between_kfs = 2;
        double kfselect_min_disparity = -1;

        // maximum duration allowed between keyframes with backend
        double kfselect_backend_max_time_sec = 3.0;

        /// By default, when the VO initializes, it sets the average depth to this
        /// value. This is because from monocular views, we can't estimate the scale.
        /// If you initialize SVO with a downward-looking camera at 1.5m height over
        /// a flat surface, then set this value to 1.5 and the resulting map will
        /// be approximately in the right scale.
        double init_map_scale = 1.0;

        /// By default, the orientation of the first camera-frame is set to be the
        /// identity (camera facing in z-direction, camera-right is in x-direction,
        /// camera-down is in y-direction) and the map scale is initialized with the
        /// option init_map_scale. However, if you would like to provide another
        /// initial orientation and inital scene depth, then activate this flag.
        /// If activated, you need to provide attitude and depth measurements using
        /// the functions addAttitudeMeasurement(), addDepthMeasurement() in the
        /// base class.
        /// This option is useful for in-flight initialization of SVO.
        bool init_use_att_and_depth = false;

        /// (!) During sparse image alignment (see [1]), we find the pose relative
        /// to the last frame by minimizing the photometric error between the frames.
        /// This KLT-style optimizer is pyramidal to allow more motion between two
        /// frames.
        /// Depending on the image size, you should increase this value.
        /// For images of the size 640x480 we set this value to 4. If your image is
        /// double the resolution, increase to 5, and so on.
        size_t img_align_max_level = 4;

        /// (!) During sparse image alignment, we stop optimizing at this level for
        /// efficiency reasons. If you have the time, you can go down to the zero'th
        /// level.
        /// Depending on the image size, you should increase this value.
        /// For images of the size 640x480 we set this value to 2. If your image is
        /// double the resolution, increase to 3, and so on.
        size_t img_align_min_level = 2;

        /// control whether to use robustification in image alignment
        bool img_align_robustification = false;

        /// If you are using a gyroscope and provide an attitude estimate
        /// together with the image in the function AddImage() then this parameter
        /// specifies how much you trust your gyroscope. If you set it to 2.0
        /// (default) it means that the gyroscope attitude is valued two times more
        /// than the actualy orientation estimate from the visual measurements.
        double img_align_prior_lambda_rot = 0.0;

        /// Internally, we have a very basic constant velocity motion model.
        /// similarly to lambda_rot, this parameter trades-off the visual measurements
        /// with the constant velocity prior. By default this weight is way below 1.
        double img_align_prior_lambda_trans = 0.0;

        /// If you choose to extract many features then sparse image alignment may
        /// become too slow. You can limit the number of features for this step with
        /// this parameter to randomly sample N features that are used for alignment.
        size_t img_align_max_num_features = 0;

        /// Whether or not to include the distortion when calculating the jacobian.
        /// For small FoV pinhole projection, it is safe to leave it as false.
        /// For fisheye lens, set this to true.
        bool img_align_use_distortion_jacobian = false;

        /// Estimate an affine transformation for illumination/exposure change.
        /// If you observe bad tracking because of illumination/exposure change,
        /// enabling these parameters might help.
        /// Normally it is OK to leave them as default.
        //bool img_align_est_illumination_gain = false;
        //bool img_align_est_illumination_offset = false;
        //bool img_align_est_ab = false;
        bool img_align_estimate_alpha = false;
        bool img_align_estimate_beta = false;
        bool img_align_estimate_ab = false;

        bool save_time_consumption = false;
        /// (!) This parameter is the reprojection error threshold during pose
        /// optimization. If the distance between a feature and the projected pixel
        /// position of the corresponding 3D point is further than this threshold
        /// appart (on the zero'th level pyramid), then the feature is removed from
        /// the frame. With a good camera and image resolution of 640x480 a threshold
        /// of 2.0 is typically ok. If you use shitty cameras (rolling shutter),
        /// higher resolution cameras, cameras with imperfect calibration etc. you
        /// might increase this threshold. But first, check the tracefile for the
        /// average reprojection threshold. We made the experice that with GoPro
        /// cameras, we had to increase this threshold.
        double poseoptim_thresh = 2.0;

        /// This is the same parameter as img_align_prior_lambda_rot but for
        /// the pose optimization instead of the sparse image alignment. Only used
        /// if you provide gyroscope measurements.
        double poseoptim_prior_lambda = 0.0;

        /// This parameter controls whether the pose optimizer works on:
        /// - unit plane: preferable for pinhole model
        /// - unit sphere: omnidirectional camera model(e.g. fisheye, catadioptric)
        bool poseoptim_using_unit_sphere = false;

        /// By default SVO does not do bundle adjustment but it optimizes the pose
        /// and the structure separately. This is not optimal but much faster. This
        /// parameters specifies how many 3D points should be randomly selected at
        /// every frame and be optimized. For speed reasons, we don't optimize all
        /// points at every iteration. Set to -1 if you want to do so anyway.
        int structure_optimization_max_pts = 20;

        /// Location where the tracefile is saved.
        std::string trace_dir = "/tmp";

        std::string save_pose_path = "/SDCARD/miloc/maps/default/visual/trajectory.txt";

        std::string save_kf_pose_path = "/SDCARD/miloc/maps/default/visual/kf_trajectory.txt";

        std::string vins_config = "/tmp";
        /// Minimum number of features that should be tracked. If the number falls
        /// bellow then the stage is set to STAGE_RELOCALIZING and the tracking
        /// quality to TRACKING_INSUFFICIENT until we find more features again.
        size_t quality_min_fts = 50;

        /// Minimum number of features that should be detected in initialize
        size_t init_min_fts = 100;
        
        /// If from one frame to the other we suddenly track much less features,
        /// this can be an indication that something is wrong and we set the stage
        /// to STAGE_RELOCALIZING and the tracking quality to TRACKING_INSUFFICIENT.
        int quality_max_fts_drop = 40;

        /// Once we are in relocalization mode, we allow a fixed number of images
        /// to try and relocalize before we reset() and set to STAGE_PAUSED.
        size_t relocalization_max_trials = 100;

        /// EXPERIMENTAL Should IMU measurements be used.
        bool use_imu = false;
        bool use_imu_only_for_gravityalign = false;
        bool use_vins_backend = false;
        bool backend_opt = false;
        bool vins_backend_multi_thread = false;

        //bool use_exposure_composition = false;
        bool use_pixel_value_align_to_first = false;
        bool use_photometric_calibration = false;

        /// EXPERIMENTAL Update seeds with old keyframes.
        bool update_seeds_with_old_keyframes = false;

        /// EXPERIMENTAL Asynchronous reprojection (for multi-camera svo)
        bool use_async_reprojectors = false;

        /// Trace statistics for benchmarking
        bool trace_statistics = false;

        /// we check whether the backend scale has stablized
        double backend_scale_stable_thresh = 0.02;
        
        bool light_optimize = true;

        /// EXPERIMENTAL:
        /// If the time from the last good tracking frame till the first frame that
        /// the system is initialized, we still think the relative pose with respect
        /// to the global map / loop closure database is still good.
        double global_map_lc_timeout_sec_ = 3.0;
    };

    enum class Stage
    {
        kPaused,        ///< Stage at the beginning and after reset
        kInitializing,  ///< Stage until the first frame with enough features is found
        kTracking,      ///< Stage when SVO is running and everything is well
        kRelocalization ///< Stage when SVO looses tracking and it tries to relocalize
    };
    extern const std::unordered_map<mivins::Stage, std::string, EnumClassHash>
        kStageName;

    enum class TrackingQuality
    {
        kInsufficient,
        kBad,
        kGood
    };
    extern const std::unordered_map<mivins::TrackingQuality, std::string,
                                    EnumClassHash>
        kTrackingQualityName;

    enum class UpdateResult
    {
        kDefault,
        kKeyframe,
        kFailure
    };
    extern const std::unordered_map<mivins::UpdateResult, std::string, EnumClassHash>
        kUpdateResultName;

    /// Base class for various VO pipelines. Manages the map and the state machine.
    class ChannelFrameBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::mutex mutex_t;
        typedef std::unique_lock<mutex_t> ulock_t;
        typedef std::function<bool(const Transformation &pose)> NewKeyFrameCriteria;

        /// Default constructor
        ChannelFrameBase(
            const BaseOptions &base_options,
            const ReprojectorOptions &reprojector_options,
            const DepthOptimizationOptions &depthfilter_options,
            const DetectorOptions &detector_options,
            const InitializationOptions &init_options,
            const FeatureTrackerOptions &tracker_options,
            const CameraBundle::Ptr &cameras);

        virtual ~ChannelFrameBase();

        // no copy
        ChannelFrameBase(const ChannelFrameBase &) = delete;
        ChannelFrameBase &operator=(const ChannelFrameBase &) = delete;

        void savefile();
        void finish_savefile();

        /// @name Main Interface
        ///
        /// These are the main functions to be used in this class when interfacing with
        /// the odometry pipeline.
        ///
        /// @{

        /// Start processing. Call this function at the beginning or to restart once
        /// the stage is set to paused.
        void Start() { set_start_ = true; }

        /// Will reset the map as soon as the current frame is finished processing.
        void Reset() { set_reset_ = true; }

        /// Get the current stage of the algorithm.
        inline Stage stage() const { return stage_; }
        inline std::string StageStr() const { return kStageName.at(stage_); }

        bool AddFrameBundle(const FrameBundlePtr &frame_bundle);

        bool AddImageBundle(
            const std::vector<cv::Mat> &imgs,
            const uint64_t timestamp);
        bool AddImageBundle(
            const std::vector<cv::Mat> &imgs,
            const std::map<int, cv::Mat> &depths,
            const uint64_t timestamp,
            std::vector<float> &exposure_times);

        bool AddImageBundle(
            const std::vector<cv::Mat> &imgs,
            std::vector<float> &exposure_times,
            const uint64_t timestamp);
        void SetRotationPrior(const Quaternion &R_imu_world);
        void SetRotationIncrementPrior(const Quaternion &R_lastimu_newimu);
        void SetPositionPrior(const Eigen::Vector3d &t_imu_world);

        inline bool IsBackendValid() const
        {
            //const bool ptr_valid = bundle_adjustment_? true:false;
            //return ptr_valid && bundle_adjustment_type_ != BundleAdjustmentType::kNone;
            const bool ptr_valid = vins_backend_ ? true : false;
            return ptr_valid;
        }

        inline bool IsBackendScaleInitialised() const
        {
            return backend_scale_initialized_;
        }

        inline bool HasGlobalMap() const
        {
#ifdef SVO_GLOBAL_MAP
            return global_map_ ? true : false;
#else
            return false;
#endif
        }

        inline bool DoesGlobalMapHaveInitialBA() const
        {
            return global_map_has_initial_ba_;
        }

        FrameBundlePtr GetLastFrames() const { return last_frames_; }

        /// @}

        /// Has the pipeline started? (set_start_ is set to false when started).
        inline bool HasStarted() const { return !set_start_; }

        /// Get the current map.
        inline const FrontendMapPtr &Map() const { return map_; }

        /// Get camera bundle.
        inline const CameraBundle::Ptr &GetNCamera() const { return cams_; }

        /// Get tracking quality.
        inline TrackingQuality trackingQuality() const { return tracking_quality_; }
        inline std::string TrackingQualityStr() const
        {
            return kTrackingQualityName.at(tracking_quality_);
        }

        /// Get update result
        inline UpdateResult updateResult() const { return update_res_; }
        inline std::string UpdateResultStr() const
        {
            return kUpdateResultName.at(update_res_);
        }

        /// Get the processing time of the previous iteration.
        inline double LastProcessingTime() const { return timer_.getTime(); }

        /// Get the number of feature observations of the last frame.
        inline size_t LastNumObservations() const { return num_obs_last_; }

        inline const BundleAdjustmentPtr &GetVinsBackend() const
        {
            return vins_backend_;
        }

        /// Set pose of first frame by specifying the IMU pose
        inline void SetInitialImuPose(const Transformation &T_world_imu)
        {
            T_world_imuinit = T_world_imu;
        }

        /// Set pose of first frame by specifying the camera pose
        inline void SetInitialCamPose(const Transformation &T_world_cam, size_t cam_index = 0)
        {
            T_world_imuinit = T_world_cam * cams_->get_T_C_B(cam_index);
        }

        /// Get the set of spatially closest keyframes of the last frame.
        std::vector<FramePtr> CloseKeyframes() const;

        /// Set compensation parameters.
        void SetCompensation(const bool do_compensation);

        /// Set the first frame (used for synthetic datasets in benchmark node)
        virtual void SetFirstFrames(const std::vector<FramePtr> &first_frames);

        /// @name Debug Interface
        /// These parameters should be private but are currently not for easier debugging.
        /// It is unlikely that you need them.
        ///
        /// @{

        /// Options for BaseFrameHandler module
        BaseOptions options_;

        /// Camera model, can be ATAN, Pinhole or Ocam (see vikit)
        CameraBundle::Ptr cams_;

        /// Current frame-bundle that is being processed
        FrameBundlePtr new_frames_;

        /// Last frame-bundle that was processed. Can be nullptr.
        FrameBundlePtr last_frames_;

        /// Custom callback to check if new keyframe is required
        NewKeyFrameCriteria need_new_kf_;

        /// Default keyframe selection criterion.
        virtual bool NeedNewKf(const Transformation &T_f_w);
        
        /// Default keyframe selection criterion.
        virtual bool NeedNewKeyframe();
        
        /// Calculation vel and omg for constant speed model
        void calcVelAndOmg();

        /// Translation prior computed from simple constant velocity assumption
        Vector3d t_lastimu_newimu_;

        /// Initial orientation
        Transformation T_world_imuinit;

        // SVO Modules
        SparseImgAlignBasePtr sparse_img_align_;
        std::vector<ReprojectorPtr> reprojectors_;
        PoseOptimizerPtr pose_optimizer_;
        DepthOptimizationPtr depth_filter_;
        InitializerPtr initializer_;
        ImuHandlerPtr imu_handler_;
        OdomHandlerPtr odom_handler_;
        //Estimator *estimator;
        bool init_sucess;
        std::ofstream savetwb;
        std::ofstream savekf;
        std::ofstream savet;
        double last_margedkf_t;
        std::vector<double> Twb2Twbaselink(const Transformation &T_w_b);
#ifdef SVO_GLOBAL_MAP
        GlobalMapPtr global_map_;
#endif
        /// @}

    protected:
        Stage stage_;                              //!< Current stage of the algorithm.
        bool set_reset_;                           //!< Flag that the user can set. Will reset the system before the next iteration.
        bool set_start_;                           //!< Flag the user can set to start the system when the next image is received.
        bool frontend_abnormal_;                         //!< Flag that judge if frontend is abnormal.
        FrontendMapPtr map_;                       //!< FrontendLocalMap of keyframes created by the slam system.
        vk::Timer timer_;                          //!< Stopwatch to measure time to process frame.
        size_t num_obs_last_;                      //!< Number of observations in the previous frame.
        TrackingQuality tracking_quality_;         //!< An estimate of the tracking quality based on the number of tracked features.
        UpdateResult update_res_;                  //!< Update result of last frame bundle
        size_t frame_counter_ = 0;                 //!< Number of frames processed since started
        double depth_median_;                      //!< Median depth at last frame
        double depth_min_;                         //!< Min depth at last frame
        double depth_max_;

        float depth_img_min_ = 0.3f;
        float depth_img_max_ = 40.0f;
        float depth_img_scale_ = 1000.0f;

        std::vector<std::vector<FramePtr>> overlap_kfs_;
        
        bool first_img_ = true;
        bool is_abnormal_ = false;
        Transformation abnormal_pose_;

        size_t bad_reproj_cnt_ = 0;
        size_t bad_optimize_cnt_ = 0;
        bool bad_optimize_ = false;
        bool bad_reproj_ = false;


        // Rotation prior.
        bool have_rotation_prior_ = false;
        Quaternion R_imu_world_;
        Quaternion R_imulast_world_;

        // 6 DoF Motion prior
        bool have_motion_prior_ = false;
        Transformation T_newimu_lastimu_prior_;

        bool have_position_prior_ = false;
        Eigen::Vector3d t_imu_world_;

        // backend related
        BundleAdjustmentType bundle_adjustment_type_ = BundleAdjustmentType::kNone;
        Eigen::Matrix<double, 9, 1> speed_bias_backend_latest_;
        Transformation T_WS_backend_latest_;
        double timestamp_backend_latest_;
        bool backend_reinit_ = false;

        // relocalization
        FramePtr reloc_keyframe_;
        FrameBundlePtr reloc_keyframe_bundle_;
        size_t relocalization_n_trials_; //!< With how many frames did we try to relocalize?

        void SetInitialPose(const FrameBundlePtr &frame_bundle) const;

        size_t SparseImageAlignment();

        size_t ProjectMapInFrame();

        size_t OptimizePose();

        void OptimizeStructure(
            const FrameBundlePtr &frames,
            int max_n_pts,
            int max_iter);

        void UpgradeSeedsToFeatures(const FramePtr &frame);

        /// Reset the map and frame handler to start from scratch.
        void ResetVisionFrontendCommon();

        /// Change the states of relevant modules to indicate recovery mode
        void SetRecovery(const bool recovery);

        inline bool IsInRecovery() const
        {
            return loss_without_correction_;
        }

        /// Pipeline implementation in derived class.
        virtual UpdateResult ProcessFrameBundle() = 0;

        /// Reset the frame handler. Implement in derived class.
        virtual void ResetAll() { ResetVisionFrontendCommon(); }

        /// Reset backend
        virtual void ResetBackend();

        /// Set the tracking quality based on the number of tracked features.
        virtual void SetTrackingQuality(const size_t num_observations);

        /// Set all cells in detector to occupied in which a point reprojected during
        /// the reprojection stage.
        virtual void SetDetectorOccupiedCells(
            const size_t reprojector_grid_idx,
            const DetectorPtr &feature_detector);

        /// Get motion prior, between last and new frames expressed in IMU frame.
        virtual void GetMotionPrior(const bool use_velocity_in_frame);
        
        /// Set motion prior for new frames
        virtual void SetMotionPrior(const FrameBundlePtr& frame_bundle);

        /// Helpers for ceres backend
        bool backend_scale_initialized_ = false;
        double last_kf_time_sec_ = -1.0;

        // global map related
        bool global_map_has_initial_ba_ = false;

        std::string tempVINS_config = "";

        // status for loss
        bool loss_without_correction_ = false;
        double last_good_tracking_time_sec_ = -1.0;
        virtual int GetType() = 0; // 0 1 2 3  ;   rgbdfisheye : 3
        bool CreateLandmarkWithDepth(
            const FramePtr &frame_ref, const FramePtr &frame_cur,
            const int id_ref, const int id_cur, PointPtr &point);

    public:
        BundleAdjustmentPtr vins_backend_;

        Eigen::Vector3d omg_;
        Eigen::Vector3d vel_;
        int window_size_;
        int abnormal_cnt_ = 0;
        bool isFrontendOk();
        bool isBackendOK(); 
        void SetVinsBackend(const BundleAdjustmentPtr &vins_ba);
        int64_t GetLastFramesTimestamp();
        std::vector<Transformation> GetLastFramesCamPose();
        Transformation GetLastFramesIMUPose();
        bool IsLastKeyFrame();
        double GetLastFramesTimestampSec();
        Transformation GetLastFramesImuPose();

        int getAbnormalCount();
        void SetAbnormalResult(bool is_abnormal);
        void SetAbnormalPose(const Transformation& abnormal_pose);
        void ResetAllEnd();

    };

} // namespace nslam

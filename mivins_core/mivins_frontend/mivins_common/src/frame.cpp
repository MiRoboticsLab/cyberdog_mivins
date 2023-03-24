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

#include <mivins/common/frame.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <stdexcept>
#include <fast/fast.h>
#include <mivins/utils/math_utils.h>
#include <mivins/utils/cv_utils.h>

#include <mivins/common/logging.h>
#include <mivins/common/point.h>
#include <mivins/common/camera.h>

namespace mivins
{

    int Frame::global_frame_id = 0;

    Frame::Frame(
        const CameraPtr &cam,
        const cv::Mat &img,
        const int64_t timestamp_ns,
        size_t n_pyr_levels)
        : frame_id_(global_frame_id++) // TEMPORARY
          ,
          cam_(cam), key_pts_(5, std::make_pair(-1, BearingVector())), timestamp_(timestamp_ns), aff(0, 0), exposure_time(1.0)
    {
        InitFrame(img, n_pyr_levels);
    }
    Frame::Frame(
        const CameraPtr &cam,
        const cv::Mat &img,
        const int64_t timestamp_ns,
        const float exposure_time,
        const size_t n_pyr_levels)
        : frame_id_(global_frame_id++) // TEMPORARY
          ,
          cam_(cam), key_pts_(5, std::make_pair(-1, BearingVector())), timestamp_(timestamp_ns), aff(0, 0), exposure_time(exposure_time)
    {
        InitFrame(img, n_pyr_levels);
    }

    Frame::Frame(
        const int id,
        const int64_t timestamp_ns,
        const CameraPtr &cam,
        const Transformation &T_world_cam)
        : frame_id_(id), cam_(cam), T_f_w_(T_world_cam.Inverse()), key_pts_(5, std::make_pair(-1, BearingVector())), timestamp_(timestamp_ns)
    {
    }

    Frame::Frame(
        const CameraPtr &cam,
        const cv::Mat &img,
        const cv::Mat &imgDepth,
        const int64_t timestamp_ns,
        size_t n_pyr_levels,
        const float exposure_time)
        : frame_id_(global_frame_id++) // TEMPORARY
          ,
          cam_(cam), key_pts_(5, std::make_pair(-1, BearingVector())), timestamp_(timestamp_ns), exposure_time(exposure_time), aff(0, 0)
    {
        InitFrame(img, n_pyr_levels);
        InitFrameDepth(imgDepth);
    }

    Frame::~Frame()
    {
    }

    void Frame::InitFrameDepth(const cv::Mat &imgDepth)
    {
        if (!imgDepth.empty())
            depth_image_ = imgDepth;
    }
    // get the valid depth value(meter), from depth image
    float Frame::GetValidDepthFromImage(int row, int col)
    {
        if (depth_image_.empty())
            return 0.0f;
        float l = 0.0f, r = 0.0f, t = 0.0f, b = 0.0f;
        float lt = 0.0f, rt = 0.0f, lb = 0.0f, rb = 0.0f;
        float center = 0.0f;
        if ((col - 1) < 0 || (col + 1) >= depth_image_.cols || (row - 1) < 0 || (row + 1) >= depth_image_.rows)
            return 0.0f;
        center = (float)depth_image_.at<ushort>(row, col) / cam_->getDepthScale();
        l = (float)depth_image_.at<ushort>(row, col - 1) / cam_->getDepthScale();
        r = (float)depth_image_.at<ushort>(row, col + 1) / cam_->getDepthScale();
        t = (float)depth_image_.at<ushort>(row - 1, col) / cam_->getDepthScale();
        b = (float)depth_image_.at<ushort>(row + 1, col) / cam_->getDepthScale();

        lt = (float)depth_image_.at<ushort>(row - 1, col - 1) / cam_->getDepthScale();
        rt = (float)depth_image_.at<ushort>(row - 1, col + 1) / cam_->getDepthScale();
        lb = (float)depth_image_.at<ushort>(row + 1, col - 1) / cam_->getDepthScale();
        rb = (float)depth_image_.at<ushort>(row + 1, col + 1) / cam_->getDepthScale();

        float depth_mean = (fabs(l - center) + fabs(r - center) + fabs(t - center) + fabs(b - center) +
                            fabs(lt - center) + fabs(lb - center) + fabs(rt - center) + fabs(rb - center)) /
                           8.0f;
        //std::cout << "======================================depth mean = " <<  depth_mean << std::endl;
        if (depth_mean < 0.1f)
            return center; //(center + l + r + t + b + lt + lb + rt + rb)/9.0f;
        else
            return 0.0f;
    }

    void Frame::InitFrame(const cv::Mat &img, size_t n_pyr_levels)
    {
        CHECK_EQ(key_pts_[0].first, -1);
        CHECK_EQ(key_pts_[4].first, -1);

        // check image
        CHECK(!img.empty());
        CHECK_EQ(img.cols, static_cast<int>(cam_->imageWidth()));
        CHECK_EQ(img.rows, static_cast<int>(cam_->imageHeight()));

        if (img.type() == CV_8UC1)
        {
            frame_utils::CreateImgPyramid(img, n_pyr_levels, img_pyr_);
        }
        else if (img.type() == CV_8UC3)
        {
            cv::Mat gray_image;
            cv::cvtColor(img, gray_image, cv::COLOR_BGR2GRAY);
            frame_utils::CreateImgPyramid(gray_image, n_pyr_levels, img_pyr_);
        }
        else
        {
            LOG(FATAL) << "Unknown image type " << img.type() << "!";
        }
        accumulated_w_T_correction_.setIdentity();
    }

    void Frame::SetKeyframe()
    {
        is_keyframe_ = true;
        SetKeyPoints();
    }

    void Frame::DeleteLandmark(const size_t &feature_index)
    {
        landmark_vec_.at(feature_index) = nullptr;
    }

    void Frame::ResizeFeatureStorage(size_t num)
    {
        if (static_cast<size_t>(px_vec_.cols()) < num)
        {
            const size_t n_new = num - num_features_;

            px_vec_.conservativeResize(Eigen::NoChange, num);
            f_vec_.conservativeResize(Eigen::NoChange, num);
            score_vec_.conservativeResize(num);
            level_vec_.conservativeResize(num);
            grad_vec_.conservativeResize(Eigen::NoChange, num);
            invmu_sigma2_a_b_vec_.conservativeResize(Eigen::NoChange, num);
            track_id_vec_.conservativeResize(num);

            type_vec_.resize(num, FeatureType::kCorner);
            landmark_vec_.resize(num, nullptr);
            seed_ref_vec_.resize(num);
            in_ba_graph_vec_.resize(num, false);

            // initial values
            level_vec_.tail(n_new).setZero();
            track_id_vec_.tail(n_new).setConstant(-1);
            score_vec_.tail(n_new).setConstant(-1);
        }
        else if (num < static_cast<size_t>(px_vec_.cols()))
        {
            SVO_ERROR_STREAM("Downsizing storage not implemented. cols = " << px_vec_.cols()
                                                                           << " , desired = " << num << ", num features = " << num_features_);
        }
    }

    void Frame::ClearFeatureStorage()
    {
        px_vec_.resize(Eigen::NoChange, 0);
        f_vec_.resize(Eigen::NoChange, 0);
        score_vec_.resize(0);
        level_vec_.resize(0);
        grad_vec_.resize(Eigen::NoChange, 0);
        invmu_sigma2_a_b_vec_.resize(Eigen::NoChange, 0);
        track_id_vec_.resize(0);
        type_vec_.clear();
        landmark_vec_.clear();
        seed_ref_vec_.clear();
        in_ba_graph_vec_.clear();
        num_features_ = 0;
    }

    void Frame::CopyFeaturesFrom(const Frame &other)
    {
        px_vec_ = other.px_vec_;
        f_vec_ = other.f_vec_;
        score_vec_ = other.score_vec_;
        level_vec_ = other.level_vec_;
        grad_vec_ = other.grad_vec_;
        type_vec_ = other.type_vec_;
        landmark_vec_ = other.landmark_vec_;
        seed_ref_vec_ = other.seed_ref_vec_;
        invmu_sigma2_a_b_vec_ = other.invmu_sigma2_a_b_vec_;
        track_id_vec_ = other.track_id_vec_;
        num_features_ = other.num_features_;
        in_ba_graph_vec_ = other.in_ba_graph_vec_;
    }

    FeatureWrapper Frame::GetFeatureWrapper(size_t index)
    {
        CHECK_LT(index, static_cast<size_t>(px_vec_.cols()));
        return FeatureWrapper(
            type_vec_[index], px_vec_.col(index), f_vec_.col(index),
            grad_vec_.col(index), score_vec_(index), level_vec_(index), landmark_vec_[index],
            seed_ref_vec_[index], track_id_vec_(index));
    }

    FeatureWrapper Frame::GetEmptyFeatureWrapper()
    {
        return GetFeatureWrapper(num_features_);
    }

    void Frame::SetKeyPoints()
    {
        const FloatType cu = cam_->imageWidth() / 2;
        const FloatType cv = cam_->imageHeight() / 2;

        for (size_t i = 0; i < num_features_; ++i)
        {
            if (landmark_vec_[i] == nullptr || type_vec_[i] == FeatureType::kOutlier)
                continue;

            const FloatType &u = px_vec_(0, i);
            const FloatType &v = px_vec_(1, i);

            // center
            if (key_pts_[0].first == -1)
                key_pts_[0] = std::make_pair(i, landmark_vec_[i]->pos3d_in_w);

            else if (std::max(std::fabs(u - cu), std::fabs(v - cv)) < std::max(std::fabs(px_vec_(0, key_pts_[0].first) - cu),
                                                                               std::fabs(px_vec_(1, key_pts_[0].first) - cv)))
                key_pts_[0] = std::make_pair(i, landmark_vec_[i]->pos3d_in_w);

            // corner
            if (u >= cu && v >= cv)
            {
                if (key_pts_[1].first == -1)
                    key_pts_[1] = std::make_pair(i, landmark_vec_[i]->pos3d_in_w);
                else if ((u - cu) * (v - cv) > (px_vec_(0, key_pts_[1].first) - cu) * (px_vec_(1, key_pts_[1].first) - cv))
                    key_pts_[1] = std::make_pair(i, landmark_vec_[i]->pos3d_in_w);
            }
            if (u >= cu && v < cv)
            {
                if (key_pts_[2].first == -1)
                    key_pts_[2] = std::make_pair(i, landmark_vec_[i]->pos3d_in_w);
                //else if((u-cu) * (v-cv)
                //        > (px_vec_(0, key_pts_[2].first) - cu) * (px_vec_(1, key_pts_[2].first)-cv))
                else if ((u - cu) * (cv - v) > (px_vec_(0, key_pts_[2].first) - cu) * (cv - px_vec_(1, key_pts_[2].first)))
                    key_pts_[2] = std::make_pair(i, landmark_vec_[i]->pos3d_in_w);
            }
            //if(u < cv && v < cv)
            if (u < cu && v < cv)
            {
                if (key_pts_[3].first == -1)
                    key_pts_[3] = std::make_pair(i, landmark_vec_[i]->pos3d_in_w);
                else if ((u - cu) * (v - cv) > (px_vec_(0, key_pts_[3].first) - cu) * (px_vec_(1, key_pts_[3].first) - cv))
                    key_pts_[3] = std::make_pair(i, landmark_vec_[i]->pos3d_in_w);
            }
            //if(u < cv && v >= cv)
            if (u < cu && v >= cv)
            {
                if (key_pts_[4].first == -1)
                    key_pts_[4] = std::make_pair(i, landmark_vec_[i]->pos3d_in_w);
                //else if((u-cu) * (v-cv)
                //        > (px_vec_(0, key_pts_[4].first) - cu) * (px_vec_(1, key_pts_[4].first)-cv))
                else if ((cu - u) * (v - cv) > (cu - px_vec_(0, key_pts_[4].first)) * (px_vec_(1, key_pts_[4].first) - cv))
                    key_pts_[4] = std::make_pair(i, landmark_vec_[i]->pos3d_in_w);
            }
        }
    }

    bool Frame::IsVisible(const Eigen::Vector3d &xyz_w,
                          Eigen::Vector2d *px) const
    {
        Eigen::Vector3d xyz_f = T_f_w_ * xyz_w;
        if (cam_->GetType() == Camera::Type::kPinhole)
        {
            Eigen::Vector2d px_top_left(0.0, 0.0);
            Eigen::Vector3d f_top_left;
            cam_->backProject3(px_top_left, &f_top_left);
            f_top_left.normalize();
            const Eigen::Vector3d z(0.0, 0.0, 1.0);
            const double min_cos = f_top_left.dot(z);
            const double cur_cos = xyz_f.normalized().dot(z);
            if (cur_cos < min_cos)
            {
                return false;
            }
        }

        if (px)
        {
            return cam_->project3(xyz_f, px).isKeypointVisible();
        }
        else
        {
            Eigen::Vector2d px_temp;
            return cam_->project3(xyz_f, &px_temp).isKeypointVisible();
        }
    }

    const cv::Mat &Frame::GetMask() const
    {
        return cam_->GetMask();
    }

    double Frame::GetErrorMultiplier() const
    {
        return cam_->errorMultiplier();
    }

    double Frame::GetAngleError(double img_err) const
    {
        return cam_->GetAngleError(img_err);
    }

    void Frame::Jacobian_xyz2image_imu(
        const mivins::Camera &cam,
        const Transformation &T_cam_imu,
        const Eigen::Vector3d &p_in_imu,
        Eigen::Matrix<double, 2, 6> &J)
    {
        Eigen::Matrix<double, 3, 6> G_x; // Generators times pose
        G_x.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        G_x.block<3, 3>(0, 3) = -vk::skew(p_in_imu);
        const Eigen::Vector3d p_in_cam = T_cam_imu * p_in_imu;

        Eigen::Matrix<double, 2, 3> J_proj; // projection derivative
        Eigen::Vector2d out_point;
        cam.project3(p_in_cam, &out_point, &J_proj);

        J = J_proj * T_cam_imu.GetRotation().GetRotationMatrix() * G_x;
    }

    FrameBundle::FrameBundle(const std::vector<FramePtr> &frames)
        : frames_(frames)
    {
        static BundleId bundle_counter = 0;
        bundle_id_ = bundle_counter++;
        for (const FramePtr &frame : frames)
        {
            frame->bundle_id_ = bundle_id_;
        }
    }

    FrameBundle::FrameBundle(const std::vector<FramePtr> &frames, const int bundle_id)
        : frames_(frames)
    {
        for (const FramePtr &frame : frames)
        {
            frame->bundle_id_ = bundle_id;
        }
    }

    size_t FrameBundle::NumFeatures() const
    {
        size_t n = 0;
        for (const FramePtr &f : frames_)
            n += f->NumFeatures();
        return n;
    }

    size_t FrameBundle::NumTrackedFeatures() const
    {
        size_t n = 0;
        for (const FramePtr &f : frames_)
            n += f->NumTrackedFeatures();
        return n;
    }

    size_t FrameBundle::NumTrackedLandmarks() const
    {
        size_t n = 0;
        for (const FramePtr &f : frames_)
            n += f->NumTrackedLandmarks();
        return n;
    }

    size_t FrameBundle::NumLandmarks() const
    {
        size_t n = 0;
        for (const FramePtr &f : frames_)
            n += f->NumLandmarks();
        return n;
    }

    size_t FrameBundle::NumLandmarksInBA() const
    {
        size_t n = 0;
        for (const FramePtr &f : frames_)
            n += f->NumLandmarksInBA();
        return n;
    }

    size_t FrameBundle::NumTrackedLandmarksInBA() const
    {
        size_t n = 0;
        for (const FramePtr &f : frames_)
            n += f->NumTrackedLandmarksInBA();
        return n;
    }

    size_t FrameBundle::NumFixedLandmarks() const
    {
        size_t n = 0;
        for (const FramePtr &f : frames_)
            n += f->NumFixedLandmarks();
        return n;
    }

    /// Utility functions for the Frame class
    namespace frame_utils
    {

        void CreateImgPyramid(const cv::Mat &img_level_0, int n_levels, ImgPyramid &pyr)
        {
            CHECK_EQ(img_level_0.type(), CV_8U);
            CHECK_GT(img_level_0.rows, 0);
            CHECK_GT(img_level_0.cols, 0);
            CHECK_GT(n_levels, 0);

            pyr.resize(n_levels);
            pyr[0] = img_level_0;
            for (int i = 1; i < n_levels; ++i)
            {
                pyr[i] = cv::Mat(pyr[i - 1].rows / 2, pyr[i - 1].cols / 2, CV_8U);
                HalfSample(pyr[i - 1], pyr[i]);
            }
        }

        bool GetSceneDepth(const FramePtr &frame, double &depth_median, double &depth_min, double &depth_max)
        {
            bool bUseDepthInScene = false; //TODO param
            std::vector<double> depth_vec;
            depth_vec.reserve(frame->num_features_);
            depth_min = std::numeric_limits<double>::max();
            depth_max = 0;
            double depth = 0;
            const Position ref_pos = frame->GetCameraPosInWorld();
            for (size_t i = 0; i < frame->num_features_; ++i)
            {
                double p_u, p_v;
                if (frame->landmark_vec_[i])
                {
                    depth = (frame->T_cam_world() * frame->landmark_vec_[i]->pos3d_in_w).norm();
                    if (bUseDepthInScene)
                    {
                        if (!frame->depth_image_.empty())
                        { //DEPTHTODO
                            p_u = frame->px_vec_(0, i);
                            p_v = frame->px_vec_(1, i);
                            if (abs(frame->GetValidDepthFromImage(round(p_v), round(p_u)) - depth) < 1.0 && frame->GetValidDepthFromImage(round(p_v), round(p_u)) < frame->cam_->getDepthMax())
                            {
                                depth = frame->GetValidDepthFromImage(round(p_v), round(p_u));
                            }
                        }
                    }
                }
                else if (frame->seed_ref_vec_[i].keyframe)
                {
                    const SeedRef &seed_ref = frame->seed_ref_vec_[i];
                    const Position pos = seed_ref.keyframe->T_world_cam() *
                                         seed_ref.keyframe->GetSeedPosInFrame(seed_ref.seed_id);
                    depth = (pos - ref_pos).norm();
                    if (bUseDepthInScene)
                    {
                        if (!frame->depth_image_.empty())
                        {
                            p_u = frame->px_vec_(0, i);
                            p_v = frame->px_vec_(1, i);
                            if (abs(frame->GetValidDepthFromImage(round(p_v), round(p_u)) - depth) < 1.0 && frame->GetValidDepthFromImage(round(p_v), round(p_u)) < frame->cam_->getDepthMax())
                            {
                                depth = frame->GetValidDepthFromImage(round(p_v), round(p_u));
                            }
                        }
                    }
                }
                else
                {
                    continue;
                }

                depth_vec.push_back(depth);
                depth_min = std::min(depth, depth_min);
                depth_max = std::max(depth, depth_max);
            }
            if (depth_vec.empty())
            {
                SVO_WARN_STREAM("Cannot set scene depth. Frame has no point-observations!");
                return false;
            }
            depth_median = vk::getMedian(depth_vec);
            return true;
        }

        void ComputeNormalizedBearingVectors(
            const Keypoints &px_vec,
            const Camera &cam,
            Bearings *f_vec)
        {
            CHECK_NOTNULL(f_vec);
            std::vector<bool> success;
            cam.backProject3(px_vec, f_vec, &success);
            for (const bool s : success)
            {
                CHECK(s);
            }
            *f_vec = f_vec->array().rowwise() / f_vec->colwise().norm().array();
        }

    } // namespace frame_utils
} // namespace mivins

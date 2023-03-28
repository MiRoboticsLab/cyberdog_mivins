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

#include <mivins/direct/feature_detector.h>
#include <mivins/direct/fast_detection_gpu.h>

#include <Eigen/Dense>
#include <fast/fast.h>
#include <mivins/utils/cv_utils.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <mivins/utils/memory.h>
#include <mivins/common/frame.h>
#include <mivins/common/camera.h>
#include <mivins/common/logging.h>
#include <mivins/direct/feature_detector_utilities.h>

//# define USE_FAST_DETECT_CUDA
//# define USE_EDGELET_DETECT_CUDA
namespace mivins
{

    namespace fd_utils = feature_detector_utils;


    void ExtractorNode::DivideNode(
      ExtractorNode &n1, ExtractorNode &n2, 
      ExtractorNode &n3, ExtractorNode &n4)
    {
        const int halfX = ceil(static_cast<float>(UR.x - UL.x) / 2);
        const int halfY = ceil(static_cast<float>(BR.y - UL.y) / 2);

        //Define boundaries of childs
        n1.UL = UL;
        n1.UR = cv::Point2i(UL.x + halfX, UL.y);
        n1.BL = cv::Point2i(UL.x, UL.y + halfY);
        n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
        n1.corners.reserve(corners.size());

        n2.UL = n1.UR;
        n2.UR = UR;
        n2.BL = n1.BR;
        n2.BR = cv::Point2i(UR.x, UL.y + halfY);
        n2.corners.reserve(corners.size());

        n3.UL = n1.BL;
        n3.UR = n1.BR;
        n3.BL = BL;
        n3.BR = cv::Point2i(n1.BR.x, BL.y);
        n3.corners.reserve(corners.size());

        n4.UL = n3.UR;
        n4.UR = n2.BR;
        n4.BL = n3.BR;
        n4.BR = BR;
        n4.corners.reserve(corners.size());

        //Associate points to childs
        for(size_t i = 0; i < corners.size(); ++i)
        {
            const Corner &kp = corners[i];
            if(kp.x < n1.UR.x)
            {
                if(kp.y < n1.BR.y)
                    n1.corners.push_back(kp);
                else
                    n3.corners.push_back(kp);
            }
            else if(kp.y < n1.BR.y)
                n2.corners.push_back(kp);
            else
                n4.corners.push_back(kp);
        }

        if(n1.corners.size() == 1)
            n1.no_more = true;
        if(n2.corners.size() == 1)
            n2.no_more = true;
        if(n3.corners.size() == 1)
            n3.no_more = true;
        if(n4.corners.size() == 1)
            n4.no_more = true;
    }

    //------------------------------------------------------------------------------
    AbstractDetector::AbstractDetector(
        const DetectorOptions &options,
        const CameraPtr &cam)
        : m_options(options), grid_(m_options.cell_size,
                                    std::ceil(static_cast<double>(cam->imageWidth()) / m_options.cell_size),
                                    std::ceil(static_cast<double>(cam->imageHeight()) / m_options.cell_size)),
          closeness_check_grid_(m_options.cell_size / m_options.sec_grid_fineness,
                                std::ceil(m_options.sec_grid_fineness * static_cast<double>(cam->imageWidth()) / m_options.cell_size),
                                std::ceil(m_options.sec_grid_fineness * static_cast<double>(cam->imageHeight()) / m_options.cell_size))
    {
    }

    //------------------------------------------------------------------------------
    void AbstractDetector::Detect(const FramePtr &frame)
    {
        size_t max_n_features = grid_.size();
        Detect(frame->img_pyr_, frame->GetMask(), max_n_features, frame->px_vec_,
               frame->score_vec_, frame->level_vec_, frame->grad_vec_, frame->type_vec_);
        frame->num_features_ = frame->px_vec_.cols();
        frame->landmark_vec_.resize(frame->num_features_, nullptr);
        frame->seed_ref_vec_.resize(frame->num_features_);
        frame->invmu_sigma2_a_b_vec_.resize(Eigen::NoChange, frame->num_features_);
        frame_utils::ComputeNormalizedBearingVectors(frame->px_vec_, *frame->cam(), &frame->f_vec_);
    }

    //------------------------------------------------------------------------------
    void FastDetector::Detect(
        const ImgPyramid &img_pyr,
        const cv::Mat &mask,
        const size_t max_n_features,
        Keypoints &px_vec,
        Scores &score_vec,
        Levels &level_vec,
        Gradients &grad_vec,
        FeatureTypes &types_vec)
    {
        Corners corners(
            grid_.n_cols * grid_.n_rows,
            Corner(0, 0, m_options.threshold_primary, 0, 0.0f));
        fd_utils::FastDetect(
            img_pyr, m_options.threshold_primary, m_options.border,
            m_options.min_level, m_options.max_level, corners, grid_);
        fd_utils::FillFeatures(
            corners, FeatureType::kCorner, mask, m_options.threshold_primary,
            max_n_features, px_vec, score_vec, level_vec, grad_vec, types_vec, grid_);

        ResetGrid();
    }

    //------------------------------------------------------------------------------
    void GradientDetector::Detect(
        const ImgPyramid &img_pyr,
        const cv::Mat &mask,
        const size_t max_n_features,
        Keypoints &px_vec,
        Scores &score_vec,
        Levels &level_vec,
        Gradients &grad_vec,
        FeatureTypes &types_vec)
    {
        // Compute pyramid of derivative max magnitude
        ImgPyramid mag_pyr(img_pyr.size());
        for (size_t i = 0; i < img_pyr.size(); ++i)
        {
            fd_utils::ComputeDerivMaxMagnitude(img_pyr[i], mag_pyr[i]);
        }

        cv::Mat mag_ss;
        mag_pyr[0].convertTo(mag_ss, CV_32F, 1.0f / 500.0f);
        //const int stride=mag_ss.cols;
#if 0
  const size_t max_level=img_pyr.size();

  if(false)
  {
    // Multiply across scale space
    float* mag_ss_ptr = (float*) mag_ss.data;
    for(int y=0; y<mag_ss.rows; ++y)
    {
      for(int x=0; x<mag_ss.cols; ++x, ++mag_ss_ptr)
      {
        for(size_t L=1; L<max_level; ++L)
        {
          const float scale = 1.0f / (1<<L);
          const int u = static_cast<int>(scale*x+0.5f);
          const int v = static_cast<int>(scale*y+0.5f);
          *mag_ss_ptr += 1.0/500.0f*mag_pyr[L].at<uint8_t>(v,u); // += ?
        }
      }
    }
  }
#endif

        // Nonmax suppression
        Corners corners;
        fd_utils::Nonmax(mag_ss, m_options.threshold_primary, &corners);
        fd_utils::SetCornerAngles(img_pyr, &corners);

        // Create feature for every corner that has high enough corner score
        fd_utils::FillFeatures(
            corners, FeatureType::kEdgelet, mask, m_options.threshold_secondary,
            max_n_features, px_vec, score_vec, level_vec, grad_vec, types_vec, grid_);
    }

    void GradientDetectorGrid::Detect(
        const ImgPyramid &img_pyr,
        const cv::Mat &mask,
        const size_t max_n_features,
        Keypoints &px_vec,
        Scores &score_vec,
        Levels &level_vec,
        Gradients &grad_vec,
        FeatureTypes &types_vec)
    {
        Corners corners(
            grid_.n_cols * grid_.n_rows,
            Corner(0, 0, m_options.threshold_secondary, 0, 0.0f));
        fd_utils::EdgeletDetector_V2(
            img_pyr, m_options.threshold_secondary, m_options.border,
            m_options.min_level, m_options.max_level, corners, grid_);
        fd_utils::FillFeatures(
            corners, FeatureType::kEdgelet, mask, m_options.threshold_secondary,
            max_n_features, px_vec, score_vec, level_vec, grad_vec, types_vec, grid_);

        ResetGrid();
    }

    //------------------------------------------------------------------------------
    void FastGradDetector::Detect(
        const ImgPyramid &img_pyr,
        const cv::Mat &mask,
        const size_t max_n_features,
        Keypoints &px_vec,
        Scores &score_vec,
        Levels &level_vec,
        Gradients &grad_vec,
        FeatureTypes &types_vec)
    {
        {
            // Detect fast corners.
            Corners corners(
                grid_.n_cols * grid_.n_rows,
                Corner(0, 0, m_options.threshold_primary, 0, 0.0f));
#ifdef USE_FAST_DETECT_CUDA
            fd_utils::FastDetectorGpu(img_pyr, m_options.threshold_primary, m_options.border,
                    m_options.min_level, m_options.max_level, corners, grid_);
#else
            fd_utils::FastDetect(
                img_pyr, m_options.threshold_primary, m_options.border,
                m_options.min_level, m_options.max_level, corners, grid_);
#endif
            fd_utils::FillFeatures(
                corners, FeatureType::kCorner, mask, m_options.threshold_primary,
                max_n_features, px_vec, score_vec, level_vec, grad_vec, types_vec, grid_);
        }

        int max_features = static_cast<int>(max_n_features) - px_vec.cols();
        if (max_features > 0)
        {
            // Detect edgelets.
            Corners corners(
                grid_.n_cols * grid_.n_rows,
                Corner(0, 0, m_options.threshold_secondary, 0, 0.0f));
#ifdef USE_EDGELET_DETECT_CUDA
            fd_utils::EdgeletDetectorV2Gpu(img_pyr, m_options.threshold_secondary, m_options.border,
                        m_options.min_level, m_options.max_level, corners, grid_);
#else
            fd_utils::EdgeletDetector_V2(
                img_pyr, m_options.threshold_secondary, m_options.border,
                m_options.min_level, m_options.max_level, corners, grid_);
#endif
            fd_utils::FillFeatures(
                corners, FeatureType::kEdgelet, mask, m_options.threshold_secondary,
                max_features, px_vec, score_vec, level_vec, grad_vec, types_vec, grid_);
        }

        ResetGrid();
    }

    //------------------------------------------------------------------------------
    void ShiTomasiGradDetector::Detect(
        const ImgPyramid &img_pyr,
        const cv::Mat &mask,
        const size_t max_n_features,
        Keypoints &px_vec,
        Scores &score_vec,
        Levels &level_vec,
        Gradients &grad_vec,
        FeatureTypes &types_vec)
    {
        {
            // Detect shitomasi corners.
            Corners corners(
                grid_.n_cols * grid_.n_rows,
                Corner(0, 0, m_options.threshold_shitomasi, 0, 0.0f));
            fd_utils::ShiTomasiDetect(
                img_pyr, m_options.threshold_shitomasi, m_options.border,
                m_options.min_level, m_options.max_level, corners, grid_, closeness_check_grid_);
            fd_utils::FillFeatures(
                corners, FeatureType::kCorner, mask, m_options.threshold_shitomasi,
                max_n_features, px_vec, score_vec, level_vec, grad_vec, types_vec, grid_);
        }

        int max_features = static_cast<int>(max_n_features) - px_vec.cols();
        if (max_features > 0)
        {
            // Detect fast corners.
            Corners corners_fast(
                grid_.n_cols * grid_.n_rows,
                Corner(0, 0, m_options.threshold_primary, 0, 0.0f));
            fd_utils::FastDetect(
                img_pyr, m_options.threshold_primary, m_options.border,
                m_options.min_level, m_options.max_level, corners_fast, grid_);
            fd_utils::FillFeatures(
                corners_fast, FeatureType::kCorner, mask, m_options.threshold_primary,
                max_n_features, px_vec, score_vec, level_vec, grad_vec, types_vec, grid_);

            // Detect edgelets.
            Corners corners_grad(
                grid_.n_cols * grid_.n_rows,
                Corner(0, 0, m_options.threshold_secondary, 0, 0.0f));
            fd_utils::EdgeletDetector_V2(
                img_pyr, m_options.threshold_secondary, m_options.border,
                m_options.min_level, m_options.max_level, corners_grad, grid_);
            fd_utils::FillFeatures(
                corners_grad, FeatureType::kEdgelet, mask, m_options.threshold_secondary,
                max_features, px_vec, score_vec, level_vec, grad_vec, types_vec, grid_);
        }

        ResetGrid();
    }

    //------------------------------------------------------------------------------

    void ShiTomasiDetector::Detect(
        const ImgPyramid &img_pyr,
        const cv::Mat &mask,
        const size_t max_n_features,
        Keypoints &px_vec,
        Scores &score_vec,
        Levels &level_vec,
        Gradients &grad_vec,
        FeatureTypes &types_vec)
    {
        {
            // Detect shitomasi corners.
            Corners corners(
                grid_.n_cols * grid_.n_rows,
                Corner(0, 0, m_options.threshold_shitomasi, 0, 0.0f));
            fd_utils::ShiTomasiDetect(
                img_pyr, m_options.threshold_shitomasi, m_options.border,
                m_options.min_level, m_options.max_level, corners, grid_,
                closeness_check_grid_);
            fd_utils::FillFeatures(
                corners, FeatureType::kMapPoint, mask, m_options.threshold_shitomasi,
                max_n_features, px_vec, score_vec, level_vec, grad_vec, types_vec, grid_);
        }
        ResetGrid();
    }

//------------------------------------------------------------------------------
//! @todo (MWE) kept the old version around as there might be changes still and
//! we want to have the same structure and workflow within the detectors.
#if 0
void AllPixelsDetector::Detect(
    const ImgPyramid& img_pyr,
    const cv::Mat& mask,
    const size_t max_n_features,
    Keypoints& px_vec,
    Levels& level_vec,
    Gradients& grad_vec,
    FeatureTypes& types_vec)
{
  Corners corners;
  for(int y=1; y<=img_pyr[0].rows-1; ++y)
  {
    for(int x=1; x<=img_pyr[0].cols-1; ++x)
    {
      corners.push_back(Corner(x, y, std::numeric_limits<double>::max(), 0, 0.0f));
    }
  }
  fd_utils::SetCornerAngles(img_pyr, &corners);

  fd_utils::FillFeatures(
        corners, FeatureType::kEdgelet, mask, 0.0, max_n_features, px_vec, score_vec,
        level_vec, grad_vec, types_vec, grid_);

  ResetGrid();
}
#else // new
    void AllPixelsDetector::Detect(
        const ImgPyramid &img_pyr,
        const cv::Mat &mask,
        const size_t /*max_n_features*/,
        Keypoints &px_vec,
        Scores &score_vec,
        Levels &level_vec,
        Gradients &grad_vec,
        FeatureTypes &types_vec)
    {
        const int width = img_pyr.at(m_options.sampling_level).cols;
        const int height = img_pyr.at(m_options.sampling_level).rows;
        const int pyr_init_scale = 1 << m_options.sampling_level;
        const int border = m_options.border;
        const size_t num_features = (width - 2 * border) * (height - 2 * border);

        px_vec.resize(Eigen::NoChange, num_features);
        score_vec.setConstant(num_features, 1.0);
        level_vec.setConstant(num_features, m_options.level);
        grad_vec.resize(Eigen::NoChange, num_features);
        types_vec.assign(num_features, mivins::FeatureType::kCorner);
        size_t feature_index = 0;
        for (int y = border; y < height - border; ++y)
        {
            for (int x = border; x < width - border; ++x)
            {
                px_vec.col(feature_index++) = mivins::Keypoint(x * pyr_init_scale, y * pyr_init_scale);
            }
        }
    }
#endif

    void CannyDetector::Detect(
        const ImgPyramid &img_pyr,
        const cv::Mat &mask,
        const size_t max_n_features,
        Keypoints &px_vec,
        Scores &score_vec,
        Levels &level_vec,
        Gradients &grad_vec,
        FeatureTypes &types_vec)
    {
        // Compute pyramid of derivative max magnitude
        cv::Mat canny_edges;
        fd_utils::DetectCannyEdges(img_pyr[m_options.sampling_level], canny_edges);

        const int width = img_pyr.at(m_options.sampling_level).cols;
        const int height = img_pyr.at(m_options.sampling_level).rows;
        const int border = 1;
        const int pyr_init_scale = 1 << m_options.sampling_level;
        const int max_num_features = cv::countNonZero(canny_edges == 255);

        px_vec.resize(Eigen::NoChange, max_num_features);
        size_t feature_index = 0;
        for (int y = border; y < height - border; ++y)
        {
            for (int x = border; x < width - border; ++x)
            {
                if (canny_edges.at<uchar>(y, x))
                {
                    px_vec.col(feature_index++) = mivins::Keypoint(x * pyr_init_scale, y * pyr_init_scale);
                }
            }
        }

        size_t num_features = feature_index - 1;
        px_vec.conservativeResize(Eigen::NoChange, num_features);
        score_vec.setConstant(num_features, 1.0);
        level_vec.setConstant(num_features, m_options.level);
        grad_vec.resize(Eigen::NoChange, num_features);
        types_vec.assign(num_features, mivins::FeatureType::kCorner);
    }

    void SobelDetector::Detect(
        const ImgPyramid &img_pyr,
        const cv::Mat &mask,
        const size_t max_n_features,
        Keypoints &px_vec,
        Scores &score_vec,
        Levels &level_vec,
        Gradients &grad_vec,
        FeatureTypes &types_vec)
    {
        // Compute pyramid of derivative max magnitude
        cv::Mat sobel_edges;
        fd_utils::DetectSobelEdges(img_pyr[m_options.sampling_level], sobel_edges);

        const int width = img_pyr.at(m_options.sampling_level).cols;
        const int height = img_pyr.at(m_options.sampling_level).rows;
        const int border = 1;
        const int pyr_init_scale = 1 << m_options.sampling_level;
        const int max_num_features = cv::countNonZero(sobel_edges == 255);

        px_vec.resize(Eigen::NoChange, max_num_features);
        size_t feature_index = 0;
        for (int y = border; y < height - border; ++y)
        {
            for (int x = border; x < width - border; ++x)
            {
                if (sobel_edges.at<uchar>(y, x))
                {
                    px_vec.col(feature_index++) = mivins::Keypoint(x * pyr_init_scale, y * pyr_init_scale);
                }
            }
        }

        size_t num_features = feature_index - 1;
        px_vec.conservativeResize(Eigen::NoChange, num_features);
        score_vec.setConstant(num_features, 1.0);
        level_vec.setConstant(num_features, m_options.level);
        grad_vec.resize(Eigen::NoChange, num_features);
        types_vec.assign(num_features, mivins::FeatureType::kCorner);
    }
    
    void SobelGradDetector::Detect(
    const ImgPyramid& img_pyr,
    const cv::Mat& mask,
    const size_t max_n_features,
    Keypoints& px_vec,
    Scores& score_vec,
    Levels& level_vec,
    Gradients& grad_vec,
    FeatureTypes& types_vec)
    {
        size_t feature_index = 0;

        cv::Mat blurred_input;
        cv::Mat grad_x, grad_y;
        cv::Mat abs_grad_x, abs_grad_y, abs_grad_img;
        const int pyr_init_scale = 1 << m_options.sampling_level;

        int scale = 1;
        int delta = 0;
        int ddepth = CV_16S;
        cv::blur(img_pyr[m_options.sampling_level], blurred_input, cv::Size(2, 2) );
        cv::Sobel( blurred_input, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
        cv::convertScaleAbs( grad_x, abs_grad_x );
        cv::Sobel( blurred_input, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
        cv::convertScaleAbs( grad_y, abs_grad_y );
        cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, abs_grad_img);  
        
        int nx = 25, ny = 15;
        int roix_size = ceil(abs_grad_img.cols / nx);
        int roiy_size = ceil(abs_grad_img.rows / ny);
        cv::Mat mask_edge = cv::Mat::zeros(abs_grad_img.rows, abs_grad_img.cols, CV_8U);
        for (int i = 0; i < nx; i++) 
        {
            for (int j = 0; j < ny; j++) 
            {
                cv::Rect roi_ij(roix_size * i, roiy_size * j, roix_size, roiy_size);
                cv::Mat mag_ij = abs_grad_img(roi_ij);

                double max_val;
                cv::Point max_loc;
                cv::minMaxLoc(mag_ij, 0, &max_val, 0, &max_loc);

                // select the strongest
                // double thr_gradmin = 1e-2;
                double thr_gradmin = 1.0f;
                cv::Mat mask_ij = cv::Mat::zeros(roiy_size, roix_size, CV_8U);
                if (max_val > thr_gradmin) 
                    mask_ij.at<uchar>(max_loc) = 255;
                if (max_val != 0)
                    mask_edge(roi_ij).setTo(255, mask_ij);
            }
        }

        std::vector<cv::Point> points;
        cv::findNonZero(mask_edge, points);
        if (points.size() == 0) 
            return;

        px_vec.resize(Eigen::NoChange, points.size());
        for(size_t i = 0; i < points.size(); ++i)
        {
            int x = points[i].x;
            int y = points[i].y;
            px_vec.col(feature_index++) = Keypoint(x*pyr_init_scale, y*pyr_init_scale);
        }

        size_t num_features = feature_index - 1;
        px_vec.conservativeResize(Eigen::NoChange, num_features);
        score_vec.setConstant(num_features, 1.0);
        level_vec.setConstant(num_features, m_options.level);
        grad_vec.resize(Eigen::NoChange, num_features);
        types_vec.assign(num_features, FeatureType::kCorner);

        int max_features = static_cast<int>(max_n_features); // - px_vec.cols();
        if(max_features > 0)
        {
            // Detect edgelets.
            Corners corners(
                  grid_.n_cols * grid_.n_rows,
                  Corner(0, 0, m_options.threshold_secondary, 0, 0.0f));
            fd_utils::EdgeletDetector_V2(
                  img_pyr, m_options.threshold_secondary, m_options.border,
                  m_options.min_level, m_options.max_level, corners, grid_);
            fd_utils::FillFeatures(
                  corners, FeatureType::kEdgelet, mask, m_options.threshold_secondary,
                  max_features, px_vec, score_vec, level_vec, grad_vec, types_vec, grid_);
        }
    }

    //------------------------------------------------------------------------------
    void GradientHuangMumfordDetector::Detect(
        const ImgPyramid &img_pyr,
        const cv::Mat &mask,
        const size_t max_n_features,
        Keypoints &px_vec,
        Scores &score_vec,
        Levels &level_vec,
        Gradients &grad_vec,
        FeatureTypes &types_vec)
    {
        // Compute pyramid of derivative max magnitude
        ImgPyramid mag_pyr_32f(img_pyr.size());
        for (size_t i = 0; i < img_pyr.size(); ++i)
        {
            fd_utils::ComputeDerivHuangMumford(img_pyr[i], mag_pyr_32f[i]);
        }

        const int width = img_pyr.at(m_options.sampling_level).cols;
        const int height = img_pyr.at(m_options.sampling_level).rows;
        const int border = 1;
        const int pyr_init_scale = 1 << m_options.sampling_level;

        cv::Mat mag_level_32f = mag_pyr_32f[m_options.sampling_level];
        cv::Mat mag_level_thresholded;
        cv::threshold(mag_level_32f, mag_level_thresholded, m_options.threshold_primary,
                      1.0, cv::THRESH_BINARY_INV);
        const int max_num_features = cv::countNonZero(mag_level_thresholded == 1.0);

#if 0
  cv::imshow("mag_level_32f", mag_level_32f);
  cv::imshow("mag_level_thresholded", mag_level_thresholded);
  cv::waitKey(0);
#endif
        px_vec.resize(Eigen::NoChange, max_num_features);

        size_t feature_index = 0;
        for (int y = border; y < height - border; ++y)
        {
            for (int x = border; x < width - border; ++x)
            {
                if (mag_level_thresholded.at<uchar>(y, x))
                {
                    px_vec.col(feature_index++) = mivins::Keypoint(x * pyr_init_scale, y * pyr_init_scale);
                }
            }
        }

        size_t num_features = feature_index - 1;

        // Resize px_vec to the actual number of features while keeping the data.
        px_vec.conservativeResize(Eigen::NoChange, num_features);
        score_vec.setConstant(num_features, 1.0);
        level_vec.setConstant(num_features, m_options.level);
        grad_vec.resize(Eigen::NoChange, num_features);
        types_vec.assign(num_features, mivins::FeatureType::kCorner);

        //  // Nonmax suppression
        //  Corners corners;
        //  fd_utils::Nonmax(mag_ss, m_options.threshold_primary, &corners);
        //  fd_utils::SetCornerAngles(img_pyr, &corners);

        //  // Create feature for every corner that has high enough corner score
        //  fd_utils::FillFeatures(
        //        corners, FeatureType::kEdgelet, mask, m_options.threshold_secondary,
        //        px_vec, level_vec, grad_vec, types_vec, grid_);
    }

} // namespace mivins

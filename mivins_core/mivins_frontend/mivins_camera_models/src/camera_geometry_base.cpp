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

#include "mivins/camera_models/camera_geometry_base.h"

#include <string>
#include <utility>

#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>

#include <mivins/camera_models/camera-yaml-serialization.h>
#include <mivins/camera_models/yaml-serialization.h>
#include <mivins/utils/path_utils.h>

namespace vk
{
    namespace cameras
    {

        CameraGeometryBase::CameraGeometryBase(const int width, const int height)
            : width_(width), height_(height), bgamma_(false), bvignette_(false)
        {
        }

        CameraGeometryBase::Ptr CameraGeometryBase::loadFromYaml(
            const std::string &yaml_file)
        {
            try
            {
                YAML::Node doc = YAML::LoadFile(yaml_file.c_str());
                CameraGeometryBase::Ptr cam = doc.as<CameraGeometryBase::Ptr>();

                std::string basename = mivins::getBaseName(yaml_file);
                if (basename.empty())
                {
                    return cam;
                }
                const YAML::Node &mask = doc["mask"];
                if (mask)
                {
                    cam->loadMask(basename + "/" + mask.as<std::string>());
                }
                return cam;
            }
            catch (const std::exception &ex)
            {
                LOG(ERROR) << "Failed to load Camera from file " << yaml_file << " with the error: \n"
                           << ex.what();
            }
            // Return nullptr in the failure case.
            return CameraGeometryBase::Ptr();
        }

        void CameraGeometryBase::backProject3(
            const Eigen::Ref<const Eigen::Matrix2Xd> &keypoints,
            Eigen::Matrix3Xd *out_bearing_vectors, std::vector<bool> *success) const
        {
            const int num_keypoints = keypoints.cols();
            CHECK_NOTNULL(out_bearing_vectors)->resize(Eigen::NoChange, num_keypoints);
            CHECK_NOTNULL(success)->resize(num_keypoints);

            for (int i = 0; i < num_keypoints; ++i)
            {
                Eigen::Vector3d bearing_vector;
                (*success)[i] = backProject3(keypoints.col(i), &bearing_vector);
                out_bearing_vectors->col(i) = bearing_vector;
            }
        }

        void CameraGeometryBase::setDepthMin(const float depth_min)
        {
            depth_min_ = depth_min;
        }

        void CameraGeometryBase::setDepthMax(const float depth_max)
        {
            depth_max_ = depth_max;
        }

        void CameraGeometryBase::setDepthScale(const float depth_scale)
        {
            depth_scale_ = depth_scale;
        }

        float CameraGeometryBase::getDepthMin()
        {
            return depth_min_;
        }

        float CameraGeometryBase::getDepthMax()
        {
            return depth_max_;
        }

        float CameraGeometryBase::getDepthScale()
        {
            return depth_scale_;
        }

        void CameraGeometryBase::setMask(const cv::Mat &mask)
        {
            CHECK_EQ(height_, mask.rows);
            CHECK_EQ(width_, mask.cols);
            CHECK_EQ(mask.type(), CV_8UC1);
            mask_ = mask;
        }

        void CameraGeometryBase::loadMask(const std::string &mask_file)
        {
            cv::Mat mask(cv::imread(mask_file, 0));
            if (mask.data)
                setMask(mask);
            else
                LOG(FATAL) << "Unable to load mask file.";
        }

        bool CameraGeometryBase::isMasked(
            const Eigen::Ref<const Eigen::Vector2d> &keypoint) const
        {
            return keypoint[0] < 0.0 ||
                   keypoint[0] >= static_cast<double>(width_) ||
                   keypoint[1] < 0.0 ||
                   keypoint[1] >= static_cast<double>(height_) ||
                   (!mask_.empty() &&
                    mask_.at<uint8_t>(static_cast<int>(keypoint[1]),
                                      static_cast<int>(keypoint[0])) == 0);
        }

        Eigen::Vector2d CameraGeometryBase::createRandomKeypoint() const
        {
            Eigen::Vector2d out;
            do
            {
                out.setRandom();
                out(0) = std::abs(out(0)) * imageWidth();
                out(1) = std::abs(out(1)) * imageHeight();
            } while (isMasked(out));

            return out;
        }

        void CameraGeometryBase::loadVignette(const std::string &vignette_file)
        {
            if (vignette_file == "")
            {
                printf("NO PHOTOMETRIC Calibration!\n");
                bvignette_ = false;
                return;
            }
            cv::Mat srcImage = cv::imread(vignette_file.c_str(), 0);
            int width, height, channels;
            height = srcImage.rows;
            width = srcImage.cols;
            channels = srcImage.channels();
            std::cout << height << "," << width << std::endl;

            float *vignetteMap = new float[width * height];
            float maxV = 0;
            for (int i = 0; i < height; i++)
            {
                uchar *data = srcImage.ptr<uchar>(i);
                for (int j = 0; j < width; j++)
                {
                    if ((float)data[j] > maxV)
                        maxV = (float)data[j];
                }
            }

            int k = 0;
            for (int i = 0; i < height; i++)
            {
                uchar *data = srcImage.ptr<uchar>(i);
                for (int j = 0; j < width; j++)
                {
                    vignetteMap[k] = (float)data[j] / maxV;
                    k++;
                }
            }

            vignette_map_inv_ = new float[width * height];
            for (int i = 0; i < k; i++)
                vignette_map_inv_[i] = 1.0f / vignetteMap[i];
            bvignette_ = true;
            std::cout << "height = " << height << "width = " << width << "channels = " << channels << std::endl;
            std::cout << "input v success-------------------------------------" << std::endl;
            // cv::imshow("【原始图】",srcImage);
        }

        void CameraGeometryBase::loadGamma(const std::string &gamma_file)
        {
            std::ifstream gfile(gamma_file.c_str());
            if (!gfile.good())
            {
                printf("readGandVignette:Could not open pcalib file!\n");
                bgamma_ = false;
                return;
            }

            std::string line;
            std::getline(gfile, line);
            std::istringstream l1i(line);
            // begin迭代器, end迭代器来初始化
            std::vector<float> Gvec = std::vector<float>(std::istream_iterator<float>(l1i), std::istream_iterator<float>());
            int GDepth = Gvec.size();

            if (GDepth < 256)
            {
                printf("readGandVignette: invalid format! got %d entries in first line, expected at least 256!\n", (int)Gvec.size());
                bgamma_ = false;
                return;
            }
            float *g_temp = new float[GDepth];
            for (int i = 0; i < GDepth; i++)
                g_temp[i] = Gvec[i];
            for (int i = 0; i < GDepth - 1; i++)
            {
                if (g_temp[i + 1] <= g_temp[i])
                {
                    printf("PhotometricUndistorter: G invalid! it has to be strictly increasing, but it isnt!\n");
                    bgamma_ = false;
                    return;
                }
            }
            // 对响应值进行标准化
            float min = g_temp[0];
            float max = g_temp[GDepth - 1];

            for (int i = 0; i < GDepth; i++)
            {
                gamma_data_[i] = 255.0 * (g_temp[i] - min) / (max - min);
                //cout<<i<<":"<<G[i]<<endl;
            }
            bgamma_ = true;
            // 如果没有标定值, 则初始化为0-255 线性分布值
            // if(setting_photometricCalibration==0)
            // {
            //   for(int i=0;i<GDepth;i++) G[i]=255.0f*i/(float)(GDepth-1);
            // }
            std::cout << "input g success-------------------------------------" << std::endl;
        }
        void CameraGeometryBase::photometricUndistorter(cv::Mat &img)
        {
            if (!bgamma_ || !bvignette_ || img.type() == CV_16UC1) //
                return;

            // for(int row = 0; row < height_; ++row) {
            //   for(int col = 0; col < width_; ++col) {
            //     int idx = row * width_ + col;
            //     int data = gamma_data_[img.at<uchar>(row, col)];
            //     data *= vignette_map_inv_[idx];
            //     if(data > 255.0)
            //       data = 255.0;
            //     img.at<uchar>(row, col) = data;
            //   }
            // }
            cv::Mat irrimg(img.rows, img.cols, CV_8UC1);
            int k = 0;
            for (int i = 0; i < img.rows; i++)
            {
                uchar *img_lptr = irrimg.ptr<uchar>(i);
                for (int j = 0; j < img.cols; j++)
                {
                    float data = gamma_data_[(int)img.at<uint8_t>(i, j)];
                    data *= vignette_map_inv_[k];
                    if (data > 255)
                        data = 255;
                    img.at<uint8_t>(i, j) = (uint8_t)round(data);
                    img_lptr[j] = (uchar)data;
                    k++;
                }
            }

            std::cout << "calculate photo success-------------------------------------" << std::endl;
        }

    } // namespace cameras
} // namespace vk

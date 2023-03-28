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

#include <mivins/camera_models/ncamera.h>

#include <string>
#include <utility>
#include <glog/logging.h>

#include <mivins/camera_models/camera_geometry_base.h>
#include <mivins/camera_models/ncamera-yaml-serialization.h>
#include <mivins/camera_models/yaml-serialization.h>
#include <mivins/utils/path_utils.h>

namespace vk
{
    namespace cameras
    {

        NCamera::NCamera(
            const TransformationVector &T_C_B,
            const std::vector<Camera::Ptr> &cameras,
            const std::string &label)
            : T_C_B_(T_C_B), cameras_(cameras), label_(label)
        {
            initInternal();
        }

        NCamera::Ptr NCamera::loadFromYaml(const std::string &yaml_file)
        {
            try
            {
                YAML::Node doc = YAML::LoadFile(yaml_file.c_str());
                NCamera::Ptr ncam = doc.as<NCamera::Ptr>();

                std::string basename = mivins::getBaseName(yaml_file);
                if (basename.empty())
                {
                    return ncam;
                }
                const YAML::Node &base = doc["base"];

                const YAML::Node &cameras_node = doc["cameras"];
                CHECK_EQ(cameras_node.size(), ncam->numCameras())
                    << "YAML file and NCamera are not consistent.";
                for (size_t i = 0; i < cameras_node.size(); i++)
                {
                    const YAML::Node &cam_node = (cameras_node[i])["camera"];
                    const YAML::Node &mask = cam_node["mask"];
                    Camera::Ptr cam = ncam->getCameraShared(i);
                    if (mask)
                    {
                        cam->loadMask(basename + "/" + mask.as<std::string>());
                    }
                    const YAML::Node &depth_min = cam_node["depth_min"];

                    if (depth_min)
                    {
                        std::cout << "depth min: " << depth_min.as<float>() << std::endl;
                        cam->setDepthMin(depth_min.as<float>());
                    }
                    const YAML::Node &depth_max = cam_node["depth_max"];
                    if (depth_max)
                    {
                        std::cout << "depth max: " << depth_max.as<float>() << std::endl;
                        cam->setDepthMax(depth_max.as<float>());
                    }
                    const YAML::Node &depth_scale = cam_node["depth_scale"];
                    if (depth_scale)
                    {
                        std::cout << "depth scale: " << depth_scale.as<float>() << std::endl;
                        cam->setDepthScale(depth_scale.as<float>());
                    }

                    const YAML::Node &vignette = cam_node["vignette-path"];
                    if (vignette)
                    {
                        std::cout << "vignette file: " << base.as<std::string>() + vignette.as<std::string>() << std::endl; //(basename + "/" + vignette.as<std::string>())
                        cam->loadVignette(base.as<std::string>() + vignette.as<std::string>());                             //basename + "/" + vignette.as<std::string>()
                    }

                    const YAML::Node &gamma = cam_node["gamma-path"];
                    if (gamma)
                    {
                        std::cout << "gamma file: " << base.as<std::string>() + gamma.as<std::string>() << std::endl; //(basename + "/" + gamma.as<std::string>())
                        cam->loadGamma(base.as<std::string>() + gamma.as<std::string>());
                    }
                }

                return ncam;
            }
            catch (const std::exception &ex)
            {
                LOG(ERROR) << "Failed to load NCamera from file " << yaml_file << " with the error: \n"
                           << ex.what();
            }
            // Return nullptr in the failure case.
            return NCamera::Ptr();
        }

        void NCamera::initInternal()
        {
            CHECK_EQ(cameras_.size(), T_C_B_.size());
            for (size_t i = 0; i < cameras_.size(); ++i)
            {
                CHECK_NOTNULL(cameras_[i].get());
            }
        }

        const Transformation &NCamera::get_T_C_B(size_t camera_index) const
        {
            CHECK_LT(camera_index, cameras_.size());
            return T_C_B_[camera_index];
        }

        const TransformationVector &NCamera::getTransformationVector() const
        {
            return T_C_B_;
        }

        const Camera &NCamera::getCamera(size_t camera_index) const
        {
            CHECK_LT(camera_index, cameras_.size());
            CHECK_NOTNULL(cameras_[camera_index].get());
            return *cameras_[camera_index];
        }

        Camera::Ptr NCamera::getCameraShared(size_t camera_index)
        {
            CHECK_LT(camera_index, cameras_.size());
            return cameras_[camera_index];
        }

        Camera::ConstPtr NCamera::getCameraShared(size_t camera_index) const
        {
            CHECK_LT(camera_index, cameras_.size());
            return cameras_[camera_index];
        }

        void NCamera::printParameters(std::ostream &out, const std::string &s) const
        {
            out << s << std::endl;
            for (size_t i = 0; i < cameras_.size(); ++i)
            {
                out << "Camera #" << i << std::endl;
                cameras_[i]->printParameters(out, "");
                out << "  T_C_B = " << T_C_B_.at(i) << std::endl;
            }
        }

        const std::vector<Camera::Ptr> &NCamera::getCameraVector() const
        {
            return cameras_;
        }

        void NCamera::maskProcess(const int idx, cv::Mat &img)
        {
            cameras_[idx]->maskProcess(img);
        }

        void NCamera::photometricUndistorter(const int idx, cv::Mat &img)
        {
            cameras_[idx]->photometricUndistorter(img);
        }

    } // namespace cameras
} // namespace vikit

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

#include "mivins/camera_models/camera-yaml-serialization.h"

#include <mivins/camera_models/yaml-serialization.h>
#include <mivins/camera_models/cameras.h>

#include "mivins/camera_models/camera_geometry_base.h"
#include "mivins/camera_models/atan_distortion.h"
#include "mivins/camera_models/camera_geometry.h"
#include "mivins/camera_models/equidistant_distortion.h"
#include "mivins/camera_models/no_distortion.h"
#include "mivins/camera_models/pinhole_projection.h"
#include "mivins/camera_models/mei_projection.h"
#include "mivins/camera_models/radial_tangential_distortion.h"
#include "mivins/camera_models/omni_geometry.h"

namespace YAML
{

    bool convert<std::shared_ptr<vk::cameras::CameraGeometryBase>>::decode(
        const Node &node,
        vk::cameras::CameraGeometryBase::Ptr &camera)
    {
        camera.reset();
        try
        {
            if (!node.IsMap())
            {
                LOG(ERROR) << "Unable to get parse the camera because the node is not a map.";
                return true;
            }

            std::string camera_type;
            unsigned image_width;
            unsigned image_height;
            Eigen::VectorXd intrinsics;
            const YAML::Node distortion_config = node["distortion"];
            std::string distortion_type;
            Eigen::VectorXd distortion_parameters;

            if (!distortion_config)
            {
                distortion_type = "none";
            }

            if (YAML::safeGet(distortion_config, "type", &distortion_type) &&
                YAML::safeGet(distortion_config, "parameters", &distortion_parameters) &&
                YAML::safeGet(node, "type", &camera_type) &&
                YAML::safeGet(node, "image_width", &image_width) &&
                YAML::safeGet(node, "image_height", &image_height) &&
                YAML::safeGet(node, "intrinsics", &intrinsics))
            {
                // TODO: implement Projection::areParametersValid();

                if (camera_type == "pinhole" && distortion_type == "none")
                {
                    std::cout << "load pinhole camera without distortion" << std::endl;
                    camera.reset(new vk::cameras::PinholeGeometry(
                        image_width, image_height,
                        vk::cameras::PinholeProjection<vk::cameras::NoDistortion>(
                            intrinsics,
                            vk::cameras::NoDistortion())));
                }
                else if (camera_type == "pinhole" && distortion_type == "radial-tangential")
                {
                    camera.reset(new vk::cameras::PinholeRadTanGeometry(
                        image_width, image_height,
                        vk::cameras::PinholeProjection<vk::cameras::RadialTangentialDistortion>(
                            intrinsics,
                            vk::cameras::RadialTangentialDistortion(distortion_parameters))));
                }
                else if (camera_type == "mei" && distortion_type == "radial-tangential")
                {
                    camera.reset(new vk::cameras::MeiRadTanGeometry(
                        image_width, image_height,
                        vk::cameras::MeiProjection<vk::cameras::RadialTangentialDistortion>(
                            intrinsics,
                            vk::cameras::RadialTangentialDistortion(distortion_parameters))));
                }
                else if (camera_type == "pinhole" && distortion_type == "equidistant")
                {
                    camera.reset(new vk::cameras::PinholeEquidistantGeometry(
                        image_width, image_height,
                        vk::cameras::PinholeProjection<vk::cameras::EquidistantDistortion>(
                            intrinsics,
                            vk::cameras::EquidistantDistortion(distortion_parameters))));
                }
                else if (camera_type == "pinhole" && distortion_type == "fisheye")
                {
                    camera.reset(new vk::cameras::PinholeAtanGeometry(
                        image_width, image_height,
                        vk::cameras::PinholeProjection<vk::cameras::AtanDistortion>(
                            intrinsics,
                            vk::cameras::AtanDistortion(distortion_parameters))));
                }
                else if (camera_type == "omni")
                {
                    camera.reset(new vk::cameras::OmniGeometry(
                        image_width, image_height, intrinsics));
                }
                else
                {
                    LOG(ERROR) << "Unknown camera and distortion combination!";
                }
            }

            if (node["label"])
            {
                camera->setLabel(node["label"].as<std::string>());
            }
        }
        catch (const std::exception &e)
        {
            LOG(ERROR) << "Yaml exception during parsing: " << e.what();
            camera.reset();
            return true;
        }
        return true;
    }

    Node convert<vk::cameras::CameraGeometryBase::Ptr>::encode(
        const vk::cameras::CameraGeometryBase::Ptr &camera)
    {
        return convert<vk::cameras::CameraGeometryBase>::encode(*CHECK_NOTNULL(camera.get()));
    }

    bool convert<vk::cameras::CameraGeometryBase>::decode(
        const Node & /*node*/, vk::cameras::CameraGeometryBase & /*camera*/)
    {
        LOG(FATAL) << "Not implemented!";
        return false;
    }

    namespace internal
    {

        template <typename DistortionType>
        void encodeDistortion(const DistortionType &distortion, Node *distortion_node);

        template <>
        void encodeDistortion(
            const vk::cameras::NoDistortion &distortion, Node *distortion_node)
        {
            CHECK_NOTNULL(distortion_node);
            (*distortion_node)["type"] = "none";
            (*distortion_node)["parameters"] = Eigen::VectorXd(0, 1);
        }

        template <>
        void encodeDistortion(
            const vk::cameras::RadialTangentialDistortion &distortion,
            Node *distortion_node)
        {
            CHECK_NOTNULL(distortion_node);
            (*distortion_node)["type"] = "radial-tangential";
            Eigen::VectorXd parameters(4, 1);
            parameters << distortion.k1_, distortion.k2_, distortion.p1_, distortion.p2_;
            (*distortion_node)["parameters"] = parameters;
        }

        template <>
        void encodeDistortion(
            const vk::cameras::EquidistantDistortion &distortion,
            Node *distortion_node)
        {
            CHECK_NOTNULL(distortion_node);
            (*distortion_node)["type"] = "equidistant";
            Eigen::VectorXd parameters(4, 1);
            parameters << distortion.k1_, distortion.k2_, distortion.k3_, distortion.k4_;
            (*distortion_node)["parameters"] = parameters;
        }

        template <>
        void encodeDistortion(
            const vk::cameras::AtanDistortion &distortion,
            Node *distortion_node)
        {
            CHECK_NOTNULL(distortion_node);
            (*distortion_node)["type"] = "fisheye";
            Eigen::VectorXd parameters(1, 1);
            parameters << distortion.s_;
            (*distortion_node)["parameters"] = parameters;
        }

        template <typename DistortionType>
        bool encodePinhole(const vk::cameras::CameraGeometryBase &camera,
                           Node *camera_node)
        {
            typedef vk::cameras::PinholeProjection<DistortionType> Projection;
            typedef const vk::cameras::CameraGeometry<Projection> *ConstGeometryPtr;

            CHECK_NOTNULL(camera_node);
            if (ConstGeometryPtr pinhole_camera = dynamic_cast<ConstGeometryPtr>(&camera))
            {
                (*camera_node)["type"] = "pinhole";
                Eigen::VectorXd intrinsics(4, 1);
                const Projection *projection =
                    pinhole_camera->template projection<Projection>();
                intrinsics << projection->fx_, projection->fy_, projection->cx_,
                    projection->cy_;
                (*camera_node)["intrinsics"] = intrinsics;

                Node distortion_node;
                encodeDistortion(projection->distortion_, &distortion_node);
                (*camera_node)["distortion"] = distortion_node;

                return true;
            }

            return false;
        }

    } // namespace internal

    Node convert<vk::cameras::CameraGeometryBase>::encode(
        const vk::cameras::CameraGeometryBase &camera)
    {
        Node camera_node;

        camera_node["label"] = camera.getLabel();
        camera_node["image_height"] = camera.imageHeight();
        camera_node["image_width"] = camera.imageWidth();

        if (!internal::encodePinhole<vk::cameras::NoDistortion>(
                camera, &camera_node) &&
            !internal::encodePinhole<vk::cameras::RadialTangentialDistortion>(
                camera, &camera_node) &&
            !internal::encodePinhole<vk::cameras::EquidistantDistortion>(
                camera, &camera_node) &&
            !internal::encodePinhole<vk::cameras::AtanDistortion>(
                camera, &camera_node))
        {
            std::ostringstream oss;
            camera.printParameters(oss);
            LOG(FATAL) << "Camera encoding currently not supported, camera is "
                       << std::endl
                       << oss.str();
        }

        return camera_node;
    }

} // namespace YAML

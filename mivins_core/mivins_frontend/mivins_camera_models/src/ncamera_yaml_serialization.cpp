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

#include <mivins/camera_models/cameras.h>
#include <mivins/camera_models/ncamera.h>
#include <mivins/camera_models/camera-yaml-serialization.h>
#include <mivins/camera_models/ncamera-yaml-serialization.h>
#include <mivins/camera_models/yaml-serialization.h>

namespace YAML
{

    bool convert<std::shared_ptr<vk::cameras::NCamera>>::decode(
        const Node &node,
        vk::cameras::NCamera::Ptr &ncamera)
    {
        ncamera.reset();
        try
        {
            if (!node.IsMap())
            {
                LOG(ERROR) << "Unable to parse the ncamera because the node is not a map.";
                return true;
            }

            // Parse the label.
            std::string label = "";
            if (!YAML::safeGet<std::string>(node, "label", &label))
            {
                LOG(ERROR) << "Unable to get the label for the ncamera.";
                return true;
            }

            /*
    // Parse the id.
    vk::cameras::NCameraId ncam_id;
    std::string id_string;
    if (!node["id"] || !YAML::safeGet<std::string>(node, "id", &id_string)) {
      LOG(WARNING) << "Unable to get the id for the ncamera. Generating new random id.";
      ncam_id.randomize();
    } else {
      ncam_id.fromHexString(id_string);
    }
    */

            // Parse the cameras.
            const Node &cameras_node = node["cameras"];
            if (!cameras_node.IsSequence())
            {
                LOG(ERROR) << "Unable to parse the cameras because the camera node is not a sequence.";
                return true;
            }

            size_t num_cameras = cameras_node.size();
            if (num_cameras == 0)
            {
                LOG(ERROR) << "Number of cameras is 0.";
                return true;
            }
            vk::cameras::TransformationVector T_Ci_B;
            typedef std::shared_ptr<vk::cameras::Camera> CameraPtr;
            std::vector<CameraPtr> cameras;
            for (size_t camera_index = 0; camera_index < num_cameras; ++camera_index)
            {
                // Decode the camera
                const Node &camera_node = cameras_node[camera_index];
                if (!camera_node)
                {
                    LOG(ERROR) << "Unable to get camera node for camera " << camera_index;
                    return true;
                }
                if (!camera_node.IsMap())
                {
                    LOG(ERROR) << "Camera node for camera " << camera_index << " is not a map.";
                    return true;
                }
                CameraPtr camera;
                if (!YAML::safeGet(camera_node, "camera", &camera))
                {
                    LOG(ERROR) << "Unable to retrieve camera " << camera_index;
                    return true;
                }

                // Get the transformation matrix T_B_C (takes points from the frame C to frame B).
                Eigen::Matrix4d T_B_C_raw;
                if (!YAML::safeGet(camera_node, "T_B_C", &T_B_C_raw))
                {
                    LOG(ERROR) << "Unable to get extrinsic transformation T_B_C for camera " << camera_index;
                    return true;
                }
                // This call will fail hard if the matrix is not a rotation matrix.
                // vk::cameras::Quaternion q_B_C = vk::cameras::Quaternion(
                //     static_cast<Eigen::Matrix3d>(T_B_C_raw.block<3, 3>(0, 0)));
                // {
                //     Eigen::Quaterniond q(T_B_C_raw.block<3,3>(0,0));
                //     Eigen::Quaterniond q_new_old(cos(-2.0*3.14159/180), 0, sin(-2.0*3.14159/180) , 0);
                //     Eigen::Quaterniond q_new = q_new_old * q;
                //     T_B_C_raw.block<3,3>(0,0) = q_new.normalized().toRotationMatrix();
                // }
                Eigen::Quaterniond q_B_C = Eigen::Quaterniond(T_B_C_raw.block<3,3>(0,0));
                q_B_C.normalize();
                vk::cameras::Transformation T_B_C(q_B_C, T_B_C_raw.block<3, 1>(0, 3));

                // Fill in the data in the ncamera.
                cameras.push_back(camera);
                T_Ci_B.push_back(T_B_C.Inverse());
            }

            // Create the ncamera and fill in all the data.
            ncamera.reset(new vk::cameras::NCamera(T_Ci_B, cameras, label));
        }
        catch (const std::exception &ex)
        {
            LOG(ERROR) << "Yaml exception during parsing: " << ex.what();
            ncamera.reset();
            return true;
        }
        return true;
    }

    Node convert<std::shared_ptr<vk::cameras::NCamera>>::encode(
        const std::shared_ptr<vk::cameras::NCamera> &ncamera)
    {
        return convert<vk::cameras::NCamera>::encode(*CHECK_NOTNULL(ncamera.get()));
    }

    bool convert<vk::cameras::NCamera>::decode(const Node & /*node*/, vk::cameras::NCamera & /*ncamera*/)
    {
        LOG(FATAL) << "Not implemented!";
        return false;
    }

    Node convert<vk::cameras::NCamera>::encode(const vk::cameras::NCamera &ncamera)
    {
        Node ncamera_node;

        ncamera_node["label"] = ncamera.getLabel();
        /*
  if(ncamera.getId().isValid()) {
    ncamera_node["id"] = ncamera.getId().hexString();
  }
  */

        Node cameras_node;
        size_t num_cameras = ncamera.numCameras();
        for (size_t camera_index = 0; camera_index < num_cameras; ++camera_index)
        {
            Node camera_node;
            camera_node["camera"] = ncamera.getCamera(camera_index);
            camera_node["T_B_C"] = ncamera.get_T_C_B(camera_index).Inverse().GetTransformationMatrix();
            cameras_node.push_back(camera_node);
        }

        ncamera_node["cameras"] = cameras_node;

        return ncamera_node;
    }

} // namespace YAML

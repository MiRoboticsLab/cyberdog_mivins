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

#include "estimator/outlier_rejection.h"


void VinsOutlierRejection::removeOutliers(
    const FramePtr& frame, std::vector<size_t> &outliers) const
{
  size_t n_deleted_edges = 0;
  size_t n_deleted_corners = 0;

  // calculate threshold
  const double outlier_thresh = reproj_err_thresh_ 
                      / getErrorMultiplier(frame);

  for(int i = 0; i < (int)frame->num_features_; ++i)
  {
    if(frame->landmark_vec_[i] == nullptr)
      continue;

    Eigen::Vector3d obs_f = frame->f_vec_.col(i);
    Eigen::Vector2d grad = frame->grad_vec_.col(i);
    Eigen::Vector3d pt_world = getLandmarkPos(frame->landmark_vec_[i]);

    double unwhitened_error = 0.0f;
    Transformation T_c_w = frame->T_cam_world();
    if(isEdgelet(frame->type_vec_[i]))
      unwhitened_error = calEdgeResUnitPlane(obs_f, pt_world, grad, T_c_w);
    else
      unwhitened_error = calPointResUnitPlane(obs_f, pt_world, T_c_w);

    unwhitened_error *= 1.0 / (1 << frame->level_vec_(i));

    if(std::fabs(unwhitened_error) > outlier_thresh)
    {
      outliers.push_back(frame->track_id_vec_[i]);

      if(isEdgelet(frame->type_vec_[i]))
        ++n_deleted_edges;
      else
        ++n_deleted_corners;

      frame->type_vec_[i] = FeatureType::kOutlier;
      frame->seed_ref_vec_[i].keyframe.reset();
      removeLandmarkObs(frame->landmark_vec_[i], frame);
      frame->landmark_vec_[i] = nullptr;
      continue;
    }
  }

  // std::cout << "Outlier rejection: removed " << n_deleted_edges
  //           << " edgelets and " << n_deleted_corners << " corners.\n";
}

double VinsOutlierRejection::calEdgeResUnitPlane(
    const Eigen::Vector3d& obs_f, const Eigen::Vector3d& pt_world,
    const Eigen::Vector2d& grad, const Transformation& T_c_w) const
{
  const Eigen::Vector3d pt_cam(T_c_w * pt_world);

  double e = grad.dot(vk::project2(obs_f) - vk::project2(pt_cam));
  return std::abs(e);
}

double VinsOutlierRejection::calPointResUnitPlane(
    const Eigen::Vector3d& obs_f, const Eigen::Vector3d& pt_world,
    const Transformation& T_c_w) const
{
  const Eigen::Vector3d pt_cam(T_c_w * pt_world);

  Eigen::Vector2d e = vk::project2(obs_f) - vk::project2(pt_cam);
  return e.norm();
}
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

// included in global.h
// #include <svo/common/frame.h>
// #include <svo/common/point.h>
// #include <svo/common/types.h>
// #include <svo/common/transformation.h>

#include "common/global.h"

class VinsOutlierRejection
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<VinsOutlierRejection> Ptr;

  VinsOutlierRejection(const double reproj_err_thresh = 2.0f):
    reproj_err_thresh_(reproj_err_thresh){}
  ~VinsOutlierRejection(){}

  void removeOutliers(const FramePtr &frame, 
      std::vector<size_t> &deleted_points) const;

  void setPixelThreshold(const double reproj_err_thresh)
  {
    reproj_err_thresh_ = reproj_err_thresh;
  }

private:
  double reproj_err_thresh_;

  double calEdgeResUnitPlane(
      const Eigen::Vector3d& obs_f,
      const Eigen::Vector3d& pt_world,
      const Eigen::Vector2d& grad,
      const Transformation& T_c_w) const;

  double calPointResUnitPlane(
      const Eigen::Vector3d& obs_f,
      const Eigen::Vector3d& pt_world,
      const Transformation& T_c_w) const;
};

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

#pragma once

#include <iostream>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>

#include "integration_base.h"

#include "common/parameters.h"
#include "common/common_lib.h"

class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
public:
  
  IMUFactor() = delete;
  
  IMUFactor(IntegrationBasePtr _pre_integration);
  
  ~IMUFactor();

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

private:

  IntegrationBasePtr pre_integration;

};


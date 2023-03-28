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

#include <cstdlib>
#include <pthread.h>
#include <ceres/ceres.h>
#include <unordered_map>

#include "common/common_lib.h"

const int NUM_THREADS = 4;

struct ResidualBlockInfo
{
  ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function, 
                    std::vector<double *> _parameter_blocks, std::vector<int> _drop_set)
    : cost_function(_cost_function), loss_function(_loss_function), 
      parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

  void Evaluate();

  ceres::CostFunction *cost_function;
  ceres::LossFunction *loss_function;
  std::vector<double *> parameter_blocks;
  std::vector<int> drop_set;

  double **raw_jacobians;
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
  Eigen::VectorXd residuals;

  int localSize(int size)
  {
    return size == 7 ? 6 : size;
  }
};

struct ThreadsStruct
{
  std::vector<ResidualBlockInfo *> sub_factors;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  std::unordered_map<long, int> parameter_block_idx; //local size
  std::unordered_map<long, int> parameter_block_size; //global size
};

class MarginalizationInfo
{
  public:
  MarginalizationInfo(){valid = true;};
  ~MarginalizationInfo();
  int localSize(int size) const;
  int globalSize(int size) const;
  void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);
  void preMarginalize();
  void marginalize();
  std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);

  Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &Amm);

  int m, n;
  std::vector<ResidualBlockInfo *> factors;
  std::unordered_map<long, int> parameter_block_idx; //local size
  std::unordered_map<long, int> parameter_block_size; //global size
  std::unordered_map<long, double *> parameter_block_data;

  int sum_block_size;

  std::vector<int> keep_block_size; //global size
  std::vector<int> keep_block_idx;  //local size
  std::vector<double *> keep_block_data;

  Eigen::MatrixXd linearized_jacobians;
  Eigen::VectorXd linearized_residuals;
  const double eps = 1e-8;
  bool valid;
};

class MarginalizationFactor : public ceres::CostFunction
{
  public:
  MarginalizationFactor(MarginalizationInfo* _marginalization_info);
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

  MarginalizationInfo* marginalization_info;
};

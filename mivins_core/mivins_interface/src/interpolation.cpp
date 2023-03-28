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

#include <svo/interpolation.h>

namespace loose_couple
{
Interpolation::Interpolation()
{
}

Interpolation::~Interpolation()
{
}

bool Interpolation::poseInterpolation(
    const double ts1, const Eigen::Quaterniond &q1, const Eigen::Vector3d &p1, 
    const double ts2, const Eigen::Quaterniond &q2, const Eigen::Vector3d &p2,
    const double ts_inter, Eigen::Quaterniond &q_inter, Eigen::Vector3d &p_inter)
{
  p_inter.setZero();
  q_inter.setIdentity();

  if (ts_inter < ts1 || ts_inter > ts2) 
    return false;

  double scalar1 = (ts_inter - ts1) / (ts2 - ts1);
  q_inter = quatSlerp(q1.normalized(), q2.normalized(), scalar1);
  
  double scalar2 = 1.0 - scalar1;
  p_inter = scalar2 * p1 + scalar1 * p2;

  return true;
}

bool Interpolation::poseInterpolation(
    const double ts1, const Eigen::Matrix4d &pose1, 
    const double ts2, const Eigen::Matrix4d &pose2, 
    const double ts_inter, Eigen::Matrix4d &pose_inter)
{
  pose_inter.setIdentity();
  if (ts_inter < ts1 || ts_inter > ts2) return false;

  Eigen::Quaterniond q1(pose1.block<3, 3>(0, 0));
  Eigen::Quaterniond q2(pose2.block<3, 3>(0, 0));
  
  double scalar1 = (ts_inter - ts1) / (ts2 - ts1);
  Eigen::Quaterniond q_out = quatSlerp(q1.normalized(), q2.normalized(), scalar1);
  pose_inter.block<3,3>(0, 0) = q_out.toRotationMatrix();
 
  double scalar2 = 1.0 - scalar1;
  pose_inter.block<3, 1>(0, 3) = scalar2 * pose1.block<3, 1>(0, 3) 
                               + scalar1 * pose2.block<3, 1>(0, 3);
  return true;
}

bool Interpolation::linearInterpolation(
    double ts1, const Eigen::Vector3d &vel1,
    double ts2, const Eigen::Vector3d &vel2,
    double ts_inter, Eigen::Vector3d &vel_inter)
{
  vel_inter = Eigen::Vector3d::Zero();
  if (ts_inter < ts1 || ts_inter > ts2) 
    return false;

  double scalar1 = (ts_inter - ts1) / (ts2 - ts1);
  double scalar2 = 1.0 - scalar1;

  vel_inter = scalar2 * vel1 + scalar1 * vel2;
  return true;
}

Eigen::Quaterniond Interpolation::quatSlerp(Eigen::Quaterniond qa, Eigen::Quaterniond qb, double scalar) 
{
  // quaternion to return
  Eigen::Quaterniond qm;
  // Calculate angle between them.
  double cosHalfTheta = qa.w() * qb.w() + qa.x() * qb.x() + qa.y() * qb.y() + qa.z() * qb.z();

  // If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
  // the shorter path. Fix by reversing one quaternion.
  if(cosHalfTheta < 0.0f)
  {
    qb.w() = -qb.w();
    qb.x() = -qb.x();
    qb.y() = -qb.y();
    qb.z() = -qb.z();
    cosHalfTheta = -cosHalfTheta;
  }

  // if qa=qb or qa=-qb then theta = 0 and we can return qa
  if (abs(cosHalfTheta) >= 1.0)
  {
    qm.w() = qa.w();
    qm.x() = qa.x();
    qm.y() = qa.y();
    qm.z() = qa.z();
    return qm;
  }
  // Calculate temporary values.
  double halfTheta = acos(cosHalfTheta);
  double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
  // if theta = 180 degrees then result is not fully defined
  // we could rotate around any axis normal to qa or qb
  if (fabs(sinHalfTheta) < 0.001)
  { // fabs is floating point absolute
    qm.w() = (qa.w() * 0.5 + qb.w() * 0.5);
    qm.x() = (qa.x() * 0.5 + qb.x() * 0.5);
    qm.y() = (qa.y() * 0.5 + qb.y() * 0.5);
    qm.z() = (qa.z() * 0.5 + qb.z() * 0.5);
    return qm;
  }
  
  double ratioA = sin((1 - scalar) * halfTheta) / sinHalfTheta;
  double ratioB = sin(scalar * halfTheta) / sinHalfTheta; 
  //calculate Quaternion.
  qm.w() = (qa.w() * ratioA + qb.w() * ratioB);
  qm.x() = (qa.x() * ratioA + qb.x() * ratioB);
  qm.y() = (qa.y() * ratioA + qb.y() * ratioB);
  qm.z() = (qa.z() * ratioA + qb.z() * ratioB);
  return qm;
}

Eigen::Vector3d Interpolation::R2ypr(const Eigen::Matrix3d &R, bool use_angle)
{
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  if(use_angle)
    return ypr / M_PI * 180.0;
  else
    return ypr;
}

Eigen::Matrix3d Interpolation::ypr2R(const Eigen::Vector3d &ypr, bool use_angle)
{
  double y = ypr(0);
  double p = ypr(1);
  double r = ypr(2);

  if(use_angle)
  {
    y = ypr(0) / 180.0 * M_PI;
    p = ypr(1) / 180.0 * M_PI;
    r = ypr(2) / 180.0 * M_PI;
  }

  Eigen::Matrix3d Rz;
  Rz << cos(y), -sin(y), 0,
      sin(y),  cos(y), 0,
         0,       0, 1;

  Eigen::Matrix3d Ry;
  Ry << cos(p), 0., sin(p),
        0., 1.,     0.,
     -sin(p), 0., cos(p);

  Eigen::Matrix3d Rx;
  Rx << 1.,     0.,      0.,
      0., cos(r), -sin(r),
      0., sin(r),  cos(r);

  return Rz * Ry * Rx;
}
}


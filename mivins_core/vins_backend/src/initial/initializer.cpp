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

#include "initial/initializer.h"

Initializer::Initializer(std::vector<StateGroup> *states, 
  Eigen::Vector3d *g, const FrameManagerPtr &frame_manager)
{
  g_ = g;
  states_ = states;
  frame_manager_ = frame_manager;

  init_alignment_ = std::make_shared<InitialAlignment>();
}

Initializer::~Initializer()
{
  
}

void Initializer::setImuAvailable(bool use_imu)
{
  use_imu_ = use_imu;
}

void Initializer::setOdomAvailable(bool use_odom)
{
  use_odom_ = use_odom;
}

bool Initializer::initialStructure(const int frame_count, 
  std::map<double, ImageFrame> &all_image_frame, 
  int &marginalization_flag)
{
  //check imu observibility
  if(use_imu_)
  {
    Eigen::Vector3d sum_g;
    std::map<double, ImageFrame>::iterator frame_it;        
    for (frame_it = all_image_frame.begin(), frame_it++; 
      frame_it != all_image_frame.end(); frame_it++)
    {
      double dt = frame_it->second.pre_integration->sum_dt_;
      Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v_ / dt;
      sum_g += tmp_g;
    }

    Eigen::Vector3d aver_g;
    aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
    std::cout << "aver_g: " << aver_g.transpose() << std::endl;

    double var = 0;
    for (frame_it = all_image_frame.begin(), frame_it++; 
      frame_it != all_image_frame.end(); frame_it++)
    {
      double dt = frame_it->second.pre_integration->sum_dt_;
      Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v_ / dt;
      var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
      //cout << "frame g " << tmp_g.transpose() << endl;
    }
    var = sqrt(var / ((int)all_image_frame.size() - 1));
    printf("IMU variation %f!\n", var);

    if(var < 0.25)
    {
      printf("IMU excitation not enouth!\n");
      //return false;
    }
  }

  // global sfm
  Eigen::Quaterniond Q[frame_count + 1];
  Eigen::Vector3d T[frame_count + 1];
  std::map<int, Eigen::Vector3d> sfm_tracked_points;
  std::vector<SFMFeature> sfm_f;
  for(auto &feature_id : frame_manager_->feature_ids_)
  {
    SFMFeature tmp_feature;
    tmp_feature.state = false;
    tmp_feature.id = feature_id;

    auto &obs_frames = frame_manager_->feature_per_ids_[feature_id].obs_frames;
    auto &feature_per_frame = frame_manager_->feature_per_ids_[feature_id].feature_per_frame;
       
    for(auto &frame_id : obs_frames)
    {
      Eigen::Vector3d pts0 = feature_per_frame[frame_id].getObsNormPlane(0);

      tmp_feature.observation.push_back(std::make_pair(frame_id, pts0.head(2)));
    }
    sfm_f.emplace_back(tmp_feature);
  }

  int l;
  Eigen::Matrix3d relative_R;
  Eigen::Vector3d relative_T;
  if (!relativePose(relative_R, relative_T, l))
  {
    printf("Not enough features or parallax; Move device around\n");
    return false;
  }

  GlobalSFM sfm;
  if(!sfm.construct(frame_count + 1, Q, T, l,
          relative_R, relative_T,
          sfm_f, sfm_tracked_points))
  {
    printf("global SFM failed!\n");
    marginalization_flag = 0;
    return false;
  }

  //solve pnp for all frame
  std::map<int, Eigen::Vector3d>::iterator it;
  std::map<double, ImageFrame>::iterator frame_it;
  frame_it = all_image_frame.begin();
  for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
  {
    // provide initial guess
    cv::Mat r, rvec, t, D, tmp_r;
    if(getTimestamp(frame_it->second.frame_bundle) == getTimestamp( (*states_)[i].frame_bundle))
    {
      frame_it->second.is_key_frame = true;
      Eigen::Matrix3d R = Q[i].toRotationMatrix() * RIC[0].transpose();
      Eigen::Vector3d t = T[i];
      frame_it->second.setPose(R, t);

      i++;
      continue;
    }

    if(getTimestamp(frame_it->second.frame_bundle) > getTimestamp((*states_)[i].frame_bundle))
    {
      i++;
    }

    Eigen::Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
    Eigen::Vector3d P_inital = - R_inital * T[i];
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    frame_it->second.is_key_frame = false;
    std::vector<cv::Point3f> pts_3_vector;
    std::vector<cv::Point2f> pts_2_vector;
    const FramePtr &frame0 = frame_it->second.frame_bundle->at(0);
    for(size_t idx0 = 0; idx0 < numFeatures(frame0); ++idx0)
    {
      int feature_id = frame0->track_id_vec_(idx0);
      if(feature_id == -1)
        continue;
      
      it = sfm_tracked_points.find(feature_id);
      if(it != sfm_tracked_points.end())
      {
        Eigen::Vector3d pts_world = it->second;
        cv::Point3f pts_3(pts_world(0), pts_world(1), pts_world(2));
        pts_3_vector.push_back(pts_3);

        Eigen::Vector3d pts0 = frame0->f_vec_.col(idx0);
        frame_manager_->cvtSphere2Plane(0, pts0, true);
        cv::Point2f pts_2(pts0(0), pts0(1));
        pts_2_vector.push_back(pts_2);
      }
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
    if(pts_3_vector.size() < 6)
    {
      std::cout << "pts_3_vector size: " << pts_3_vector.size() << std::endl;
      printf("Not enough points for solve pnp !\n");
      return false;
    }
    if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
    {
      printf("solve pnp fail!\n");
      return false;
    }
    cv::Rodrigues(rvec, r);
    Eigen::MatrixXd R_pnp,tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);
    R_pnp = tmp_R_pnp.transpose();
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    T_pnp = R_pnp * (-T_pnp);
    frame_it->second.R = R_pnp * RIC[0].transpose();
    frame_it->second.T = T_pnp;
  }
  std::cout << "............... initialStructure 5\n";

  if (visualInitialAlign(frame_count, all_image_frame))
    return true;
  else
  {
    printf("misalign visual structure with IMU\n");
    return false;
  }
}

bool Initializer::visualInitialAlign(const int frame_count, 
          std::map<double, ImageFrame> &all_image_frame)
{
  Eigen::VectorXd x;
  //solve scale
  bool result = init_alignment_->visualAlignment(all_image_frame, 
          *states_, *g_, x, use_imu_, use_odom_);
  if(!result)
  {
    printf("solve g failed!\n");
    return false;
  }

  // change state
  for (int i = 0; i <= frame_count; i++)
  {
    double ts = getTimestamp((*states_)[i].frame_bundle);
    (*states_)[i].R = all_image_frame[ts].R;
    (*states_)[i].P = all_image_frame[ts].T;
    all_image_frame[ts].is_key_frame = true;
  }

  if(use_imu_)
  {
    for (int i = 0; i <= WINDOW_SIZE; ++i)
      (*states_)[i].pre_integration->repropagate(Eigen::Vector3d::Zero(), (*states_)[i].Bg);
  }

  double s = (x.tail<1>())(0);
  for (int i = frame_count; i >= 0; i--)
    (*states_)[i].P = s * (*states_)[i].P - (*states_)[i].R * TIC[0] 
                      - (s * (*states_)[0].P - (*states_)[0].R * TIC[0]);

  int kv = -1;
  std::map<double, ImageFrame>::iterator frame_i;
  for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
  {
    if(frame_i->second.is_key_frame && use_imu_)
    {
      kv++;
      (*states_)[kv].V = frame_i->second.R * x.segment<3>(kv * 3);
    }
  }

  Eigen::Matrix3d R0 = Utility::g2R(*g_);
  double yaw = Utility::R2ypr(R0 * (*states_)[0].R).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  *g_ = R0 * (*g_);
  //Matrix3d rot_diff = R0 * (*states_)[0].R.transpose();
  Eigen::Matrix3d rot_diff = R0;
  for (int i = 0; i <= frame_count; i++)
  {
    (*states_)[i].P = rot_diff * (*states_)[i].P;
    (*states_)[i].R = rot_diff * (*states_)[i].R;
    (*states_)[i].V = rot_diff * (*states_)[i].V;
  }
  std::cout << "g0     " << (*g_).transpose() << std::endl;;
  std::cout <<"my R0  " << Utility::R2ypr((*states_)[0].R).transpose() << std::endl;; 

  frame_manager_->clearDepth();
  frame_manager_->triangulate();

  return true;
}

bool Initializer::relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l)
{
  // find previous frame which contians enough correspondance and parallex with newest frame
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    std::vector<size_t> corres_idx;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
    frame_manager_->getCorresponding(i, WINDOW_SIZE, corres_idx, corres);
    
    std::cout << "*** " << i << ", " << WINDOW_SIZE 
          << ", corres size: " << corres.size() 
          << std::endl;

    if (corres.size() > 20)
    {
      double sum_parallax = 0;
      double average_parallax;
      for (int j = 0; j < int(corres.size()); j++)
      {
        Eigen::Vector2d pts_0 = corres[j].first.head<2>();
        Eigen::Vector2d pts_1 = corres[j].second.head<2>();
        double parallax = (pts_0 - pts_1).norm();
        sum_parallax += parallax;
      }
      
      cv::Mat status;
      average_parallax = 1.0 * sum_parallax / int(corres.size());
      std::cout << "average_parallax: " << average_parallax * FOCAL_LENGTH << std::endl;

      if(average_parallax * FOCAL_LENGTH > MIN_PARALLAX
        && motion_estimator_.solveRelativeRT(corres, status, relative_R, relative_T))
      {
        // just for debug
        {
          int cnt = 0;
          std::vector<size_t> inlier_idx, outlier_idx;
          for(int row = 0; row < status.rows; ++row)
          {
            uchar *ptr = status.ptr<uchar>(row);
            for(int col = 0; col < status.cols; ++col)
            {
              size_t idx = corres_idx[cnt];
              if((int)*ptr != 0)
                inlier_idx.emplace_back(idx);   
              else
                outlier_idx.emplace_back(idx);   
                
              ++cnt;
            }
          }

          frame_manager_->drawFeatureMatchById(i, WINDOW_SIZE, inlier_idx, "mono_inlier");
          frame_manager_->drawFeatureMatchById(i, WINDOW_SIZE, outlier_idx, "mono_outlier");
          // frame_manager_->removeOutlier(outlier_idx);
        }


        l = i;
        printf("average_parallax %f choose l %d and newest frame to triangulate the whole structure\n", 
          average_parallax * FOCAL_LENGTH, l);
        return true;
      }
    }
  }
  return false;
}

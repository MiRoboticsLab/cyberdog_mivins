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

#include "estimator/frame_manager.h"
#include "common/global.h"

FrameManager::FrameManager(std::vector<StateGroup> *_states,
  Matrix3dVec *_ric, Vector3dVec *_tic,
  CameraBundlePtr& camera_bundle, 
  VinsBackendOptions& options)
{
  ric_ = _ric;
  tic_ = _tic;

  states_ = _states;

  reject_with_F_ = false;

  camera_bundle_ = camera_bundle;

  obs_thresh_ = options.obs_threshold;

  min_depth_ = options.min_depth;

  max_depth_ = options.max_depth;

  grain_size_ = options.grain_size;

  nkf_all_opt_size_ = options.nkf_all_opt_size;
  kf_new_opt_size_ = options.opt_kf_only ? 0 : options.kf_new_opt_size;
  kf_all_opt_size_ = options.opt_kf_only ? 0 : options.kf_all_opt_size;

  grid_width_ = options.backend_grid_width;
  grid_height_ = options.backend_grid_height;
  
  for(int i = 0; i < (int)camera_bundle_->getNumCameras(); ++i)
  {
    const Camera &cam_i = camera_bundle_->getCamera(i);
    Eigen::VectorXd intrinsic = cam_i.getIntrinsicParameters();
   
    // cam_type_ = camera_bundle->getCamera(0).getType();
   
    // if(cam_type_ == Camera::Type::kMei && intrinsic.rows() != 5)
    // {
    //   std::cout << "Mei model, the camera intrinsic should be 5\n";
    //   exit(0);
    // }
    
    cam_intrinsics_.push_back(intrinsic);
  }

  getGridBorder();
}

FrameManager::~FrameManager()
{
  reset();
}

void FrameManager::reset()
{   
  clearState();
}

void FrameManager::getGridBorder()
{
  if(camera_bundle_ == nullptr)
    return;

  for(size_t i = 0; i < camera_bundle_->getNumCameras(); ++i)
  {
    size_t img_width = camera_bundle_->getCamera(i).imageWidth();
    size_t img_height = camera_bundle_->getCamera(i).imageHeight();

    size_t grid_cols = img_width / grid_width_;
    size_t grid_rows = img_height / grid_height_;
 
    size_t left_border = (img_width % grid_width_) / 2;
    size_t right_border = left_border + grid_cols * grid_width_;

    size_t top_border = (img_height % grid_height_) / 2;
    size_t bottom_border = top_border + grid_rows * grid_height_;

    grid_border_[i]["top"] = top_border;
    grid_border_[i]["bottom"] = bottom_border;
    grid_border_[i]["left"] = left_border;
    grid_border_[i]["right"] = right_border;
  }
}

void FrameManager::updateFeatureIds()
{
  feature_ids_.clear();
  for(auto &kv : feature_per_ids_)
    feature_ids_.emplace_back(kv.first);
}

void FrameManager::clearState()
{
  feature_ids_.clear();
  feature_per_ids_.clear();
}

void FrameManager::clearDepth()
{
  for(auto &feature_per_id : feature_per_ids_)
    feature_per_id.second.estimated_depth = -1.0f;
}

bool FrameManager::inlierCheck(const FramePtr &frame, Eigen::Vector2d px)
{
  int img_width = frame->img_pyr_[0].cols;
  int img_height = frame->img_pyr_[0].rows;
  if(px.x() < 0 || px.x() > img_width 
    || px.y() < 0 || px.y() > img_height)
    return false;
  return true;
}

void FrameManager::getCameraPose(const int frame_idx, const int cam_idx, 
          Eigen::Matrix4d &T_w_c, Eigen::Matrix4d &T_c_w)
{
  Eigen::Matrix3d R_w_c = (*states_)[frame_idx].R * (*ric_)[cam_idx];
  Eigen::Vector3d t_w_c = (*states_)[frame_idx].R * (*tic_)[cam_idx] + (*states_)[frame_idx].P;

  T_w_c = Eigen::Matrix4d::Identity();
  T_w_c.block<3, 3>(0, 0) = R_w_c;
  T_w_c.block<3, 1>(0, 3) = t_w_c;

  T_c_w = Eigen::Matrix4d::Identity();
  T_c_w.block<3, 3>(0, 0) = R_w_c.transpose();
  T_c_w.block<3, 1>(0, 3) = -R_w_c.transpose() * t_w_c;
}

void FrameManager::cvtSphere2Plane(const int cam_idx, Eigen::Vector3d &pts, bool force_unit_plane)
{
  if(!USE_BEARING_VECTOR || force_unit_plane)
  {
    // assert(abs(pts.norm() - 1.0f) < 10e-3);

    // Eigen::VectorXd &intrinsic = cam_intrinsics_[cam_idx];
    // if(cam_type_ == Camera::Type::kMei)
    // {    
    //     double xi = intrinsic[4];
    //     double x = pts[0] / (pts[2] + xi);
    //     double y = pts[1] / (pts[2] + xi);
    //     double z = pts[2] / (pts[2] + xi);
    //     pts = Eigen::Vector3d(x/z, y/z, 1.0);
    //     return;
    // }  

    double x = pts.x();
    double y = pts.y();
    double z = pts.z();
    pts = Eigen::Vector3d(x / z, y / z, 1.0);
  }
}

bool FrameManager::isGoodPoint(const FramePtr &frame, const size_t ftr_idx)
{
  if (!frame->landmark_vec_[ftr_idx])
    return false;

  const PointPtr& point = frame->landmark_vec_[ftr_idx];
  if(!isPointVisible(frame, point))
    return false;

  const FeatureType &ftr_type = frame->type_vec_[ftr_idx];
  if ((!USE_EDGELET && isEdgelet(ftr_type)) 
      || ftr_type == FeatureType::kOutlier)
    return false;

  // Transformation T_c_w = frame->T_cam_world();
  // Eigen::Vector3d pt_cam = T_c_w * point->pos_;
  // bool res_min_depth = min_depth_ > 0.0f ? pt_cam.z() < min_depth_ : false;
  // bool res_max_depth = max_depth_ > 0.0f ? pt_cam.z() > max_depth_ : false;
  // if(res_min_depth || res_max_depth)
  //   return false;
  
  return true;
}

float FrameManager::getDepthValueFromImage(const FramePtr &frame, 
    const cv::Mat &depth_image, int ftr_idx, double &depth)
{
  const float depth_img_min = frame->cam_->getDepthMin();
  const float depth_img_max = frame->cam_->getDepthMax();
  const float depth_img_scale = frame->cam_->getDepthScale();
  if(depth_img_min == 0.0f || depth_img_max == 0.0f || depth_img_scale == 0.0f)
  {
    std::cout << "depth min: " << depth_img_min << "; "
        << "depth max: " << depth_img_max << "; "
        << "depth scale: " << depth_img_scale << "\n";
    std::cout << "depth_img_min, depth_img_max and depth_img_scale shouldn't be 0.0f\n";
    exit(-1);
  }

  Eigen::Vector2d px = frame->px_vec_.col(ftr_idx);
  int x = std::round(px.x()), y = std::round(px.y());
  depth = (float)depth_image.at<ushort>(y, x) / depth_img_scale;

  if(depth < depth_img_min || depth > depth_img_max)
    return true;
  
  return false;
}

bool FrameManager::checkParallax(const int frame_count, const int cam_idx)
{
  if (frame_count < 2 || last_track_num_ < 20) // || new_feature_num_ > 0.5 * last_track_num_)
    return true;

  int parallax_num = 0;
  double parallax_sum = 0;

  int i = frame_count - 1; // second last frame
  int j = frame_count - 2; // third last frame

  for(auto &kv : feature_per_ids_)
  {
    size_t feature_id = kv.first;
    auto &feature_per_id = kv.second;

    auto &obs_frames = feature_per_id.obs_frames;

    if(obs_frames.empty()) 
      continue;    

    auto iter_i = std::find(obs_frames.begin(), obs_frames.end(), i);
    auto iter_j = std::find(obs_frames.begin(), obs_frames.end(), j);
    if(iter_j == obs_frames.end() || iter_i == obs_frames.end())
      continue;

    auto &feature_per_frame = feature_per_id.feature_per_frame;

    bool obs_by_i = feature_per_frame[i].isObsByCam(cam_idx);
    bool obs_by_j = feature_per_frame[j].isObsByCam(cam_idx);

    if(!obs_by_i || obs_by_j)
      continue;

    const Eigen::Vector3d &pts_i = feature_per_frame[i].getObservation(cam_idx);
    const Eigen::Vector3d &pts_j = feature_per_frame[j].getObservation(cam_idx);
    
    double parallax = (pts_j - pts_i).head<2>().norm();
    parallax_sum  += parallax;
    ++parallax_num;
  }
  double parallax_avg = parallax_sum / parallax_num * FOCAL_LENGTH;

  std::cout << "parallax_sum: " << parallax_sum << "\t"
        << "parallax_num: " << parallax_num << "\t"
        << "parallax_avg: " << parallax_avg << "\n";

  if (parallax_num == 0)
    return true;
  else
    return parallax_avg >= 10.0;
}

bool FrameManager::checkParallax(int frame_count, const FrameBundlePtr &new_frames)
{
  if(frame_count == 0)
    return true;

  int parallax_num = 0;
  double parallax_sum = 0.0f;
  const FrameBundlePtr &prv_frames = (*states_)[frame_count - 1].frame_bundle;
 
  for(size_t cam_idx = 0; cam_idx < new_frames->size(); ++cam_idx)
  {
    const FramePtr &frame_i = prv_frames->at(cam_idx);
    const FramePtr &frame_j = new_frames->at(cam_idx);

    std::unordered_map<size_t, size_t> feature_matches;
    getFeatureMatches(frame_i, frame_j, feature_matches);

    parallax_num += feature_matches.size();

    for(auto &iter : feature_matches)
    {
      size_t idx_i = iter.first;
      size_t idx_j = iter.second;

      Eigen::Vector3d pts_i = frame_i->f_vec_.col(idx_i);
      cvtSphere2Plane(cam_idx, pts_i, true);

      Eigen::Vector3d pts_j = frame_j->f_vec_.col(idx_j);
      cvtSphere2Plane(cam_idx, pts_j, true);

      double parallax = (pts_j - pts_i).head<2>().norm();
      parallax_sum  += parallax;
    }
  }

  double parallax_avg = parallax_sum / parallax_num * FOCAL_LENGTH;
  std::cout << "parallax_avg: " << parallax_avg << "\n";

  if (parallax_num == 0)
    return true;
  else
    return parallax_avg >= MIN_PARALLAX;
}

void FrameManager::removeFailures(std::vector<size_t> &failure_index)
{
  std::lock_guard<std::mutex> lock(ftr_mutex_);
  
  for(auto &feature_id : feature_ids_)
  {
    if(feature_per_ids_.count(feature_id) == 0)
      continue;

    auto &feature_per_id = feature_per_ids_[feature_id];

    if(feature_per_id.solve_flag != 2)
      continue;

    feature_per_ids_.erase(feature_id);
    failure_index.emplace_back(feature_id);
  }
}

void FrameManager::setDepth(const int frame_count,
    const std::vector<size_t> &opt_feature_ids, 
    const std::map<size_t, size_t> &ftr_idx_map,
    double **x)
{
  int fix_cnt = 0;

  std::lock_guard<std::mutex> lock(ftr_mutex_);

  for(auto &feature_id : opt_feature_ids)
  {
    if(feature_per_ids_.count(feature_id) == 0)
      continue;
      
    int feature_index = ftr_idx_map.at(feature_id);

    double inv_depth = x[feature_index][0];
    double estimated_depth = 1.0 / inv_depth;
    auto &feature_per_id = feature_per_ids_[feature_id];
    feature_per_id.estimated_depth = estimated_depth;

    // double depth_error = estimated_depth - prv_estimated_depth;
    // if(std::abs(depth_error) <= 0.05f 
    //     && estimated_depth > 0.0f 
    //     && estimated_depth  < 5.0f)
    // {
    //   // feature_per_id.solve_flag = -1;
    //   ++fix_cnt;
    // }
    // std::cout << feature_id << ": estimated_depth: " << estimated_depth << "\n";

    /*
    if(frame_count == WINDOW_SIZE && solver_flag == SolverFlag::NON_LINEAR)
    {
      if(estimated_depth < 0.0f)
        feature_per_id.solve_flag = 2;
      else
        feature_per_id.solve_flag = -1;

      continue;        
    } 
    */

    if(feature_per_id.solve_flag == -1)
      continue;
    
    if(estimated_depth < 0.0f)
      feature_per_id.solve_flag = 2;
    else
      feature_per_id.solve_flag = 1;
  }
}

void FrameManager::removeFirstNew(const int frame_count)
{
  int marg_frame = frame_count;

  std::lock_guard<std::mutex> lock(ftr_mutex_);

  for(size_t lm_idx = 0; lm_idx < feature_ids_.size(); ++lm_idx)
  {
    const size_t feature_id = feature_ids_[lm_idx];
    if(feature_per_ids_.count(feature_id) == 0)
      continue;

    auto &feature_per_id = feature_per_ids_[feature_id];

    auto &obs_frames = feature_per_id.obs_frames;
    auto &feature_per_frame = feature_per_id.feature_per_frame;

    if(obs_frames.empty()) 
      continue;

    const int &end_frame = obs_frames.back();

    if(end_frame < marg_frame)
      continue;

    auto marg_iter = find(obs_frames.begin(), obs_frames.end(), marg_frame);
    if(marg_iter != obs_frames.end())
    {
      obs_frames.erase(marg_iter);
      feature_per_frame.erase(marg_frame);
    }

    if(obs_frames.empty())
    {
      // feature_per_ids_.erase(feature_id);
      feature_per_id.slide_out = true;
      continue;
    }
  }

  slideOutFeatures();
}

void FrameManager::removeSecondNew(const int frame_count)
{
  std::vector<int> feature_remove;
  int marg_frame = frame_count - 1;

  std::lock_guard<std::mutex> lock(ftr_mutex_);

  for(auto &feature_id : feature_ids_)
  {
    if(feature_per_ids_.count(feature_id) == 0)
      continue;

    auto &feature_per_id = feature_per_ids_[feature_id];
    auto &obs_frames = feature_per_id.obs_frames;
    auto &feature_per_frame = feature_per_id.feature_per_frame;

    if(obs_frames.empty()) 
      continue;    

    int start_frame = obs_frames.front();
    int end_frame = obs_frames.back();

    if(start_frame == frame_count)
    {
      assert(obs_frames.size() == 1);
      obs_frames.front() = start_frame - 1;
      feature_per_frame[obs_frames.front()] = feature_per_frame[start_frame];
      feature_per_frame.erase(start_frame);
    }
    else
    {    
      if(end_frame < marg_frame)
      {
        continue;
      }

      if(start_frame == marg_frame && end_frame == frame_count)
      {        
        const std::vector<int> obs_cam_ids_i = feature_per_frame[marg_frame].getObsCamIds();

        int cam_idx_i = obs_cam_ids_i.front();
        Eigen::Matrix3d marg_R = (*states_)[marg_frame].R * (*ric_)[cam_idx_i];
        Eigen::Vector3d marg_P = (*states_)[marg_frame].R * (*tic_)[cam_idx_i] + (*states_)[marg_frame].P;
        Eigen::Vector3d pts_i = feature_per_frame[marg_frame].getObservation(cam_idx_i);

        const std::vector<int> obs_cam_ids_j = feature_per_frame[frame_count].getObsCamIds();
        int cam_idx_j = obs_cam_ids_j.front();
        Eigen::Matrix3d new_R = (*states_)[frame_count].R * (*ric_)[cam_idx_j];
        Eigen::Vector3d new_P = (*states_)[frame_count].R * (*tic_)[cam_idx_j] + (*states_)[frame_count].P;
        
        const double &estimated_depth = feature_per_id.estimated_depth;

        Eigen::Vector3d pts_cam_i = pts_i * estimated_depth;
        Eigen::Vector3d pts_world = marg_R * pts_cam_i + marg_P;
        Eigen::Vector3d pts_cam_j = new_R.transpose() * (pts_world - new_P);

        if(pts_cam_j.z() > 0)
        {
          double estimated_depth = USE_BEARING_VECTOR ? pts_cam_j.norm() : pts_cam_j.z();
          feature_per_id.estimated_depth = estimated_depth;
        }
        else
          feature_per_id.solve_flag = 2;
      }

      auto marg_iter = find(obs_frames.begin(), obs_frames.end(), marg_frame);
      if(marg_iter != obs_frames.end())
      {
        obs_frames.erase(marg_iter);
        feature_per_frame.erase(marg_frame);
      }

      auto new_iter = find(obs_frames.begin(), obs_frames.end(), frame_count);
      if(new_iter != obs_frames.end())
      {
        *new_iter -= 1;
        // feature_per_frame[frame_count - 1] = feature_per_frame[frame_count];
        feature_per_frame[frame_count - 1].swap(feature_per_frame[frame_count]);
        feature_per_frame.erase(frame_count);

        assert(obs_frames.back() != frame_count);
      }

      if(obs_frames.empty())
      {
        feature_per_ids_.erase(feature_id);
        continue;
      }
    }
  }
}

void FrameManager::removeOld()
{
  std::lock_guard<std::mutex> lock(ftr_mutex_);

  for(size_t lm_idx = 0; lm_idx < feature_ids_.size(); ++lm_idx)
  {
    const size_t feature_id = feature_ids_[lm_idx];
    if(feature_per_ids_.count(feature_id) == 0)
      continue;

    auto &feature_per_id = feature_per_ids_[feature_id];
    auto &obs_frames = feature_per_id.obs_frames;
    auto &feature_per_frame = feature_per_id.feature_per_frame;

    if(obs_frames.empty()) 
      continue;

    int start_frame = obs_frames.front();
    if(start_frame != 0)
    {
      feature_per_id.slideOld();
    }
    else
    {
      feature_per_id.removeOld();

      if(obs_frames.empty())
      {
        // feature_per_ids_.erase(feature_id);
        feature_per_id.slide_out = true;
        continue;
      }
      else
        feature_per_id.slideOld();
    }
  }

  slideOutFeatures();
}

void FrameManager::removeBackShiftDepth()
{
  std::lock_guard<std::mutex> lock(ftr_mutex_);

  for(size_t lm_idx = 0; lm_idx < feature_ids_.size(); ++lm_idx)
  {
    size_t feature_id = feature_ids_[lm_idx];

    if(feature_per_ids_.count(feature_id) == 0)
      continue;

    auto &feature_per_id = feature_per_ids_[feature_id];
    auto &obs_frames = feature_per_id.obs_frames;
    auto &feature_per_frame = feature_per_id.feature_per_frame;

    if(obs_frames.empty()) 
      continue;    

    if(obs_frames.front() != 0)
    {
      feature_per_id.slideOld();
    }
    else
    {
      if(obs_frames.size() < 2)
      {
        // feature_per_ids_.erase(feature_id);
        feature_per_id.slide_out = true;
        continue;
      }

      int frame_i = obs_frames[0];   
      auto &feature_per_cam_i = feature_per_frame[frame_i];

      int cam_idx_i = feature_per_cam_i.getObsCamIds().front();
      Eigen::Vector3d pts_i = feature_per_cam_i.getObservation(cam_idx_i);

      Eigen::Matrix3d marg_R = (*states_)[frame_i].R * (*ric_)[cam_idx_i];
      Eigen::Vector3d marg_P = (*states_)[frame_i].R * (*tic_)[cam_idx_i] 
                                + (*states_)[frame_i].P;

      int frame_j = obs_frames[1];
      auto &feature_per_cam_j = feature_per_frame[frame_j];
      
      int cam_idx_j = feature_per_cam_j.getObsCamIds().front();
      Eigen::Matrix3d new_R = (*states_)[frame_j].R * (*ric_)[cam_idx_j];
      Eigen::Vector3d new_P = (*states_)[frame_j].R * (*tic_)[cam_idx_j] 
                              + (*states_)[frame_j].P;
      
      double estimated_depth_i = feature_per_id.estimated_depth;

      Eigen::Vector3d pts_cam_i = pts_i * estimated_depth_i;
      Eigen::Vector3d pts_world = marg_R * pts_cam_i + marg_P;
      Eigen::Vector3d pts_j = new_R.transpose() * (pts_world - new_P);

      if(pts_j.z() > 0)
      {
        double estimated_depth_j = USE_BEARING_VECTOR ? pts_j.norm() : pts_j.z();
        feature_per_id.estimated_depth = estimated_depth_j;
      }
      else
        feature_per_id.solve_flag = 2;

      feature_per_id.removeOld();

      feature_per_id.slideOld();
    }
  }

  slideOutFeatures();
}

void FrameManager::slideOutFeatures()
{
  for(size_t lm_idx = 0; lm_idx < feature_ids_.size(); ++lm_idx)
  {
    size_t feature_id = feature_ids_[lm_idx];

    if(feature_per_ids_.count(feature_id) == 0)
      continue;
    
    if(feature_per_ids_[feature_id].slide_out)
      feature_per_ids_.erase(feature_id);
  }  
}

void FrameManager::getDepth(const int frame_count, const SolverFlag solver_flag, 
  const bool opt, std::vector<size_t> &opt_feature_ids, double **para_Feature)
{
  int feature_index = -1;

  unsigned int obs_thresh = is_kf_ ? obs_thresh_ : obs_thresh_ + 1;
  // unsigned int obs_thresh = obs_thresh_;

  if(!opt)
  {
    for(auto feature_id : opt_feature_ids)
    {
      double estimated_depth = feature_per_ids_[feature_id].estimated_depth;
      para_Feature[++feature_index][0] = 1.0 / estimated_depth;
    }

    return;
  }

  opt_feature_ids.clear();

  std::vector<std::pair<size_t, size_t>> total_obs;
  for(auto &kv : feature_per_ids_)
  {
    size_t feature_id = kv.first;
    auto &feature_per_id = kv.second;
    size_t obs_cnt = feature_per_id.obs_frames.size();
    
    if(obs_cnt < obs_thresh)
      continue;
      
    total_obs.push_back(std::make_pair(feature_id, obs_cnt));
  }

  sort(total_obs.begin(), total_obs.end(), [](
    const std::pair<size_t, size_t>a, const std::pair<size_t, size_t>b){
      return a.second > b.second;
  });

  if(solver_flag == SolverFlag::INITIAL)
  {
    for(auto &kv : total_obs)
    {
      const size_t& feature_id = kv.first;

      double estimated_depth = feature_per_ids_[feature_id].estimated_depth;

      // if(kf_all_opt_size_ > 0 && (int)opt_feature_ids.size() >= kf_all_opt_size_)
      //   break;

      para_Feature[++feature_index][0] = 1.0 / estimated_depth;
      opt_feature_ids.emplace_back(feature_id);
    }
    // std::cout << "0. opt_feature_ids size: " << opt_feature_ids.size() << "\n";

    return;
  }

  /*
  if(!is_kf_)
  {
    for(auto &kv : total_obs)
    {
      int opt_size = opt_feature_ids.size();
      if(nkf_all_opt_size_ > 0 && opt_size >= nkf_all_opt_size_)
        break;

      const size_t& feature_id = kv.first;
      if(cur_feature_ids_.count(feature_id) == 0)
        continue;

      double estimated_depth = feature_per_ids_[feature_id].estimated_depth;
      if(!isDepthOK(estimated_depth))
        continue;

      para_Feature[++feature_index][0] = 1.0 / estimated_depth;
      opt_feature_ids.emplace_back(feature_id);
    }
    return;
  }
  */

  int kf_new_opt_size = kf_new_opt_size_;
  int kf_all_opt_size = kf_all_opt_size_;

  getGridObs(frame_count);

  size_t cam_size = camera_bundle_->getNumCameras();
  for(size_t cam_id = 0; cam_id < cam_size; ++cam_id)
  {
    for(auto grid_obs : grid_obs_[cam_id])
    {
      int add_cnt = 0;
      for(auto kv : grid_obs.second)
      {
        size_t feature_id = kv.first;
        size_t obs_cnt = kv.second;

        if(obs_cnt < obs_thresh)
          continue;

        auto iter = std::find(opt_feature_ids.begin(), opt_feature_ids.end(), feature_id);
        if(iter != opt_feature_ids.end())
          continue;

        double estimated_depth = feature_per_ids_[feature_id].estimated_depth;
        if(!isDepthOK(estimated_depth))
          continue;

        para_Feature[++feature_index][0] = 1.0 / estimated_depth;
        opt_feature_ids.emplace_back(feature_id);

        ++add_cnt;

        if(is_kf_ && add_cnt >= 1)
          break;

        if(!is_kf_ && add_cnt >= 3)
          break;
      }
    }
    // std::cout << "cam id: " << cam_id << "; "
    //           << "add_cnt: " << add_cnt << "\n";
  }
  // std::cout << "1. opt_feature_ids size: " << opt_feature_ids.size() << "\n";

  for(auto &kv : total_obs)
  {
    int opt_size = opt_feature_ids.size();
    if(is_kf_ && kf_new_opt_size > 0 && opt_size >= kf_new_opt_size)
      break;
      
    if(!is_kf_ && nkf_all_opt_size_ > 0 && opt_size >= nkf_all_opt_size_)
      break;
    
    const size_t& feature_id = kv.first;
    if(cur_feature_ids_.count(feature_id) == 0)
      continue;

    auto iter = std::find(opt_feature_ids.begin(), opt_feature_ids.end(), feature_id);
    if(iter != opt_feature_ids.end())
      continue;

    double estimated_depth = feature_per_ids_[feature_id].estimated_depth;
    if(!isDepthOK(estimated_depth))
      continue;

    para_Feature[++feature_index][0] = 1.0 / estimated_depth;
    opt_feature_ids.emplace_back(feature_id);
  }
  // std::cout << "2. opt_feature_ids size: " << opt_feature_ids.size() << "\n";

  if(!is_kf_) return;

  /*
  for(auto &kv : total_obs)
  {
    int opt_size = opt_feature_ids.size();
    if(kf_all_opt_size > 0 && opt_size >= kf_all_opt_size)
      break;

    const size_t& feature_id = kv.first;
    
    auto &obs_frames = feature_per_ids_[feature_id].obs_frames;
    if(obs_frames.front() != 0)
      continue;

    auto iter = std::find(opt_feature_ids.begin(), opt_feature_ids.end(), feature_id);
    if(iter != opt_feature_ids.end())
      continue;

    double estimated_depth = feature_per_ids_[feature_id].estimated_depth;
    if(!isDepthOK(estimated_depth))
      continue;

    para_Feature[++feature_index][0] = 1.0 / estimated_depth;
    opt_feature_ids.emplace_back(feature_id);
  }
  // std::cout << "3. opt_feature_ids size: " << opt_feature_ids.size() << "\n";
  */

  for(auto &kv : total_obs)
  {
    int opt_size = opt_feature_ids.size();
    if(kf_all_opt_size > 0 && opt_size >= kf_all_opt_size)
      break;

    const size_t& feature_id = kv.first;
    
    auto iter = std::find(opt_feature_ids.begin(), opt_feature_ids.end(), feature_id);
    if(iter != opt_feature_ids.end())
      continue;

    double estimated_depth = feature_per_ids_[feature_id].estimated_depth;
    if(!isDepthOK(estimated_depth))
      continue;

    para_Feature[++feature_index][0] = 1.0 / estimated_depth;
    opt_feature_ids.emplace_back(feature_id);
  }
  // std::cout << "4. opt_feature_ids size: " << opt_feature_ids.size() << "\n";
}

bool FrameManager::isDepthOK(const double estimated_depth)
{
  if(min_depth_ > 0.0f && estimated_depth < min_depth_)
    return false;
    
  if(max_depth_ > 0.0f && estimated_depth > max_depth_)
    return false;
  
  // if(estimated_depth < 0.0f || estimated_depth == INIT_DEPTH)
  //   return false;

  return true;
}

void FrameManager::getCorresponding(
  int frame_count_l, int frame_count_r, 
  std::vector<size_t> &corres_feature_id,
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &corres_pts)
{    
  // juse for test
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> corres_px;

  int cam_idx = 0;
  corres_pts.clear();
  corres_feature_id.clear();

  for(auto &feature_id : feature_ids_)
  {
    if(feature_per_ids_.count(feature_id) == 0)
      continue;

    auto &feature_per_id = feature_per_ids_[feature_id];
    auto &obs_frames = feature_per_id.obs_frames;
    auto &feature_per_frame = feature_per_id.feature_per_frame;

    if(obs_frames.empty())
      continue;

    std::vector<int>::iterator iter_l, iter_r;
    iter_l = std::find(obs_frames.begin(), obs_frames.end(), frame_count_l);
    iter_r = std::find(obs_frames.begin(), obs_frames.end(), frame_count_r);
    if(iter_l == obs_frames.end() || iter_r == obs_frames.end())
      continue;

    auto &feature_per_frame_l = feature_per_frame[frame_count_l];
    auto &feature_per_frame_r = feature_per_frame[frame_count_r];

    bool obs_by_l = feature_per_frame_l.isObsByCam(cam_idx);
    bool obs_by_r = feature_per_frame_r.isObsByCam(cam_idx);
    if(!obs_by_l || !obs_by_r)
      continue;

    const Eigen::Vector2d &px_l = feature_per_frame_l.getObsPixel(cam_idx);
    const Eigen::Vector3d &pts_l = feature_per_frame_l.getObsNormPlane(cam_idx);

    const Eigen::Vector2d &px_r = feature_per_frame_r.getObsPixel(cam_idx);
    const Eigen::Vector3d &pts_r = feature_per_frame_r.getObsNormPlane(cam_idx);

    corres_feature_id.emplace_back(feature_id);
    corres_px.emplace_back(std::make_pair(px_l, px_r));
    corres_pts.emplace_back(std::make_pair(pts_l, pts_r));
  }

  drawCorrespondingFeature(frame_count_l, frame_count_r, corres_px, "mono_init");
}

void FrameManager::getFeatureMatches(
  const FramePtr& frame1, const FramePtr& frame2,
  std::unordered_map<size_t, size_t>& matches_12)
{
  std::unordered_map<int, size_t> trackid_slotid_map;
  // for(size_t i = 0; i < numFeatures(frame1); ++i)
  for(unsigned int i = 0; i < frame1->track_id_vec_.rows(); ++i)
  {
    int track_id_1 = frame1->track_id_vec_(i);  // landmark id
    if(track_id_1 >= 0)                         // 只有存在对应的3的点, track_id_1才不为负值
      trackid_slotid_map[track_id_1] = i;       // <landmark id, feature id>
  }

  // for(size_t i = 0; i < numFeatures(frame2); ++i)
  for(unsigned int i = 0; i < frame2->track_id_vec_.rows(); ++i)
  {
    int track_id_2 = frame2->track_id_vec_(i);
    if(track_id_2 >= 0)
    {
      const auto it = trackid_slotid_map.find(track_id_2);
      if(it != trackid_slotid_map.end())
        matches_12[it->second] = i;
    }
  }
}

void FrameManager::addMonoFrameBundle(const int frame_count, const FrameBundlePtr &new_frames)
{
  std::unordered_map<size_t, int> match_status;
  // rejectWithF(frame_count, new_frames, match_status);
  
  int depth_num = 0;
  last_track_num_ = 0;
  new_feature_num_ = 0;

  size_t cam_size = new_frames->size();
  for(size_t cam_idx = 0; cam_idx < cam_size; ++cam_idx)
  {
    const FramePtr &frame = new_frames->at(cam_idx);
    cv::Mat depth_image = frame->depth_image_.clone();

    for(size_t ftr_idx = 0; ftr_idx < numFeatures(frame); ++ftr_idx)
    {
      if(ftr_idx >= (size_t)frame->track_id_vec_.size())
        break;

      int pyr_level = frame->level_vec_(ftr_idx);
      int feature_id = frame->track_id_vec_(ftr_idx);
      if(feature_id == -1)
        continue;

      const Eigen::Vector2d &pixel = frame->px_vec_.col(ftr_idx);
      const Eigen::Vector3d &bearing_vector = frame->f_vec_.col(ftr_idx);

      if(feature_per_ids_.count(feature_id) == 0)
      {
        FeaturePerId feature_per_id(0.0f, 0);
        feature_per_ids_[feature_id] = feature_per_id;
        ++new_feature_num_;
      }
      else
        ++last_track_num_;

      feature_per_ids_[feature_id].addObservation(frame_count, 
          cam_size, cam_idx, ftr_idx, pyr_level, pixel, bearing_vector);

      if(!depth_image.empty()) 
      {
        double depth = 0.0f;
        float res = getDepthValueFromImage(frame, depth_image, ftr_idx, depth);
        
        if(res)
        {
          feature_per_ids_[feature_id].solve_flag = 1;
          feature_per_ids_[feature_id].estimated_depth = depth; 

          ++depth_num;
        }
      }
    }
  }

  updateFeatureIds();

  std::cout << "mono: "
            << "frame count: " << frame_count << "; " 
            // << "feature size: " << numFeatures(frame) << "; "
            << "last_track_num_: " << last_track_num_ << "; "
            << "new_feature_num_: " << new_feature_num_ << "; "
            << "depth_feature_num: " << depth_num << "\n";
}

void FrameManager::addFrameBundle(const int frame_count, 
    const FrameBundlePtr &new_frames, const bool verbose)
{
  assert(new_frames->size() == camera_bundle_->getNumCameras());

  is_kf_ = isKeyframe(new_frames);

  cur_frame_obs_.clear();
  cur_feature_ids_.clear();

  unsigned int cam_size = new_frames->size();

  for(unsigned int cam_idx = 0; cam_idx < cam_size; ++cam_idx)
  {
    int new_added_num = 0, last_track_num = 0;
    const FramePtr &frame = new_frames->at(cam_idx);
    size_t feature_size = numFeatures(frame);
    if(is_kf_ && !isKeyframe(frame))
      continue;

    for(size_t ftr_idx = 0; ftr_idx < feature_size; ++ftr_idx)
    {
      if(ftr_idx >= (size_t)frame->track_id_vec_.size())
        continue;

      const int pyr_level =  frame->level_vec_(ftr_idx);
      const int feature_id = frame->track_id_vec_(ftr_idx);

      const Eigen::Vector2d &pixel = frame->px_vec_.col(ftr_idx);
      const Eigen::Vector3d &bearing_vector = frame->f_vec_.col(ftr_idx);

      const PointPtr& point = frame->landmark_vec_[ftr_idx];
      if(point != nullptr)
      {
        assert(feature_id == getPointId(point));
        
        // if(point->obs_.size() < 2)
        //   continue;

        if(!isKeyframe(frame) && feature_per_ids_.count(feature_id) == 0)
          continue;

        bool is_constant = point->is_constant_;

        Eigen::Vector3d pt_world = getLandmarkPos(point);
        Eigen::Vector3d pt_cam = frame->T_cam_world() * pt_world;
        double estimated_depth = USE_BEARING_VECTOR ? pt_cam.norm() : pt_cam.z();

        if(!isGoodPoint(frame, ftr_idx))
          continue;

        if(feature_per_ids_.count(feature_id) == 0)
        {
          int solve_flag = is_constant ? -1 : 1;

          FeaturePerId feature_per_id(estimated_depth, solve_flag);
          feature_per_ids_[feature_id] = feature_per_id;

          ++new_added_num;
        }
        else
          ++last_track_num;

        feature_per_ids_[feature_id].addObservation(frame_count, 
            cam_size, cam_idx, ftr_idx, pyr_level, pixel, bearing_vector);

        cur_feature_ids_.insert(feature_id);
        cur_frame_obs_[cam_idx].insert(feature_id);
      }
      else if(feature_id != -1)
      {
        continue;

        if(feature_per_ids_.count(feature_id) == 0)
        {
          FeaturePerId feature_per_id(0.0f, 0);
          feature_per_ids_[feature_id] = feature_per_id;
        }

        feature_per_ids_[feature_id].addObservation(frame_count, 
            cam_size, cam_idx, ftr_idx, pyr_level, pixel, bearing_vector);

        cur_feature_ids_.insert(feature_id);
        cur_frame_obs_[cam_idx].insert(feature_id);
      }
    }

    if(verbose && is_kf_)
    {
      std::cout << "cam: " << cam_idx << "; "             
            << "track_id size: " << frame->track_id_vec_.size() << "; "
            << "feat size: " << feature_size << ", "
            << "new added: " << new_added_num << ", "
            << "last track: " << last_track_num << "\n";
    }
  }

  updateFeatureIds();
}

void FrameManager::getTrackingState(const int frame_count)
{
  if(frame_count == 0)
    return;

  int track_cnt = 0;
  int prv_frame_count = frame_count - 1;

  (*states_)[frame_count].tracked_feat_cnt = 0;

  for(auto &feature_id : cur_feature_ids_)
  {
    auto &obs_frames = feature_per_ids_[feature_id].obs_frames;

    if(obs_frames.empty())
      continue;

    auto iter = std::find(obs_frames.begin(), obs_frames.end(), prv_frame_count);
    if(iter == obs_frames.end())
      continue;

    ++track_cnt;
  }

  (*states_)[frame_count].tracked_feat_cnt = track_cnt;
  // std::cout << "tracked_feat_cnt: " << track_cnt << "\n";
}

void FrameManager::getTrackingState(
  const int frame_count, const FrameBundlePtr &frame_bundle)
{
  if(frame_count == 0)
    return;
  
  float pixel_error = 6.0f;
  int totoal_cnt = 0, match_cnt = 0;
  (*states_)[frame_count].tracked_feat_cnt = 0;

  for(size_t i = 0; i < frame_bundle->size(); ++i)
  {
    const FramePtr frame0 = (*states_)[frame_count - 1].frame_bundle->at(i);
    const FramePtr frame1 = frame_bundle->at(i);

    std::vector<uchar> match_status;
    std::unordered_map<size_t, size_t> feature_matches;
    getFeatureMatches(frame0, frame1, feature_matches);
    rejectWithF(frame0, frame1, feature_matches, match_status, pixel_error);
    // drawStereoMatches(frame0, frame1, feature_matches, match_status);

    for(int idx = 0; idx < (int)match_status.size(); ++idx)
    {
      if(match_status[idx])
        ++match_cnt;
    }
    totoal_cnt += match_cnt;
  }
  
  (*states_)[frame_count].tracked_feat_cnt = totoal_cnt;

  std::cout << "Tracking cnt: " << totoal_cnt << "; ";
  std::cout << "Match cnt: " << match_cnt << "\n";
}

void FrameManager::getGridObs(const int frame_count)
{  
  if(grid_border_.empty())
    return;
  
  if(!grid_obs_.empty())
    grid_obs_.clear();

  const FrameBundlePtr frame_bundle = (*states_)[frame_count].frame_bundle;
  
  for(unsigned int cam_idx = 0; cam_idx < frame_bundle->size(); ++cam_idx)
  {
    const size_t img_width = camera_bundle_->getCamera(cam_idx).imageWidth();
    const size_t img_height = camera_bundle_->getCamera(cam_idx).imageHeight();

    const size_t grid_cols = std::floor((float) img_width / grid_width_);
    const size_t grid_rows = std::floor((float)img_height / grid_height_);

    const FramePtr &frame = frame_bundle->at(cam_idx);
    size_t feature_size = numFeatures(frame);

    for(size_t ftr_idx = 0; ftr_idx < feature_size; ++ftr_idx)
    {
      if(ftr_idx >= (size_t)frame->track_id_vec_.size())
        break;

      Eigen::Vector2d obs_px = frame->px_vec_.col(ftr_idx);
      if(obs_px.x() < grid_border_[cam_idx]["left"] 
        || obs_px.x() >= grid_border_[cam_idx]["right"])
        continue;

      if(obs_px.y() < grid_border_[cam_idx]["top"]
        || obs_px.y() >= grid_border_[cam_idx]["bottom"])
        continue;

      int feature_id = frame->track_id_vec_(ftr_idx);
      
      if(!cur_feature_ids_.count(feature_id))
        continue;

      int col_begin = obs_px.x() - grid_border_[cam_idx]["left"];
      int row_begin = obs_px.y() - grid_border_[cam_idx]["top"];
      int col = std::floor((float)col_begin / grid_width_);
      int row = std::floor((float)row_begin / grid_height_);

      int grid_num = row * grid_cols + col;

      size_t obs_cnt = 0;
      obs_cnt = feature_per_ids_[feature_id].obs_frames.size();

      const std::pair<size_t, size_t> ftr_obs_cnt(feature_id, obs_cnt);
      grid_obs_[cam_idx][grid_num].emplace_back(ftr_obs_cnt);
    }

    for(auto &grid_obs : grid_obs_[cam_idx])
    {
      sort(grid_obs.second.begin(), grid_obs.second.end(), [](
        const std::pair<size_t, size_t>a, const std::pair<size_t, size_t>b){
          return a.second > b.second;
      });
    }
  }
}


bool FrameManager::triangulateDepth(
  const Eigen::Matrix4d& T_cur_ref,
  const Eigen::Vector3d& f_ref,
  const Eigen::Vector3d& f_cur,
  double &depth)
{
  Eigen::Matrix3d R_cur_ref = T_cur_ref.block<3, 3>(0, 0);
  Eigen::Vector3d t_cur_ref = T_cur_ref.block<3, 1>(0, 3);

  Eigen::Matrix<double, 3, 2> A; 
  A << R_cur_ref * f_ref, f_cur;
  const Eigen::Matrix2d AtA = A.transpose() * A;

  depth = 0.0f;
  if(AtA.determinant() > 0.000001)
  {
    const Eigen::Vector2d depth2 = -AtA.inverse() * A.transpose() * t_cur_ref;
    depth = std::fabs(depth2[0]);
    return true;
  }
  return false;
}

// 三角化特征点, 获取其在世界系下的3D坐标
void FrameManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
            Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
  Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
  design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
  design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
  design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
  Eigen::Vector4d triangulated_point;
  triangulated_point =
        design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

void FrameManager::triangulatePoint(const Eigen::Matrix4d &T_rel, 
  const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j, double &depth)
{
  cv::Mat_<double> Ti(3, 4);
  Ti << 1, 0, 0, 0, 
        0, 1, 0, 0, 
        0, 0, 1, 0;

  cv::Mat_<double> Tj(3, 4);
  Tj << T_rel(0, 0), T_rel(0, 1), T_rel(0, 2), T_rel(0, 3), 
        T_rel(1, 0), T_rel(1, 1), T_rel(1, 2), T_rel(1, 3), 
        T_rel(2, 0), T_rel(2, 1), T_rel(2, 2), T_rel(2, 3);

  cv::Mat_<double> px_norm_i(2, 1);
  cv::Mat_<double> px_norm_j(2, 1);
  px_norm_i << pts_i.x() / pts_i.z(), pts_i.y() / pts_i.z();
  px_norm_j << pts_j.x() / pts_j.z(), pts_j.y() / pts_j.z();

  cv::Mat pts_3d(4, 1, CV_64FC4);
  Eigen::Vector3d pts_cam_i = Eigen::Vector3d::Zero();
  cv::triangulatePoints(Ti, Tj, px_norm_i, px_norm_j, pts_3d);
  // pts_cam_i.x() = pts_3d.at<double>(0, 0) / pts_3d.at<double>(3, 0);
  // pts_cam_i.y() = pts_3d.at<double>(1, 0) / pts_3d.at<double>(3, 0);
  // pts_cam_i.z() = pts_3d.at<double>(2, 0) / pts_3d.at<double>(3, 0);
  depth = pts_3d.at<double>(2, 0) / pts_3d.at<double>(3, 0);
}

bool FrameManager::solvePoseByPnP( 
    const std::vector<cv::Point2f> &pts_2d, 
    const std::vector<cv::Point3f> &pts_3d,
    Eigen::Matrix3d &R, Eigen::Vector3d &P)
{
  Eigen::Matrix3d R_initial;
  Eigen::Vector3d P_initial;

  R_initial = R.transpose();
  P_initial = -(R_initial * P);

  if (pts_2d.size() < 4)
  {
    printf("feature tracking not enough, please slowly move you device! \n");
    return false;
  }
  cv::Mat r, rvec, t, D, tmp_r;
  cv::eigen2cv(R_initial, tmp_r);
  cv::Rodrigues(tmp_r, rvec);
  cv::eigen2cv(P_initial, t);
  cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
  bool pnp_succ;
  pnp_succ = cv::solvePnP(pts_3d, pts_2d, K, D, rvec, t, 1);
  //pnp_succ = solvePnPRansac(pts_3d, pts_2d, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

  if(!pnp_succ)
  {
    printf("pnp failed ! \n");
    return false;
  }
  cv::Rodrigues(rvec, r);
  Eigen::MatrixXd R_pnp;
  cv::cv2eigen(r, R_pnp);
  Eigen::MatrixXd T_pnp;
  cv::cv2eigen(t, T_pnp);

  R = R_pnp.transpose();
  P = R * (-T_pnp);

  return true;
}

bool FrameManager::initFramePoseByPnP(
    const int frame_count, Eigen::Matrix3d &R_w_b, Eigen::Vector3d &t_w_b)
{
  if(frame_count == 0)
    return false;
  
  size_t cam_size = camera_bundle_->getNumCameras();
  for(size_t cam_idx_i = 0; cam_idx_i < cam_size; ++cam_idx_i)
  {
    std::vector<cv::Point2f> pts_2d;
    std::vector<cv::Point3f> pts_3d;
    const std::set<size_t> &cur_frame_obs = cur_frame_obs_[cam_idx_i];

    for(auto feature_id : cur_frame_obs)
    {
      auto &feature_per_id = feature_per_ids_[feature_id];
      const int &solve_flag = feature_per_id.solve_flag;
      const double &estimated_depth = feature_per_id.estimated_depth;

      if(estimated_depth <= 0.0f || estimated_depth == INIT_DEPTH)
        continue;

      if(!isDepthOK(estimated_depth))
        continue;

      auto &obs_frames = feature_per_id.obs_frames;
      auto &feature_per_frame = feature_per_id.feature_per_frame;

      if(obs_frames.empty()) 
        continue;    

      auto feature_per_cam_i = feature_per_frame[frame_count];
      const Eigen::Vector3d &cur_pts = feature_per_cam_i.getObsNormPlane(cam_idx_i);

      cv::Point2f point2d(cur_pts.x(), cur_pts.y());

      size_t start_frame = obs_frames.front();
      auto &feature_per_cam_j = feature_per_frame[start_frame];

      const Eigen::Matrix3d &R_w_b_j = (*states_)[start_frame].R;
      const Eigen::Vector3d &t_w_b_j = (*states_)[start_frame].P;

      std::vector<int> obs_cam_ids_j = feature_per_cam_j.getObsCamIds();      
      if(obs_cam_ids_j.empty())
        continue;

      int cam_idx_j = obs_cam_ids_j.front();
      const Eigen::Vector3d &pts_j = feature_per_cam_j.getObservation(cam_idx_j);

      Eigen::Vector3d pts_cam_j = pts_j * estimated_depth;
      Eigen::Vector3d pts_imu_j = (*ric_)[cam_idx_j] * pts_cam_j + (*tic_)[cam_idx_j];
      Eigen::Vector3d pts_world = R_w_b_j * pts_imu_j + t_w_b_j;

      cv::Point3f point3d(pts_world.x(), pts_world.y(), pts_world.z());
      
      pts_3d.emplace_back(point3d);
      pts_2d.emplace_back(point2d);
    }

    std::cout << "pts_2d size: " << pts_2d.size() << "; "
              << "pts_3d size: " << pts_3d.size() << "\n";

    const Eigen::Matrix3d &R_b_c_i = (*ric_)[cam_idx_i];
    const Eigen::Vector3d &t_b_c_i = (*tic_)[cam_idx_i];

    Eigen::Matrix3d R_w_b_i = (*states_)[frame_count].R;
    Eigen::Vector3d t_w_b_i = (*states_)[frame_count].P;

    Eigen::Matrix3d R_w_c_i = R_w_b_i * R_b_c_i;
    Eigen::Vector3d t_w_c_i = R_w_b_i * t_b_c_i + t_w_b_i;

    if(solvePoseByPnP(pts_2d, pts_3d, R_w_c_i, t_w_c_i))
    {
      R_w_b = R_w_c_i * R_b_c_i.transpose(); 
      t_w_b = -R_w_c_i * R_b_c_i.transpose() * t_b_c_i + t_w_c_i;
      return true;
    }
  }

  return false;
}

void FrameManager::triangulate()
{   
  int mono_triangulate_cnt = 0;
  int stereo_triangulate_cnt = 0;

  for(auto &feature_id : feature_ids_)
  {
    if(feature_per_ids_.count(feature_id) == 0)
      continue;

    auto &feature_per_id = feature_per_ids_[feature_id];

    if(feature_per_id.estimated_depth > 0)
      continue;

    auto &obs_frames = feature_per_id.obs_frames;
    auto &feature_per_frame = feature_per_id.feature_per_frame;

    if(obs_frames.empty()) 
      continue;    

    std::vector<int> obs_cams;
    int frame_idx = obs_frames.front();
    for(auto &frame_id : obs_frames)
    {
      obs_cams = feature_per_frame[frame_id].getObsCamIds();
      if(obs_cams.size() >= 2)
      {
        frame_idx = frame_id;
        break;
      }
    }
    
    if(obs_cams.size() >= 2)
    {
      int imu_i = frame_idx;
      
      int cam_idx_0 = obs_cams[0];
      int cam_idx_1 = obs_cams[1];

      auto &feature_per_cam = feature_per_frame[imu_i];
      
      const Eigen::Vector3d &pts0 = feature_per_cam.getObsBearingVector(cam_idx_0);
      const Eigen::Vector3d &pts1 = feature_per_cam.getObsBearingVector(cam_idx_1);

      double depth = 0.0f;
      Eigen::Matrix4d T_c0_c1 = Eigen::Matrix4d::Identity();
      T_c0_c1.block<3, 3>(0, 0) = (*ric_)[cam_idx_0].transpose() * (*ric_)[cam_idx_1];
      T_c0_c1.block<3, 1>(0, 3) = (*ric_)[cam_idx_0].transpose() * ((*tic_)[cam_idx_1] - (*tic_)[cam_idx_0]);

      triangulateDepth(T_c0_c1, pts1, pts0, depth);
      Eigen::Vector3d pts_cam = pts1 * depth;
      double estimated_depth = USE_BEARING_VECTOR ? pts_cam.norm() : pts_cam.z();
       
      if (estimated_depth > 0.1f)
        feature_per_id.estimated_depth = estimated_depth;
      else
        feature_per_id.estimated_depth = INIT_DEPTH;

      stereo_triangulate_cnt =  stereo_triangulate_cnt + (int)(estimated_depth > 0.1f);

      continue;
    }
    else if(obs_frames.size() > 1)
    {
      for(size_t idx = 1; idx < obs_frames.size(); ++idx)
      {
        int imu_i = obs_frames[0];
        int imu_j = obs_frames[idx];

        auto &feature_per_frame_i = feature_per_frame[imu_i];
        auto &feature_per_frame_j = feature_per_frame[imu_j];

        int cam_idx_i = feature_per_frame_i.getObsCamIds().front();
        int cam_idx_j = feature_per_frame_j.getObsCamIds().front();

        const Eigen::Vector3d &pts_i = feature_per_frame_i.getObsBearingVector(cam_idx_i);
        const Eigen::Vector3d &pts_j = feature_per_frame_j.getObsBearingVector(cam_idx_j);

        Eigen::Matrix4d T_w_ci, T_ci_w;
        getCameraPose(imu_i, cam_idx_i, T_w_ci, T_ci_w);

        Eigen::Matrix4d T_w_cj, T_cj_w;
        getCameraPose(imu_j, cam_idx_j, T_w_cj, T_cj_w);

        double depth = 0.0f;
        double estimated_depth = 0.0f;

        Eigen::Matrix4d T_ci_cj = T_ci_w * T_w_cj;
        // Eigen::Vector3d norm_pt_i = pts_i / pts_i.z();
        // Eigen::Vector3d norm_pt_j = pts_j / pts_j.z();
        Eigen::Vector3d pts_i_tmp = T_ci_cj.block<3, 3>(0, 0) * pts_j;
        float cos_parallax_rays = pts_i.dot(pts_i_tmp);
        if (cos_parallax_rays > 0.9997)
        {   
          feature_per_id.estimated_depth = INIT_DEPTH;
          feature_per_id.solve_flag = 0;
          continue;
        }

        Eigen::Matrix4d T_cj_ci = T_cj_w * T_w_ci;            
        triangulateDepth(T_cj_ci, pts_i, pts_j, depth);
        Eigen::Vector3d pts_cam_i = pts_i * depth;
        depth = USE_BEARING_VECTOR ? pts_cam_i.norm() : pts_cam_i.z();
        estimated_depth = depth;

        /*
        Eigen::Matrix<double, 3, 4> pose_i, pose_j;
        pose_i = T_ci_w.block<3, 4>(0, 0);
        pose_j = T_cj_w.block<3, 4>(0, 0);
        Eigen::Vector2d point_i = (pts_i / pts_i.z()).head(2);
        Eigen::Vector2d point_j = (pts_j / pts_j.z()).head(2);
        Eigen::Vector3d point3d = Eigen::Vector3d::Identity();
        triangulatePoint(pose_i, pose_j, point_i, point_j, point3d);
        Eigen::Vector3d pts_cam = T_ci_w.block<3, 3>(0, 0) * point3d + T_ci_w.block<3, 1>(0, 3);
        std::cout  << pts_cam.z() << "; ";
        // estimated_depth = pts_cam.z();


        triangulatePoint(T_cj_ci, pts_i, pts_j, depth);
        std::cout << depth << std::endl;
        // estimated_depth = depth;
        */

        if (estimated_depth > 0.1f)
        {
          feature_per_id.estimated_depth = estimated_depth;
          feature_per_id.solve_flag = 1;
          break;
        }
        else
        {
          feature_per_id.estimated_depth = INIT_DEPTH;
          feature_per_id.solve_flag = 0;
        }

        mono_triangulate_cnt = mono_triangulate_cnt + (int)(estimated_depth > 0.1f);

      }

      continue;
    }

    if(obs_frames.size() < obs_thresh_)
      continue;
    
    Eigen::MatrixXd svd_A(2 * obs_frames.size(), 4);

    int imu_i = obs_frames.front();
    auto &feature_per_frame_i = feature_per_frame[imu_i];
    int cam_idx_i = feature_per_frame_i.getObsCamIds().front();

    Eigen::Vector3d t0 = (*states_)[imu_i].R * (*tic_)[cam_idx_i] + (*states_)[imu_i].P;
    Eigen::Matrix3d R0 = (*states_)[imu_i].R * (*ric_)[cam_idx_i];

    int svd_idx = 0;
    for(auto &imu_j : obs_frames)
    {
      auto &feature_per_frame_j = feature_per_frame[imu_j];
      int cam_idx_j = feature_per_frame_j.getObsCamIds().front();

      Eigen::Vector3d t1 = (*states_)[imu_j].R * (*tic_)[cam_idx_j] + (*states_)[imu_j].P;
      Eigen::Matrix3d R1 = (*states_)[imu_j].R * (*ric_)[cam_idx_j];

      Eigen::Vector3d t = R0.transpose() * (t1 - t0);
      Eigen::Matrix3d R = R0.transpose() * R1;
      Eigen::Matrix<double, 3, 4> P;
      P.leftCols<3>() = R.transpose();
      P.rightCols<1>() = -R.transpose() * t;

      const Eigen::Vector3d &pts = feature_per_frame_j.getObsBearingVector(cam_idx_j);

      svd_A.row(svd_idx++) = pts[0] * P.row(2) - pts[2] * P.row(0);
      svd_A.row(svd_idx++) = pts[1] * P.row(2) - pts[2] * P.row(1);

      if(imu_i == imu_j)
        continue;
    }

    assert(svd_idx == svd_A.rows());
    Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, 
              Eigen::ComputeThinV).matrixV().rightCols<1>();
    double svd_method = svd_V[2] / svd_V[3];
    
    std::cout << svd_method << std::endl;
    feature_per_id.estimated_depth = svd_method;
    feature_per_id.solve_flag = 1;

    if (feature_per_id.estimated_depth < 0.1)
    {
      feature_per_id.estimated_depth = INIT_DEPTH;
      feature_per_id.solve_flag = 0;
    }

    mono_triangulate_cnt = mono_triangulate_cnt + (int)(svd_method > 0.1f);

  }
  std::cout << "mono triangulate count: " << mono_triangulate_cnt << "\n";
  std::cout << "stereo triangulate count: " << stereo_triangulate_cnt << "\n";
}

void FrameManager::reTriangulate(const std::vector<size_t> &opt_feature_ids)
{
  int re_triangulate_cnt = 0;

  std::lock_guard<std::mutex> lock(ftr_mutex_);

  for(size_t lm_idx = 0; lm_idx < feature_ids_.size(); ++lm_idx)
  {
    const size_t feature_id = feature_ids_[lm_idx];
    if(feature_per_ids_.count(feature_id) == 0)
      continue;

    auto &feature_per_id = feature_per_ids_[feature_id];

    auto iter = std::find(opt_feature_ids.begin(), opt_feature_ids.end(), feature_id);
    if(iter != opt_feature_ids.end())
      continue;

    auto &obs_frames = feature_per_id.obs_frames;
    auto &feature_per_frame = feature_per_id.feature_per_frame;

    if(obs_frames.empty()) 
      continue;    

    size_t obs_cnt = obs_frames.size();
    if(obs_cnt < obs_thresh_) 
      continue;

    size_t imu_i = obs_frames.front();
    auto &feature_per_frame_i = feature_per_frame[imu_i];
    int cam_idx_i = feature_per_frame_i.getObsCamIds().front();
    Eigen::Vector3d pts_i = feature_per_frame_i.getObsBearingVector(cam_idx_i);

    Eigen::Matrix4d T_w_ci, T_ci_w;
    getCameraPose(imu_i, cam_idx_i, T_w_ci, T_ci_w);

    double estimated_depth = 0.0f;
    double prv_depth = feature_per_id.estimated_depth;

    if(1)
    {
      for(size_t idx = obs_frames.size() - 1; idx >= 1; --idx)
      {
        size_t imu_j = obs_frames[idx];
        auto &feature_per_frame_j = feature_per_frame[imu_j];
        int cam_idx_j = feature_per_frame_j.getObsCamIds().front();
        Eigen::Vector3d pts_j = feature_per_frame_j.getObsBearingVector(cam_idx_j);

        Eigen::Matrix4d T_w_cj, T_cj_w;
        getCameraPose(imu_j, cam_idx_j, T_w_cj, T_cj_w);

        Eigen::Matrix4d T_ci_cj = T_ci_w * T_w_cj;
        Eigen::Vector3d pts_i_tmp = T_ci_cj.block<3, 3>(0, 0) * pts_j;
        float cos_parallax_rays = pts_i.dot(pts_i_tmp);
        if (cos_parallax_rays > 0.9997)
          continue;

        double depth_i = 0.0f;
        Eigen::Matrix4d T_cj_ci = T_cj_w * T_w_ci;            
        bool res = triangulateDepth(T_cj_ci, pts_i, pts_j, depth_i);
        if(res)
        {
          Eigen::Vector3d pts_cam_i = pts_i * depth_i;
          estimated_depth = USE_BEARING_VECTOR ? pts_cam_i.norm() : pts_cam_i.z();
        }
        else
          continue;

        // estimated_depth = 0.5 * (estimated_depth + prv_depth);
        feature_per_id.estimated_depth = estimated_depth;
        feature_per_id.solve_flag = 1;

        if (feature_per_id.estimated_depth < 0.1)
          feature_per_id.solve_flag = 2;

        ++re_triangulate_cnt;

        break;
      }
    }
    else
    {
      int obs_cnt = 0;
      for(auto &i : obs_frames)
      {
        const std::vector<int> obs_cam_id = feature_per_frame[i].getObsCamIds();
        obs_cnt += obs_cam_id.size();
      }

      int svd_idx = 0;
      Eigen::MatrixXd svd_A(2 * obs_cnt, 4);

      for(auto &imu_j : obs_frames)
      {
        auto &feature_per_frame_j = feature_per_frame[imu_j];
        const std::vector<int> obs_cam_ids = feature_per_frame_j.getObsCamIds();

        for(auto &cam_idx_j : obs_cam_ids)
        {
          Eigen::Vector3d pts_j = feature_per_frame_j.getObsBearingVector(cam_idx_j);

          Eigen::Matrix4d T_w_cj, T_cj_w;
          getCameraPose(imu_j, cam_idx_j, T_w_cj, T_cj_w);

          // Eigen::Matrix4d T_ci_cj = T_ci_w * T_w_cj;
          // Eigen::Vector3d pts_i_tmp = T_ci_cj.block<3, 3>(0, 0) * pts_j;
          // float cos_parallax_rays = pts_i.dot(pts_i_tmp);
          // if (imu_i != imu_j && cos_parallax_rays > 0.9997)
          //   continue;

          Eigen::Matrix4d T_cj_ci = T_cj_w * T_w_ci;

          svd_A.row(svd_idx++) = -pts_j(2) * T_cj_ci.row(1) + pts_j(1) * T_cj_ci.row(2);
          svd_A.row(svd_idx++) = pts_j(2) * T_cj_ci.row(0) - pts_j(0) * T_cj_ci.row(2);
        }
      }

      assert(svd_idx == svd_A.rows());

      Eigen::Vector4d pts_cam_i = Eigen::JacobiSVD<Eigen::MatrixXd>( 
                svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
      pts_cam_i /= pts_cam_i(3);
      
      estimated_depth = USE_BEARING_VECTOR ? 
                  pts_cam_i.head<3>().norm() : pts_cam_i(2);

      // estimated_depth = 0.5 * (estimated_depth + prv_depth);
      feature_per_id.estimated_depth = estimated_depth;
      feature_per_id.solve_flag = 1;

      if (feature_per_id.estimated_depth < 0.1)
        feature_per_id.solve_flag = 2;
      
      ++re_triangulate_cnt;
    }
  }

  if(!is_kf_)
    std::cout << "opt_feature_ids size: " << opt_feature_ids.size() << "; "
              << "re_triangulate_cnt size: " << re_triangulate_cnt << "; "
              << "total feature ids size: " << feature_per_ids_.size() << std::endl;
}

double FrameManager::reprojectionError(Eigen::Matrix3d &Ri, Eigen::Vector3d &Pi, 
    Eigen::Matrix3d &rici, Eigen::Vector3d &tici,
    Eigen::Matrix3d &Rj, Eigen::Vector3d &Pj, 
    Eigen::Matrix3d &ricj, Eigen::Vector3d &ticj, 
    double depth, Eigen::Vector3d &uvi, Eigen::Vector3d &uvj)
{
  if(!USE_BEARING_VECTOR)
  {
    Eigen::Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Eigen::Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Eigen::Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    return residual.norm();
  }
  else
  {
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = uvj.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if(a == tmp)
      tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);

    Eigen::Matrix<double, 2, 3> tangent_base;
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();

    Eigen::Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Eigen::Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);

    Eigen::Vector2d residual = tangent_base * (pts_cj.normalized() - uvj.normalized());
    return residual.norm();
  }
}

void FrameManager::outliersRejection(std::vector<size_t> &remove_index)
{
  std::lock_guard<std::mutex> lock(ftr_mutex_);

  for(size_t lm_idx = 0; lm_idx < feature_ids_.size(); ++lm_idx)
  {
    size_t feature_id = feature_ids_[lm_idx];
    if(feature_per_ids_.count(feature_id) == 0)
      continue;
    
    auto &feature_per_id = feature_per_ids_[feature_id];

    double err = 0;
    int err_cnt = 0;

    auto &obs_frames = feature_per_id.obs_frames;
    auto &feature_per_frame = feature_per_id.feature_per_frame;

    if (obs_frames.size() < obs_thresh_)
      continue;

    int imu_i = obs_frames.front();
    double depth = feature_per_id.estimated_depth;

    auto &feature_per_frame_i = feature_per_frame[imu_i];
    const std::vector<int> obs_cam_ids_i = feature_per_frame_i.getObsCamIds();

    int cam_idx_i = obs_cam_ids_i.front();
    Eigen::Vector3d pts_i = feature_per_frame_i.getObservation(cam_idx_i);

    for(auto &imu_j : obs_frames)
    {
      auto &feature_per_frame_j = feature_per_frame[imu_j];
      const std::vector<int> obs_cam_ids_j = feature_per_frame_j.getObsCamIds();

      for(auto &cam_idx_j : obs_cam_ids_j)
      {
        Eigen::Vector3d pts_j = feature_per_frame_j.getObservation(cam_idx_j);

        double tmp_error = reprojectionError(
                          (*states_)[imu_i].R, (*states_)[imu_i].P, 
                          (*ric_)[cam_idx_i], (*tic_)[cam_idx_i], 
                          (*states_)[imu_j].R, (*states_)[imu_j].P, 
                          (*ric_)[cam_idx_j], (*tic_)[cam_idx_j],
                          depth, pts_i, pts_j);
        err += tmp_error;
        err_cnt++;
      }
    }

    double ave_err = err / err_cnt;
    if(ave_err * FOCAL_LENGTH > 3.0f)
      feature_per_id.is_outlier = true;
    else
      feature_per_id.is_outlier = false;
  }

 for(auto &kv : feature_per_ids_)
 {
   if(kv.second.is_outlier)
    remove_index.emplace_back(kv.first);
 }
}

void FrameManager::removeOutlier(std::vector<size_t> &outlier_ids)
{
  std::lock_guard<std::mutex> lock(ftr_mutex_);

  for(auto &feature_id : outlier_ids)
  {
    if(feature_per_ids_.count(feature_id) == 0)
      continue;

    feature_per_ids_.erase(feature_id);
  }
}

bool FrameManager::getPoseWithPnPRansac(const int frame_count, const int cam_idx,
                const int matches_num_thresh, int &matches_num,
                Eigen::Matrix3d &R_cur, Eigen::Vector3d &t_cur)
{
  if(frame_count < 1) 
    return false;

  FrameBundlePtr &frame_bundles_cur = (*states_)[frame_count].frame_bundle;
  FrameBundlePtr &frame_bundles_prv = (*states_)[frame_count - 1].frame_bundle;

  if(!frame_bundles_prv || !frame_bundles_cur)
    return false;

  Eigen::Matrix4d T_i_c = Eigen::Matrix4d::Identity();
  T_i_c.block<3, 3>(0, 0) = (*ric_)[cam_idx];
  T_i_c.block<3, 1>(0, 3) = (*tic_)[cam_idx];

  Eigen::Matrix4d T_c_i = Eigen::Matrix4d::Identity();
  T_c_i.block<3, 3>(0, 0) = (*ric_)[cam_idx].transpose();
  T_c_i.block<3, 1>(0, 3) = -(*ric_)[cam_idx].transpose() * (*tic_)[cam_idx];

  const FramePtr &frame_cur = frame_bundles_cur->at(cam_idx);
  const FramePtr &frame_prv = frame_bundles_prv->at(cam_idx);

  std::unordered_map<size_t, size_t> feature_matches;
  getFeatureMatches(frame_prv, frame_cur, feature_matches);

  std::vector<cv::Point2f> cur_matched_norm;
  std::vector<cv::Point3f> prv_matched_3d;

  cv::Mat &depth_image_prv = frame_prv->depth_image_;
  const float depth_img_min = frame_prv->cam_->getDepthMin();
  const float depth_img_max = frame_prv->cam_->getDepthMax();
  const float depth_img_scale = frame_prv->cam_->getDepthScale();
  if(depth_img_min == 0.0f || depth_img_max == 0.0f || depth_img_scale == 0.0f)
  {
    std::cout << "depth min: " << depth_img_min << "; "
        << "depth max: " << depth_img_max << "; "
        << "depth scale: " << depth_img_scale << "\n";
    std::cout << "depth_img_min, depth_img_max and depth_img_scale shouldn't be 0.0f\n";
    exit(-1);
  }

  int matched_3d = 0;
  for(auto &it : feature_matches)
  {
    size_t idx_prv = it.first;
    size_t idx_cur = it.second;
    size_t feautre_id = frame_cur->track_id_vec_(idx_cur);
    assert(frame_cur->track_id_vec_(idx_cur) == frame_prv->track_id_vec_(idx_prv));

    Eigen::Vector2d px_prv = frame_prv->px_vec_.col(idx_prv);
    int x_prv = round(px_prv.x());
    int y_prv = round(px_prv.y());

    // TODO: if use                 
    float depth_prv = (float)depth_image_prv.at<ushort>(y_prv, x_prv) / depth_img_scale;

    if(depth_prv < depth_img_min || depth_prv > depth_img_max)
      continue;

    Eigen::Vector3d f_cur = frame_cur->f_vec_.col(idx_cur);
    Eigen::Vector3d f_prv = frame_prv->f_vec_.col(idx_prv);            
    
    Eigen::Vector3d norm_cur = f_cur / f_cur.z();
    Eigen::Vector3d norm_prv = f_prv / f_prv.z();

    Eigen::Vector3d pts_cam_prv = norm_prv * depth_prv;

    cv::Point2f pt_2d_cur(norm_cur.x(), norm_cur.y());
    cv::Point3f pt_3d_prv(pts_cam_prv.x(), pts_cam_prv.y(), pts_cam_prv.z());

    cur_matched_norm.emplace_back(pt_2d_cur);
    prv_matched_3d.emplace_back(pt_3d_prv);
    
    ++matched_3d;
  }

  matches_num = (int)prv_matched_3d.size();
  if((int)prv_matched_3d.size() < matches_num_thresh)
  {
    printf("The number of 3d point from depth image (%d) is too little, less than 25\n", 
        (int)prv_matched_3d.size());
    return false;
  }

  cv::Mat inliers;
  cv::Mat r, rvec, t, D;
  cv::Mat K = cv::Mat::eye(3, 3, CV_32FC1);
  Eigen::VectorXd &intrinsic = cam_intrinsics_[cam_idx];
  float focal_length = std::sqrt(intrinsic(0) * intrinsic(1));

  if (CV_MAJOR_VERSION < 3)
  {
    cv::solvePnPRansac(prv_matched_3d, cur_matched_norm, K, D, rvec, 
            t, true, 100, 10.0 / focal_length, 100, inliers);
  }
  else
  {
    if (CV_MINOR_VERSION < 2)
    cv::solvePnPRansac(prv_matched_3d, cur_matched_norm, K, D, rvec, 
          t, true, 100, sqrt(10.0 / focal_length), 0.99, inliers);
    else
    cv::solvePnPRansac(prv_matched_3d, cur_matched_norm, K, D, rvec, 
          t, true, 100, 10.0 / focal_length, 0.99, inliers);
  }
  std::cout << "*******************************\n";

  Eigen::Matrix3d R_cur_prv;
  Eigen::Vector3d t_cur_prv; 
  cv::Rodrigues(rvec, r);
  cv::cv2eigen(r, R_cur_prv);
  cv::cv2eigen(t, t_cur_prv);

  Eigen::Matrix3d R_prv_cur = R_cur_prv.transpose();
  Eigen::Vector3d t_prv_cur = -R_cur_prv.transpose() * t_cur_prv;

  Eigen::Matrix4d T_prv_cur = Eigen::Matrix4d::Identity();
  T_prv_cur.block<3, 3>(0, 0) = R_prv_cur;
  T_prv_cur.block<3, 1>(0, 3) = t_prv_cur;

  T_prv_cur = T_i_c * T_prv_cur * T_c_i;
  R_prv_cur = T_prv_cur.block<3, 3>(0, 0);
  t_prv_cur = T_prv_cur.block<3, 1>(0, 3);

  // std::cout << "T_prv_cur\n" << T_prv_cur << std::endl;

  Eigen::Matrix3d R_prv = (*states_)[frame_count - 1].R;
  Eigen::Vector3d t_prv = (*states_)[frame_count - 1].P;

  R_cur = R_prv * R_prv_cur;
  t_cur = R_prv * t_prv_cur + t_prv;

  return true;
}


void FrameManager::monoRejectWithF(const int frame_count, 
      const int cam_index,
      const FrameBundlePtr &new_frames, 
      std::unordered_map<size_t, int> &match_status)
{
  if(!reject_with_F_ || frame_count < 1)
    return;

  int cols = new_frames->at(0)->img_pyr_[0].cols;
  int rows = new_frames->at(0)->img_pyr_[0].rows;

  const FramePtr &frame_i = (*states_)[frame_count - 1].frame_bundle->at(cam_index);
  const FramePtr &frame_j = new_frames->at(cam_index);

  std::unordered_map<size_t, size_t> feature_matches;
  getFeatureMatches(frame_i, frame_j, feature_matches);

  int match_size = feature_matches.size();
  std::vector<cv::Point2f> un_pts_i(match_size);
  std::vector<cv::Point2f> un_pts_j(match_size);

  int idx = 0;
  for(auto &iter : feature_matches)
  {
    size_t idx_i = iter.first;
    size_t idx_j = iter.second;

    Eigen::Vector3d pts_i = frame_i->f_vec_.col(idx_i);
    cvtSphere2Plane(0, pts_i, true);
    un_pts_i[idx] = cv::Point2f(pts_i.x(), pts_i.y());

    Eigen::Vector3d pts_j = frame_j->f_vec_.col(idx_j);
    cvtSphere2Plane(0, pts_j, true);
    un_pts_j[idx] = cv::Point2f(pts_j.x(), pts_j.y());

    ++idx;
  }

  std::vector<uchar> status;
  cv::findFundamentalMat(un_pts_i, un_pts_j, cv::FM_RANSAC, 1.0 / FOCAL_LENGTH, 0.99, status);

  idx = 0;
  int inlier_cnt = 0;
  for(auto &iter : feature_matches)
  {
    size_t idx_i = iter.first;
    size_t idx_j = iter.second;
    int feature_id = frame_i->track_id_vec_(idx_i);

    assert(frame_i->track_id_vec_(idx_i) 
      == frame_j->track_id_vec_(idx_j));
  
    if(feature_id == -1) continue;

    if (status[idx])
    {
      match_status[feature_id] = 1;
      ++inlier_cnt;
    }
    else
      match_status[feature_id] = 0;

    ++idx;
  }

  std::cout << "monoRejectWithF: "
        << "match size: " << match_size << "; " 
        << "inlier size: " << inlier_cnt << "\n";
}

void FrameManager::stereoRejectWithF(
  const FrameBundlePtr &new_frames, 
  const std::unordered_map<size_t, size_t> &feature_matches,
  std::unordered_map<size_t, int> &match_status)
{
  if(!reject_with_F_)
    return;

  int cols = new_frames->at(0)->img_pyr_[0].cols;
  int rows = new_frames->at(0)->img_pyr_[0].rows;

  const FramePtr &frame_i = new_frames->at(0);
  const FramePtr &frame_j = new_frames->at(1);

  int match_size = feature_matches.size();
  std::vector<cv::Point2f> un_pts_i(match_size);
  std::vector<cv::Point2f> un_pts_j(match_size);

  int idx = 0;
  for(auto &iter : feature_matches)
  {
    size_t idx_i = iter.first;
    size_t idx_j = iter.second;

    Eigen::Vector2d tmp_i;
    Eigen::Vector3d pts_i = frame_i->f_vec_.col(idx_i);
    cvtSphere2Plane(0, pts_i, true);
    un_pts_i[idx] = cv::Point2f(pts_i.x(), pts_i.y());

    Eigen::Vector2d tmp_j;
    Eigen::Vector3d pts_j = frame_j->f_vec_.col(idx_j);
    cvtSphere2Plane(1, pts_j, true);
    un_pts_j[idx] = cv::Point2f(pts_j.x(), pts_j.y());

    ++idx;
  }

  std::vector<uchar> status;
  cv::findFundamentalMat(un_pts_i, un_pts_j, cv::FM_RANSAC, 0.5 / FOCAL_LENGTH, 0.99, status);

  idx = 0;
  int inlier_cnt = 0;
  for(auto &iter : feature_matches)
  {
    size_t idx_i = iter.first;
    size_t idx_j = iter.second;
    int feature_id = frame_i->track_id_vec_(idx_i);

    assert(frame_i->track_id_vec_(idx_i) 
      == frame_j->track_id_vec_(idx_j));
  
    if(feature_id == -1) continue;

    if (status[idx])
    {
      match_status[feature_id] = 1;
      ++inlier_cnt;
    }
    else
      match_status[feature_id] = 0;

    ++idx;
  }

  // just for test
  if(1)
  {
    cv::Mat track_img;
    cv::Mat left_img = frame_i->img_pyr_[0];
    cv::Mat right_img = frame_j->img_pyr_[0];
    int cols = left_img.cols;
    cv::hconcat(left_img, right_img, track_img);
    if(track_img.channels() == 1)
      cv::cvtColor(track_img, track_img, cv::COLOR_GRAY2RGB);
    
    for(auto &iter : feature_matches)
    {
      size_t idx_i = iter.first;
      size_t idx_j = iter.second;
      size_t feature_id = frame_i->track_id_vec_(idx_i);
      
      assert(frame_i->track_id_vec_(idx_i) == frame_j->track_id_vec_(idx_j));

      cv::Point2d left_point(frame_i->px_vec_.col(idx_i)[0], frame_i->px_vec_.col(idx_i)[1]);
      cv::Point2d right_point(frame_j->px_vec_.col(idx_j)[0] + cols, frame_j->px_vec_.col(idx_j)[1]);

      cv::circle(track_img, left_point, 2, cv::Scalar(0, 255, 0), 2);
      cv::circle(track_img, right_point, 2, cv::Scalar(0, 255, 0), 2);

      if(match_status[feature_id] == 1)
        cv::line(track_img, left_point, right_point, cv::Scalar(0, 255, 255), 1, 8, 0);
      else if(match_status[feature_id] == 0)
        cv::line(track_img, left_point, right_point, cv::Scalar(0, 0, 255), 1, 8, 0);
    }

    char save_path[256];
    std::string home_path = getenv("HOME");
    double timestamp = getTimestamp(new_frames);
    snprintf(save_path, sizeof(save_path), "%s/debug/%.9lf.jpg", home_path.c_str(), timestamp);
    cv::imwrite(save_path, track_img);
  }

  std::cout << "stereoRejectWithF: "
      << "match size: " << match_size << "; " 
      << "inlier size: " << inlier_cnt << "\n";
}

void FrameManager::rejectWithF(const int frame_count, 
  const FrameBundlePtr &new_frames, 
  std::unordered_map<size_t, int> &match_status)
{
  return;

  if(frame_count < 1)
    return;

  int cols = new_frames->at(0)->img_pyr_[0].cols;
  int rows = new_frames->at(0)->img_pyr_[0].rows;

  const FramePtr &frame_i = (*states_)[frame_count - 1].frame_bundle->at(0);
  const FramePtr &frame_j = new_frames->at(0);

  std::unordered_map<size_t, size_t> feature_matches;
  getFeatureMatches(frame_i, frame_j, feature_matches);

  int match_size = feature_matches.size();
  std::vector<cv::Point2f> un_pts_i(match_size);
  std::vector<cv::Point2f> un_pts_j(match_size);

  int idx = 0;
  int cam_idx_i = frame_i->nframe_index_;
  int cam_idx_j = frame_j->nframe_index_;
  for(auto &iter : feature_matches)
  {
    size_t idx_i = iter.first;
    size_t idx_j = iter.second;

    Eigen::Vector3d pts_i = frame_i->f_vec_.col(idx_i);
    cvtSphere2Plane(cam_idx_i, pts_i, true);
    un_pts_i[idx] = cv::Point2f(pts_i.x(), pts_i.y());

    Eigen::Vector3d pts_j = frame_j->f_vec_.col(idx_j);
    cvtSphere2Plane(cam_idx_j, pts_j, true);
    un_pts_j[idx] = cv::Point2f(pts_j.x(), pts_j.y());

    ++idx;
  }

  std::vector<uchar> status;
  cv::findFundamentalMat(un_pts_i, un_pts_j, cv::FM_RANSAC, 1.0 / FOCAL_LENGTH, 0.99, status);


  idx = 0;
  int inlier_cnt = 0;
  for(auto &iter : feature_matches)
  {
    size_t idx_i = iter.first;
    size_t idx_j = iter.second;
    int feature_id = frame_i->track_id_vec_(idx_i);

    assert(frame_i->track_id_vec_(idx_i) 
      == frame_j->track_id_vec_(idx_j));
  
    if(feature_id == -1) continue;

    if (status[idx])
    {
      match_status[feature_id] = 1;
      ++inlier_cnt;
    }
    else
      match_status[feature_id] = 0;

    ++idx;
  }

  std::cout << frame_count << ": " << " "; 
  std::cout << "match size: " << match_size << "; " 
      << "inlier size: " << inlier_cnt << std::endl;
}

void FrameManager::rejectWithF(
  const FramePtr &frame_i, const FramePtr &frame_j, 
  const std::unordered_map<size_t, size_t> &feature_matches,
  std::vector<uchar> &status, const double pixel_error)
{
  if(feature_matches.size() < 8)
    return;

  std::vector<cv::Point2f> un_pts_i(feature_matches.size());
  std::vector<cv::Point2f> un_pts_j(feature_matches.size());

  int idx = 0;
  int cam_idx_i = frame_i->nframe_index_;
  int cam_idx_j = frame_j->nframe_index_;

  for(auto &iter : feature_matches)
  {
    size_t idx_i = iter.first;
    size_t idx_j = iter.second;

    Eigen::Vector3d pts_i = frame_i->f_vec_.col(idx_i);
    cvtSphere2Plane(cam_idx_i, pts_i, true);
    un_pts_i[idx] = cv::Point2f(pts_i.x(), pts_i.y());

    Eigen::Vector3d pts_j = frame_j->f_vec_.col(idx_j);
    cvtSphere2Plane(cam_idx_j, pts_j, true);
    un_pts_j[idx] = cv::Point2f(pts_j.x(), pts_j.y());

    ++idx;
  }

  cv::findFundamentalMat(un_pts_i, un_pts_j, cv::FM_RANSAC, 
                pixel_error / FOCAL_LENGTH, 0.99, status);
}

bool FrameManager::drawStereoMatches(
  const FramePtr &frame0, const FramePtr &frame1,
  const std::unordered_map<size_t, size_t> &feature_matches,
  const std::vector<uchar> &match_status)
{
  if(!DEBUG)
    return false;

  cv::Mat track_img;
  cv::Mat left_img = frame0->img_pyr_[0];
  cv::Mat right_img = frame1->img_pyr_[0];
  int cols = left_img.cols;
  cv::hconcat(left_img, right_img, track_img);
  if(track_img.channels() == 1)
    cv::cvtColor(track_img, track_img, cv::COLOR_GRAY2RGB);

  cv::Scalar missed_color(0, 0, 255);
  cv::Scalar matched_color(0, 255, 0);
  cv::Scalar point_color(0, 255, 255);

  int idx = 0;
  for(auto &iter : feature_matches)
  {
    // if(idx % 5 != 0)
    //     continue;

    cv::Scalar line_color;
    if(match_status.empty())
      line_color = matched_color;
    else
        line_color = match_status[idx] ? 
            matched_color : missed_color;
    
    size_t idx0 = iter.first;
    size_t idx1 = iter.second;
    cv::Point2d left_point(frame0->px_vec_.col(idx0)[0], frame0->px_vec_.col(idx0)[1]);
    cv::Point2d right_point(frame1->px_vec_.col(idx1)[0] + cols, frame1->px_vec_.col(idx1)[1]);
    cv::circle(track_img, left_point, 2, point_color, 2);
    cv::circle(track_img, right_point, 2, point_color, 2);
    cv::line(track_img, left_point, right_point, line_color, 1, 8, 0);
    idx++;
  }

  cv::imshow("stereo_match", track_img);

  char save_path[256];
  std::string home_path = getenv("HOME");
  snprintf(save_path, sizeof(save_path), "%s/debug/stereo_match.jpg", home_path.c_str());
  cv::imwrite(save_path, track_img);
  cv::waitKey(1);

  return true;
}

bool FrameManager::drawCorrespondingFeature(int frame_count_l, int frame_count_r, 
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &corres_px, const std::string name)
{
  if(!DEBUG)
    return false;

  FrameBundlePtr &frame_bundle_l = (*states_)[frame_count_l].frame_bundle;
  const FramePtr &frame_l = frame_bundle_l->at(0);
  cv::Mat img_l = frame_l->img_pyr_[0];

  FrameBundlePtr &frame_bundle_r = (*states_)[frame_count_r].frame_bundle;
  const FramePtr &frame_r = frame_bundle_r->at(0);
  cv::Mat img_r = frame_r->img_pyr_[0];


  cv::Mat track_img;
  int cols = img_l.cols;
  cv::hconcat(img_l, img_r, track_img);
  if(track_img.channels() == 1)
    cv::cvtColor(track_img, track_img, cv::COLOR_GRAY2RGB);

  for(auto px : corres_px)
  {
    Eigen::Vector2d px_l = px.first;
    Eigen::Vector2d px_r = px.second;

    cv::Point2d point_l(px_l.x(), px_l.y());
    cv::Point2d point_r(px_r.x() + cols, px_r.y());
    cv::circle(track_img, point_l, 2, cv::Scalar(0, 255, 0), 2);
    cv::circle(track_img, point_r, 2, cv::Scalar(0, 255, 0), 2);
    cv::line(track_img, point_l, point_r, cv::Scalar(0, 255, 255), 1, 8, 0);
  }

  char save_path[256];
  std::string home_path = getenv("HOME");
  snprintf(save_path, sizeof(save_path), "%s/debug/%s_%d_%d.jpg", 
      home_path.c_str(), name.c_str(), frame_count_l, frame_count_r);
  std::cout << "save_path: " << save_path << std::endl;
  cv::imwrite(save_path, track_img);
  
  return true;
}

bool FrameManager::drawFeatureMatchById(int frame_count_l, int frame_count_r, 
  const std::vector<size_t> &feature_ids, const std::string name)
{
  if(!DEBUG)
    return false;

  FrameBundlePtr &frame_bundle_l = (*states_)[frame_count_l].frame_bundle;
  const FramePtr &frame_l = frame_bundle_l->at(0);
  cv::Mat img_l = frame_l->img_pyr_[0];

  FrameBundlePtr &frame_bundle_r = (*states_)[frame_count_r].frame_bundle;
  const FramePtr &frame_r = frame_bundle_r->at(0);
  cv::Mat img_r = frame_r->img_pyr_[0];

  cv::Mat track_img;
  int cols = img_l.cols;
  cv::hconcat(img_l, img_r, track_img);
  if(track_img.channels() == 1)
    cv::cvtColor(track_img, track_img, cv::COLOR_GRAY2RGB);

  for(auto &feature_id : feature_ids)
  {
    if(feature_per_ids_.count(feature_id))
      continue;

    auto &feture_per_frame = feature_per_ids_[feature_id].feature_per_frame;
    const Eigen::Vector2d &px_l = feture_per_frame[frame_count_l].getObsPixel(0);

    const Eigen::Vector2d &px_r = feture_per_frame[frame_count_r].getObsPixel(0);

    cv::Point2d point_l(px_l.x(), px_l.y());
    cv::Point2d point_r(px_r.x() + cols, px_r.y());
    cv::circle(track_img, point_l, 2, cv::Scalar(0, 255, 0), 2);
    cv::circle(track_img, point_r, 2, cv::Scalar(0, 255, 0), 2);
    cv::line(track_img, point_l, point_r, cv::Scalar(0, 255, 255), 1, 8, 0);
  }

  char save_path[256];
  std::string home_path = getenv("HOME");
  snprintf(save_path, sizeof(save_path), "%s/debug/%s_%d_%d.jpg", 
      home_path.c_str(), name.c_str(), frame_count_l, frame_count_r);
  std::cout << "save_path: " << save_path << std::endl;
  cv::imwrite(save_path, track_img);
  
  return true;
}

bool FrameManager::drawMonoFeatureTrack(int i, int j)
{
  if(!DEBUG)
    return false;

  if(i < 0 || j < 0 || !(*states_)[i].frame_bundle || !(*states_)[j].frame_bundle)
    return false;

  size_t cam_size = (*states_)[i].frame_bundle->size();
  for(size_t cam_idx = 0; cam_idx < cam_size; ++cam_idx)
  {   
    const FramePtr &frame_i = (*states_)[i].frame_bundle->at(cam_idx);
    const FramePtr &frame_j = (*states_)[j].frame_bundle->at(cam_idx);

    std::unordered_map<size_t, size_t> feature_matches;
    getFeatureMatches(frame_i, frame_j, feature_matches);

    cv::Mat track_img;
    cv::Mat img_i = frame_i->img_pyr_[0];
    cv::Mat img_j = frame_j->img_pyr_[0];
    int cols = img_i.cols;
    cv::hconcat(img_i, img_j, track_img);
    if(track_img.channels() == 1)
      cv::cvtColor(track_img, track_img, cv::COLOR_GRAY2RGB);

    int count = 0;
    cv::Scalar point_color;
    for(auto &iter : feature_matches)
    {
      count++;
      if(count % 5 != 0)
        continue;
      size_t idx_i = iter.first;
      size_t idx_j = iter.second;

      size_t feature_id = frame_i->track_id_vec_(idx_i);
      assert(frame_i->track_id_vec_(idx_i) == frame_j->track_id_vec_(idx_j));
      if(feature_per_ids_[feature_id].estimated_depth > 0.0f)
        point_color = cv::Scalar(0, 255, 0);
      else
        point_color = cv::Scalar(0, 0, 255);
      
      cv::Point2d point_i(frame_i->px_vec_.col(idx_i)[0], frame_i->px_vec_.col(idx_i)[1]);
      cv::Point2d point_j(frame_j->px_vec_.col(idx_j)[0] + cols, frame_j->px_vec_.col(idx_j)[1]);
      cv::circle(track_img, point_i, 2, point_color, 2);
      cv::circle(track_img, point_j, 2, point_color, 2);
      cv::line(track_img, point_i, point_j, cv::Scalar(0, 255, 255), 1, 8, 0);
    }

    // cv::namedWindow("feature_track", cv::WINDOW_AUTOSIZE);
    // cv::imshow("feature_track", track_img);
    // cv::waitKey(1);

    char save_path[256];
    std::string home_path = getenv("HOME");
    snprintf(save_path, sizeof(save_path), "%s/debug/%d-%d_%ld.jpg", 
        home_path.c_str(), i, j, cam_idx);
    cv::imwrite(save_path, track_img);
  }

  return true;
}

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

#include <set>
#include <map>
#include <list>
#include <deque>
#include <algorithm>
#include <vector>
#include <numeric>
#include <mutex>
#include <shared_mutex>

#include <eigen3/Eigen/Dense>

#include "common/common_lib.h"
#include "common/parameters.h"

// included in global.h
// #include <svo/common/frame.h>
// #include <svo/common/types.h>
// #include <svo/imu_handler.h>
// #include <svo/odom_handler.h>
// #include <svo/common/camera.h>

struct PointInFramebundle
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::VectorXi obs_cam_id;
  Eigen::VectorXi obs_ftr_idx;
  Eigen::VectorXi obs_pyr_level;

  Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor> obs_pixel;
  Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor> obs_velocity;
  Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor> obs_norm_plane;
  Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor> obs_bearing_vector;

  // std::shared_timed_mutex mutex;

  PointInFramebundle(const int cam_size)
  {
    obs_cam_id.conservativeResize(cam_size);    
    obs_ftr_idx.conservativeResize(cam_size);
    obs_pyr_level.conservativeResize(cam_size);

    obs_pixel.conservativeResize(Eigen::NoChange, cam_size);
    obs_velocity.conservativeResize(Eigen::NoChange, cam_size);
    obs_norm_plane.conservativeResize(Eigen::NoChange, cam_size);
    obs_bearing_vector.conservativeResize(Eigen::NoChange, cam_size);

    obs_cam_id.setConstant(-1);
  }

  PointInFramebundle() = default;

  ~PointInFramebundle() = default;

  void swap(PointInFramebundle &tmp)
  {
    obs_cam_id.swap(tmp.obs_cam_id);
    obs_ftr_idx.swap(tmp.obs_ftr_idx);
    obs_pyr_level.swap(tmp.obs_pyr_level);

    obs_pixel.swap(tmp.obs_pixel);
    obs_velocity.swap(tmp.obs_velocity);
    obs_norm_plane.swap(tmp.obs_norm_plane);
    obs_bearing_vector.swap(tmp.obs_bearing_vector);
  }

  inline void addObservation(
      const int cam_id, const size_t ftr_idx, const int pyr_level, 
      const Eigen::Vector2d &pixel, const Eigen::Vector3d &bearing_vector, 
      const Eigen::Vector3d &velocity = Eigen::Vector3d::Zero())
  {
    obs_cam_id(cam_id) = 1;
    obs_ftr_idx(cam_id) = ftr_idx;
    obs_pyr_level(cam_id) = 0;

    obs_pixel.col(cam_id) = pixel;
    obs_velocity.col(cam_id) = velocity;
    obs_bearing_vector.col(cam_id) = bearing_vector;
    obs_norm_plane.col(cam_id) = bearing_vector / bearing_vector.z();
  }

  inline void addObservation(
      const int cam_id, const size_t ftr_idx, const int pyr_level)
  {
    obs_cam_id(cam_id) = 1;
    obs_ftr_idx(cam_id) = ftr_idx;
    obs_pyr_level(cam_id) = 0;
  }

  inline bool isObsByCam(const int cam_id)
  {
    if(obs_cam_id(cam_id) > 0)
      return true;

    return false;
  }

  inline std::vector<int> getObsCamIds()
  {
    std::vector<int> res;

    for(int i = 0; i < obs_cam_id.rows(); ++i)
      if(obs_cam_id(i) > 0)
        res.emplace_back(i);

    return res;
  }

  inline size_t getObsFtrIdx(const int cam_id)
  {
    return obs_ftr_idx(cam_id);
  }

  inline int getObsPyrLevel(const int cam_id)
  {
    return obs_pyr_level(cam_id);
  }

  inline Eigen::Vector2d getObsPixel(const int cam_id)
  {
    return obs_pixel.col(cam_id);
  }

  inline Eigen::Vector3d getObsNormPlane(const int cam_id)
  {
    return obs_norm_plane.col(cam_id);;
  }

  inline Eigen::Vector3d getObsBearingVector(const int cam_id)
  {
    return obs_bearing_vector.col(cam_id);
  }

  inline Eigen::Vector3d getObsVelocity(const int cam_id)
  {
    return obs_velocity.col(cam_id);
  }

  inline Eigen::Vector3d getObservation(const int cam_id)
  {
    if(USE_BEARING_VECTOR)
      return getObsBearingVector(cam_id);
    else
      return getObsNormPlane(cam_id);
  }
};

struct FeaturePerId
{
public:

  int solve_flag = 0;
  bool is_plane = false;
  bool is_outlier = false;
  bool slide_out = false;
  double estimated_depth = 0.0f;

  std::vector<int> obs_frames;
  Eigen::aligned_unordered_map<size_t, PointInFramebundle> feature_per_frame;

  FeaturePerId(const double _estimated_depth, const int _solve_flag)
  {
    solve_flag = _solve_flag;
    estimated_depth = _estimated_depth;
  }

  FeaturePerId() = default;

  ~FeaturePerId() = default;

  inline void addObservation(
      const int frame_count, const int cam_size, const int cam_id, 
      const size_t ftr_idx, const int pyr_level, const Eigen::Vector2d &pixel, 
      const Eigen::Vector3d &bearing_vector, const Eigen::Vector3d &velocity = Eigen::Vector3d::Zero())
  {
    if(feature_per_frame.count(frame_count) == 0)
      feature_per_frame[frame_count] = PointInFramebundle(cam_size);
      
    feature_per_frame[frame_count].addObservation(cam_id, 
        ftr_idx, pyr_level, pixel, bearing_vector, velocity);
    
    if(obs_frames.empty()) 
      obs_frames.emplace_back(frame_count);
    else
      if(std::find(obs_frames.begin(), obs_frames.end(), frame_count) == obs_frames.end())
        obs_frames.emplace_back(frame_count);
  }

  inline void addObservation(const int frame_count, const int cam_size, 
            const int cam_id, const size_t ftr_idx, const int pyr_level)
  {
    if(feature_per_frame.count(frame_count) == 0)
      feature_per_frame[frame_count] = PointInFramebundle(cam_size);

    feature_per_frame[frame_count].addObservation(cam_id, ftr_idx, pyr_level);
    
    if(obs_frames.empty()) 
      obs_frames.emplace_back(frame_count);
    else
      if(std::find(obs_frames.begin(), obs_frames.end(), frame_count) == obs_frames.end())
        obs_frames.emplace_back(frame_count);
  }

  inline void removeOld()
  {
    int start_frame = obs_frames.front();   

    obs_frames.erase(obs_frames.begin());
    feature_per_frame.erase(start_frame);
  }

  inline void slideOld()
  {
    for(unsigned int i = 0; i < obs_frames.size(); ++i)
    {
      size_t frame_id = obs_frames[i];
      obs_frames[i] = frame_id - 1;
      // auto &feature_per_cam = feature_per_frame[frame_id];
      // feature_per_frame[frame_id - 1] = feature_per_cam;
      // feature_per_frame.erase(frame_id);
      feature_per_frame[frame_id - 1].swap(feature_per_frame[frame_id]);
      feature_per_frame.erase(frame_id);
    }
  }
};

class FrameManager
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrameManager(std::vector<StateGroup> *_states,
        Matrix3dVec *_ric, Vector3dVec *_tic, 
        CameraBundlePtr& camera_bundle,
        VinsBackendOptions& options);

  ~FrameManager();

  void reset();

  void clearState();

  void clearDepth();

  void setRic(Eigen::Matrix3d _ric[]);

  bool checkParallax(const int frame_count, const int cam_idx);

  bool checkParallax(int frame_count, const FrameBundlePtr &new_frames);

  void cvtSphere2Plane(const int cam_idx, Eigen::Vector3d &pts, bool force_unit_plane = false);

  void removeFailures(std::vector<size_t> &failure_index);

  void removeSecondNew(const int frame_count);

  void removeFirstNew(const int frame_count);

  void removeOld();

  void getDepth(const int frame_count, const SolverFlag solver_flag, const bool opt, 
                std::vector<size_t> &opt_feature_ids, double **para_Feature);

  // void setDepth(const Eigen::VectorXd &x);

  void setDepth(const int frame_count, const std::vector<size_t> &opt_feature_ids, 
            const std::map<size_t, size_t> &ftr_idx_map, double **x);

  bool getPoseWithPnPRansac(const int frame_count, const int cam_idx,
            const int matches_num_thresh, int &matches_num,
            Eigen::Matrix3d &R_cur, Eigen::Vector3d &t_cur);

  bool inlierCheck(const FramePtr &frame, Eigen::Vector2d px);

  void removeOutlier(std::vector<size_t> &outlier_index);

  void outliersRejection(std::vector<size_t> &remove_index);

  void monoRejectWithF(const int frame_count, 
             const int cam_index,
             const FrameBundlePtr &new_frames, 
             std::unordered_map<size_t, int> &match_status);

  void stereoRejectWithF(const FrameBundlePtr &new_frames, 
               const std::unordered_map<size_t, size_t> &feature_matches,
               std::unordered_map<size_t, int> &match_status);

  void rejectWithF(const int frame_count, const FrameBundlePtr &new_frames, std::unordered_map<size_t, int> &match_status);

  double reprojectionError(Eigen::Matrix3d &Ri, Eigen::Vector3d &Pi, Eigen::Matrix3d &rici, Eigen::Vector3d &tici,
            Eigen::Matrix3d &Rj, Eigen::Vector3d &Pj, Eigen::Matrix3d &ricj, Eigen::Vector3d &ticj, 
            double depth, Eigen::Vector3d &uvi, Eigen::Vector3d &uvj);

  void removeBackShiftDepth();

  void getCorresponding(int frame_count_l, int frame_count_r, std::vector<size_t> &corres_feature_id,
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &corres_pts);

  void getFeatureMatches(const FramePtr& frame1, const FramePtr& frame2,
            std::unordered_map<size_t, size_t>& matches_12);

  void addMonoFrameBundle(const int frame_count, const FrameBundlePtr &new_frames);

  void addFrameBundle(const int frame_count, const FrameBundlePtr &new_frames, const bool verbose = true);

  void getTrackingState(const int frame_count);

  void getTrackingState(const int frame_count, const FrameBundlePtr &frame_bundle);

  bool solvePoseByPnP(const std::vector<cv::Point2f> &pts_2d, 
                      const std::vector<cv::Point3f> &pts_3d,
                      Eigen::Matrix3d &R, Eigen::Vector3d &P);

  void triangulate();

  void reTriangulate(const std::vector<size_t> &opt_feature_ids);

  bool initFramePoseByPnP(const int frame_count, Eigen::Matrix3d &R_w_b, Eigen::Vector3d &t_w_b);

  bool drawStereoMatches(const FramePtr &frame0, const FramePtr &frame1,
               const std::unordered_map<size_t, size_t> &feature_matches,
               const std::vector<uchar> &match_status = {});
    
  bool drawCorrespondingFeature(int frame_count_l, int frame_count_r, 
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &corres_px, const std::string name);

  bool drawFeatureMatchById(int frame_count_l, int frame_count_r, const std::vector<size_t> &idxs, const std::string name);

  bool drawMonoFeatureTrack(int i, int j);

  void rejectWithF(const FramePtr &frame_i, const FramePtr &frame_j, 
      const std::unordered_map<size_t, size_t> &feature_matches,
      std::vector<uchar> &status, const double pixel_error = 1.0f);

  int planePointsDetection(const double plane_d);

private:

  void getGridBorder();

  void updateFeatureIds();

  void slideOutFeatures();

  void getGridObs(const int frame_count);

  bool isDepthOK(const double estimated_depth);

  void getCameraPose(const int frame_idx, const int cam_idx, 
      Eigen::Matrix4d &T_w_c, Eigen::Matrix4d &T_c_w);

  bool triangulateDepth(const Eigen::Matrix4d& T_cur_ref, const Eigen::Vector3d& f_ref, 
          const Eigen::Vector3d& f_cur, double &depth);

  void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
          Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);

  void triangulatePoint(const Eigen::Matrix4d &T_rel, const Eigen::Vector3d &pts_i, 
          const Eigen::Vector3d &pts_j, double &depth);

  float getDepthValueFromImage(const FramePtr &frame, 
      const cv::Mat &depth_image, int feat_idx, double &depth);

  bool isGoodPoint(const FramePtr &frame, const size_t ftr_idx);

  Eigen::Vector2d detectionFunction(const Eigen::Matrix4d &T_w_ci, const Eigen::Matrix4d &T_cj_ci,
                                    const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
                                    const double d);

  void drawPlanePoints(const Eigen::Vector4d &plane_w);


private:
  bool is_kf_ = false;
  
  int grain_size_ = 1;
  int kf_new_opt_size_ = 0;
  int kf_all_opt_size_ = 0;
  int nkf_all_opt_size_ = 0;

  float min_depth_ = -1.0f;
  float max_depth_ = -1.0f;
  
  int grid_width_ = 100;
  int grid_height_ = 100;

  std::map<size_t, std::map<std::string, size_t>> grid_border_;
  std::map<size_t, std::map<size_t, std::vector<std::pair<size_t, size_t>>>> grid_obs_;

public:

  // Eigen::Matrix3d *ric;
  // Eigen::Vector3d *tic;
  Matrix3dVec *ric_;
  Vector3dVec *tic_;

  std::vector<StateGroup> *states_;
  std::mutex ftr_mutex_;

  bool reject_with_F_;

  CameraBundlePtr camera_bundle_ = nullptr;

  std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> cam_intrinsics_;

  Camera::CameraGeometryBase::Type cam_type_;

  int last_track_num_;
  int new_feature_num_;
  unsigned int obs_thresh_;

  std::set<size_t> cur_feature_ids_;
  std::map<size_t, std::set<size_t>> cur_frame_obs_;
  
  std::vector<size_t> feature_ids_;
  std::unordered_map<size_t, FeaturePerId> feature_per_ids_;
};

using FrameManagerPtr = std::shared_ptr<FrameManager>; 

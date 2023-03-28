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

#include <mivins/common/types.h>
#include <mivins/common/frame.h>
#include <mivins/tracker/feature_tracker_obs.h>
#include <mivins/tracker/feature_tracker_tools.h>

namespace mivins
{
    namespace feature_tracker_tools
    {

        double GetTracksDisparityPercentile(
            const FeatureTracksPerCam &tracks,
            double pivot_ratio)
        {
            CHECK_GT(pivot_ratio, 0.0) << "pivot_ratio needs to be in (0,1)";
            CHECK_LT(pivot_ratio, 1.0) << "pivot_ratio needs to be in (0,1)";

            if (tracks.empty())
                return 0.0;

            // compute all disparities.
            std::vector<double> disparities;
            disparities.reserve(tracks.size());
            for (const FeatureTrackPerId &track : tracks)
                disparities.push_back(track.GetDisparity());

            // compute percentile.
            const size_t pivot = std::floor(pivot_ratio * disparities.size());
            CHECK_LT(pivot, disparities.size());
            std::nth_element(disparities.begin(), disparities.begin() + pivot, disparities.end(),
                             std::greater<double>());
            return disparities[pivot];
        }

        void GetFeatureMatches(
            const Frame &frame1, const Frame &frame2,
            std::vector<std::pair<size_t, size_t>> *matches_12)
        {
            CHECK_NOTNULL(matches_12);

            // Create lookup-table with track-ids from frame 1.
            std::unordered_map<int, size_t> trackid_slotid_map;
            for (size_t i = 0; i < frame1.num_features_; ++i)
            {
                int track_id_1 = frame1.track_id_vec_(i);
                if (track_id_1 >= 0)
                    trackid_slotid_map[track_id_1] = i;
            }

            // Create list of matches.
            matches_12->reserve(frame2.num_features_);
            for (size_t i = 0; i < frame2.num_features_; ++i)
            {
                int track_id_2 = frame2.track_id_vec_(i);
                if (track_id_2 >= 0)
                {
                    const auto it = trackid_slotid_map.find(track_id_2);
                    if (it != trackid_slotid_map.end())
                        matches_12->push_back(std::make_pair(it->second, i));
                }
            }
        }

        void GetFeatureMatches(
            const FramePtr &frame1, const FramePtr &frame2,
            std::unordered_map<size_t, size_t> &matches_12)
        {
            std::unordered_map<int, size_t> trackid_slotid_map;
            for (size_t i = 0; i < frame1->num_features_; ++i)
            {
                int track_id_1 = frame1->track_id_vec_(i);
                if (track_id_1 >= 0)
                    trackid_slotid_map[track_id_1] = i;
            }

            for (size_t i = 0; i < frame2->num_features_; ++i)
            {
                int track_id_2 = frame2->track_id_vec_(i);
                if (track_id_2 >= 0)
                {
                    const auto it = trackid_slotid_map.find(track_id_2);
                    if (it != trackid_slotid_map.end())
                        matches_12[it->second] = i;
                }
            }
        }

        void DrawFeatureMatches(
            const FramePtr &frame1, const FramePtr &frame2,
            std::unordered_map<size_t, size_t> &matches_12)
        {
            cv::Mat track_img;
            cv::Mat img1 = frame1->img_pyr_[0];
            cv::Mat img2 = frame2->img_pyr_[0];
            int cols = img1.cols;
            cv::hconcat(img1, img2, track_img);
            if (track_img.channels() == 1)
                cv::cvtColor(track_img, track_img, cv::COLOR_GRAY2RGB);

            int count = 0;
            for (auto &iter : matches_12)
            {
                count++;
                if (count % 5 != 0)
                    continue;
                size_t idx1 = iter.first;
                size_t idx2 = iter.second;
                cv::Point2d point1(frame1->px_vec_.col(idx1)[0], frame1->px_vec_.col(idx1)[1]);
                cv::Point2d point2(frame2->px_vec_.col(idx2)[0] + cols, frame2->px_vec_.col(idx2)[1]);
                cv::circle(track_img, point1, 2, cv::Scalar(0, 255, 0), 2);
                cv::circle(track_img, point2, 2, cv::Scalar(0, 255, 0), 2);
                cv::line(track_img, point1, point2, cv::Scalar(0, 255, 255), 1, 8, 0);
            }

            // cv::imshow("track_img", track_img);
            // cv::waitKey(1);

            char save_path[256];
            std::string home_path = getenv("HOME");
            snprintf(save_path, sizeof(save_path), "%s/debug/%d_%d.jpg", home_path.c_str(), frame1->GetFrameId(), frame2->GetFrameId());
            cv::imwrite(save_path, track_img);
        }

    } // namespace feature_tracker_tools
} // namespace mivins

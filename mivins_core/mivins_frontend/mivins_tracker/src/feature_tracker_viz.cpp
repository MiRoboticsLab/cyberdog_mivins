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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mivins/common/frame.h>
#include <mivins/tracker/feature_tracker_viz.h>
#include <mivins/tracker/feature_tracker.h>

namespace mivins
{

    void visualizeTracks(
        const FeatureTracker &tracker, size_t frame_index, int sleep)
    {
        const FeatureTracksPerCam tracks = tracker.GetActiveTracks(frame_index);
        if (tracks.empty())
        {
            VLOG(1) << "No features to visualize.";
            return;
        }
        VLOG(5) << "Tracker: Visualize " << tracks.size() << " tracks.";

        cv::Mat img_8u = tracks.at(0).back().GetFrame()->img();
        cv::Mat img_rgb(img_8u.size(), CV_8UC3);
        cv::cvtColor(img_8u, img_rgb, cv::COLOR_GRAY2RGB);
        int frame_id = tracks.at(0).back().GetFrame()->GetFrameId();
        for (size_t i = 0; i < tracks.size(); ++i)
        {
            const FeatureTrackPerId &track = tracks.at(i);
            CHECK_EQ(frame_id, track.back().GetFrame()->GetFrameId());
            cv::line(
                img_rgb,
                cv::Point2f(track.front().GetPx()(0), track.front().GetPx()(1)),
                cv::Point2f(track.back().GetPx()(0), track.back().GetPx()(1)),
                cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("tracking result", img_rgb);
        cv::waitKey(sleep);
    }

} // namespace mivins

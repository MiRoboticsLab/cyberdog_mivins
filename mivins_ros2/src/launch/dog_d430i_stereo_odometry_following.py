# Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
#from convert_calibration import ConvertCalibration

sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from convert_calibration import ConvertCalibration
def generate_launch_description():
    calib_path = os.path.join(
        get_package_share_directory("mivins"), "param/d430/realsense_stereo_d430_new.yaml"
    )
    config_file = os.path.join(
        get_package_share_directory("mivins"),
        "param/d430/realsense_vio_stereo_following.yaml",
    )

    calib_dir = "/params/camera/calibration"
    calib_converted = os.path.join(
        get_package_share_directory("mivins"),
        "param/d430/realsense_stereo_d430_mivins.yaml",
    )
    if ConvertCalibration(calib_dir, calib_converted):
        calib_path = calib_converted

    print(calib_path)
    namespace = LaunchConfiguration("namespace", default="")
    return LaunchDescription(
        [
            # launch a node
            Node(
                package="mivins",
                executable="mivins_node",
                namespace=namespace,
                name="mivinsfollowing",
                output="screen",
                parameters=[
                    {"cam0_topic": "camera/infra1/image_rect_raw"},
                    {"cam1_topic": "camera/infra2/image_rect_raw"},
                    {"imu_topic": "camera/imu"},
                    {"odom_topic": "odom_out"},
                    {"status_topic": "motion_status"},
                    {"calib_file": calib_path},
                    {"config_file": config_file},
                    {"mode_type": 3},
                    {"mode_stable": 1},
                ],
            )
        ]
    )

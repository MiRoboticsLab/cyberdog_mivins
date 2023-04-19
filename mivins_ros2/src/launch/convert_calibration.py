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

from collections import OrderedDict
import yaml
import numpy as np
import numpy.matlib


def represent_ordereddict(dumper, data):
    value = []

    for item_key, item_value in data.items():
        node_key = dumper.represent_data(item_key)
        node_value = dumper.represent_data(item_value)

        value.append((node_key, node_value))

    return yaml.nodes.MappingNode("tag:yaml.org,2002:map", value)


yaml.add_representer(OrderedDict, represent_ordereddict)

cam_params = []
T_B_O = np.matlib.identity(4, dtype=float)


def ReadCamIntrinsic(input_file):
    with open(input_file, "r") as f:
        calib = yaml.load(f, Loader=yaml.FullLoader)

        for idx in range(len(calib)):
            label = "cam{0}".format(idx)

            rostopic = calib[label]["rostopic"]
            camera_model = calib[label]["camera_model"]
            distortion_model = calib[label]["distortion_model"]
            if distortion_model == "radtan":
                distortion_model = "radial-tangential"
            resolution = calib[label]["resolution"]
            intrinsics = calib[label]["intrinsics"]
            distortion_coeffs = calib[label]["distortion_coeffs"]
            if len(distortion_coeffs) == 5:
                del distortion_coeffs[2]

            T_B_C = np.matlib.identity(4, dtype=float)

            param = {
                "label": label,
                "rostopic": rostopic,
                "camera_model": camera_model,
                "distortion_model": distortion_model,
                "resolution": resolution,
                "distortion_coeffs": distortion_coeffs,
                "intrinsics": intrinsics,
                "T_B_C": T_B_C,
            }

            cam_params.append(param)


def ReadImuExtrinsic(input_file):
    with open(input_file, "r") as f:
        calib = yaml.load(f, Loader=yaml.FullLoader)

        label = "cam0"
        T_cam0_imu = np.array(calib[label]["T_cam_imu"])
        T_B_C = np.linalg.inv(T_cam0_imu)
        for param in cam_params:
            if param["label"] == label:
                param["T_B_C"] = T_B_C
                break


def ReadCamExtrinsic(input_file):
    with open(input_file, "r") as f:
        calib = yaml.load(f, Loader=yaml.FullLoader)

        T_imu_cam0 = np.matlib.identity(4, dtype=float)
        for param in cam_params:
            if param["label"] == "cam0":
                T_imu_cam0 = param["T_B_C"]
                break

        for idx in range(len(calib)):
            label = "cam{0}".format(idx)

            if idx == 0:
                continue

            T_camN_cam0 = np.array(calib[label]["T_cn_c0"])
            T_B_C = np.matmul(T_imu_cam0, np.linalg.inv(T_camN_cam0))

            for param in cam_params:
                if param["label"] == label:
                    param["T_B_C"] = T_B_C
                    break


def ReadOdomExtrinsic(input_file):
    global T_B_O
    with open(input_file, "r") as f:
        calib = yaml.load(f, Loader=yaml.FullLoader)

        T_imu_cam0 = np.matlib.identity(4, dtype=float)
        for param in cam_params:
            if param["label"] == "cam0":
                T_imu_cam0 = param["T_B_C"]
                break

        T_cam_odom = np.array(calib["cam0"]["T_cam_imu"])
        T_B_O = np.matmul(T_imu_cam0, T_cam_odom)


def ReadCalibFiles(calib_dir):
    cam_intrinsic = calib_dir + "/params_intrinsic.yaml"
    imu_extrinsic = calib_dir + "/params_imu_extrinsic.yaml"
    cam_extrinsic = calib_dir + "/params_extrinsic.yaml"
    odom_extrinsic = calib_dir + "/params_bodyimu_extrinsic.yaml"

    ReadCamIntrinsic(cam_intrinsic)
    ReadImuExtrinsic(imu_extrinsic)
    ReadCamExtrinsic(cam_extrinsic)
    ReadOdomExtrinsic(odom_extrinsic)


def WriteMivinsCalib(output_file):
    cams = []
    cam_n = 2

    for id in range(cam_n):
        idx = id
        if id == 1:
            idx = 2

        cam = OrderedDict()

        cam["camera"] = OrderedDict()
        cam["camera"]["label"] = cam_params[id]["label"]
        cam["camera"]["id"] = id
        cam["camera"]["line-delay-nanoseconds"] = 0
        cam["camera"]["image_height"] = cam_params[idx]["resolution"][1]
        cam["camera"]["image_width"] = cam_params[idx]["resolution"][0]
        cam["camera"]["type"] = cam_params[idx]["camera_model"]

        cam["camera"]["intrinsics"] = OrderedDict()
        cam["camera"]["intrinsics"]["cols"] = 1
        cam["camera"]["intrinsics"]["rows"] = len(cam_params[idx]["intrinsics"])
        cam["camera"]["intrinsics"]["data"] = cam_params[idx]["intrinsics"]

        cam["camera"]["distortion"] = OrderedDict()
        cam["camera"]["distortion"]["type"] = cam_params[idx]["distortion_model"]
        cam["camera"]["distortion"]["parameters"] = OrderedDict()
        cam["camera"]["distortion"]["parameters"]["cols"] = 1
        cam["camera"]["distortion"]["parameters"]["rows"] = len(
            cam_params[idx]["distortion_coeffs"]
        )
        cam["camera"]["distortion"]["parameters"]["data"] = cam_params[idx][
            "distortion_coeffs"
        ]

        cam["camera"]["intrinsics"] = OrderedDict()
        cam["camera"]["intrinsics"]["cols"] = 1
        cam["camera"]["intrinsics"]["rows"] = len(cam_params[idx]["intrinsics"])
        cam["camera"]["intrinsics"]["data"] = cam_params[idx]["intrinsics"]

        cam["T_B_C"] = OrderedDict()
        cam["T_B_C"]["cols"] = 4
        cam["T_B_C"]["rows"] = 4
        cam["T_B_C"]["data"] = cam_params[idx]["T_B_C"].flatten().tolist()

        cam["camera"]["description"] = cam_params[idx]["rostopic"]
        cams.append(cam)

    f = open(output_file, "w")
    P_cams = OrderedDict()

    P_cams["label"] = "Realsense"
    P_cams["cameras"] = cams

    f.write(yaml.dump(P_cams, default_flow_style=None))

    f.write("\n")

    P_imu = OrderedDict()
    P_imu["imu_params"] = OrderedDict()
    P_imu["imu_params"]["delay_imu_cam"] = 0.0
    P_imu["imu_params"]["max_imu_delta_t"] = 0.01
    P_imu["imu_params"]["acc_max"] = 176.0
    P_imu["imu_params"]["omega_max"] = 17

    P_imu["imu_params"]["sigma_acc_c"] = 0.5
    P_imu["imu_params"]["sigma_omega_c"] = 0.01

    P_imu["imu_params"]["sigma_acc_bias_c"] = 0.001
    P_imu["imu_params"]["sigma_omega_bias_c"] = 0.0001

    P_imu["imu_params"]["imu_rate"] = 200
    P_imu["imu_params"]["g"] = 9.8082
    P_imu["imu_params"]["sigma_integration"] = 0.0

    P_imu_init = OrderedDict()
    P_imu_init["imu_initialization"] = OrderedDict()
    P_imu_init["imu_initialization"]["velocity"] = [0.0, 0.0, 0.0]
    P_imu_init["imu_initialization"]["omega_bias"] = [0.0, 0.0, 0.0]
    P_imu_init["imu_initialization"]["acc_bias"] = [0.0, 0.0, 0.0]
    P_imu_init["imu_initialization"]["velocity_sigma"] = 2.0
    P_imu_init["imu_initialization"]["omega_bias_sigma"] = 0.01
    P_imu_init["imu_initialization"]["acc_bias_sigma"] = 0.1

    f.write(yaml.dump(P_imu, default_flow_style=None))
    f.write("\n")
    f.write(yaml.dump(P_imu_init, default_flow_style=None))
    f.write("\n")

    P_odom = OrderedDict()
    P_odom["odometry_params"] = OrderedDict()
    P_odom["odometry_params"]["delay_odom"] = 0.0
    P_odom["odometry_params"]["max_odom_delta_t"] = 0.0
    P_odom["odometry_params"]["linear_velocity_n"] = 0.2
    P_odom["odometry_params"]["angular_velocity_n"] = 0.1
    P_odom["odometry_params"]["vel_scale"] = [1.0, 1.0, 1.0]
    P_odom["odometry_params"]["gyr_scale"] = [1.0, 1.0, 1.0]
    P_odom["odometry_params"]["T_B_O"] = OrderedDict()
    P_odom["odometry_params"]["T_B_O"]["cols"] = 4
    P_odom["odometry_params"]["T_B_O"]["rows"] = 4
    P_odom["odometry_params"]["T_B_O"]["data"] = T_B_O.flatten().tolist()

    f.write(yaml.dump(P_odom, default_flow_style=None))
    f.write("\n")

    f.close()


def ConvertCalibration(calib_dir, output_file):
    try:
        ReadCalibFiles(calib_dir)
    except Exception as e:
        print("Can not read calibration files.")
        print("Reason:", e)
        return False
    else:
        print("Success in reading calibration files.")

    try:
        WriteMivinsCalib(output_file)
    except Exception as e:
        print("Can not write calibration file.")
        print("Reason:", e)
        return False
    else:
        print("Success in writing calibration files.")

    return True

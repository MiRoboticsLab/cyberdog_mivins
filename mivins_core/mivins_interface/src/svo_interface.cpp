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

#include <svo/svo_interface.h>

#include <svo/svo_factory.h>
#include <mivins/common/frame.h>
#include <mivins/frontend_local_map.h>
#include <svo/param.h>
#include <mivins/channel_imu.h>
#include <mivins/channel_odom.h>
#include <mivins/common/camera.h>
#include <mivins/common/conversions.h>
#include <mivins/common/logging.h>
//#include <mivins/frame_handler_rgbd.h>
#include <mivins/channel_frame_mono.h>
#include <mivins/channel_frame_stereo.h>
#include <mivins/channel_frame_rgbdfisheye.h>
//#include <mivins/frame_handler_triple_with_depth.h>
#include <mivins/initialization.h>
#include <mivins/direct/depth_optimization.h>

// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/exact_time.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <image_transport/subscriber_filter.h>
// #include <image_transport/image_transport.h>
// #include <sensor_msgs/image_encodings.h>
// #include <cv_bridge/cv_bridge.h>
#include <mivins/utils/timer.h>
#include <svo/vins_backend_factory.h> // wcp add

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

using namespace std;

namespace mivins
{

    SvoInterface::SvoInterface(const std::string config_file, const ::std::string calib_file)
    {
        calib_file_ = calib_file;
        config_file_ = config_file;
        YAML::Node config = YAML::LoadFile(config_file_);
        pipeline_type_ = PipelineType(getParam<int>(config, "pipeline", 1));
        remote_key_topic_ =
            getParam<std::string>(config, "remote_key_topic", "svo/remote_key");
        automatic_reinitialization_ =
            getParam<bool>(config, "automatic_reinitialization", false);
        set_initial_attitude_from_gravity_ =
            getParam<bool>(config, "set_initial_attitude_from_gravity", true);
        use_constraint_odom_ = 
            getParam<bool>(config, "use_constraint_odom", false);
        use_processed_odom_ =
            getParam<bool>(config, "use_processed_odom", false);
        update_pose_ =
            getParam<bool>(config, "use_pose_update", false);

        switch (pipeline_type_)
        {
        // case PipelineType::kRgbd:
        //   std::cout << "PipelineType: kRgbd\n";
        //   svo_ = factory::makeRgbd(config_file_, calib_file_);
        //   break;
        case PipelineType::kMono:
            //std::cout << "PipelineType: kMono\n";
            SVO_DEBUG_STREAM("PipelineType: kMono");
            svo_ = factory::makeMono(config_file_, calib_file_);
            break;
        case PipelineType::kStereo:
            //std::cout << "PipelineType: kStereo\n";
            SVO_DEBUG_STREAM("PipelineType: kStereo");
            svo_ = factory::makeStereo(config_file_, calib_file_);
            break;
        // case PipelineType::kTripleWithStereo:
        //   std::cout << "PipelineType: kTripleWithStereo\n";
        //   svo_ = factory::makeTripleWithStereo(config_file_, calib_file_);
        //   break;
        case PipelineType::kTripleWithDepth:
            //std::cout << "PipelineType: kTripleWithDepth\n";
            SVO_DEBUG_STREAM("PipelineType: kTripleWithDepth");
            svo_ = factory::makeTripleWithDepth(config_file_, calib_file_);
            break;
        default:
            LOG(FATAL) << "Unknown pipeline";
            break;
        }

        if (getParam<bool>(config, "use_imu", false))
        {
            //std::cout << "use_imu........" << std::endl;
            SVO_DEBUG_STREAM("use_imu........");
            imu_handler_ = factory::getImuHandler(config_file_, calib_file_);
            svo_->imu_handler_ = imu_handler_;
            header_tracker_ = std::make_shared<ekf3dof::HeadTracker>();
            header_tracker_->Resume();
        }

        if (getParam<bool>(config, "use_odom", false))
        {
            odom_handler_ = factory::getOdomHandler(calib_file_);
            svo_->odom_handler_ = odom_handler_;
        }
        use_imu_only_for_gravityalign_ = false;
        if (getParam<bool>(config, "use_imu_only_for_gravityalign_", false))
        {
            //std::cout << "use_imu_only_for_gravityalign_........" << std::endl;
            SVO_DEBUG_STREAM("use_imu_only_for_gravityalign_........");
        }
        if(getParam<bool>(config, "use_loose_couple", false))
        {
            if(odom_handler_)
            {
                //std::cout << "Loose couple would be used.\n";
                SVO_DEBUG_STREAM("Loose couple would be used.");
                loose_couple_pose_.setIdentity();
                loose_couple_ = factory::getLooseCoupleHandler(
                                    config_file_, calib_file_);
                automatic_reinitialization_ = false;
            }
        }

        if(update_pose_)
        {
            if(odom_handler_)
            {
                //std::cout << "Pose update would be used.\n";
                SVO_DEBUG_STREAM("Pose update would be used.");

                pose_updater_ = factory::getPoseUpdateHandler(
                                    config_file_);
            }
        }

        // wcp TODO:
        if (getParam<bool>(config, "backend_opt", false))
        {
            //std::cout << "------------------------------------------------ make vins backend begin\n";
            SVO_DEBUG_STREAM("------------------------------------------------ make vins backend begin");
            vins_backend_interface_ = vins_backend_factory::makeBackend(config_file_, svo_->GetNCamera(), (int)pipeline_type_);
            svo_->SetVinsBackend(vins_backend_interface_);

            if (use_imu_only_for_gravityalign_)
            {
                vins_backend_interface_->setImuHandler(nullptr);
            }
            else
            {
                vins_backend_interface_->setImuHandler(imu_handler_);
            }
            vins_backend_interface_->setOdomHandler(odom_handler_);
            //std::cout << "---------------------------------------------------make vins backend end\n";
            SVO_DEBUG_STREAM("----------------------------------------------------make vins backend end");
        }

        exposure_time_enable = false;
        if (getParam<bool>(config, "exposure_time_enable", false))
        {
            exposure_time_enable = true;
            string l_times = getParam<string>(config, "photometric_calib_path", "/tmp/") + "cam0/times.txt";
            string r_times = getParam<string>(config, "photometric_calib_path", "/tmp/") + "cam1/times.txt";
            loadTimes(l_times, 0);
            loadTimes(r_times, 1);
            //std::cout << "ltimes size=" << ltimes.size() << "," << rtimes.size() << std::endl;
            SVO_DEBUG_STREAM("ltimes size=" << ltimes.size() << "," << rtimes.size());
        }

        svo_->Start();
    }
    std::vector<double> SvoInterface::GetImuBaselinkCalib() 
    {
        if(!odom_handler_) odom_handler_ = factory::getOdomHandler(calib_file_);
        Eigen::Quaterniond q = odom_handler_->odom_calib_.T_B_O.GetRotation().ToImplementation();
        Eigen::Vector3d p = odom_handler_->odom_calib_.T_B_O.GetPosition();
        std::vector<double> baselink_to_imu_calib = {p[0], p[1], p[2], q.x(), q.y(), q.z(), q.w()};
        return baselink_to_imu_calib;
    }
    std::vector<double> SvoInterface::GetCameraImuCalib() 
    {
        Eigen::Quaterniond q = svo_->cams_->get_T_C_B(0).GetRotation().ToImplementation();
        Eigen::Vector3d p = svo_->cams_->get_T_C_B(0).GetPosition();
        std::vector<double> imu_to_camera_calib = {p[0], p[1], p[2], q.x(), q.y(), q.z(), q.w()};
        return imu_to_camera_calib;
    }
    SvoInterface::~SvoInterface()
    {
        // svo_->ResetAllEnd();
        //std::cout<< "SvoInterface Destructed SVO1" << std::endl;
        VLOG(1) << "Destructed SVO.";
    }
    void SvoInterface::SvoShutdown()
    {
        //loose_couple_->reset();
        if(loose_couple_)
        {
            loose_couple_pose_.setIdentity();
            loose_couple_ = factory::getLooseCoupleHandler(
                            config_file_, calib_file_);
        }
        svo_->ResetAllEnd();
        svo_->abnormal_cnt_ = 0;
        imu_handler_->reset();
        odom_handler_->reset();
        header_tracker_.reset();
        header_tracker_ = std::make_shared<ekf3dof::HeadTracker>();
        header_tracker_->Resume();
        prv_img_ts_ = 0;
        prv_odom_ts_ = 0;
        prv_imu_ts_ = 0;
        b_first_get_init_with_vio_g_ = false;

        init_stable_mode_ = false;
        stable_mode_first_stage_ = false;
        robot_status_ = -1;

        pose_follow_.setIdentity();
        pos_odom_last_.setZero();
        odom_last_.setOrigin();
        odom_processed_ = false;
        if(pose_updater_)
            pose_updater_ = factory::getPoseUpdateHandler(config_file_);

        svo_->Start();
        //std::cout<< "SvoInterface Destructed SVO2" << std::endl;
        SVO_DEBUG_STREAM("SvoInterface Destructed SVO2");
    }

    void SvoInterface::savefile()
    {
        svo_->savefile();
    }

    void SvoInterface::finishfile()
    {
        svo_->finish_savefile();
    }

    bool SvoInterface::setImuPrior(const int64_t timestamp_nanoseconds)
    {
        if (svo_->GetVinsBackend())
        {
            //if we use backend, this will take care of setting priors
            if (!svo_->HasStarted())
            {
                //when starting up, make sure we already have IMU measurements
                if (imu_handler_ && imu_handler_->GetMeasurementsCopy().size() < 10u)
                {
                    return false;
                }
            }
            // return true;
        }

        if (imu_handler_ && !svo_->HasStarted() && set_initial_attitude_from_gravity_)
        {
            // set initial orientation
            Quaternion R_imu_world;
            if (imu_handler_->GetInitialAttitude(
                    timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
                    R_imu_world))
            {
                VLOG(3) << "Set initial orientation from accelerometer measurements.";
                {
                    Eigen::Quaterniond q_bw(R_imu_world.w(),R_imu_world.x(),R_imu_world.y(),R_imu_world.z());
                    const float pi = 3.14159265358979323846264;
                    cout<<"q(zyx) = "<<R_imu_world.w()<<","<<R_imu_world.x()<<","<<R_imu_world.y()<<","<<R_imu_world.z()<<std::endl;
                    cout<<"yaw pitch roll(zyx) = "<<q_bw.matrix().eulerAngles(2,1,0).transpose()*180/pi<<endl;
                }
                svo_->SetRotationPrior(R_imu_world);
            }
            else
            {
                return false;
            }
        }
        else if (imu_handler_ && svo_->GetLastFrames())
        {
            // set incremental rotation prior
            Quaternion R_lastimu_newimu;
            if (imu_handler_->GetRelativeRotationPrior(
                    svo_->GetLastFrames()->GetMinTimestampNanoseconds() *
                        common::conversions::kNanoSecondsToSeconds,
                    timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
                    false, false, R_lastimu_newimu))
            {
                VLOG(3) << "Set incremental rotation prior from IMU.";
                svo_->SetRotationIncrementPrior(R_lastimu_newimu);
            }
        }
        return true;
    }

    bool SvoInterface::setImuPrior_3dof(const int64_t timestamp_nanoseconds)
    {
        if (svo_->GetVinsBackend())
        {
            //if we use backend, this will take care of setting priors
            if (!svo_->HasStarted())
            {
                //when starting up, make sure we already have IMU measurements
                if (imu_handler_ && imu_handler_->GetMeasurementsCopy().size() < 10u)
                {
                    return false;
                }
            }
            // return true;
        }

        if (imu_handler_ && !svo_->HasStarted() && set_initial_attitude_from_gravity_)
        {
            // // set initial orientation
            // Quaternion R_imu_world;
            // if (imu_handler_->GetInitialAttitude(
            //         timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
            //         R_imu_world))
            // {
            //     VLOG(3) << "Set initial orientation from accelerometer measurements.";
            //     svo_->SetRotationPrior(R_imu_world);
            // }
            // else
            // {
            //     return false;
            // }

            // set initial orientation
            ImuMeasurements imu_measurements;
            if(!imu_handler_->GetMeasurementsContainingEdges(timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds, imu_measurements, false))
            {
                return false;
            }
            else if(imu_measurements.size() < 600)
            {
                return false;
            }
            for(int i = (int)imu_measurements.size() - 1; i >= 0; --i)
            {
                double t = imu_measurements[i].timestamp_;
                Eigen::Vector3d gyr = imu_measurements[i].angular_velocity_;
                Eigen::Vector3d acc = imu_measurements[i].linear_acceleration_;

                ekf3dof::GyroscopeData sample_gyr;
                sample_gyr.sensor_timestamp_ns = t * 1e9;
                sample_gyr.system_timestamp = t * 1e9;
                sample_gyr.data = ekf3dof::Vector3(gyr.x(), gyr.y(), gyr.z());
                header_tracker_->OnGyroscopeData(sample_gyr);

                ekf3dof::AccelerometerData sample_acc;
                sample_acc.sensor_timestamp_ns = t * 1e9;
                sample_acc.system_timestamp = t * 1e9;
                sample_acc.data = ekf3dof::Vector3(acc.x(), acc.y(), acc.z());
                header_tracker_->OnAccelerometerData(sample_acc);
            }
            std::array<float, 3> out_position;      //x,y,z
            std::array<float, 4> out_orientation;   //x,y,z,w
            if(!header_tracker_->GetPose(timestamp_nanoseconds, out_position, out_orientation))
            {
                return false;
            }
            Quaternion R_imu_world(out_orientation[3], out_orientation[0], out_orientation[1], out_orientation[2]);
            {
                Eigen::Quaterniond q_bw(R_imu_world.w(),R_imu_world.x(),R_imu_world.y(),R_imu_world.z());
                const float pi = 3.14159265358979323846264;
                //cout<<"q(zyx) = "<<out_orientation[3]<<","<<out_orientation[0]<<","<<out_orientation[1]<<","<<out_orientation[2]<<std::endl;
                cout<<"yaw pitch roll(zyx bw) = "<<q_bw.matrix().eulerAngles(2,1,0).transpose()*180/pi<<endl;
                Eigen::Quaterniond q_cb = svo_->cams_->get_T_C_B(0).GetRotation().ToImplementation();
                cout<<"yaw pitch roll(zyx bc) = "<< q_cb.matrix().inverse().eulerAngles(2,1,0).transpose()*180/pi<<endl;
                //yaw pitch roll(r_bc zyx) =  90.0145 -179.823 -101.443
                Eigen::Matrix3d rotation_wc = (q_bw.matrix().inverse())*(q_cb.matrix().inverse());
                cout<<"yaw pitch roll(zyx wc) = "<<rotation_wc.eulerAngles(2,1,0).transpose()*180/pi<<endl;
            }
            VLOG(3) << "Set initial orientation from accelerometer measurements.";
            //std::cout<< " setImuPrior 11111 --------------------------------------------------------------------------------------------------------- " << std::endl;
            //std::cout<<"q(R_imu_world):"<<R_imu_world.x()<<" "<<R_imu_world.y()<<" "<<R_imu_world.z()<<" "<<R_imu_world.w()<<std::endl;
            svo_->SetRotationPrior(R_imu_world);

        }
        else if (imu_handler_ && svo_->GetLastFrames())
        {
            // set incremental rotation prior
            // Quaternion R_lastimu_newimu;
            // if (imu_handler_->GetRelativeRotationPrior(
            //         svo_->GetLastFrames()->GetMinTimestampNanoseconds() *
            //             common::conversions::kNanoSecondsToSeconds,
            //         timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
            //         false, false, R_lastimu_newimu))
            // {
            //     VLOG(3) << "Set incremental rotation prior from IMU.";
            //     svo_->SetRotationIncrementPrior(R_lastimu_newimu);
            // }

            ImuMeasurements measurements;
            double old_cam_timestamp = svo_->GetLastFrames()->GetMinTimestampNanoseconds() * common::conversions::kNanoSecondsToSeconds;
            double new_cam_timestamp = timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds;
            if (!imu_handler_->GetMeasurements(old_cam_timestamp, new_cam_timestamp, false, measurements))
                return false;
            std::array<float, 3> out_position;      //x,y,z
            std::array<float, 4> out_orientation;   //x,y,z,w
            if(!header_tracker_->GetPose(svo_->GetLastFrames()->GetMinTimestampNanoseconds(), out_position, out_orientation))
            {
                return false;
            }
            Quaternion R_imuold_world(out_orientation[3], out_orientation[0], out_orientation[1], out_orientation[2]);


            for(int i = (int)measurements.size() - 1; i >= 0; --i)
            {
                double t = measurements[i].timestamp_;
                Eigen::Vector3d gyr = measurements[i].angular_velocity_;
                Eigen::Vector3d acc = measurements[i].linear_acceleration_;

                ekf3dof::GyroscopeData sample_gyr;
                sample_gyr.sensor_timestamp_ns = t * 1e9;
                sample_gyr.system_timestamp = t * 1e9;
                sample_gyr.data = ekf3dof::Vector3(gyr.x(), gyr.y(), gyr.z());
                header_tracker_->OnGyroscopeData(sample_gyr);

                ekf3dof::AccelerometerData sample_acc;
                sample_acc.sensor_timestamp_ns = t * 1e9;
                sample_acc.system_timestamp = t * 1e9;
                sample_acc.data = ekf3dof::Vector3(acc.x(), acc.y(), acc.z());
                header_tracker_->OnAccelerometerData(sample_acc);
            }
            if(!header_tracker_->GetPose(timestamp_nanoseconds, out_position, out_orientation))
            {
                return false;
            }
            Quaternion R_imunew_world(out_orientation[3], out_orientation[0], out_orientation[1], out_orientation[2]);
            Quaternion R_lastimu_newimu = R_imuold_world * R_imunew_world.Inverse();
            //svo_->SetRotationIncrementPrior(R_lastimu_newimu);
            svo_->SetRotationPrior(R_imunew_world);
            //std::cout<< " setImuPrior 22222 --------------------------------------------------------------------------------------------------------- " << std::endl;
        }
        return true;

    }

    bool SvoInterface::setOrientationPrior(const int64_t timestamp_nanoseconds)
    {
        if (svo_->GetVinsBackend())
        {
            //if we use backend, this will take care of setting priors
            if (!svo_->HasStarted())
            {
                //when starting up, make sure we already have IMU measurements
                if (imu_handler_ && imu_handler_->GetMeasurementsCopy().size() < 10u)
                {
                    return false;
                }
            }
            // return true;
        }

        if (odom_handler_ && imu_handler_ && !svo_->HasStarted() && set_initial_attitude_from_gravity_)
        {
            // set initial pose
            Quaternion R_imu_world; 
            Eigen::Vector3d t_imu_world;
            Transformation T_w_o;
            if (odom_handler_->GetCorresPoseProcessed(
                timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds, 
                T_w_o))
            {
                Transformation T_w_b = T_w_o * odom_handler_->odom_calib_.T_B_O.Inverse();
                R_imu_world = T_w_b.Inverse().GetRotation();
                t_imu_world = T_w_b.Inverse().GetPosition();
                svo_->SetRotationPrior(R_imu_world);
                svo_->SetPositionPrior(t_imu_world);
            }
            else
            {
                return false;
            }
        }
        else if (imu_handler_ && svo_->GetLastFrames())
        {
            // set incremental rotation prior
            Quaternion R_lastimu_newimu;
            if (imu_handler_->GetRelativeRotationPrior(
                    svo_->GetLastFrames()->GetMinTimestampNanoseconds() *
                        common::conversions::kNanoSecondsToSeconds,
                    timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
                    false, false, R_lastimu_newimu))
            {
                VLOG(3) << "Set incremental rotation prior from IMU.";
                svo_->SetRotationIncrementPrior(R_lastimu_newimu);
            }
        }
        return true;
    }

    int SvoInterface::getStage()
    {
        if(stable_mode_first_stage_)
        {
            if(init_stable_mode_)
                return (int)Stage::kTracking;
            else
                return (int)Stage::kInitializing;
        }
        if((int)svo_->stage() == (int)Stage::kTracking) {
            b_first_get_init_with_vio_g_ = true;
        }
        if(loose_couple_&&b_first_get_init_with_vio_g_)
          return (int)Stage::kTracking;
        return (int)svo_->stage();
    }

    int SvoInterface::getTrackingQuality() const
    {
        return static_cast<int>(svo_->trackingQuality());
    }

    int SvoInterface::getCamSize() const
    {
        return (int)svo_->cams_->getNumCameras();
    }

    int SvoInterface::getPipelineType() const
    {
        return (int)pipeline_type_;
    }

    bool SvoInterface::imuHandlerValid() const
    {
        return imu_handler_ != nullptr;
    }

    bool SvoInterface::odomHandlerValid() const
    {
        return odom_handler_ != nullptr;
    }

    void SvoInterface::processImageData(const int64_t ts,
                                        const std::vector<cv::Mat> &images,
                                        const std::map<int, cv::Mat> &depths)
    {
        if(ts * 1e-9 < prv_img_ts_)
        {
            printf("Img timestamp is disorder: ");
            printf("Prv img ts: %.9lf; Cur img ts: %.9lf\n", prv_img_ts_, ts * 1e-9);
            return;
        }
        prv_img_ts_ = ts * 1e-9;

        //for following
        if(stable_mode_first_stage_)
            return;
        else
        {
            if(update_pose_)
            {
                if(!setOrientationPrior(ts))
                {
                    VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
                    return;
                }
            }
            else
            {
                //if (!setImuPrior(ts))
                if(!setImuPrior_3dof(ts))
                {
                    VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
                    return;
                }
            }

            // if(!svo_->vins_backend_)
            // {
            //   vins_backend_interface_ = vins_backend_factory::makeBackend(config_file_, svo_->GetNCamera(), (int)pipeline_type_);
            //   svo_->SetVinsBackend(vins_backend_interface_);
            //   vins_backend_interface_->setImuHandler(imu_handler_);
            //   vins_backend_interface_->setOdomHandler(odom_handler_);
            // }

            std::vector<float> exposure_times;
            if (exposure_time_enable)
            {
                float lt = 1;
                auto liter = ltimes.find(ts);
                if (liter != ltimes.end())
                {
                    lt = liter->second;
                    std::cout<<"find...l..."<<liter->second <<std::endl;
                }
                exposure_times.push_back(lt);

                float rt = 1;
                auto riter = rtimes.find(ts);
                if (riter != rtimes.end())
                {
                    rt = riter->second;
                    std::cout<<"find...r..."<<riter->second <<std::endl;
                }
                exposure_times.push_back(rt);
            }
            
            abnormalProcess(ts);

            svo_->AddImageBundle(images, depths, ts, exposure_times);
            
            
            if(data_align_)
                data_align_->align(ts * 1e-9);
            
            looseCoupleProcess();

            updatePose(ts);

            if (svo_->stage() == Stage::kPaused && automatic_reinitialization_)
                svo_->Start();
        }
    }

    void SvoInterface::inputImuData(const double ts,
                                    const Eigen::Vector3d &acc_imu,
                                    const Eigen::Vector3d &gyr_imu)
    {
        if(ts < prv_imu_ts_)
        {
            printf("Imu timestamp is disorder: ");
            printf("Prv imu ts: %.9lf; Cur imu ts: %.9lf\n", prv_imu_ts_, ts);
            return;
        }
        prv_imu_ts_ = ts;

        const ImuMeasurement m(ts, gyr_imu, acc_imu);
        if (imu_handler_)
            imu_handler_->AddImuMeasurement(m);
        else
            SVO_ERROR_STREAM("SvoNode has no ChannelImu");
            
        if(!data_align_) data_align_.reset(new DataAlign());
        data_align_->inputImu(ts, gyr_imu, acc_imu);
    }

    void SvoInterface::inputOdomData(const double ts,
                                     const Eigen::Quaterniond &orientation, 
                                     const Eigen::Vector3d &position, 
                                     const Eigen::Vector3d &linear_velocity,
                                     const Eigen::Vector3d &angular_velocity)
    {
        if(orientation.coeffs().hasNaN() || position.hasNaN() || linear_velocity.hasNaN() || angular_velocity.hasNaN())
            return;

        if(ts < prv_odom_ts_)
        {
            printf("Odom timestamp is disorder: ");
            printf("Prv odom ts: %.9lf; Cur odom ts: %.9lf\n", prv_odom_ts_, ts);
            return;
        }
        prv_odom_ts_ = ts;
        
        const OdomMeasurement m(ts, orientation, position, linear_velocity, angular_velocity);
        if(!stable_mode_first_stage_)
        {
            if(update_pose_)
            {
                OdomInfo odom_info = processOdom(ts, orientation, position, linear_velocity, angular_velocity);
                const OdomMeasurement m_processed(odom_info.timestamp, odom_info.orientation, 
                    odom_info.position, odom_info.linear_velocity, odom_info.angular_velocity);

                if(use_processed_odom_)
                {
                    if(odom_handler_)
                    {
                        odom_handler_->AddOdomMeasurement(m_processed);
                        odom_handler_->AddOdomMeasurementProcessed(m_processed);
                    }
                    else
                        SVO_ERROR_STREAM("SvoNode has no ChannelOdom");

                    if(!data_align_) data_align_.reset(new DataAlign());
                    data_align_->inputOdom(ts, m_processed.orientation_, m_processed.position_, 
                        m_processed.linear_velocity_, m_processed.angular_velocity_);
                }
                else
                {
                    if(odom_handler_)
                    {
                        odom_handler_->AddOdomMeasurement(m);
                        odom_handler_->AddOdomMeasurementProcessed(m_processed);
                    }
                    else
                        SVO_ERROR_STREAM("SvoNode has no ChannelOdom");

                    if(!data_align_) data_align_.reset(new DataAlign());
                    data_align_->inputOdom(ts, orientation, position, linear_velocity, angular_velocity);
                }
            }
            else
            {
                if(use_processed_odom_)
                {
                    OdomInfo odom_info = processOdom(ts, orientation, position, linear_velocity, angular_velocity);
                    const OdomMeasurement m_processed(odom_info.timestamp, odom_info.orientation, 
                        odom_info.position, odom_info.linear_velocity, odom_info.angular_velocity);

                    if(odom_handler_)
                        odom_handler_->AddOdomMeasurement(m_processed);
                    else    
                        SVO_ERROR_STREAM("SvoNode has no ChannelOdom");

                    if(!data_align_) data_align_.reset(new DataAlign());
                    data_align_->inputOdom(ts, m_processed.orientation_, m_processed.position_, 
                        m_processed.linear_velocity_, m_processed.angular_velocity_);
                }
                else
                {
                    if(odom_handler_)
                        odom_handler_->AddOdomMeasurement(m);
                    else
                        SVO_ERROR_STREAM("SvoNode has no ChannelOdom");

                    if(!data_align_) data_align_.reset(new DataAlign());
                    data_align_->inputOdom(ts, orientation, position, linear_velocity, angular_velocity);
                }
            }
        }
        else
        {
            if(!odom_handler_)
                return;

            OdomInfo odom_info = processOdom(ts, orientation, position, linear_velocity, angular_velocity);
            const OdomMeasurement m_processed(odom_info.timestamp, odom_info.orientation, 
                odom_info.position, odom_info.linear_velocity, odom_info.angular_velocity);

            odom_handler_->AddOdomMeasurementProcessed(m_processed);

            if(!init_stable_mode_)
                init_stable_mode_ = true;
          
            Transformation T_w_o;
            if(odom_handler_->GetLatestPoseProcessed(T_w_o))
                pose_follow_ = T_w_o;
        }

    }

    OdomInfo SvoInterface::processOdom(const double ts,
                                       const Eigen::Quaterniond &orientation, 
                                       const Eigen::Vector3d &position, 
                                       const Eigen::Vector3d &linear_velocity,
                                       const Eigen::Vector3d &angular_velocity)
    {
        Eigen::Quaterniond ori_cur;
        Eigen::Vector3d pos_cur;
        Eigen::Vector3d linear_velocity_cur;
        Eigen::Vector3d angular_velocity_cur;

        Eigen::Quaterniond ori = orientation;
        if(use_constraint_odom_)
        {
            if (robot_status_!=MotionMode::UpStair &&
                robot_status_!=MotionMode::DownStair &&
                robot_status_!=MotionMode::Climbing)
            {
                Eigen::Matrix3d rot = orientation.toRotationMatrix();
                Eigen::Vector3d rpy = vk::dcm2rpy(rot);
                rpy(0) = 0.0;
                rpy(1) = 0.0;
                ori = Eigen::Quaterniond(vk::rpy2dcm(rpy));
            }
        }

        if(!odom_processed_)
        {
            ori_cur = ori.normalized();
            // pos_cur = -(ori_cur*odom_handler_->odom_calib_.T_B_O.Inverse().GetPosition()); // set t_wb to (0,0,0)
            pos_cur = Eigen::Vector3d::Zero();
            linear_velocity_cur  = linear_velocity;
            angular_velocity_cur = angular_velocity;

            odom_last_.angular_velocity = angular_velocity_cur;
            odom_last_.linear_velocity  = linear_velocity_cur;
            odom_last_.orientation = ori_cur;
            odom_last_.position = pos_cur;
            odom_last_.timestamp = ts;
            pos_odom_last_ = position;

            odom_processed_ = true;
        }
        else
        {
            ori_cur = ori.normalized();
            if(stable_mode_first_stage_)
            {
                double dt = ts-odom_last_.timestamp;
                Eigen::Vector3d pos_odom_delta = position-pos_odom_last_;
                Eigen::Vector3d pos_odom_rel = odom_last_.orientation.inverse()*pos_odom_delta;
                pos_odom_rel(2) = 0.0;
                Eigen::Vector3d pos_proj_delta = odom_last_.orientation*(pos_odom_rel*(pos_odom_delta.norm()/pos_odom_rel.norm()));
                pos_cur = odom_last_.position+pos_proj_delta;

                Eigen::Vector3d delta_p = odom_last_.orientation.inverse()*(pos_cur-odom_last_.position);
                linear_velocity_cur = delta_p/dt;
                Eigen::Quaterniond delta_q  = odom_last_.orientation.inverse()*ori_cur;
                Eigen::Vector3d delta_theta = 2.0* delta_q.normalized().vec();
                angular_velocity_cur = delta_theta/dt;
            }
            else
            {
                if(robot_status_ == MotionMode::POWER_ON || robot_status_ == MotionMode::LYING || robot_status_ == MotionMode::STANDING)
                {
                    Eigen::Vector3d pos_odom_delta = position-pos_odom_last_;
                    pos_cur = odom_last_.position+pos_odom_delta;
                    linear_velocity_cur  = linear_velocity;
                    angular_velocity_cur = angular_velocity;
                }
                else
                {
                    double dt = ts-odom_last_.timestamp;
                    Eigen::Vector3d pos_odom_delta = position-pos_odom_last_;
                    Eigen::Vector3d pos_odom_rel = odom_last_.orientation.inverse()*pos_odom_delta;
                    pos_odom_rel(2) = 0.0;
                    Eigen::Vector3d pos_proj_delta = odom_last_.orientation*(pos_odom_rel*(pos_odom_delta.norm()/pos_odom_rel.norm()));
                    pos_cur = odom_last_.position+pos_proj_delta;

                    Eigen::Vector3d delta_p = odom_last_.orientation.inverse()*(pos_cur-odom_last_.position);
                    linear_velocity_cur = delta_p/dt;
                    Eigen::Quaterniond delta_q  = odom_last_.orientation.inverse()*ori_cur;
                    Eigen::Vector3d delta_theta = 2.0* delta_q.normalized().vec();
                    angular_velocity_cur = delta_theta/dt;
                }
            }

            if(pos_cur.hasNaN())
                pos_cur = odom_last_.position;
            if(ori_cur.coeffs().hasNaN())
                ori_cur = odom_last_.orientation;
            if(linear_velocity_cur.hasNaN())
                linear_velocity_cur = odom_last_.linear_velocity;
            if(angular_velocity_cur.hasNaN())
                angular_velocity_cur = odom_last_.angular_velocity;

            odom_last_.angular_velocity = angular_velocity_cur;
            odom_last_.linear_velocity  = linear_velocity_cur;
            odom_last_.orientation = ori_cur;
            odom_last_.position = pos_cur;
            odom_last_.timestamp = ts;

            pos_odom_last_ = position;
        }

        return OdomInfo(ts, ori_cur, pos_cur, linear_velocity_cur, angular_velocity_cur);
    }

    void SvoInterface::inputStatusData(const int motion_id)
    {
        robot_status_ = motion_id;
    }

    int SvoInterface::getRobotStatus()
    {
        return robot_status_;
    }

    bool SvoInterface::isLastKeyFrame()
    {
        return svo_->IsBackendValid();
    }

    double SvoInterface::getLastProcessingTime()
    {
        return svo_->LastProcessingTime();
    }

    int64_t SvoInterface::getLastNumObservations()
    {
        return svo_->LastNumObservations();
    }

    // int64_t SvoInterface::GetLastFramesTimestamp()
    // {
    //   return svo_->GetLastFramesTimestamp();
    //
    // }

    // Transformation SvoInterface::getLastFramesImuPose()
    // {
    //   return svo_->getLastFramesImuPose();
    // }

    // std::vector<int> SvoInterface::getLastFramesId()
    // {
    //   return svo_->getLastFramesId();
    // }

    // std::vector<cv::Mat> SvoInterface::getLastFramesImage()
    // {
    //   return svo_->getLastFramesImage();
    // }

    std::vector<std::vector<double>> SvoInterface::GetLastFramesPose(const std::string sensor_type)
    {
        Transformation T_w_b;
        std::vector<std::vector<double>> last_frame_pose;
        
        //for following
        if(stable_mode_first_stage_)
        {
            if(init_stable_mode_)
                T_w_b = pose_follow_*odom_handler_->odom_calib_.T_B_O.Inverse();
            else
                return last_frame_pose;
        }
        else
        {
            if(loose_couple_)
            {
                if(!loose_couple_->first_init_)
                {
                    T_w_b = loose_couple_pose_;
                    std::cout<<"loose matrix:" << loose_couple_pose_.GetRotationMatrix()<< std::endl;
                    std::cout << "abnormal count: " << svo_->getAbnormalCount() << "; ";
                    if(loose_couple_->use_vio_pose_)
                        std::cout << "use vio pose\n";
                    else
                        std::cout << "use odom pose\n";
                }
                else
                {
                   printf("Warning: Waiting for initialize!\n");
                   return last_frame_pose;
                }
            }
            else if(svo_ && svo_->last_frames_)
                T_w_b = svo_->GetLastFramesImuPose();
            else
            {
                printf("ERROR: Both loose_couple_ and sov are nullptr!\n");
                return last_frame_pose;
            }

            if(update_pose_)
            {  
                if(pose_updater_->getStage()==PoseUpdateStage::kStart && prv_img_ts_!=0.0)
                {
                    Transformation T_w_o;
                    if(odom_handler_->GetCorresPoseProcessed(prv_img_ts_, T_w_o))
                        T_w_b = T_w_o*odom_handler_->odom_calib_.T_B_O.Inverse();
                    else
                        return last_frame_pose;
                }
                else if(prv_img_ts_!=0.0)
                {
                    Eigen::Matrix3d R = pose_updater_->getRot();
                    Eigen::Vector3d t = pose_updater_->getTrl();
                    Eigen::Vector3d zd = pose_updater_->getCompZ();

                    Eigen::Vector3d pos = R*T_w_b.GetPosition()+t+zd;
                    Eigen::Matrix3d rot = R*T_w_b.GetRotationMatrix();

                    T_w_b = Transformation(Quaternion(rot), pos);
                }
                else
                    return last_frame_pose;
            }
        }
            
        if(sensor_type == "imu")
        {
            Eigen::Quaterniond q = T_w_b.GetRotation().ToImplementation();
            Eigen::Vector3d p = T_w_b.GetPosition();
            std::vector<double> pose = {p[0], p[1], p[2], q.x(), q.y(), q.z(), q.w()};
            last_frame_pose.push_back(pose);            
        }
        else if(sensor_type == "cam")
        {            
            CameraBundle::Ptr camera_bundle = svo_->GetNCamera();
            for(int i = 0; i < (int)camera_bundle->getNumCameras(); ++i)
            {
                Transformation T_c_b = camera_bundle->get_T_C_B(i);
                Transformation T_w_c = T_w_b * T_c_b.Inverse();
                
                Eigen::Quaterniond q = T_w_c.GetRotation().ToImplementation();
                Eigen::Vector3d p = T_w_c.GetPosition();
                std::vector<double> pose = {p[0], p[1], p[2], q.x(), q.y(), q.z(), q.w()};
                last_frame_pose.push_back(pose);                
            }
        }
        else if(sensor_type == "odom")
        {
            if(odom_handler_)
            {
                Transformation T_b_o = odom_handler_->odom_calib_.T_B_O;
                Transformation T_w_o = T_w_b * T_b_o;
                
                Eigen::Quaterniond q = T_w_o.GetRotation().ToImplementation();
                Eigen::Vector3d p = T_w_o.GetPosition();
                std::vector<double> pose = {p[0], p[1], p[2], q.x(), q.y(), q.z(), q.w()};
                last_frame_pose.push_back(pose);                                
            }
            else
                printf("odom is not used!\n");
        }
        else
            printf("%s is not support!\n", sensor_type.c_str());
            
        return last_frame_pose;
    }
    
    bool SvoInterface::getAlignedOdom(const double timestamp, Eigen::Matrix4d &odom_pose)
    {
        if(!data_align_)
            return false;
            
        return data_align_->getAlignedOdom(timestamp, odom_pose);
    }
    
    bool SvoInterface::getLatestAlignedOdom(double &timestamp, Eigen::Matrix4d &odom_pose)
    {
        if(!data_align_)
            return false;
            
        return data_align_->getLatestAlignedOdom(timestamp, odom_pose);
    }

    std::vector<std::vector<double>> SvoInterface::GetLastFramesCamPose()
    {
        std::vector<std::vector<double>> lastFrameCamPose;
        std::vector<Transformation> transformations = svo_->GetLastFramesCamPose();
        for (size_t i = 0; i < transformations.size(); ++i)
        {

            Eigen::Quaterniond q = transformations[i].GetRotation().ToImplementation();
            Eigen::Vector3d p = transformations[i].GetPosition();
            std::vector<double> camPose = {p[0], p[1], p[2], q.x(), q.y(), q.z(), q.w()};
            lastFrameCamPose.push_back(camPose);
        }
        return lastFrameCamPose;
    }

    std::vector<double> SvoInterface::GetLastFramesIMUPose() {
        Transformation imu_transformation = svo_->GetLastFramesIMUPose();
        Eigen::Quaterniond q = imu_transformation.GetRotation().ToImplementation();
        Eigen::Vector3d p = imu_transformation.GetPosition();
        std::vector<double> imuPose = {p[0], p[1], p[2], q.x(), q.y(), q.z(), q.w()};
        return imuPose;
    }

    void SvoInterface::loadTimes(std::string file, int id_camera)
    {
        std::ifstream tr;
        tr.open(file.c_str());
        while (!tr.eof() && tr.good())
        {
            std::string line;
            char buf[1000];
            tr.getline(buf, 1000);

            long long id;
            double stamp;
            float exposure = 0;
            if (3 == sscanf(buf, "%lld %lf %f", &id, &stamp, &exposure))
            {
                if (id_camera == 0)
                {
                    ltimes.insert(pair<int64_t, float>(static_cast<int64_t>(id), exposure));
                }
                else if (id_camera == 1)
                {
                    rtimes.insert(pair<int64_t, float>(static_cast<int64_t>(id), exposure));
                }
            }
        }
    }
    
    void SvoInterface::looseCoupleProcess()
    {
        if(!data_align_ || !loose_couple_)
            return;
            
        bool is_backend_ok = svo_->isBackendOK();
            
        loose_couple_->setAlignHandler(data_align_);

        bool vio_state = false;
        if(svo_->stage() == Stage::kTracking)
            vio_state = true;
        
        bool is_keyframe = svo_->IsLastKeyFrame();
        double timestamp = svo_->GetLastFramesTimestampSec();
        
        Transformation pose = svo_->GetLastFramesImuPose();
        
        Eigen::Matrix4d T_w_b = pose.GetTransformationMatrix();
        loose_couple_->addFrameBundle(timestamp, T_w_b, is_keyframe, vio_state, is_backend_ok);
        Eigen::Matrix4d fused_pose = loose_couple_->getFusedPose();

        Eigen::Vector3d fused_t(fused_pose.block<3, 1>(0, 3));
        Eigen::Quaterniond fused_q(fused_pose.block<3, 3>(0, 0));
        loose_couple_pose_ = Transformation(fused_q.normalized(), fused_t);

        is_abnormal_ = loose_couple_->isAbnormal();
        svo_->SetAbnormalResult(is_abnormal_);
        if(is_abnormal_)
        {
          loose_couple_->reset();
          svo_->ResetAllEnd();
        }
    }
    
    void SvoInterface::abnormalProcess(const int64_t timestamp_nanoseconds)
    {
      if(!data_align_ || !loose_couple_ || svo_->stage() == Stage::kTracking)
          return;

      loose_couple_->setAlignHandler(data_align_);

      double timestamp = timestamp_nanoseconds * 1e-9;
      Eigen::Matrix4d fused_pose = loose_couple_->getFusedPose(timestamp);
      Eigen::Vector3d fused_t(fused_pose.block<3, 1>(0, 3));
      Eigen::Quaterniond fused_q(fused_pose.block<3, 3>(0, 0));
      Transformation abnormal_pose(fused_q.normalized(), fused_t);
      svo_->SetAbnormalPose(abnormal_pose);
    }

    void SvoInterface::updatePose(const int64_t timestamp_nanoseconds)
    {
        if(!update_pose_)
            return;

        double timestamp = timestamp_nanoseconds * 1e-9;
        Transformation T_wb_i, T_wb_o;
        bool is_kf = false;

        if(loose_couple_)
        {
            if(!loose_couple_->first_init_)
                T_wb_i = loose_couple_pose_;
            else
                return;
        }
        else if(svo_)
        {
            if(svo_->last_frames_)
                T_wb_i = svo_->last_frames_->Get_T_W_B();
            else
                return;
        }
        else
            return;

        Transformation T_wo;
        if(odom_handler_->GetCorresPoseProcessed(timestamp, T_wo))
            T_wb_o = T_wo*odom_handler_->odom_calib_.T_B_O.Inverse();
        else
            return;

        if(svo_)
            if(svo_->last_frames_ && svo_->last_frames_->IsKeyframe())
                is_kf = true;

        pose_updater_->updatePose(T_wb_i, T_wb_o, is_kf);
    }

    // std::vector<Eigen::MatrixXd> SvoInterface::getLastFramesPts()
    // {
    //   return svo_->getLastFramesPts();
    // }

    // std::vector<Eigen::Vector4d> SvoInterface::getMapRegionPoints()
    // {
    //   return svo_->getMapRegionPoints();
    // }

    // std::unordered_map<int, Transformation> SvoInterface::getCloseKfPoses()
    // {
    //   return svo_->getCloseKfPoses();
    // }

    // std::unordered_map<int, Eigen::Vector4d> SvoInterface::getCloseKfPts()
    // {
    //   return svo_->getCloseKfPts();
    // }

    // std::vector<Eigen::Vector3d> SvoInterface::getMapSeeds()
    // {
    //   return svo_->getMapSeeds();
    // }

    // std::vector<Eigen::Vector3d> SvoInterface::getMapSeedsUncertainty()
    // {
    //   return svo_->getMapSeedsUncertainty();
    // }

    // std::vector<Transformation> SvoInterface::getActiveKeyframePoses()
    // {
    //   return svo_->getActiveKeyframePoses();
    // }

    // std::vector<cv::Mat> SvoInterface::getLastFramePyr(const int cam_idx)
    // {
    //   return svo_->getLastFramePyr(cam_idx);
    // }

    // Eigen::MatrixXd SvoInterface::getLastFrameFeaturePx(const int cam_idx)
    // {
    //   return svo_->getLastFrameFeaturePx(cam_idx);
    // }

    // Eigen::MatrixXd SvoInterface::getLastFrameFeatureGrad(const int cam_idx)
    // {
    //   return svo_->getLastFrameFeatureGrad(cam_idx);
    // }

    // Eigen::MatrixXi SvoInterface::getLastFrameFeatureStatus(const int cam_idx)
    // {
    //   return svo_->getLastFrameFeatureStatus(cam_idx);
    // }

    // std::vector<cv::Mat> SvoInterface::getNewFramePyr(const int cam_idx)
    // {
    //   return svo_->getNewFramePyr(cam_idx);
    // }

} // namespace mivins

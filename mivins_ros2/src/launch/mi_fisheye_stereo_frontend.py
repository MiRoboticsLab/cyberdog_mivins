import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    calib_path = os.path.join(get_package_share_directory('vins'), 'param/calib/mi_fisheye_stereo.yaml') 
    vins_path = os.path.join(get_package_share_directory('vins'), 'param/frontend_imu/mi_fisheye_stereo_imu_config.yaml')    
    print(calib_path) 
    namespace = LaunchConfiguration('namespace', default='')
    return LaunchDescription([
        # launch a node        
        Node(
            package='vins',
            executable='mivins_node',
            namespace=namespace,
            name='vins',
            output='screen',
            parameters=[
                {'cam0_topic': '/image_left'},
                {'cam1_topic': '/image_right'},
                {'imu_topic': '/mi0017090/camera/imu'},
                {'calib_file': calib_path},
                {'vins_config': vins_path},
                {'vins_folder':vins_path},
                {'pipeline_is_stereo':bool(True)},
                {'pipeline_type':int(1)}, 
                {'save_time_consumption':bool(True)},
                #{'vignette_enable':bool(True)},
                #{'gamma_enable':bool(True)},
                #{'exposure_time_enable':bool(False)},
                #{'photometric_calib_path':'/home/gc/me/dataset/dog/fisheye/dso/'},
                {'max_fts':180},
                {'max_n_kfs':20},
                {'map_scale':float(1.0)},
                {'T_world_imuinit/qx':float(0.0)},
                {'T_world_imuinit/qy':float(0.0)},
                {'T_world_imuinit/qz':float(0.0)},
                {'T_world_imuinit/qw':float(1.0)},
                {'kfselect_criterion':'FORWARD'},
                {'kfselect_numkfs_upper_thresh':120},
                {'kfselect_numkfs_lower_thresh':float(40.0)},
                {'kfselect_min_dist_metric':float(0.2)},
                {'kfselect_min_angle':float(10.0)},
                {'kfselect_min_disparity':float(25.0)},
                {'update_seeds_with_old_keyframes':bool(True)},
                {'init_min_disparity':float(25.0)},
                {'grid_size':25},
                {'reprojector_max_n_kfs':5},
                {'scan_epi_unit_sphere':bool(True)},
                {'poseoptim_using_unit_sphere':bool(True)},
                {'use_imu':bool(False)},
                {'publish_every_nth_dense_input':5},
                {'publish_marker_scale':float(0.8)},
                {'img_align_use_distortion_jacobian':bool(True)},
                {'use_ceres_backend':bool(False)},
                {'use_vins_backend':bool(True)},
                {'use_vins_init':bool(True)},
                {'vins_backend_multi_thread':bool(True)},
                #{'use_pixel_value_align_to_first':bool(False)},
                #{'use_photometric_calibration':bool(True)},
                {'kfselect_backend_max_time_sec':float(1.0)},
                {'use_async_reprojectors':bool(True)},
                {'poseoptim_prior_lambda':float(0.1)},
                {'img_align_prior_lambda_rot':float(0.0)},
                {'img_align_prior_lambda_trans':float(0.0)},
                {'use_threaded_depthfilter':bool(False)},
                {'quality_min_fts':40},
                {'quality_max_drop_fts':80},
                {'max_depth_inv':float(0.05)},
                {'min_depth_inv':float(1.0)},
                {'mean_depth_inv':float(0.3)},
                {'kfselect_min_num_frames_between_kfs':0},
                #{'img_align_est_ab':bool(True)},
                {'img_align_est_illumination_gain':bool(True)},
                {'img_align_est_illumination_offset':bool(True)},
                {'depth_filter_affine_est_offset':bool(True)},
                {'depth_filter_affine_est_gain':bool(True)},
                {'reprojector_affine_est_offset':bool(True)},
                {'reprojector_affine_est_gain':bool(True)}            ]
        )
    ])

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from convert_calibration import ConvertCalibration

def generate_launch_description():
    calib_path = os.path.join(get_package_share_directory('vins'), 'param/d430/realsense_stereo_d430_new.yaml')
    config_file = os.path.join(get_package_share_directory('vins'), 'param/d430/realsense_vio_stereo.yaml')

    calib_dir = "/params/camera/calibration"
    calib_converted = os.path.join(get_package_share_directory('vins'), 'param/d430/realsense_stereo_d430_mivins.yaml')
    if(ConvertCalibration(calib_dir, calib_converted)):
        calib_path = calib_converted

    print(calib_path) 
    namespace = LaunchConfiguration('namespace', default='')
    return LaunchDescription([
        # launch a node        
        Node(
            package='vins',
            executable='mivins_node',
            namespace=namespace,
            name='mivinsmapping',
            output='screen',
            parameters=[
                {'cam0_topic': 'camera/infra1/image_rect_raw'},
                {'cam1_topic': 'camera/infra2/image_rect_raw'},
                {'imu_topic' : 'camera/imu'},
                {'odom_topic': 'odom_out'},
                {'status_topic': 'motion_status'},
                {'calib_file': calib_path},
                {'config_file': config_file},
                {'mode_type': 1},
                {'pose_path': '/SDCARD/miloc/maps/default/visual/trajectory.txt'}]
            ),

        #Node(
        #    package='rviz2',
        #    namespace='',
        #    executable='rviz2',
        #    name='rviz2',
        #    arguments=['-d' + os.path.join(get_package_share_directory('vins'), 'rviz_config/mivins_rviz2_config.rviz2.rviz')]
        #)
    ])
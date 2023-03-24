import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    calib_path = os.path.join(get_package_share_directory('vins'), 'param/d430/realsense_stereo_d430.yaml')
    config_file = os.path.join(get_package_share_directory('vins'), 'param/d430/realsense_vio_stereo.yaml')
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
                {'cam0_topic': '/camera/infra1/image_rect_raw'},
                {'cam1_topic': '/camera/infra2/image_rect_raw'},
                {'imu_topic': '/camera/imu'},
                {'odom_topic': '/odom_out'},
                {'calib_file': calib_path},
                {'config_file': config_file},
                            ]
            ),

        #Node(
        #    package='rviz2',
        #    namespace='',
        #    executable='rviz2',
        #    name='rviz2',
        #    arguments=['-d' + os.path.join(get_package_share_directory('vins'), 'rviz_config/mivins_rviz2_config.rviz2.rviz')]
        #)
    ])
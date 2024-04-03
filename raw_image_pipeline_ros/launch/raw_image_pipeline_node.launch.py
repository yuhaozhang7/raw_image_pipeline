import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    ld = launch.LaunchDescription([
        DeclareLaunchArgument(
            name='camera_name',
            default_value='cam0'
        ),
        DeclareLaunchArgument(
            name='input_topic',
            default_value='alphasense_driver_ros/cam0'
        ),
        DeclareLaunchArgument(
            name='input_type',
            default_value='color'
        ),
        DeclareLaunchArgument(
            name='output_prefix',
            default_value='alphasense_driver_ros/cam0'
        ),
        DeclareLaunchArgument(
            name='output_frame',
            default_value='cam0_sensor_frame_helper'
        ),
        DeclareLaunchArgument(
            name='launch_prefix',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='output_encoding',
            default_value='BGR'
        ),
        DeclareLaunchArgument(
            name='skip_number_of_images_for_slow_topic',
            default_value='5'
        ),
        DeclareLaunchArgument(
            name='use_gpu',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='disable_compression_plugins',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='debayer/enabled',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='flip/enabled',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='white_balance/enabled',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='color_calibration/enabled',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='gamma_correction/enabled',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='vignetting_correction/enabled',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='color_enhancer/enabled',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='undistortion/enabled',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='debayer_encoding',
            default_value='auto'
        ),
        DeclareLaunchArgument(
            name='flip/angle',
            default_value='0'
        ),
        DeclareLaunchArgument(
            name='white_balance/method',
            default_value='ccc'
        ),
        DeclareLaunchArgument(
            name='white_balance/clipping_percentile',
            default_value='10.'
        ),
        DeclareLaunchArgument(
            name='white_balance/saturation_bright_thr',
            default_value='0.8'
        ),
        DeclareLaunchArgument(
            name='white_balance/saturation_dark_thr',
            default_value='0.2'
        ),
        DeclareLaunchArgument(
            name='white_balance/temporal_consistency',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='color_calibration/calibration_file',
            default_value=get_package_share_directory(
                'raw_image_pipeline') + '/config/alphasense_color_calib_example.yaml'
        ),
        DeclareLaunchArgument(
            name='gamma_correction/method',
            default_value='custom'
        ),
        DeclareLaunchArgument(
            name='gamma_correction/k',
            default_value='0.9'
        ),
        DeclareLaunchArgument(
            name='vignetting_correction/scale',
            default_value='1.5'
        ),
        DeclareLaunchArgument(
            name='vignetting_correction/a2',
            default_value='0.001'
        ),
        DeclareLaunchArgument(
            name='vignetting_correction/a4',
            default_value='0.000001'
        ),
        DeclareLaunchArgument(
            name='color_enhancer/saturation_gain',
            default_value='1.2'
        ),
        DeclareLaunchArgument(
            name='undistortion/balance',
            default_value='0.5'
        ),
        DeclareLaunchArgument(
            name='undistortion/fov_scale',
            default_value='1.2'
        ),
        DeclareLaunchArgument(
            name='undistortion/calibration_file',
            default_value=get_package_share_directory(
                'raw_image_pipeline') + '/config/alphasense_calib_example.yaml'
        ),
        launch_ros.actions.Node(
            package='raw_image_pipeline_ros',
            executable='raw_image_pipeline_ros_node',
            name=LaunchConfiguration('camera_name'),
            output='screen',
            parameters=[
                {
                    'input_topic': LaunchConfiguration('input_topic')
                },
                {
                    'input_type': LaunchConfiguration('input_type')
                },
                {
                    'output_prefix': LaunchConfiguration('output_prefix')
                },
                {
                    'output_encoding': LaunchConfiguration('output_encoding')
                },
                {
                    'output_frame': LaunchConfiguration('output_frame')
                },
                {
                    'skip_number_of_images_for_slow_topic': LaunchConfiguration('skip_number_of_images_for_slow_topic')
                },
                {
                    'use_gpu': LaunchConfiguration('use_gpu')
                },
                {
                    'debug': LaunchConfiguration('debug')
                },
                {
                    'debayer/enabled': LaunchConfiguration('debayer/enabled')
                },
                {
                    'debayer/encoding': LaunchConfiguration('debayer_encoding')
                },
                {
                    'flip/enabled': LaunchConfiguration('flip/enabled')
                },
                {
                    'flip/angle': LaunchConfiguration('flip/angle')
                },
                {
                    'white_balance/enabled': LaunchConfiguration('white_balance/enabled')
                },
                {
                    'white_balance/method': LaunchConfiguration('white_balance/method')
                },
                {
                    'white_balance/clipping_percentile': LaunchConfiguration('white_balance/clipping_percentile')
                },
                {
                    'white_balance/saturation_bright_thr': LaunchConfiguration('white_balance/saturation_bright_thr')
                },
                {
                    'white_balance/saturation_dark_thr': LaunchConfiguration('white_balance/saturation_dark_thr')
                },
                {
                    'white_balance/temporal_consistency': LaunchConfiguration('white_balance/temporal_consistency')
                },
                {
                    'color_calibration/enabled': LaunchConfiguration('color_calibration/enabled')
                },
                {
                    'color_calibration/calibration_file': LaunchConfiguration('color_calibration/calibration_file')
                },
                {
                    'gamma_correction/enabled': LaunchConfiguration('gamma_correction/enabled')
                },
                {
                    'gamma_correction/method': LaunchConfiguration('gamma_correction/method')
                },
                {
                    'gamma_correction/k': LaunchConfiguration('gamma_correction/k')
                },
                {
                    'vignetting_correction/enabled': LaunchConfiguration('vignetting_correction/enabled')
                },
                {
                    'vignetting_correction/scale': LaunchConfiguration('vignetting_correction/scale')
                },
                {
                    'vignetting_correction/a2': LaunchConfiguration('vignetting_correction/a2')
                },
                {
                    'vignetting_correction/a4': LaunchConfiguration('vignetting_correction/a4')
                },
                {
                    'color_enhancer/enabled': LaunchConfiguration('color_enhancer/enabled')
                },
                {
                    'color_enhancer/saturation_gain': LaunchConfiguration('color_enhancer/saturation_gain')
                },
                {
                    'undistortion/enabled': LaunchConfiguration('undistortion/enabled')
                },
                {
                    'undistortion/balance': LaunchConfiguration('undistortion/balance')
                },
                {
                    'undistortion/fov_scale': LaunchConfiguration('undistortion/fov_scale')
                },
                {
                    'undistortion/calibration_file': LaunchConfiguration('undistortion/calibration_file')
                },
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

#!/bin/bash
rosparam set /visensor_node/cam0_aec_enable false
rosparam set /visensor_node/cam0_agc_enable false
rosparam set /visensor_node/cam0_coarse_shutter_width 5
rosparam set /visensor_node/cam0_global_analog_gain 64
rosparam set /visensor_node/cam1_aec_enable false
rosparam set /visensor_node/cam1_agc_enable false
rosparam set /visensor_node/cam1_coarse_shutter_width 5
rosparam set /visensor_node/cam1_global_analog_gain 64
rosparam set /visensor_node/cam2_aec_enable false
rosparam set /visensor_node/cam2_agc_enable false
rosparam set /visensor_node/cam2_coarse_shutter_width 5
rosparam set /visensor_node/cam2_global_analog_gain 64
rosparam set /visensor_node/cam3_aec_enable false
rosparam set /visensor_node/cam3_agc_enable false
rosparam set /visensor_node/cam3_coarse_shutter_width 5
rosparam set /visensor_node/cam3_global_analog_gain 64
rosrun visensor_node visensor_node _camRate:=2&
rosrun image_view image_view image:=/cam0/image_raw&
rosrun image_view image_view image:=/cam1/image_raw

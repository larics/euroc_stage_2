# VI-Sensor & Firefly to VI-Sensor Calibration Procedure#

Description: In this packet, you find scripts to calibrate

* the VI-Sensor (intrinsics & extrinsics).
* the Asctec IMU extrinsics wrt. VI-Sensor IMU.
* The extrinsics that are used in MSF that take into account pose measurements from the VI-Sensor to stabilize an Asctec UAV.

1. Calibrate VI-Sensor camera intrinsics and extrinsics. To this end, in calibration_template/camera_calibration/runx, you find scripts to start the sensor driver, set the correct sensor settings (for the non-devel version of the driver) and record the image data into a bag. To this end, during recording, the VI-sensor should be moved 1-2 m away from the calibration target in order to capture the full imager are with the target. Make sure to make slow movements as motion blur tends to worsen the calibration results. Make sure that the target is well lit either with direct sunlight (or better) with an artificial light source. Also make sure that the settings of the VI-Sensor are correct: The exposure should be very short (3-5) and the gain should be at 64 for each camera. This should result in an image that is well lit in the are of the target (but not overexposed) and otherwise very dark.

2. Run the calibration for each recorded run and select the one with the lowest std of the reprojection error (found in the results.txt file).

3. Copy the camchain-runX.yaml file into the top folder (calibration_template) and rename it to camchain.yaml. We will use this file for the IMU-camera calibration.

4. Record calibration sequence for camera - imu extrinsics. Not only record /imu0 messages but also /fcu/imu coming from the Asctec UAV. Do not forget to adjust frequency of /fcu/imu in fcu_parameters.yaml in mav_startup to 200 Hz. For the calibration, you can reduce the ekf_state_out frequency to 10 Hz to not put too much pressure on the serial bus. DO NOT FORGET TO reset the values after recording. Adjust the record scripts to account for the correct namespace of the IMU topic of the Firefly. During recording, wave the VI-Sensor in front of the target. Make sure to include enough translational and rotational motion. Try to make smooth motions as very rugged motion might saturate the gyroscope.

5. Calibrate camera to VI-Sensor IMU using the calibrate_imu.sh script for each recorded run.

6. For the best sequence, rename the camchain-xxx.yaml file to something you remember, since this is your new calibration and gets overwritten by the next step. To use this calibration with aslam, use the provided scripts in kalibr to convert it to the aslam format with the following command: `kalibr_aslam_config --cam camchain-superawesomenewcalibration.yaml`

7. Calibrate the Asctec IMU as well using the calibrate_imu_asctec.sh. To this end, adjust the topic name of the Asctec IMU topic in the file parameter_files/asctec.yaml to account for the correct namespace.

8. Adjust config.yaml of this firefly_visensor_calibration packet using the newly found camera-imu transforms of the VI-Sensor and VI-Sensor-Asctec IMU.

9. Start the launch file of this packet and paste the output into the msf_parameters.yaml file of the mav_startup packet.

Author: Sammy Omari (omaris@ethz.ch, sammy.omari@skybotix.com)

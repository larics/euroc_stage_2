#!/bin/bash
kalibr_calibrate_imu_camera --cam ../../camchain.yaml --target ../../parameter_files/april_6x6.yaml --imu ../../parameter_files/asctec.yaml --bag run3.bag --time-calibration

#!/bin/bash
rosbag record /cam0/image_raw /cam1/image_raw /imu0 /mpu0 /mpu1 /fcu/imu -O run2

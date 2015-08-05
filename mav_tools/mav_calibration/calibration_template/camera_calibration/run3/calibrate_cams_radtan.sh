#!/bin/bash
kalibr_calibrate_cameras --target ../../parameter_files/april_6x6.yaml --bag run3.bag --models pinhole-radtan pinhole-radtan --topics /cam0/image_raw /cam1/image_raw 




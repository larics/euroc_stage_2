# README #

Read this before trying to run or contribute.

### What is this repository for? ###

Refactoring of aslam_visual_inertial (Leutenegger et al. IJRR 2014)

### How do I get set up? ###

You will the catkin build system and the following catkin packages:

* eigen_catkin
* opencv2_catkin
* glog_catkin
* ceres_catkin
* ethzasl_brisk
* opengv
* libvisensor_devel

then clone into your catkin workspace

### Building the project in debug mode

To change the cmake build type for the whole project use:

    catkin build --force-cmake -cmake-args -DCMAKE_BUILD_TYPE=Release 
    
### Running

Make sure to use the right config file in okvis_node/config/ that is called by the launch file in okvis_node/launch/. Specify there whether or not to use the driver for directly interfacing with the VI Sensor.
To launch, run:

    roslaunch okvis_node okvis_node.launch

### Contribution guidelines ###

* Programming guidelines: please follow https://github.com/ethz-asl/programming_guidelines/wiki/Cpp-Coding-Style-Guidelines .
* Writing tests: please write unit tests (gtest).
* Code review: for major contribution, please create a pull request. The pull request will be reviewed by an admin before merging.

### Who do I talk to? ###

* Repo owner or admin

### Calibration ###

* Get Kalibr by following the instructions here https://github.com/ethz-asl/kalibr/wiki/installation . If you decide to build from source and you run ROS indigo checkout pull request 3:

    git fetch origin pull/3/head:request3
    git checkout request3

* Follow https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration to calibrate intrinsic and extrinsic parameters of the cameras. If you receive an error message that the tool was unable to make an initial guess on focal length, make sure that your recorded dataset contains frames that have the whole calibration target in view.
* Follow https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration to get estimates for the spatial parameters of the cameras with respect to the IMU.

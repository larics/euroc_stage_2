QUICKSTART                         {#mainpage}
==========

Read this before trying to use OKVIS.

## Installation ##

### ROS ###

OKVIS currently still uses the catkin build system and depends on ROS (we tested with Indigo LTS). Follow the instructions on http://www.ros.org/install/ to install ROS. 
Future versions will be only optionally dependent on ROS.
Make sure you have some tools around (if you install ros-desktop-full, you should already have it)

    sudo apt-get install python-rosinstall python-catkin-tools python-setuptools

### Create a New Workspace (Optional) ###

We recommend using a separate workspace, but this is completely optional. 

Proceed as follows (adopted from https://github.com/ethz-asl/aslam_install/wiki/Installing-the-aslam-libraries).
So, first make a directory where you would like to store your code:

    mkdir ~/okvis_ws
    cd ~/okvis_ws

Now we initialize the ros workspace and source directory

    mkdir src
    cd src
    source /opt/ros/indigo/setup.bash
    wstool init 
    catkin_init_workspace

### Download the Sources ###

Download the script https://www.doc.ic.ac.uk/~sleutene/software/okvis/download/download_sources.sh into your workspace's src/ folder.
Get everything you need by typing 

    source download_sources.sh

Use the same login.

### Building ###

Now you should be able to build your workspace (cd into the workspace root, not src/) with

    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

or, if you only need this software:

    catkin build okvis_node --cmake-args -DCMAKE_BUILD_TYPE=Release

### Setup the workspace ###

Assuming you had set up a workspace at ~/okvis_ws

    cd ~/okvis_ws
    source devel/setup.bash

This sets your environment variables to use this ROS workspace. You may want to add

    source ~/okvis_ws/devel/setup.bash

to your .bashrc file so that this is set up every time you log in.

## Config / Calibration ##

Edit a config file into okvis_node/config and create a launchfile in launch/.
Make sure all the parameters are sensible! The trajectory/landmark output will only ever be as good as your parameters. 

For calibration use kalibr, see https://github.com/ethz-asl/kalibr.
You will want to adopt IMU parameters similar to the ones given in other p2 config files.

## Running ##

Make sure a roscore is running. For visualisation, you may use rviz. A respective configuration is available: okvis_node/rviz.rviz. 
Use rostopic list to get an idea of the topics available.

### Offline (Bag) Processing ##

Run

    rosrun okvis okvis_node_synchronous <config_file.config> <bag_file.bag>

### Live Processing ##

You will have two options: either using the visensor driver directly (less overhead), set the useDriver option to True for this.

Otherwise, the node will listen to the ros topics specified in the launch file. This means, you can use this either with the visensor_node that publishes output or by playing back a bag. Just note the overhead.

    roslaunch okvis_node <launch_file.launch>

## Updates of the Pre-Release ##

Just follow the installation and build process from "Download the Sources", in case we provide updates.

## Problems ##

Bug reports, experiences,... please send by e-mail to s.leutenegger@imperial.ac.uk.

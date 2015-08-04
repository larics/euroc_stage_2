#!/bin/bash

#sudo apt-get install meld gitk git-cola indicator-multiload -y

# install some codecs
#sudo apt-get install libavformat-extra-53 libavcodec-extra-53 ubuntu-restricted-extras -y

# --- SET UP CCACHE ---
sudo apt-get install -y ccache

echo 'export PATH="/usr/lib/ccache:$PATH"' | tee -a ~/.bashrc \
&& source ~/.bashrc && echo $PATH


# --- SET UP ROS ---

# add ros repository to package manager
UBUNTU_VERSION=$(lsb_release -a)

if [[ $UBUNTU_VERSION == *14.04* ]]
then
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
elif [[ $UBUNTU_VERSION == *13.10* ]]
then
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu saucy main" > /etc/apt/sources.list.d/ros-latest.list'
else
	echo "Unknown Ubuntu Version for ROS Indigo"
fi



# add key to ros repository
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

# update package manager
sudo apt-get update

# install ros indigo
sudo apt-get install ros-indigo-ros-base -y

# initialize rosdep
sudo rosdep init
rosdep update

# ros python install helper
sudo apt-get install python-rosinstall -y

# install node manager
sudo apt-get install ros-indigo-node-manager-fkie -y

echo
echo "*** SETTING UP ROS ***"
echo

cd ~

source /opt/ros/indigo/setup.bash


#install stuff needed for catkin to work
sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev build-essential -y

# install dependencies for loibvisensor
sudo apt-get install libssh2-1-dev -y

# install stuff needed for aslam
sudo apt-get install python-setuptools python-rosinstall ipython libeigen3-dev libboost-all-dev doxygen libopencv-dev ros-indigo-vision-opencv ros-indigo-image-transport-plugins ros-indigo-cmake-modules python-software-properties software-properties-common libpoco-dev python-matplotlib python-git python-pip ipython libtbb-dev libblas-dev liblapack-dev python-catkin-tools autoconf libtool ros-indigo-camera-info-manager -y

# install stuff needed for mapping
sudo apt-get install ros-indigo-octomap ros-indigo-octomap-ros  ros-indigo-octomap-msgs ros-indigo-diagnostic-updater ros-indigo-pcl-conversions ros-indigo-pcl-ros ros-indigo-interactive-markers ros-indigo-laser-geometry libyaml-cpp-dev ros-indigo-resource-retriever ros-indigo-image-geometry ros-indigo-tf-conversions ros-indigo-tf-tools -y
#ros-indigo-octomap-rviz-plugins ros-indigo-octovis

# install stuff needed for vehicle monitor
sudo apt-get install ros-indigo-dynamic-edt-3d -y

# install stuff needed for flight_manager
sudo apt-get install ros-indigo-geometry -y

# install stuff needed for map-api (this is not the full list, but enough to get it compiling)
# sudo apt-get install ros-indigo-desktop-full doxygen liblapack-dev libblas-dev \
# autotools-dev dh-autoreconf libboost-all-dev python-setuptools git g++ \
# nautilus-open-terminal synaptic vim default-jre libreadline-dev \
# libgtest-dev libglew-dev clang-format-3.4 python-git pylint checkstyle \
# python-termcolor ros-indigo-camera-info-manager*
sudo apt-get install liblapack-dev libblas-dev libreadline-dev libglew-dev liblog4cplus-dev cimg-dev -y

# stuff needed for monodense-mapping
sudo apt-get install ros-indigo-rqt-gui-cpp freeglut3-dev -y

# create catkin workspace
source /opt/ros/indigo/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
rosws init

wstool set aslam_install --git git@github.com:ethz-asl/aslam_install.git -y
wstool set catkin_simple --git git@github.com:catkin/catkin_simple.git -y
wstool update


cd ~/catkin_ws/
catkin build

cd ~

# add ros to bash
sh -c 'echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc'

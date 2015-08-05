/*
* Copyright (c) 2014, Sammy Omari
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int32_t main (int32_t argc,char *argv[]) {

  ros::init(argc, argv, "firefly_visensor_calibration");
  ros::NodeHandle nh("~");

  XmlRpc::XmlRpcValue params;
  nh.getParam("visensor", params);
  if(!params.hasMember("T_cam_imu")){
    ROS_ERROR("visensor does not contain T_cam_imu");
    return 1;
  }
  XmlRpc::XmlRpcValue T_C_I;

  T_C_I = params["T_cam_imu"];
  std::cout << T_C_I[0][0] << std::endl;
  tf::Matrix3x3 R_CAM0_ADIS(T_C_I[0][0], T_C_I[0][1], T_C_I[0][2], T_C_I[1][0], T_C_I[1][1], T_C_I[1][2], T_C_I[2][0], T_C_I[2][1], T_C_I[2][2] );
  tf::Vector3 t_CAM0_ADIS(T_C_I[0][3], T_C_I[1][3], T_C_I[2][3]);
  tf::Transform tf_CAM0_ADIS(R_CAM0_ADIS, t_CAM0_ADIS);

  nh.getParam("firefly", params);
  if(!params.hasMember("T_cam_imu")){
    ROS_ERROR("firefly does not contain T_cam_imu");
    return 1;
  }
  T_C_I = params["T_cam_imu"];
  tf::Matrix3x3 R_CAM0_ASC(T_C_I[0][0], T_C_I[0][1], T_C_I[0][2], T_C_I[1][0], T_C_I[1][1], T_C_I[1][2], T_C_I[2][0], T_C_I[2][1], T_C_I[2][2] );
  tf::Vector3 t_CAM0_ASC(T_C_I[0][3], T_C_I[1][3], T_C_I[2][3]);
  tf::Transform tf_CAM0_ASC(R_CAM0_ASC, t_CAM0_ASC);


  tf::Transform tf_ADIS_ASC = tf_CAM0_ADIS.inverse() * tf_CAM0_ASC;
  tf::Quaternion q_ASC_ADIS = tf_ADIS_ASC.inverse().getRotation();
  tf::Vector3 t_ASC_ADIS = tf_ADIS_ASC.inverse().getOrigin();

  std::setprecision(10);
  std::cout << "/pose_sensor/pose_sensor/init/q_ic/x: " << q_ASC_ADIS.x() <<  std::endl;
  std::cout << "/pose_sensor/pose_sensor/init/q_ic/y: " << q_ASC_ADIS.y() <<  std::endl;
  std::cout << "/pose_sensor/pose_sensor/init/q_ic/z: " << q_ASC_ADIS.z() <<  std::endl;
  std::cout << "/pose_sensor/pose_sensor/init/q_ic/w: " << q_ASC_ADIS.w() <<  std::endl;
  std::cout << std::endl;
  std::cout << "/pose_sensor/pose_sensor/init/p_ic/x: " << t_ASC_ADIS.x() <<  std::endl;
  std::cout << "/pose_sensor/pose_sensor/init/p_ic/y: " << t_ASC_ADIS.y() <<  std::endl;
  std::cout << "/pose_sensor/pose_sensor/init/p_ic/z: " << t_ASC_ADIS.z() <<  std::endl;
  return 0;
}

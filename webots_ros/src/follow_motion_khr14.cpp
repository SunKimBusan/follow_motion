// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <signal.h>
#include "ros/ros.h"
#define _USE_MATH_DEFINES
#include <math.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// include files to use services like 'robot_get_time'.
// srv files needed to use webots service can be found in the /srv folder where you found this example.
// for more info on how to create and use services with ROS refer to their website: http://wiki.ros.org/
// here 'webots_ros' is the name of the package used for this node. Replace it by your own package.
#include <webots_ros/get_float.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <webots_ros/robot_get_device_list.h>

#include <webots_ros/get_int.h>
#include <webots_ros/get_string.h>

#include "openpose_ros_msgs/OpenPoseHumanList3D.h"
#include <Eigen/Dense>

// include files to use standard message types in topic
// Webots only use basic messages type defined in ROS library
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <joint_filter.h>
#include <Point.h>

#define TIME_STEP 32;

static int controllerCount;
static std::vector<std::string> controllerList;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;
std::map<std::string, double> joint_angles_;
std::map <int, Eigen::Vector3d> body_position_;

enum BodyParts{
  Nose,
  Neck,
  RShoulder,
  RElbow,
  RWrist,
  LShoulder,
  LElbow,
  LWrist,
  MidHip,
  RHip,
  RKnee,
  RAnkle,
  LHip,
  LKnee,
  LAnkle,
  REye,
  LEye,
  REar,
  LEar,
  LBigToe,
  LSmallToe,
  LHeel,
  RBigToe,
  RSmallToe,
  RHeel
};

Point point_wow = Point(3, 4);

double spike_M = M_PI/180*90;
int spike_n = 5;
int max_seq_len = 50;
Filter joint_filter = Filter(spike_M, spike_n, max_seq_len);

ros::ServiceClient motorSetPositionClient_1;
webots_ros::set_float motorSetPositionSrv_1;
ros::ServiceClient motorGetTargetPositionClient_1;
webots_ros::get_float motorGetTargetPositionSrv_1;

ros::ServiceClient motorSetPositionClient_2;
webots_ros::set_float motorSetPositionSrv_2;
ros::ServiceClient motorGetTargetPositionClient_2;
webots_ros::get_float motorGetTargetPositionSrv_2;

ros::ServiceClient motorSetPositionClient_3;
webots_ros::set_float motorSetPositionSrv_3;
ros::ServiceClient motorGetTargetPositionClient_3;
webots_ros::get_float motorGetTargetPositionSrv_3;

ros::ServiceClient motorSetPositionClient_4;
webots_ros::set_float motorSetPositionSrv_4;
ros::ServiceClient motorGetTargetPositionClient_4;
webots_ros::get_float motorGetTargetPositionSrv_4;

ros::ServiceClient motorSetPositionClient_5;
webots_ros::set_float motorSetPositionSrv_5;
ros::ServiceClient motorGetTargetPositionClient_5;
webots_ros::get_float motorGetTargetPositionSrv_5;

ros::ServiceClient motorSetPositionClient_6;
webots_ros::set_float motorSetPositionSrv_6;
ros::ServiceClient motorGetTargetPositionClient_6;
webots_ros::get_float motorGetTargetPositionSrv_6;

ros::ServiceClient motorSetPositionClient_7;
webots_ros::set_float motorSetPositionSrv_7;
ros::ServiceClient motorGetTargetPositionClient_7;
webots_ros::get_float motorGetTargetPositionSrv_7;

ros::ServiceClient motorSetPositionClient_8;
webots_ros::set_float motorSetPositionSrv_8;
ros::ServiceClient motorGetTargetPositionClient_8;
webots_ros::get_float motorGetTargetPositionSrv_8;

void cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
    try
    {
      cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

void quit(int sig) {
  ROS_INFO("User stopped the 'robot_information_parser' node.");
  ros::shutdown();
  exit(0);
}

bool getShoulderLength(const openpose_ros_msgs::OpenPoseHuman3D &person, double& length){

  ROS_INFO("getShoulderLength");


  Eigen::Vector2d neck(0.0, 0.0), r_shoulder(0.0, 0.0), l_shoulder(0.0, 0.0), nose(0.0, 0.0);
  double r_sho_len = 0.0, l_sho_len = 0.0, nose_len = 0.0;

  for(int ix = 0; ix < person.body_key_points_with_prob.size(); ix++){
    openpose_ros_msgs::PointWithProb3D body_part = person.body_key_points_with_prob[ix];
    // Neck
    if(ix == 1){
      if(body_part.prob == 0.0)
        return false;
      neck = Eigen::Vector2d(body_part.x, body_part.y);
    }
    // RShoulder
    if(ix == 2){
      r_shoulder = Eigen::Vector2d(body_part.x, body_part.y);
    }
    // LShoulder
    if(ix == 5){
      l_shoulder = Eigen::Vector2d(body_part.x, body_part.y);
    }
    // Nose
    if(ix == 0){
      nose = Eigen::Vector2d(body_part.x, body_part.y);
    }
  }

  if(r_shoulder != Eigen::Vector2d(0.0, 0.0)){
    Eigen::Vector2d r_len_vec = r_shoulder - neck;
    r_sho_len = r_len_vec.norm();
  }
  if(l_shoulder != Eigen::Vector2d(0.0, 0.0)){
    Eigen::Vector2d l_len_vec = l_shoulder - neck;
    l_sho_len = l_len_vec.norm();
  }
  if(nose != Eigen::Vector2d(0.0, 0.0)){
    Eigen::Vector2d nose_len_vec = nose - neck;
    nose_len = nose_len_vec.norm();
  }

  length = r_sho_len + l_sho_len + nose_len;
  return true;
}

bool calcJointAngle(Eigen::Vector3d upper, Eigen::Vector3d lower, double& target_angle){
  // calc
  double inter_value = upper.dot(lower) / (upper.norm() * lower.norm());
  target_angle = acos(inter_value);
  return true;
}

void checkMaxAngle(const double maximum, double &target_angle){
  if(fabs(target_angle) > maximum)
    target_angle = target_angle > 0 ? maximum : -maximum;
}

void checkMinAngle(const double minimum, double &target_angle){
  if(fabs(target_angle) < minimum)
    target_angle = target_angle > 0 ? minimum : -minimum;
}

void calcJointStates(const openpose_ros_msgs::OpenPoseHuman3D &person_to_follow){
  ROS_INFO("calcJointStates");

  joint_angles_.clear();

  for(int ix = 0; ix < person_to_follow.body_key_points_with_prob.size(); ix++){
    openpose_ros_msgs::PointWithProb3D body_part = person_to_follow.body_key_points_with_prob[ix];
    body_position_[ix] = Eigen::Vector3d(body_part.x, body_part.y, body_part.z);
  }

  double link1 = 1;
  double link2 = 1;
  double theta0;
  double theta1;
  double theta2;
  double theta3;

  Eigen::Vector3d shoulder_to_elbow_vector;
  Eigen::Vector3d elbow_to_wrist_vector;
  Eigen::Vector3d e_s_vector;
  Eigen::Vector3d w_s_vector;
  Eigen::Vector3d a_vector;
  Eigen::Vector3d b_vector;
  Eigen::Vector3d c_vector;
  Eigen::Vector3d d_vector;
  Eigen::Vector3d b_minus_a_vector;
  Eigen::Vector3d z_basis;
  double alpha;
  double beta;
  double k_m;
  double x_m;
  double y_m;
  double z_m;
  Eigen::Vector3d m_vector;
  Eigen::Vector3d m_minus_c_vector;
  Eigen::Vector3d x_basis;
  Eigen::Vector3d n_vector;
  double gamma1;
  double beta1;
  double delta;
  double alpha2;
  double beta2;
  double dp_x;
  double dp_y;
  double dp_z;
  Eigen::Vector3d dp_vector;
  Eigen::Vector3d d_minus_dp_vector;
  Eigen::Vector3d y_basis;
  Eigen::Matrix3d k_s_R;
  double sin_theta0;
  Eigen::Matrix4d transform_1_0_ch3;
  Eigen::Matrix4d rotation_z;
  double theta;
  double cos_;
  double sin_;
  Eigen::Matrix4d rotation_x;
  Eigen::Matrix4d transform_2_1_ch3;
  Eigen::Vector4d w_s_vector_4;
  Eigen::Vector4d w_s_vector_4_dash;
  double sin_theta2;

  // Left Arm
  if(body_position_[LShoulder] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[LElbow] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[LWrist] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[RShoulder] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[MidHip] != Eigen::Vector3d(0.0, 0.0, 0.0)){

    shoulder_to_elbow_vector = body_position_[LElbow] - body_position_[LShoulder];
    elbow_to_wrist_vector = body_position_[LWrist] - body_position_[LElbow];

    // human vector to robot vector
    e_s_vector = shoulder_to_elbow_vector/shoulder_to_elbow_vector.norm()*link1;
    w_s_vector = shoulder_to_elbow_vector/shoulder_to_elbow_vector.norm()*link1 + elbow_to_wrist_vector/elbow_to_wrist_vector.norm()*link2;

    // rotation matrix camera to left shoulder
    a_vector = body_position_[RShoulder];
    b_vector = body_position_[LShoulder];
    c_vector = body_position_[MidHip];
    d_vector = Eigen::Vector3d(0.0, 0.0, 0.0);

    b_minus_a_vector = b_vector - a_vector;
    z_basis = b_minus_a_vector/b_minus_a_vector.norm();

    alpha = pow(b_vector.coeff(0)-a_vector.coeff(0),2)+pow(b_vector.coeff(1)-a_vector.coeff(1),2)+pow(b_vector.coeff(2)-a_vector.coeff(2),2);
    beta = (b_vector.coeff(0)-a_vector.coeff(0))*(a_vector.coeff(0)-c_vector.coeff(0))
                  +(b_vector.coeff(1)-a_vector.coeff(1))*(a_vector.coeff(1)-c_vector.coeff(1))
                  +(b_vector.coeff(2)-a_vector.coeff(2))*(a_vector.coeff(2)-c_vector.coeff(2));
    k_m = -beta/alpha;
    x_m = (b_vector.coeff(0)-a_vector.coeff(0))*k_m + a_vector.coeff(0);
    y_m = (b_vector.coeff(1)-a_vector.coeff(1))*k_m + a_vector.coeff(1);
    z_m = (b_vector.coeff(2)-a_vector.coeff(2))*k_m + a_vector.coeff(2);
    m_vector = Eigen::Vector3d(x_m, y_m, z_m);

    m_minus_c_vector = m_vector - c_vector;
    x_basis = m_minus_c_vector/m_minus_c_vector.norm();

    n_vector = (a_vector-c_vector).cross(b_vector-c_vector);
    n_vector = n_vector/n_vector.norm();

    gamma1 = d_vector.coeff(0)*n_vector.coeff(1)-d_vector.coeff(1)*n_vector.coeff(0);
    beta1 = d_vector.coeff(0)*n_vector.coeff(2)-d_vector.coeff(2)*n_vector.coeff(0);
    delta = -n_vector.coeff(0)*c_vector.coeff(0)-n_vector.coeff(1)*c_vector.coeff(1)-n_vector.coeff(2)*c_vector.coeff(2);
    alpha2 = n_vector.coeff(0)+(pow(n_vector.coeff(1),2)+pow(n_vector.coeff(2),2))/n_vector.coeff(0);
    beta2 = delta-(n_vector.coeff(1)*gamma1+n_vector.coeff(2)*beta1)/n_vector.coeff(0);
    dp_x = -beta2/alpha2;
    dp_y = n_vector.coeff(1)/n_vector.coeff(0)*dp_x-gamma1/n_vector.coeff(0);
    dp_z = n_vector.coeff(2)/n_vector.coeff(0)*dp_x-beta1/n_vector.coeff(0);
    dp_vector = Eigen::Vector3d(dp_x, dp_y, dp_z);

    d_minus_dp_vector = d_vector - dp_vector;
    y_basis = d_minus_dp_vector/d_minus_dp_vector.norm();

    k_s_R << x_basis.coeff(0), y_basis.coeff(0), z_basis.coeff(0),
             x_basis.coeff(1), y_basis.coeff(1), z_basis.coeff(1),
             x_basis.coeff(2), y_basis.coeff(2), z_basis.coeff(2);
    k_s_R = k_s_R.inverse().eval();

    // transform robot vector to shoulder frame
    e_s_vector = k_s_R * e_s_vector;
    w_s_vector = k_s_R * w_s_vector;

    theta1 = asin(e_s_vector.coeff(2)/link1);
    if(e_s_vector.coeff(0)>0)
      theta1 = M_PI-theta1;

    sin_theta0 = e_s_vector.coeff(1)/(-link1*cos(theta1));

    if(sin_theta0 >= 0)
      theta0 = acos(e_s_vector.coeff(0)/(-link1*cos(theta1)));
    else
      theta0 = -acos(e_s_vector.coeff(0)/(-link1*cos(theta1)));

    theta = theta0 + M_PI;
    cos_ = cos(theta);
    sin_ = sin(theta);
    rotation_z << cos_, -sin_, 0.0d, 0.0d,
                  sin_, cos_, 0.0d, 0.0d,
                  0.0d, 0.0d, 1.0d, 0.0d,
                  0.0d, 0.0d, 0.0d, 1.0d;
    theta = M_PI/2;
    cos_ = cos(theta);
    sin_ = sin(theta);
    rotation_x << 1.0d, 0.0d, 0.0d, 0.0d,
                  0.0d, cos_, -sin_, 0.0d,
                  0.0d, sin_, cos_, 0.0d,
                  0.0d, 0.0d, 0.0d, 1.0d;
    transform_1_0_ch3 = rotation_z*rotation_x;

    theta = theta1 + M_PI/2;
    cos_ = cos(theta);
    sin_ = sin(theta);
    rotation_z << cos_, -sin_, 0.0d, 0.0d,
                  sin_, cos_, 0.0d, 0.0d,
                  0.0d, 0.0d, 1.0d, 0.0d,
                  0.0d, 0.0d, 0.0d, 1.0d;
    theta = M_PI/2;
    cos_ = cos(theta);
    sin_ = sin(theta);
    rotation_x << 1.0d, 0.0d, 0.0d, 0.0d,
                  0.0d, cos_, -sin_, 0.0d,
                  0.0d, sin_, cos_, 0.0d,
                  0.0d, 0.0d, 0.0d, 1.0d;
    transform_2_1_ch3 = rotation_z*rotation_x;

    w_s_vector_4 = Eigen::Vector4d(w_s_vector.coeff(0), w_s_vector.coeff(1), w_s_vector.coeff(2), 1);
    w_s_vector_4_dash = transform_2_1_ch3.inverse()*transform_1_0_ch3.inverse()*w_s_vector_4;

    theta3 = acos((w_s_vector_4_dash.coeff(2)-link1)/link2);

    sin_theta2 = -w_s_vector_4_dash.coeff(0)/(link2*sin(theta3));
    if(sin_theta2 >= 0)
      theta2 = acos(w_s_vector_4_dash.coeff(1)/(link2*sin(theta3)));
    else
      theta2 = -acos(w_s_vector_4_dash.coeff(1)/(link2*sin(theta3)));

    // LeftShoulder
    //theta0 = joint_filter.setRawSeq1(theta0);
    if (theta0 < -M_PI)
      theta0 = -M_PI;
    else if (theta0 > M_PI/6)
      theta0 = M_PI/6;

    // LeftArm
    //theta1 = joint_filter.setRawSeq2(theta1);
    if (theta1 < 0)
      theta1 = 0;
    else if (theta1 > M_PI*3/4)
      theta1 = M_PI*3/4;

    // LeftElbow
    //theta2 = joint_filter.setRawSeq3(theta2);
    if (theta2 < -M_PI/4)
      theta2 = -M_PI/4;
    else if (theta2 > M_PI*3/4)
      theta2 = M_PI*3/4;

    // LeftForearm
    //theta3 = joint_filter.setRawSeq4(theta3);
    if (theta3 < 0)
      theta3 = 0;
    else if (theta3 > M_PI*5/6)
      theta3 = M_PI*5/6;

    joint_angles_["l_sho_pitch"] = theta0;
    joint_angles_["l_sho_roll"] = theta1;
    joint_angles_["l_el_pitch"] = theta2;
    joint_angles_["l_el"] = theta3;
  }

  // Right Arm
  if(body_position_[RShoulder] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[RElbow] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[RWrist] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[LShoulder] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[MidHip] != Eigen::Vector3d(0.0, 0.0, 0.0)){

    shoulder_to_elbow_vector = body_position_[RElbow] - body_position_[RShoulder];
    elbow_to_wrist_vector = body_position_[RWrist] - body_position_[RElbow];

    // human vector to robot vector
    e_s_vector = shoulder_to_elbow_vector/shoulder_to_elbow_vector.norm()*link1;
    w_s_vector = shoulder_to_elbow_vector/shoulder_to_elbow_vector.norm()*link1 + elbow_to_wrist_vector/elbow_to_wrist_vector.norm()*link2;

    // rotation matrix camera to left shoulder
    a_vector = body_position_[RShoulder];
    b_vector = body_position_[LShoulder];
    c_vector = body_position_[MidHip];
    d_vector = Eigen::Vector3d(0.0, 0.0, 0.0);

    b_minus_a_vector = b_vector - a_vector;
    z_basis = b_minus_a_vector/b_minus_a_vector.norm();

    alpha = pow(b_vector.coeff(0)-a_vector.coeff(0),2)+pow(b_vector.coeff(1)-a_vector.coeff(1),2)+pow(b_vector.coeff(2)-a_vector.coeff(2),2);
    beta = (b_vector.coeff(0)-a_vector.coeff(0))*(a_vector.coeff(0)-c_vector.coeff(0))
                  +(b_vector.coeff(1)-a_vector.coeff(1))*(a_vector.coeff(1)-c_vector.coeff(1))
                  +(b_vector.coeff(2)-a_vector.coeff(2))*(a_vector.coeff(2)-c_vector.coeff(2));
    k_m = -beta/alpha;
    x_m = (b_vector.coeff(0)-a_vector.coeff(0))*k_m + a_vector.coeff(0);
    y_m = (b_vector.coeff(1)-a_vector.coeff(1))*k_m + a_vector.coeff(1);
    z_m = (b_vector.coeff(2)-a_vector.coeff(2))*k_m + a_vector.coeff(2);
    m_vector = Eigen::Vector3d(x_m, y_m, z_m);

    m_minus_c_vector = m_vector - c_vector;
    x_basis = m_minus_c_vector/m_minus_c_vector.norm();

    n_vector = (a_vector-c_vector).cross(b_vector-c_vector);
    n_vector = n_vector/n_vector.norm();

    gamma1 = d_vector.coeff(0)*n_vector.coeff(1)-d_vector.coeff(1)*n_vector.coeff(0);
    beta1 = d_vector.coeff(0)*n_vector.coeff(2)-d_vector.coeff(2)*n_vector.coeff(0);
    delta = -n_vector.coeff(0)*c_vector.coeff(0)-n_vector.coeff(1)*c_vector.coeff(1)-n_vector.coeff(2)*c_vector.coeff(2);
    alpha2 = n_vector.coeff(0)+(pow(n_vector.coeff(1),2)+pow(n_vector.coeff(2),2))/n_vector.coeff(0);
    beta2 = delta-(n_vector.coeff(1)*gamma1+n_vector.coeff(2)*beta1)/n_vector.coeff(0);
    dp_x = -beta2/alpha2;
    dp_y = n_vector.coeff(1)/n_vector.coeff(0)*dp_x-gamma1/n_vector.coeff(0);
    dp_z = n_vector.coeff(2)/n_vector.coeff(0)*dp_x-beta1/n_vector.coeff(0);
    dp_vector = Eigen::Vector3d(dp_x, dp_y, dp_z);

    d_minus_dp_vector = d_vector - dp_vector;
    y_basis = d_minus_dp_vector/d_minus_dp_vector.norm();

    k_s_R << x_basis.coeff(0), y_basis.coeff(0), z_basis.coeff(0),
             x_basis.coeff(1), y_basis.coeff(1), z_basis.coeff(1),
             x_basis.coeff(2), y_basis.coeff(2), z_basis.coeff(2);
    k_s_R = k_s_R.inverse().eval();

    // transform robot vector to shoulder frame
    e_s_vector = k_s_R * e_s_vector;
    w_s_vector = k_s_R * w_s_vector;

    theta1 = asin(e_s_vector.coeff(2)/link1);
    if(e_s_vector.coeff(0)>0)
      theta1 = -M_PI-theta1;

    sin_theta0 = e_s_vector.coeff(1)/(-link1*cos(theta1));

    if(sin_theta0 >= 0)
      theta0 = acos(e_s_vector.coeff(0)/(-link1*cos(theta1)));
    else
      theta0 = -acos(e_s_vector.coeff(0)/(-link1*cos(theta1)));

    theta = theta0 + M_PI;
    cos_ = cos(theta);
    sin_ = sin(theta);
    rotation_z << cos_, -sin_, 0.0d, 0.0d,
                  sin_, cos_, 0.0d, 0.0d,
                  0.0d, 0.0d, 1.0d, 0.0d,
                  0.0d, 0.0d, 0.0d, 1.0d;
    theta = M_PI/2;
    cos_ = cos(theta);
    sin_ = sin(theta);
    rotation_x << 1.0d, 0.0d, 0.0d, 0.0d,
                  0.0d, cos_, -sin_, 0.0d,
                  0.0d, sin_, cos_, 0.0d,
                  0.0d, 0.0d, 0.0d, 1.0d;
    transform_1_0_ch3 = rotation_z*rotation_x;

    theta = theta1 + M_PI/2;
    cos_ = cos(theta);
    sin_ = sin(theta);
    rotation_z << cos_, -sin_, 0.0d, 0.0d,
                  sin_, cos_, 0.0d, 0.0d,
                  0.0d, 0.0d, 1.0d, 0.0d,
                  0.0d, 0.0d, 0.0d, 1.0d;
    theta = M_PI/2;
    cos_ = cos(theta);
    sin_ = sin(theta);
    rotation_x << 1.0d, 0.0d, 0.0d, 0.0d,
                  0.0d, cos_, -sin_, 0.0d,
                  0.0d, sin_, cos_, 0.0d,
                  0.0d, 0.0d, 0.0d, 1.0d;
    transform_2_1_ch3 = rotation_z*rotation_x;

    w_s_vector_4 = Eigen::Vector4d(w_s_vector.coeff(0), w_s_vector.coeff(1), w_s_vector.coeff(2), 1);
    w_s_vector_4_dash = transform_2_1_ch3.inverse()*transform_1_0_ch3.inverse()*w_s_vector_4;

    theta3 = acos((w_s_vector_4_dash.coeff(2)-link1)/link2);

    sin_theta2 = -w_s_vector_4_dash.coeff(0)/(link2*sin(theta3));
    if(sin_theta2 >= 0)
      theta2 = acos(w_s_vector_4_dash.coeff(1)/(link2*sin(theta3)));
    else
      theta2 = -acos(w_s_vector_4_dash.coeff(1)/(link2*sin(theta3)));

    // RightShoulder
    //theta0 = joint_filter.setRawSeq5(theta0);
    if (theta0 < -M_PI)
      theta0 = -M_PI;
    else if (theta0 > M_PI/6)
      theta0 = M_PI/6;

    // RightArm
    //theta1 = joint_filter.setRawSeq6(theta1);
    if (theta1 > 0)
      theta1 = 0;
    else if (theta1 < -M_PI*3/4)
      theta1 = -M_PI*3/4;

    // RightElbow
    //theta2 = joint_filter.setRawSeq7(theta2);
    if (theta2 > M_PI/4)
      theta2 = M_PI/4;
    else if (theta2 < -M_PI*3/4)
      theta2 = -M_PI*3/4;

    // RightForearm
    //theta3 = joint_filter.setRawSeq8(theta3);
    if (theta3 < 0)
      theta3 = 0;
    else if (theta3 > M_PI*5/6)
      theta3 = M_PI*5/6;

    joint_angles_["r_sho_pitch"] = theta0;
    joint_angles_["r_sho_roll"] = theta1;
    joint_angles_["r_el_pitch"] = theta2;
    joint_angles_["r_el"] = theta3;

  }

}

void publishJointStates(){
  ROS_INFO("publishJointStates");

  if(joint_angles_.size() == 0)
    return;

  float l_sho_pitch = (float) joint_angles_["l_sho_pitch"];
  float l_sho_roll = (float) joint_angles_["l_sho_roll"];
  float l_el_pitch = (float) joint_angles_["l_el_pitch"];
  float l_el = (float) joint_angles_["l_el"];

  float r_sho_pitch = (float) joint_angles_["r_sho_pitch"];
  float r_sho_roll = (float) joint_angles_["r_sho_roll"];
  float r_el_pitch = (float) joint_angles_["r_el_pitch"];
  float r_el = (float) joint_angles_["r_el"];

  // change direction for webots motor's standard
  motorSetPositionSrv_6.request.value = -l_sho_pitch;
  motorSetPositionSrv_2.request.value = l_sho_roll;
  motorSetPositionSrv_8.request.value = -l_el_pitch;
  motorSetPositionSrv_4.request.value = l_el;

  motorSetPositionSrv_5.request.value = -r_sho_pitch;
  motorSetPositionSrv_1.request.value = -r_sho_roll;
  motorSetPositionSrv_7.request.value = r_el_pitch;
  motorSetPositionSrv_3.request.value = r_el;

  motorSetPositionClient_6.call(motorSetPositionSrv_6);
  motorSetPositionClient_2.call(motorSetPositionSrv_2);
  motorSetPositionClient_8.call(motorSetPositionSrv_8);
  motorSetPositionClient_4.call(motorSetPositionSrv_4);

  motorSetPositionClient_5.call(motorSetPositionSrv_5);
  motorSetPositionClient_1.call(motorSetPositionSrv_1);
  motorSetPositionClient_7.call(motorSetPositionSrv_7);
  motorSetPositionClient_3.call(motorSetPositionSrv_3);

}

void humanPoseCallback(const openpose_ros_msgs::OpenPoseHumanList3D::ConstPtr &msg) {

  ROS_INFO("humanPoseCallback");

  // find closest person
  if(msg->human_list.size() == 0)
    return;

  int index_of_closest_person = -1;
  double shoulder_size = 0.0;
  for(int ix = 0; ix < msg->human_list.size(); ix++){
    double s_length = 0.0;
    bool result = getShoulderLength(msg->human_list[ix], s_length);
    // check the largest
    if(result == true && s_length > shoulder_size){
      shoulder_size = s_length;
      index_of_closest_person = ix;
    }
  }

  if(index_of_closest_person == -1)
    return;

  // calcJointStates
  calcJointStates(msg->human_list[index_of_closest_person]);

  // publish joint angle
  publishJointStates();

}


int main(int argc, char **argv) {

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)){
    ros::console::notifyLoggerLevelsChanged();
  }

  std::string controllerName;
  std::vector<std::string> deviceList;
  // create a node named 'robot_information_parser' on ROS network
  ros::init(argc, argv, "follow_motion", ros::init_options::AnonymousName);
  float i, step;
  ros::NodeHandle n;

  signal(SIGINT, quit);

  // subscribe to the topic model_name to get the list of availables controllers
  ros::Subscriber nameSub = n.subscribe("model_name", 100, controllerNameCallback);
  while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();

  // if there is more than one controller available, it let the user choose
  if (controllerCount == 1)
    controllerName = controllerList[0];
  else {
    int wantedController = 0;
    std::cout << "Choose the # of the controller you want to use:\n";
    std::cin >> wantedController;
    if (1 <= wantedController && wantedController <= controllerCount)
      controllerName = controllerList[wantedController - 1];
    else {
      ROS_ERROR("Invalid number for controller choice.");
      return 1;
    }
  }
  // leave topic once it is not necessary anymore
  nameSub.shutdown();

  // call get_type and get_model services to get more general information about the robot
  ros::ServiceClient getTypeClient = n.serviceClient<webots_ros::get_int>(controllerName + "/robot/get_type");
  webots_ros::get_int getTypeSrv;
  ros::ServiceClient getModelClient = n.serviceClient<webots_ros::get_string>(controllerName + "/robot/get_model");
  webots_ros::get_string getModelSrv;

  getTypeClient.call(getTypeSrv);
  if (getTypeSrv.response.value == 40)
    ROS_INFO("This controller is on a basic robot.");
  else
    ROS_INFO("This controller is on a supervisor robot.");

  if (getModelClient.call(getModelSrv)) {
    if (!getModelSrv.response.value.empty())
      ROS_INFO("The model of this robot is %s.", getModelSrv.response.value.c_str());
    else
      ROS_ERROR("The robot doesn't seems to have a model.");
  } else
    ROS_ERROR("Could not get the model of this robot.");

  // call deviceList service to get the list of the name of the devices available on the controller and print it
  // the deviceListSrv object contains 2 members: request and response. Their fields are described in the corresponding .srv
  // file
  ros::ServiceClient deviceListClient =
    n.serviceClient<webots_ros::robot_get_device_list>(controllerName + "/robot/get_device_list");
  webots_ros::robot_get_device_list deviceListSrv;

  if (deviceListClient.call(deviceListSrv)) {
    deviceList = deviceListSrv.response.list;
    ROS_INFO("The controller has %lu devices availables:", deviceList.size());
    for (unsigned int i = 0; i < deviceList.size(); i++)
      ROS_INFO("Device [%d]: %s.", i, deviceList[i].c_str());
  } else
    ROS_ERROR("Failed to call service deviceList.");

  motorSetPositionClient_1 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "RightArm" + "/set_position");
  motorGetTargetPositionClient_1 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "RightArm" + "/get_target_position");
  motorSetPositionClient_2 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "LeftArm" + "/set_position");
  motorGetTargetPositionClient_2 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "LeftArm" + "/get_target_position");

  motorSetPositionClient_3 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "RightForearm" + "/set_position");
  motorGetTargetPositionClient_3 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "RightForearm" + "/get_target_position");
  motorSetPositionClient_4 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "LeftForearm" + "/set_position");
  motorGetTargetPositionClient_4 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "LeftForearm" + "/get_target_position");

  motorSetPositionClient_5 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "RightShoulder" + "/set_position");
  motorGetTargetPositionClient_5 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "RightShoulder" + "/get_target_position");
  motorSetPositionClient_6 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "LeftShoulder" + "/set_position");
  motorGetTargetPositionClient_6 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "LeftShoulder" + "/get_target_position");

  motorSetPositionClient_7 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "RightElbow" + "/set_position");
  motorGetTargetPositionClient_7 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "RightElbow" + "/get_target_position");
  motorSetPositionClient_8 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "LeftElbow" + "/set_position");
  motorGetTargetPositionClient_8 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "LeftElbow" + "/get_target_position");

  // subscribe openpose
  ros::Subscriber human_pose_sub_ = n.subscribe("/openpose_ros/human_list3D", 1, humanPoseCallback);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber camera_sub_ = it.subscribe("/camera/color/image_raw", 1, cameraCallback);

  ROS_INFO("I AM HERE");

  ros::spin();
  cv::destroyWindow("view");
}

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


void calcJointStates(const openpose_ros_msgs::OpenPoseHuman3D &person_to_follow){
  ROS_INFO("calcJointStates");

  joint_angles_.clear();

  for(int ix = 0; ix < person_to_follow.body_key_points_with_prob.size(); ix++){
    openpose_ros_msgs::PointWithProb3D body_part = person_to_follow.body_key_points_with_prob[ix];
    body_position_[ix] = Eigen::Vector3d(body_part.x, body_part.y, body_part.z);
  }

  bool calc_result = false;

  // calc left shoulder
  if(body_position_[Neck] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[LShoulder] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[LElbow] != Eigen::Vector3d(0.0, 0.0, 0.0)){

    double l_shoulder_roll = 0.0;

    Eigen::Vector3d neck_vector = body_position_[Neck] - body_position_[LShoulder];
    Eigen::Vector3d shoulder_vector = body_position_[LShoulder] - body_position_[LElbow];
    Eigen::Vector3d arm_vector = body_position_[Neck] - body_position_[LElbow];

    // get a shoulder pitch angle
    calc_result = calcJointAngle(neck_vector, shoulder_vector, l_shoulder_roll);

    Eigen::Vector3d shoulder_direction = shoulder_vector.cross(arm_vector);

    if(shoulder_direction.coeff(2) > 0)
      l_shoulder_roll *= (-1);

    checkMaxAngle(80 * M_PI / 180.0, l_shoulder_roll);
    joint_angles_["l_sho_roll"] = l_shoulder_roll;
  }

  // calc right shoulder
  if(body_position_[Neck] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[RShoulder] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[RElbow] != Eigen::Vector3d(0.0, 0.0, 0.0)){

    double r_shoulder_roll = 0.0;

    Eigen::Vector3d neck_vector = body_position_[Neck] - body_position_[RShoulder];
    Eigen::Vector3d shoulder_vector = body_position_[RShoulder] - body_position_[RElbow];
    Eigen::Vector3d arm_vector = body_position_[Neck] - body_position_[RElbow];

    // get a shoulder pitch angle
    calc_result = calcJointAngle(neck_vector, shoulder_vector, r_shoulder_roll);

    Eigen::Vector3d shoulder_direction = shoulder_vector.cross(arm_vector);

    if(shoulder_direction.coeff(2) > 0)
      r_shoulder_roll *= (-1);

    checkMaxAngle(80 * M_PI / 180.0, r_shoulder_roll);
    joint_angles_["r_sho_roll"] = r_shoulder_roll;
  }

  // calc left arm
  if(body_position_[LShoulder] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[LElbow] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[LWrist] != Eigen::Vector3d(0.0, 0.0, 0.0)){

    double l_elbow = 0.0;

    Eigen::Vector3d shoulder_vector = body_position_[LShoulder] - body_position_[LElbow];
    Eigen::Vector3d elbow_vector = body_position_[LElbow] - body_position_[LWrist];
    Eigen::Vector3d arm_vector = body_position_[LShoulder] - body_position_[LWrist];

    calc_result = calcJointAngle(shoulder_vector,elbow_vector, l_elbow);

    Eigen::Vector3d elbow_direction = elbow_vector.cross(arm_vector);

    if(elbow_direction.coeff(2) < 0)
      l_elbow *= (-1);

    checkMaxAngle(120 * M_PI / 180.0, l_elbow);
    joint_angles_["l_el"] = l_elbow;
  }

  // calc right arm
  if(body_position_[RShoulder] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[RElbow] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[RWrist] != Eigen::Vector3d(0.0, 0.0, 0.0)){

    double r_elbow = 0.0;

    Eigen::Vector3d shoulder_vector = body_position_[RShoulder] - body_position_[RElbow];
    Eigen::Vector3d elbow_vector = body_position_[RElbow] - body_position_[RWrist];
    Eigen::Vector3d arm_vector = body_position_[RShoulder] - body_position_[RWrist];

    calc_result = calcJointAngle(shoulder_vector,elbow_vector, r_elbow);

    Eigen::Vector3d elbow_direction = elbow_vector.cross(arm_vector);

    if(elbow_direction.coeff(2) < 0)
      r_elbow *= (-1);

    checkMaxAngle(120 * M_PI / 180.0, r_elbow);
    joint_angles_["r_el"] = r_elbow;
  }

  // calc left shoulder pitch
  if(body_position_[Neck] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[LShoulder] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[MidHip] != Eigen::Vector3d(0.0, 0.0, 0.0)){

    double l_shoulder_pitch = 0.0;

    Eigen::Vector3d shoulder_vector = body_position_[LShoulder] - body_position_[LElbow];
    Eigen::Vector3d spine_vector = body_position_[Neck] - body_position_[MidHip];

    // get a shoulder pitch angle
    calc_result = calcJointAngle(spine_vector, shoulder_vector, l_shoulder_pitch);

    checkMaxAngle(170 * M_PI / 180.0, l_shoulder_pitch);
    joint_angles_["l_sho_pitch"] = l_shoulder_pitch;
  }

  // calc right shoulder pitch
  if(body_position_[Neck] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[RShoulder] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[MidHip] != Eigen::Vector3d(0.0, 0.0, 0.0)){

    double r_shoulder_pitch = 0.0;

    Eigen::Vector3d shoulder_vector = body_position_[RShoulder] - body_position_[RElbow];
    Eigen::Vector3d spine_vector = body_position_[Neck] - body_position_[MidHip];

    // get a shoulder pitch angle
    calc_result = calcJointAngle(spine_vector, shoulder_vector, r_shoulder_pitch);

    checkMaxAngle(170 * M_PI / 180.0, r_shoulder_pitch);
    joint_angles_["r_sho_pitch"] = r_shoulder_pitch;
  }

}

void publishJointStates(){
  ROS_INFO("publishJointStates");

  if(joint_angles_.size() == 0)
    return;

  float right_pos = (float) joint_angles_["r_sho_roll"];
  float left_pos = (float) joint_angles_["l_sho_roll"];
  float r_el_pos = (float) joint_angles_["r_el"];
  float l_el_pos = (float) joint_angles_["l_el"];
  float r_sh_pos = (float) joint_angles_["r_sho_pitch"];
  float l_sh_pos = (float) joint_angles_["l_sho_pitch"];

  ROS_INFO("right_pos %f", right_pos);
  ROS_INFO("left_pos %f", left_pos);

  motorSetPositionSrv_1.request.value = right_pos;
  motorSetPositionSrv_2.request.value = left_pos;
  motorSetPositionSrv_3.request.value = r_el_pos;
  motorSetPositionSrv_4.request.value = l_el_pos;
  motorSetPositionSrv_5.request.value = r_sh_pos;
  motorSetPositionSrv_6.request.value = l_sh_pos;

  motorSetPositionClient_1.call(motorSetPositionSrv_1);
  motorSetPositionClient_2.call(motorSetPositionSrv_2);
  motorSetPositionClient_3.call(motorSetPositionSrv_3);
  motorSetPositionClient_4.call(motorSetPositionSrv_4);
  //motorSetPositionClient_5.call(motorSetPositionSrv_5);
  //motorSetPositionClient_6.call(motorSetPositionSrv_6);

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


  motorSetPositionClient_1 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "ArmUpperR" + "/set_position");
  motorGetTargetPositionClient_1 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "ArmUpperR" + "/get_target_position");
  motorSetPositionClient_2 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "ArmUpperL" + "/set_position");
  motorGetTargetPositionClient_2 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "ArmUpperL" + "/get_target_position");
  motorSetPositionClient_3 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "ArmLowerR" + "/set_position");
  motorGetTargetPositionClient_3 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "ArmLowerR" + "/get_target_position");
  motorSetPositionClient_4 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "ArmLowerL" + "/set_position");
  motorGetTargetPositionClient_4 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "ArmLowerL" + "/get_target_position");
  motorSetPositionClient_5 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "ShoulderR" + "/set_position");
  motorGetTargetPositionClient_5 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "ShoulderR" + "/get_target_position");
  motorSetPositionClient_6 = n.serviceClient<webots_ros::set_float>(controllerName + '/' + "ShoulderL" + "/set_position");
  motorGetTargetPositionClient_6 = n.serviceClient<webots_ros::get_float>(controllerName + '/' + "ShoulderL" + "/get_target_position");

  // subscribe openpose
  ros::Subscriber human_pose_sub_ = n.subscribe("/openpose_ros/human_list3D", 1, humanPoseCallback);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber camera_sub_ = it.subscribe("/camera/color/image_raw", 1, cameraCallback);

  ROS_INFO("I AM HERE");

  ros::spin();
  cv::destroyWindow("view");
}

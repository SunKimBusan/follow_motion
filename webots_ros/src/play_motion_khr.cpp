#include <signal.h>
#include "ros/ros.h"
#define _USE_MATH_DEFINES
#include <cmath>

#include <webots_ros/get_float.h>
#include <webots_ros/set_float.h>

#include <webots_ros/robot_get_device_list.h>

#include <webots_ros/get_int.h>
#include <webots_ros/get_string.h>

#include <std_msgs/String.h>
#include <webots_ros/MotionData.h>
#include <Eigen/Dense>

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

static int controllerCount;
static std::vector<std::string> controllerList;

std::map<std::string, double> joint_angles_;
std::map <int, Eigen::Vector3d> body_position_;

enum BodyParts{
  Spine,
  Nose,
  Lshoulder,
  Lelbow,
  Lwrist,
  Rshoulder,
  Relbow,
  Rwrist
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

ros::ServiceClient motorSetPositionClient_7;
webots_ros::set_float motorSetPositionSrv_7;
ros::ServiceClient motorGetTargetPositionClient_7;
webots_ros::get_float motorGetTargetPositionSrv_7;

ros::ServiceClient motorSetPositionClient_8;
webots_ros::set_float motorSetPositionSrv_8;
ros::ServiceClient motorGetTargetPositionClient_8;
webots_ros::get_float motorGetTargetPositionSrv_8;

//ros::ServiceClient set_dxl_position_client;
//dynamixel_workbench_msgs::DynamixelCommand dxl_position;

const char* port_name = "/dev/ttyUSB0";
int baud_rate = 1000000;
DynamixelWorkbench dxl_wb;
const char *logg;
bool result = false;



// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

void quit(int sig) {
  ROS_INFO("User stopped the 'play_motion' node.");
  ros::shutdown();
  exit(0);
}

void calcJointStates(const webots_ros::MotionData::ConstPtr& msg){
  //ROS_INFO("calcJointStates");

  joint_angles_.clear();

  body_position_[Spine] = Eigen::Vector3d(msg->Spine.x, msg->Spine.y, msg->Spine.z);
  body_position_[Nose] = Eigen::Vector3d(msg->Nose.x, msg->Nose.y, msg->Nose.z);
  body_position_[Lshoulder] = Eigen::Vector3d(msg->Lshoulder.x, msg->Lshoulder.y, msg->Lshoulder.z);
  body_position_[Lelbow] = Eigen::Vector3d(msg->Lelbow.x, msg->Lelbow.y, msg->Lelbow.z);
  body_position_[Lwrist] = Eigen::Vector3d(msg->Lwrist.x, msg->Lwrist.y, msg->Lwrist.z);
  body_position_[Rshoulder] = Eigen::Vector3d(msg->Rshoulder.x, msg->Rshoulder.y, msg->Rshoulder.z);
  body_position_[Relbow] = Eigen::Vector3d(msg->Relbow.x, msg->Relbow.y, msg->Relbow.z);
  body_position_[Rwrist] = Eigen::Vector3d(msg->Rwrist.x, msg->Rwrist.y, msg->Rwrist.z);

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
  shoulder_to_elbow_vector = body_position_[Lelbow] - body_position_[Lshoulder];
  elbow_to_wrist_vector = body_position_[Lwrist] - body_position_[Lelbow];

  // human vector to robot vector
  e_s_vector = shoulder_to_elbow_vector/shoulder_to_elbow_vector.norm()*link1;
  w_s_vector = shoulder_to_elbow_vector/shoulder_to_elbow_vector.norm()*link1 + elbow_to_wrist_vector/elbow_to_wrist_vector.norm()*link2;

  // rotation matrix camera to left shoulder
  a_vector = body_position_[Rshoulder];
  b_vector = body_position_[Lshoulder];
  c_vector = body_position_[Spine];
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
  if (theta0 < -M_PI)
    theta0 = -M_PI;
  else if (theta0 > M_PI/6)
    theta0 = M_PI/6;

  // LeftArm
  if (theta1 < 0)
    theta1 = 0;
  else if (theta1 > M_PI*3/4)
    theta1 = M_PI*3/4;

  // LeftElbow
  if (theta2 < -M_PI/4)
    theta2 = -M_PI/4;
  else if (theta2 > M_PI*3/4)
    theta2 = M_PI*3/4;

  // LeftForearm
  if (theta3 < 0)
    theta3 = 0;
  else if (theta3 > M_PI*5/6)
    theta3 = M_PI*5/6;

  joint_angles_["l_sho_pitch"] = theta0;
  joint_angles_["l_sho_roll"] = theta1;
  joint_angles_["l_el_pitch"] = theta2;
  joint_angles_["l_el"] = theta3;

  // Right Arm
  shoulder_to_elbow_vector = body_position_[Relbow] - body_position_[Rshoulder];
  elbow_to_wrist_vector = body_position_[Rwrist] - body_position_[Relbow];

  // human vector to robot vector
  e_s_vector = shoulder_to_elbow_vector/shoulder_to_elbow_vector.norm()*link1;
  w_s_vector = shoulder_to_elbow_vector/shoulder_to_elbow_vector.norm()*link1 + elbow_to_wrist_vector/elbow_to_wrist_vector.norm()*link2;

  // rotation matrix camera to left shoulder
  a_vector = body_position_[Rshoulder];
  b_vector = body_position_[Lshoulder];
  c_vector = body_position_[Spine];
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
  if (theta0 < -M_PI)
    theta0 = -M_PI;
  else if (theta0 > M_PI/6)
    theta0 = M_PI/6;

  // RightArm
  if (theta1 > 0)
    theta1 = 0;
  else if (theta1 < -M_PI*3/4)
    theta1 = -M_PI*3/4;

  // RightElbow
  if (theta2 > M_PI/4)
    theta2 = M_PI/4;
  else if (theta2 < -M_PI*3/4)
    theta2 = -M_PI*3/4;

  // RightForearm
  if (theta3 < 0)
    theta3 = 0;
  else if (theta3 > M_PI*5/6)
    theta3 = M_PI*5/6;

  joint_angles_["r_sho_pitch"] = theta0;
  joint_angles_["r_sho_roll"] = theta1;
  joint_angles_["r_el_pitch"] = theta2;
  joint_angles_["r_el"] = theta3;
}

int radian2value(float radian){
  // Dynamixel step 4096
  int value = floor(radian/(2*M_PI)*4096);
  return value;
}

void publishJointStates(){
  //ROS_INFO("publishJointStates");

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

  /*motorSetPositionClient_6.call(motorSetPositionSrv_6);
  motorSetPositionClient_2.call(motorSetPositionSrv_2);
  motorSetPositionClient_8.call(motorSetPositionSrv_8);
  motorSetPositionClient_4.call(motorSetPositionSrv_4);

  motorSetPositionClient_5.call(motorSetPositionSrv_5);
  motorSetPositionClient_1.call(motorSetPositionSrv_1);
  motorSetPositionClient_7.call(motorSetPositionSrv_7);
  motorSetPositionClient_3.call(motorSetPositionSrv_3);*/

  /*dxl_position.request.addr_name = "Goal_Position";

  dxl_position.request.id = 1;
  dxl_position.request.value = radian2value(l_sho_pitch+M_PI);
  set_dxl_position_client.call(dxl_position);

  dxl_position.request.id = 2;
  dxl_position.request.value = radian2value(-l_sho_roll+M_PI);
  set_dxl_position_client.call(dxl_position);

  dxl_position.request.id = 3;
  dxl_position.request.value = radian2value(l_el_pitch+M_PI);
  set_dxl_position_client.call(dxl_position);

  dxl_position.request.id = 4;
  dxl_position.request.value = radian2value(l_el+M_PI);
  set_dxl_position_client.call(dxl_position);

  dxl_position.request.id = 5;
  dxl_position.request.value = radian2value(-r_sho_pitch+M_PI);
  set_dxl_position_client.call(dxl_position);

  dxl_position.request.id = 6;
  dxl_position.request.value = radian2value(-r_sho_roll+M_PI);
  set_dxl_position_client.call(dxl_position);

  dxl_position.request.id = 7;
  dxl_position.request.value = radian2value(r_el_pitch+M_PI);
  set_dxl_position_client.call(dxl_position);

  dxl_position.request.id = 8;
  dxl_position.request.value = radian2value(-r_el+M_PI);
  set_dxl_position_client.call(dxl_position);*/

  // bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, int32_t *data, const char **log)
  int32_t present_pos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  result = dxl_wb.bulkRead(&logg);
  result = dxl_wb.getBulkReadData(&present_pos[0], &logg);
    
  int32_t goal_pos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  goal_pos[0] = radian2value(l_sho_pitch+M_PI);
  goal_pos[1] = radian2value(-l_sho_roll+M_PI);
  goal_pos[2] = radian2value(l_el_pitch+M_PI);
  goal_pos[3] = radian2value(l_el+M_PI);
  goal_pos[4] = radian2value(-r_sho_pitch+M_PI);
  goal_pos[5] = radian2value(-r_sho_roll+M_PI);
  goal_pos[6] = radian2value(r_el_pitch+M_PI);
  goal_pos[7] = radian2value(-r_el+M_PI);

  int32_t diff_pos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i=0; i<8; ++i){
    diff_pos[i] = goal_pos[i] - present_pos[i];
  }
  // Goal Position range 0 ~ 4095
  // Profile Acceleration range 0 ~ 32737
  // Profile Velocity range 0 ~ 32737
  // Profile Acceleration will not exceed 50% of Profile Velocity
  float coeff_1 = 0.1;
  float coeff_2 = 0.5;
  for (int i=1; i<=8; ++i){
    int32_t p_vel = floor(abs(coeff_1*diff_pos[i-1]/4096*32737));
    int32_t p_acc = floor(coeff_2*p_vel);
    dxl_wb.addBulkWriteParam(i, "Profile_Velocity", p_vel, &logg);
    dxl_wb.addBulkWriteParam(i, "Profile_Acceleration", p_acc, &logg);
  }
  result = dxl_wb.bulkWrite(&logg);

  for (int i=1; i<=8; ++i){
    dxl_wb.addBulkWriteParam(i, "Goal_Position", goal_pos[i-1], &logg);
  }
  result = dxl_wb.bulkWrite(&logg);
  //dxl_wb.writeRegister(4, "Goal_Position", goal_pos[3], &logg);
}

// transform this for MotionData
void motionCallback(const webots_ros::MotionData::ConstPtr& msg) {

  // calcJointStates
  calcJointStates(msg);

  // publish joint angle
  publishJointStates();
}

void init_dynamixel(){
  result = dxl_wb.init(port_name, baud_rate, &logg);
  if (result == false)
  {
    printf("%s\n", logg);
    printf("Failed to init\n");
    ROS_INFO("Failed to init.");
  }
  else
    ROS_INFO("Succeed to init.");

  uint16_t model_number = 0;
  for (int i=1; i<=8; ++i){
    result = dxl_wb.ping(i, &model_number, &logg);
    if (result == false)
      ROS_INFO("Failed to ping id %d", i);
    else
      ROS_INFO("Succeed to ping id %d", i);
  }
  
  for (int i=1; i<=8; ++i){
    result = dxl_wb.jointMode(i, 0, 0, &logg);
    if (result == false)
      ROS_INFO("Failed to init id %d", i);
    else
      ROS_INFO("Succeed to init id %d", i);
  }

  result = dxl_wb.initBulkWrite(&logg);
  if (result == false)
  {
    ROS_INFO("%s\n", logg);
  }
  else
  {
    ROS_INFO("%s\n", logg);
  }

  result = dxl_wb.initBulkRead(&logg);
  if (result == false)
  {
    ROS_INFO("%s\n", logg);
  }
  else
  {
    ROS_INFO("%s\n", logg);
  }

  for (int i=1; i<=8; i++){
    result = dxl_wb.addBulkReadParam(i, "Present_Position", &logg);
    ROS_INFO("%s\n", logg);
  }

}


int main(int argc, char **argv) {

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)){
    ros::console::notifyLoggerLevelsChanged();
  }

  std::string controllerName;
  std::vector<std::string> deviceList;
  // create a node named 'play_motion' on ROS network
  ros::init(argc, argv, "play_motion", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  signal(SIGINT, quit);

  // subscribe to the topic model_name to get the list of availables controllers
  ros::Subscriber nameSub = nh.subscribe("model_name", 100, controllerNameCallback);
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
  ros::ServiceClient getTypeClient = nh.serviceClient<webots_ros::get_int>(controllerName + "/robot/get_type");
  webots_ros::get_int getTypeSrv;
  ros::ServiceClient getModelClient = nh.serviceClient<webots_ros::get_string>(controllerName + "/robot/get_model");
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
    nh.serviceClient<webots_ros::robot_get_device_list>(controllerName + "/robot/get_device_list");
  webots_ros::robot_get_device_list deviceListSrv;

  if (deviceListClient.call(deviceListSrv)) {
    deviceList = deviceListSrv.response.list;
    ROS_INFO("The controller has %lu devices availables:", deviceList.size());
    for (unsigned int i = 0; i < deviceList.size(); i++)
      ROS_INFO("Device [%d]: %s.", i, deviceList[i].c_str());
  } else
    ROS_ERROR("Failed to call service deviceList.");

  motorSetPositionClient_1 = nh.serviceClient<webots_ros::set_float>(controllerName + '/' + "RightArm" + "/set_position");
  motorGetTargetPositionClient_1 = nh.serviceClient<webots_ros::get_float>(controllerName + '/' + "RightArm" + "/get_target_position");
  motorSetPositionClient_2 = nh.serviceClient<webots_ros::set_float>(controllerName + '/' + "LeftArm" + "/set_position");
  motorGetTargetPositionClient_2 = nh.serviceClient<webots_ros::get_float>(controllerName + '/' + "LeftArm" + "/get_target_position");

  motorSetPositionClient_3 = nh.serviceClient<webots_ros::set_float>(controllerName + '/' + "RightForearm" + "/set_position");
  motorGetTargetPositionClient_3 = nh.serviceClient<webots_ros::get_float>(controllerName + '/' + "RightForearm" + "/get_target_position");
  motorSetPositionClient_4 = nh.serviceClient<webots_ros::set_float>(controllerName + '/' + "LeftForearm" + "/set_position");
  motorGetTargetPositionClient_4 = nh.serviceClient<webots_ros::get_float>(controllerName + '/' + "LeftForearm" + "/get_target_position");

  motorSetPositionClient_5 = nh.serviceClient<webots_ros::set_float>(controllerName + '/' + "RightShoulder" + "/set_position");
  motorGetTargetPositionClient_5 = nh.serviceClient<webots_ros::get_float>(controllerName + '/' + "RightShoulder" + "/get_target_position");
  motorSetPositionClient_6 = nh.serviceClient<webots_ros::set_float>(controllerName + '/' + "LeftShoulder" + "/set_position");
  motorGetTargetPositionClient_6 = nh.serviceClient<webots_ros::get_float>(controllerName + '/' + "LeftShoulder" + "/get_target_position");

  motorSetPositionClient_7 = nh.serviceClient<webots_ros::set_float>(controllerName + '/' + "RightElbow" + "/set_position");
  motorGetTargetPositionClient_7 = nh.serviceClient<webots_ros::get_float>(controllerName + '/' + "RightElbow" + "/get_target_position");
  motorSetPositionClient_8 = nh.serviceClient<webots_ros::set_float>(controllerName + '/' + "LeftElbow" + "/set_position");
  motorGetTargetPositionClient_8 = nh.serviceClient<webots_ros::get_float>(controllerName + '/' + "LeftElbow" + "/get_target_position");

  //set_dxl_position_client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

  init_dynamixel();

  ros::Subscriber sub_motion = nh.subscribe("motion_data", 1000, motionCallback);

  ros::spin();

}

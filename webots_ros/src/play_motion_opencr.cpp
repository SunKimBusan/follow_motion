#include <signal.h>
#include "ros/ros.h"
#define _USE_MATH_DEFINES
#include <cmath>

#include <std_msgs/String.h>
#include <webots_ros/MotionData.h>
#include <Eigen/Dense>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>


std::map<std::string, double> joint_angles_;
std::map <int, Eigen::Vector3d> body_position_;

ros::Publisher pub_arm;

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

void calcJointStates(const webots_ros::MotionData::ConstPtr& msg){

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
  
  float l_sho_pitch = (float) joint_angles_["l_sho_pitch"];
  float l_sho_roll = (float) joint_angles_["l_sho_roll"];
  float l_el_pitch = (float) joint_angles_["l_el_pitch"];
  float l_el = (float) joint_angles_["l_el"];

  float r_sho_pitch = (float) joint_angles_["r_sho_pitch"];
  float r_sho_roll = (float) joint_angles_["r_sho_roll"];
  float r_el_pitch = (float) joint_angles_["r_el_pitch"];
  float r_el = (float) joint_angles_["r_el"];
  
  int32_t goal_pos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  goal_pos[0] = radian2value(l_sho_pitch+M_PI);
  goal_pos[1] = radian2value(-l_sho_roll+M_PI);
  goal_pos[2] = radian2value(l_el_pitch+M_PI);
  goal_pos[3] = radian2value(l_el+M_PI);
  goal_pos[4] = radian2value(-r_sho_pitch+M_PI);
  goal_pos[5] = radian2value(-r_sho_roll+M_PI);
  goal_pos[6] = radian2value(r_el_pitch+M_PI);
  goal_pos[7] = radian2value(-r_el+M_PI);

  pub_arm.publish(goal_pos);
}

// transform this for MotionData
void motionCallback(const webots_ros::MotionData::ConstPtr& msg) {

  // calcJointStates
  calcJointStates(msg);

  // publish joint angle
  publishJointStates();
}

int main(int argc, char **argv) {
  // create a node named 'play_motion' on ROS network
  ros::init(argc, argv, "play_motion", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::Subscriber sub_motion = nh.subscribe("motion_data", 1000, motionCallback);
  pub_arm = nh.advertise<std_msgs::Int32MultiArray>("arm_pos", 1000);
  ros::spin();
}

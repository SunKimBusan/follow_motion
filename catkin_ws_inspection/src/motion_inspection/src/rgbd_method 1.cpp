#include <signal.h>
#include "ros/ros.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include "openpose_ros_msgs/OpenPoseHumanList3D.h"
#include <Eigen/Dense>

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

}

void humanPoseCallback(const openpose_ros_msgs::OpenPoseHumanList3D::ConstPtr &msg) {
  
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

  ros::init(argc, argv, "follow_motion", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  // subscribe openpose
  ros::Subscriber human_pose_sub_ = n.subscribe("/openpose_ros/human_list3D", 1, humanPoseCallback);
  ros::spin();
}

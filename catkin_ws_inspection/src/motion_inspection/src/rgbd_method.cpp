#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <openpose_ros_msgs/OpenPoseHumanList3D.h>
#include <Eigen/Dense>

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

  for(int ix = 0; ix < person_to_follow.body_key_points_with_prob.size(); ix++){
    openpose_ros_msgs::PointWithProb3D body_part = person_to_follow.body_key_points_with_prob[ix];
    body_position_[ix] = Eigen::Vector3d(body_part.x, body_part.y, body_part.z);
  }

  if(body_position_[LShoulder] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[LElbow] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[LWrist] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[RShoulder] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[RElbow] != Eigen::Vector3d(0.0, 0.0, 0.0) &&
    body_position_[RWrist] != Eigen::Vector3d(0.0, 0.0, 0.0)){
    
    //ROS_INFO("x coordinate [%f] y coordinate [%f] z coordinate [%f]", body_position_[LShoulder].coeff(0), body_position_[LShoulder].coeff(0), body_position_[LShoulder].coeff(0));

    // Scaling
    Eigen::Vector3d body_vec_1 = body_position_[LShoulder]/100;
    Eigen::Vector3d body_vec_2 = body_position_[LElbow]/100;
    Eigen::Vector3d body_vec_3 = body_position_[LWrist]/100;
    Eigen::Vector3d body_vec_4 = body_position_[RShoulder]/100;
    Eigen::Vector3d body_vec_5 = body_position_[RElbow]/100;
    Eigen::Vector3d body_vec_6 = body_position_[RWrist]/100;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin(tf::Vector3(body_vec_1.coeff(0), body_vec_1.coeff(1), body_vec_1.coeff(2)));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lshoulder"));

    transform.setOrigin(tf::Vector3(body_vec_2.coeff(0), body_vec_2.coeff(1), body_vec_2.coeff(2)));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lelbow"));

    transform.setOrigin(tf::Vector3(body_vec_3.coeff(0), body_vec_3.coeff(1), body_vec_3.coeff(2)));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lwrist"));

    transform.setOrigin(tf::Vector3(body_vec_4.coeff(0), body_vec_4.coeff(1), body_vec_4.coeff(2)));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rshoulder"));

    transform.setOrigin(tf::Vector3(body_vec_5.coeff(0), body_vec_5.coeff(1), body_vec_5.coeff(2)));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "relbow"));

    transform.setOrigin(tf::Vector3(body_vec_6.coeff(0), body_vec_6.coeff(1), body_vec_6.coeff(2)));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rwrist"));

  }

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

}


int main(int argc, char **argv) {

  ros::init(argc, argv, "rgbd_method_node", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  // subscribe openpose
  ros::Subscriber human_pose_sub_ = n.subscribe("/openpose_ros/human_list3D", 1, humanPoseCallback);
  ros::spin();
}

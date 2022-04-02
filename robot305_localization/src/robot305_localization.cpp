/* Author: Mahoro */

#include "robot305_localization/robot305_localization.h"

namespace robotis_op
{

ROBOT305Localization::ROBOT305Localization()
 : ros_node_(),
   transform_tolerance_(0.0),
   err_tol_(0.2),
   is_moving_walking_(false)
{
  initialize();

  waist_pose_base_walking_.pose.position.x = 0.0;
  waist_pose_base_walking_.pose.position.y = 0.0;
  waist_pose_base_walking_.pose.position.z = 0.0;
  waist_pose_base_walking_.pose.orientation.x = 0.0;
  waist_pose_base_walking_.pose.orientation.y = 0.0;
  waist_pose_base_walking_.pose.orientation.z = 0.0;
  waist_pose_base_walking_.pose.orientation.w = 1.0;

  waist_pose_offset_.pose.position.x = 0.0;
  waist_pose_offset_.pose.position.y = 0.0;
  waist_pose_offset_.pose.position.z = 0.3642; //0.3402256 - 0.0907;
  waist_pose_offset_.pose.orientation.x = 0.0;
  waist_pose_offset_.pose.orientation.y = 0.0;
  waist_pose_offset_.pose.orientation.z = 0.0;
  waist_pose_offset_.pose.orientation.w = 1.0;

  waist_pose_old_.pose.position.x = 0.0;
  waist_pose_old_.pose.position.y = 0.0;
  waist_pose_old_.pose.position.z = 0.0;
  waist_pose_old_.pose.orientation.x = 0.0;
  waist_pose_old_.pose.orientation.y = 0.0;
  waist_pose_old_.pose.orientation.z = 0.0;
  waist_pose_old_.pose.orientation.w = 1.0;

  update();
}

ROBOT305Localization::~ROBOT305Localization()
{

}

void ROBOT305Localization::initialize()
{
  // subscriber
  waist_pose_msg_sub_ = ros_node_.subscribe("/robotis/waist_pose", 5,
                                             &ROBOT305Localization::waistPoseCallback, this);
//  waist_base_walking_msg_sub_ = ros_node_.subscribe("/robotis/waist_pose_base_walking", 5,
//                                                               &ROBOT305Localization::waistPoseBaseWalkingCallback, this);

  waist_reset_msg_sub_ = ros_node_.subscribe("/robotis/waist_pose_reset", 5,
                                                       &ROBOT305Localization::waistPoseResetCallback, this);

}

void ROBOT305Localization::waistPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  mutex_.lock();

  waist_pose_offset_ = *msg;
  waist_pose_.header.stamp = waist_pose_offset_.header.stamp;

  mutex_.unlock();
}

//void ROBOT305Localization::waistPoseBaseWalkingCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
//{
//  mutex_.lock();

//  Eigen::Quaterniond msg_q;
//  tf::quaternionMsgToEigen(msg->pose.orientation, msg_q);

//  Eigen::MatrixXd rpy = robotis_framework::convertQuaternionToRPY(msg_q);
//  double yaw = rpy.coeff(2,0);

//  if ( fabs(msg->pose.position.x) <= 1e-3 &&
//       fabs(msg->pose.position.y) <= 1e-3 &&
//       fabs(yaw) <= 1e-3 )
//  {
//    if (is_moving_walking_ == true)
//    {
//      ROS_INFO("Walking Pelvis Pose Update");
//      waist_pose_old_.pose.position.x += waist_pose_base_walking_new_.pose.position.x;
//      waist_pose_old_.pose.position.y += waist_pose_base_walking_new_.pose.position.y;

//      Eigen::Quaterniond pose_old_quaternion(waist_pose_old_.pose.orientation.w,
//                                             waist_pose_old_.pose.orientation.x,
//                                             waist_pose_old_.pose.orientation.y,
//                                             waist_pose_old_.pose.orientation.z);

//      Eigen::Quaterniond pose_base_quaternion(waist_pose_base_walking_.pose.orientation.w,
//                                              waist_pose_base_walking_.pose.orientation.x,
//                                              waist_pose_base_walking_.pose.orientation.y,
//                                              waist_pose_base_walking_.pose.orientation.z);

//      Eigen::Quaterniond q = pose_old_quaternion * pose_base_quaternion;
//      tf::quaternionEigenToMsg(q, waist_pose_old_.pose.orientation);

//      is_moving_walking_ = false;
//    }
//  }
//  else
//  {
//    is_moving_walking_ = true;
//  }

//  waist_pose_base_walking_ = *msg;
//  waist_pose_.header.stamp = waist_pose_base_walking_.header.stamp;

//  mutex_.unlock();
//}

void ROBOT305Localization::waistPoseResetCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "reset")
  {
    ROS_INFO("Pelvis Pose Reset");

    waist_pose_old_.pose.position.x = 0.0;
    waist_pose_old_.pose.position.y = 0.0;
    waist_pose_old_.pose.orientation.x = 0.0;
    waist_pose_old_.pose.orientation.y = 0.0;
    waist_pose_old_.pose.orientation.z = 0.0;
    waist_pose_old_.pose.orientation.w = 1.0;
  }
}

void ROBOT305Localization::process()
{
  update();

  waist_trans_.setOrigin(tf::Vector3(waist_pose_.pose.position.x,
                                      waist_pose_.pose.position.y,
                                      waist_pose_.pose.position.z)
                          );

  tf::Quaternion q(waist_pose_.pose.orientation.x,
                   waist_pose_.pose.orientation.y,
                   waist_pose_.pose.orientation.z,
                   waist_pose_.pose.orientation.w);

  waist_trans_.setRotation(q);

  ros::Duration transform_tolerance(transform_tolerance_);
  ros::Time transform_expiration = (waist_pose_.header.stamp + transform_tolerance);

  tf::StampedTransform tmp_tf_stamped(waist_trans_, transform_expiration, "world", "body_link");

  broadcaster_.sendTransform(tmp_tf_stamped);
}

void ROBOT305Localization::update()
{
  mutex_.lock();

  Eigen::Quaterniond pose_old_quaternion(waist_pose_old_.pose.orientation.w,
                                         waist_pose_old_.pose.orientation.x,
                                         waist_pose_old_.pose.orientation.y,
                                         waist_pose_old_.pose.orientation.z);

//  Eigen::Quaterniond pose_base_walking_quaternion(waist_pose_base_walking_.pose.orientation.w,
//                                                  waist_pose_base_walking_.pose.orientation.x,
//                                                  waist_pose_base_walking_.pose.orientation.y,
//                                                  waist_pose_base_walking_.pose.orientation.z);

  Eigen::Quaterniond pose_offset_quaternion(waist_pose_offset_.pose.orientation.w,
                                            waist_pose_offset_.pose.orientation.x,
                                            waist_pose_offset_.pose.orientation.y,
                                            waist_pose_offset_.pose.orientation.z);

  Eigen::Quaterniond pose_quaternion =
      pose_old_quaternion *
//      pose_base_walking_quaternion *
      pose_offset_quaternion;

//  Eigen::MatrixXd position_walking = Eigen::MatrixXd::Zero(3,1);
//  position_walking.coeffRef(0,0) =
//      waist_pose_base_walking_.pose.position.x;
//  position_walking.coeffRef(1,0) =
//      waist_pose_base_walking_.pose.position.y;

  Eigen::MatrixXd position_offset = Eigen::MatrixXd::Zero(3,1);
  position_offset.coeffRef(0,0) =
      waist_pose_offset_.pose.position.x;
  position_offset.coeffRef(1,0) =
      waist_pose_offset_.pose.position.y;

  Eigen::MatrixXd orientation = robotis_framework::convertQuaternionToRotation(pose_old_quaternion);
//  Eigen::MatrixXd position_walking_new = orientation * position_walking;
  Eigen::MatrixXd position_offset_new = orientation * position_offset;

//  waist_pose_base_walking_new_.pose.position.x = position_walking_new.coeff(0,0);
//  waist_pose_base_walking_new_.pose.position.y = position_walking_new.coeff(1,0);

  waist_pose_offset_new_.pose.position.x = position_offset_new.coeff(0,0);
  waist_pose_offset_new_.pose.position.y = position_offset_new.coeff(1,0);

  waist_pose_.pose.position.x =
      waist_pose_old_.pose.position.x +
//      waist_pose_base_walking_new_.pose.position.x +
      waist_pose_offset_new_.pose.position.x;

  waist_pose_.pose.position.y =
      waist_pose_old_.pose.position.y +
//      waist_pose_base_walking_new_.pose.position.y +
      waist_pose_offset_new_.pose.position.y;

  waist_pose_.pose.position.z =
      waist_pose_offset_.pose.position.z;

  tf::quaternionEigenToMsg(pose_quaternion, waist_pose_.pose.orientation);

  mutex_.unlock();
}

} // namespace robotis_op

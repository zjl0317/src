/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman */

#include "op2_walking_module/op2_walking_module.h"

namespace robotis_op
{

WalkingModule::WalkingModule()
    : control_cycle_msec_(8),
      DEBUG(false)
{
  enable_ = false;
  module_name_ = "walking_module";
  control_mode_ = robotis_framework::PositionControl;

  init_pose_count_ = 0;
  walking_state_ = WalkingReady;
  previous_x_move_amplitude_ = 0.0;

  op2_kd_ = new OP2KinematicsDynamics(WholeBody2);

  // result
  result_["r_hip_yaw"] = new robotis_framework::DynamixelState();
  result_["r_hip_roll"] = new robotis_framework::DynamixelState();
  result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["r_knee"] = new robotis_framework::DynamixelState();
  result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["r_ank_roll"] = new robotis_framework::DynamixelState();

  result_["l_hip_yaw"] = new robotis_framework::DynamixelState();
  result_["l_hip_roll"] = new robotis_framework::DynamixelState();
  result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["l_knee"] = new robotis_framework::DynamixelState();
  result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["l_ank_roll"] = new robotis_framework::DynamixelState();

  result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
  result_["l_sho_pitch"] = new robotis_framework::DynamixelState();

  // joint table
  joint_table_["r_hip_yaw"] = 0;
  joint_table_["r_hip_roll"] = 1;
  joint_table_["r_hip_pitch"] = 2;
  joint_table_["r_knee"] = 3;
  joint_table_["r_ank_pitch"] = 4;
  joint_table_["r_ank_roll"] = 5;

  joint_table_["l_hip_yaw"] = 6;
  joint_table_["l_hip_roll"] = 7;
  joint_table_["l_hip_pitch"] = 8;
  joint_table_["l_knee"] = 9;
  joint_table_["l_ank_pitch"] = 10;
  joint_table_["l_ank_roll"] = 11;

  joint_table_["r_sho_pitch"] = 12;
  joint_table_["l_sho_pitch"] = 13;

  target_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  init_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  joint_axis_direction_ = Eigen::MatrixXi::Zero(1, result_.size());
}

WalkingModule::~WalkingModule()
{
  queue_thread_.join();
}

void WalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  queue_thread_ = boost::thread(boost::bind(&WalkingModule::queueThread, this));
  control_cycle_msec_ = control_cycle_msec;

  // m, s, rad
  // init pose
  walking_param_.init_x_offset      = -0.010;
  walking_param_.init_y_offset      = 0.005;
  walking_param_.init_z_offset      = 0.020;
  walking_param_.init_roll_offset   = 0.0;
  walking_param_.init_pitch_offset  = 0.0 * DEGREE2RADIAN;
  walking_param_.init_yaw_offset    = 0.0 * DEGREE2RADIAN;
  walking_param_.hip_pitch_offset   = 13.0 * DEGREE2RADIAN;
  // time
  walking_param_.period_time        = 600 * 0.001;
  walking_param_.dsp_ratio          = 0.1;
  walking_param_.step_fb_ratio      = 0.28;
  // walking
  walking_param_.x_move_amplitude   = 0.0;
  walking_param_.y_move_amplitude   = 0.0;
  walking_param_.z_move_amplitude   = 0.040;    // foot height
  walking_param_.angle_move_amplitude = 0.0;
  // balance
  walking_param_.balance_enable           = false;
  walking_param_.balance_hip_roll_gain    = 0.5;
  walking_param_.balance_knee_gain        = 0.3;
  walking_param_.balance_ankle_roll_gain  = 1.0;
  walking_param_.balance_ankle_pitch_gain = 0.9;
  walking_param_.y_swap_amplitude         = 0.020;
  walking_param_.z_swap_amplitude         = 0.005;
  walking_param_.pelvis_offset            = 3.0 * DEGREE2RADIAN;
  walking_param_.arm_swing_gain           = 1.5;

  // member variable
  body_swing_y = 0;
  body_swing_z = 0;

  x_swap_phase_shift_     = M_PI;
  x_swap_amplitude_shift_ = 0;
  x_move_phase_shift_     = M_PI / 2;
  x_move_amplitude_shift_ = 0;
  y_swap_phase_shift_     = 0;
  y_swap_amplitude_shift_ = 0;
  y_move_phase_shift_     = M_PI / 2;
  z_swap_phase_shift_     = M_PI * 3 / 2;
  z_move_phase_shift_     = M_PI / 2;
  a_move_phase_shift_     = M_PI / 2;

  ctrl_running_ = false;
  real_running_ = false;
  time_ = 0;

  //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL,
  //                     L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL,
  //                     R_ARM_SWING, L_ARM_SWING
  joint_axis_direction_ <<      -1,         -1,           1,      1,            -1,            1,
                                -1,         -1,          -1,     -1,             1,            1,
                                 1,         -1;
  init_position_        <<     0.0,        0.0,         0.0,    0.0,           0.0,          0.0,
                               0.0,        0.0,         0.0,    0.0,           0.0,          0.0,
                           -48.345,     41.313;
  init_position_ *= DEGREE2RADIAN;

  ros::NodeHandle ros_node;

  std::string default_param_path = ros::package::getPath("op2_walking_module") + "/config/param.yaml";
  ros_node.param<std::string>("walking_param_path", param_path_, default_param_path);

  loadWalkingParam(param_path_);

  updateTimeParam();
  updateMovementParam();
}

void WalkingModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);

  /* ROS Service Callback Functions */
  ros::ServiceServer get_walking_param_server = ros_node.advertiseService("/robotis/walking/get_params",
                                                                          &WalkingModule::getWalkigParameterCallback,
                                                                          this);

  /* sensor topic subscribe */
  ros::Subscriber walking_command_sub = ros_node.subscribe("/robotis/walking/command", 0,
                                                           &WalkingModule::walkingCommandCallback, this);
  ros::Subscriber walking_param_sub = ros_node.subscribe("/robotis/walking/set_params", 0,
                                                         &WalkingModule::walkingParameterCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void WalkingModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Walking";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

void WalkingModule::walkingCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if(enable_ == false)
  {
    ROS_WARN("walking module is not ready.");
    return;
  }

  if (msg->data == "start")
    startWalking();
  else if (msg->data == "stop")
    stop();
  else if (msg->data == "balance on")
    walking_param_.balance_enable = true;
  else if (msg->data == "balance off")
    walking_param_.balance_enable = false;
  else if (msg->data == "save")
    saveWalkingParam(param_path_);
}

void WalkingModule::walkingParameterCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr &msg)
{
  walking_param_ = *msg;  
}

bool WalkingModule::getWalkigParameterCallback(op3_walking_module_msgs::GetWalkingParam::Request &req,
                                               op3_walking_module_msgs::GetWalkingParam::Response &res)
{
  res.parameters = walking_param_;

  return true;
}

double WalkingModule::wSin(double time, double period, double period_shift, double mag, double mag_shift)
{
  return mag * sin(2 * M_PI / period * time - period_shift) + mag_shift;
}

// m, rad
// for default op3: it was used from previos version(OP2) but it's not using now.
bool WalkingModule::computeIK(double *out, double pos_x, double pos_y, double pos_z, double ori_roll, double ori_pitch,
                              double ori_yaw)
{
  double thigh_length = 93.0 * 0.001;  //m
  double calf_length = 93.0 * 0.001;  //m
  double ankle_length = 33.5 * 0.001;  //m
  double leg_length = 219.5 * 0.001;  //m (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)

  Eigen::MatrixXd transformation_ad, transformation_da, transformation_cd, transformation_dc, transformation_ac;
  Eigen::Vector3d vector;
  double r_ac, acos_value, atan_value, value_k, value_l, value_m, value_n, value_s, value_c, theta;

  // make transform matrix
  transformation_ad = robotis_framework::getTransformationXYZRPY(pos_x, pos_y, pos_z, ori_roll, ori_pitch, ori_yaw);

  vector << pos_x + transformation_ad.coeff(0, 2) * ankle_length, pos_y + transformation_ad.coeff(1, 2) * ankle_length, (pos_z
      - leg_length) + transformation_ad.coeff(2, 2) * ankle_length;

  // Get Knee
  r_ac = vector.norm();
  acos_value = acos(
      (r_ac * r_ac - thigh_length * thigh_length - calf_length * calf_length) / (2 * thigh_length * calf_length));
  if (std::isnan(acos_value) == 1)
    return false;
  *(out + 3) = acos_value;

  // Get Ankle Roll
  transformation_da = robotis_framework::getInverseTransformation(transformation_ad);
  double tda_y = transformation_da.coeff(1, 3);
  double tda_z = transformation_da.coeff(2, 3);
  value_k = sqrt(tda_y * tda_y + tda_z * tda_z);
  value_l = sqrt(tda_y * tda_y + (tda_z - ankle_length) * (tda_z - ankle_length));
  value_m = (value_k * value_k - value_l * value_l - ankle_length * ankle_length) / (2 * value_l * ankle_length);
  if (value_m > 1.0)
    value_m = 1.0;
  else if (value_m < -1.0)
    value_m = -1.0;
  acos_value = acos(value_m);
  if (std::isnan(acos_value) == 1)
    return false;
  if (tda_y < 0.0)
    *(out + 5) = -acos_value;
  else
    *(out + 5) = acos_value;

  // Get Hip Yaw
  transformation_cd = robotis_framework::getTransformationXYZRPY(0.0, 0.0, -ankle_length, *(out + 5), 0.0, 0.0);
  transformation_dc = robotis_framework::getInverseTransformation(transformation_cd);
  transformation_ac = transformation_ad * transformation_dc;
  atan_value = atan2(-transformation_ac.coeff(0, 1), transformation_ac.coeff(1, 1));
  if (std::isinf(atan_value) == 1)
    return false;
  *(out) = atan_value;

  // Get Hip Roll
  atan_value = atan2(transformation_ac.coeff(2, 1),
                     -transformation_ac.coeff(0, 1) * sin(*(out)) + transformation_ac.coeff(1, 1) * cos(*(out)));
  if (std::isinf(atan_value) == 1)
    return false;
  *(out + 1) = atan_value;

  // Get Hip Pitch and Ankle Pitch
  atan_value = atan2(transformation_ac.coeff(0, 2) * cos(*(out)) + transformation_ac.coeff(1, 2) * sin(*(out)),
                     transformation_ac.coeff(0, 0) * cos(*(out)) + transformation_ac.coeff(1, 0) * sin(*(out)));
  if (std::isinf(atan_value) == 1)
    return false;
  theta = atan_value;
  value_k = sin(*(out + 3)) * calf_length;
  value_l = -thigh_length - cos(*(out + 3)) * calf_length;
  value_m = cos(*(out)) * vector.x() + sin(*(out)) * vector.y();
  value_n = cos(*(out + 1)) * vector.z() + sin(*(out)) * sin(*(out + 1)) * vector.x()
      - cos(*(out)) * sin(*(out + 1)) * vector.y();
  value_s = (value_k * value_n + value_l * value_m) / (value_k * value_k + value_l * value_l);
  value_c = (value_n - value_k * value_s) / value_l;
  atan_value = atan2(value_s, value_c);
  if (std::isinf(atan_value) == 1)
    return false;
  *(out + 2) = atan_value;
  *(out + 4) = theta - *(out + 3) - *(out + 2);

  return true;
}

void WalkingModule::updateTimeParam()
{
  period_time_ = walking_param_.period_time;  // * 1000;   // s -> ms
  dsp_ratio_ = walking_param_.dsp_ratio;
  ssp_ratio_ = 1 - dsp_ratio_;

  x_swap_period_time_ = period_time_ / 2;
  x_move_period_time_ = period_time_ * ssp_ratio_;
  y_swap_period_time_ = period_time_;
  y_move_period_time_ = period_time_ * ssp_ratio_;
  z_swap_period_time_ = period_time_ / 2;
  z_move_period_time_ = period_time_ * ssp_ratio_ / 2;
  a_move_period_time_ = period_time_ * ssp_ratio_;

  ssp_time_ = period_time_ * ssp_ratio_;
  l_ssp_start_time_ = (1 - ssp_ratio_) * period_time_ / 4;
  l_ssp_end_time_ = (1 + ssp_ratio_) * period_time_ / 4;
  r_ssp_start_time_ = (3 - ssp_ratio_) * period_time_ / 4;
  r_ssp_end_time_ = (3 + ssp_ratio_) * period_time_ / 4;

  phase1_time_ = (l_ssp_start_time_ + l_ssp_end_time_) / 2;
  phase2_time_ = (l_ssp_end_time_ + r_ssp_start_time_) / 2;
  phase3_time_ = (r_ssp_start_time_ + r_ssp_end_time_) / 2;

  pelvis_offset_ = walking_param_.pelvis_offset;
  pelvis_swing_ = pelvis_offset_ * 0.35;
  arm_swing_gain_ = walking_param_.arm_swing_gain;
}

void WalkingModule::updateMovementParam()
{
  // Forward/Back
  x_move_amplitude_ = walking_param_.x_move_amplitude;
  x_swap_amplitude_ = walking_param_.x_move_amplitude * walking_param_.step_fb_ratio;

  if (previous_x_move_amplitude_ == 0)
  {
    x_move_amplitude_ *= 0.5;
    x_swap_amplitude_ *= 0.5;
  }

  // Right/Left
  y_move_amplitude_ = walking_param_.y_move_amplitude / 2;
  if (y_move_amplitude_ > 0)
    y_move_amplitude_shift_ = y_move_amplitude_;
  else
    y_move_amplitude_shift_ = -y_move_amplitude_;
  y_swap_amplitude_ = walking_param_.y_swap_amplitude + y_move_amplitude_shift_ * 0.04;

  z_move_amplitude_ = walking_param_.z_move_amplitude / 2;
  z_move_amplitude_shift_ = z_move_amplitude_ / 2;
  z_swap_amplitude_ = walking_param_.z_swap_amplitude;
  z_swap_amplitude_shift_ = z_swap_amplitude_;

  // Direction
  if (walking_param_.move_aim_on == false)
  {
    a_move_amplitude_ = walking_param_.angle_move_amplitude / 2;
    if (a_move_amplitude_ > 0)
      a_move_amplitude_shift_ = a_move_amplitude_;
    else
      a_move_amplitude_shift_ = -a_move_amplitude_;
  }
  else
  {
    a_move_amplitude_ = -walking_param_.angle_move_amplitude / 2;
    if (a_move_amplitude_ > 0)
      a_move_amplitude_shift_ = -a_move_amplitude_;
    else
      a_move_amplitude_shift_ = a_move_amplitude_;
  }
}

void WalkingModule::updatePoseParam()
{
  x_offset_ = walking_param_.init_x_offset;
  y_offset_ = walking_param_.init_y_offset;
  z_offset_ = walking_param_.init_z_offset;
  r_offset_ = walking_param_.init_roll_offset;
  p_offset_ = walking_param_.init_pitch_offset;
  a_offset_ = walking_param_.init_yaw_offset;
  hit_pitch_offset_ = walking_param_.hip_pitch_offset;
}

void WalkingModule::startWalking()
{
  ctrl_running_ = true;
  real_running_ = true;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start walking");
}

void WalkingModule::stop()
{
  ctrl_running_ = false;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Stop walking");
}

bool WalkingModule::isRunning()
{
  return real_running_ || (walking_state_ == WalkingInitPose);
}

// default [angle : radian, length : m]
void WalkingModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                            std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  const double time_unit = control_cycle_msec_ * 0.001;  // ms -> s
  int joint_size = result_.size();
  double angle[joint_size];
  double balance_angle[joint_size];

  for(int _idx = 0; _idx < joint_size; _idx++)
  {
    angle[_idx]         = 0.0f;
    balance_angle[_idx] = 0.0f;
  }

  if (walking_state_ == WalkingInitPose)
  {
    int total_count = calc_joint_tra_.rows();
    for (int id = 1; id <= result_.size(); id++)
      target_position_.coeffRef(0, id) = calc_joint_tra_(init_pose_count_, id);

    init_pose_count_ += 1;
    if (init_pose_count_ >= total_count)
    {
      walking_state_ = WalkingReady;
      if (DEBUG)
        std::cout << "End moving to Init : " << init_pose_count_ << std::endl;
    }

  }
  else if (walking_state_ == WalkingReady || walking_state_ == WalkingEnable)
  {
    // present angle
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
        state_iter != result_.end(); state_iter++)
    {
      std::string _joint_name = state_iter->first;
      int joint_index = joint_table_[_joint_name];

      robotis_framework::Dynamixel *dxl = NULL;
      std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(_joint_name);
      if (dxl_it != dxls.end())
        dxl = dxl_it->second;
      else
        continue;

      goal_position_.coeffRef(0, joint_index) = dxl->dxl_state_->goal_position_;
    }

    processPhase(time_unit);

    bool get_angle = false;
    get_angle = computeLegAngle(&angle[0]);

    computeArmAngle(&angle[12]);

    double rl_gyro_err = 0.0 - sensors["gyro_x"];
    double fb_gyro_err = 0.0 - sensors["gyro_y"];

    sensoryFeedback(rl_gyro_err, fb_gyro_err, balance_angle);

    double err_total = 0.0, err_max = 0.0;
    // set goal position
    for (int idx = 0; idx < 14; idx++)
    {
      double goal_position = 0.0;
      if (get_angle == false && idx < 12)
        goal_position = goal_position_.coeff(0, idx);
      else
        goal_position = init_position_.coeff(0, idx) + angle[idx] + balance_angle[idx];

      target_position_.coeffRef(0, idx) = goal_position;

      double err = fabs(target_position_.coeff(0, idx) - goal_position_.coeff(0, idx)) * RADIAN2DEGREE;
      if (err > err_max)
        err_max = err;
      err_total += err;
    }

    // Check Enable
    if (walking_state_ == WalkingEnable && err_total > 5.0)
    {
      if (DEBUG)
        std::cout << "Check Err : " << err_max << std::endl;

      // make trajecotry for init pose
      int mov_time = err_max / 30;
      iniPoseTraGene(mov_time < 1 ? 1 : mov_time);

      // set target to goal
      target_position_ = goal_position_;

      walking_state_ = WalkingInitPose;

      ROS_WARN_STREAM_COND(DEBUG, "x_offset: " << walking_param_.init_x_offset);
      ROS_WARN_STREAM_COND(DEBUG, "y_offset: " << walking_param_.init_y_offset);
      ROS_WARN_STREAM_COND(DEBUG, "z_offset: " << walking_param_.init_z_offset);
      ROS_WARN_STREAM_COND(DEBUG, "roll_offset: " << walking_param_.init_roll_offset * RADIAN2DEGREE);
      ROS_WARN_STREAM_COND(DEBUG, "pitch_offset: " << walking_param_.init_pitch_offset * RADIAN2DEGREE);
      ROS_WARN_STREAM_COND(DEBUG, "yaw_offset: " << walking_param_.init_yaw_offset * RADIAN2DEGREE);
      ROS_WARN_STREAM_COND(DEBUG, "hip_pitch_offset: " << walking_param_.hip_pitch_offset * RADIAN2DEGREE);
      ROS_WARN_STREAM_COND(DEBUG, "period_time: " << walking_param_.period_time * 1000);
      ROS_WARN_STREAM_COND(DEBUG, "dsp_ratio: " << walking_param_.dsp_ratio);
      ROS_WARN_STREAM_COND(DEBUG, "step_forward_back_ratio: " << walking_param_.step_fb_ratio);
      ROS_WARN_STREAM_COND(DEBUG, "foot_height: " << walking_param_.z_move_amplitude);
      ROS_WARN_STREAM_COND(DEBUG, "swing_right_left: " << walking_param_.y_swap_amplitude);
      ROS_WARN_STREAM_COND(DEBUG, "swing_top_down: " << walking_param_.z_swap_amplitude);
      ROS_WARN_STREAM_COND(DEBUG, "pelvis_offset: " << walking_param_.pelvis_offset * RADIAN2DEGREE);
      ROS_WARN_STREAM_COND(DEBUG, "arm_swing_gain: " << walking_param_.arm_swing_gain);
      ROS_WARN_STREAM_COND(DEBUG, "balance_hip_roll_gain: " << walking_param_.balance_hip_roll_gain);
      ROS_WARN_STREAM_COND(DEBUG, "balance_knee_gain: " << walking_param_.balance_knee_gain);
      ROS_WARN_STREAM_COND(DEBUG, "balance_ankle_roll_gain: " << walking_param_.balance_ankle_roll_gain);
      ROS_WARN_STREAM_COND(DEBUG, "balance_ankle_pitch_gain: " << walking_param_.balance_ankle_pitch_gain);
      ROS_WARN_STREAM_COND(DEBUG, "balance : " << (walking_param_.balance_enable ? "TRUE" : "FALSE"));
    }
    else
    {
      walking_state_ = WalkingReady;
    }
  }

  // set result
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
      state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int joint_index = joint_table_[joint_name];

    result_[joint_name]->goal_position_ = target_position_.coeff(0, joint_index);

    // Todo : setting PID gain to the leg joints
    // result_[joint_name]->position_p_gain_ = walking_param_.p_gain;
    // result_[joint_name]->position_i_gain_ = walking_param_.i_gain;
    // result_[joint_name]->position_d_gain_ = walking_param_.d_gain;
  }

  // time
  if (real_running_ == true)
  {
    time_ += time_unit;
    if (time_ >= period_time_)
    {
      time_ = 0;
      previous_x_move_amplitude_ = walking_param_.x_move_amplitude * 0.5;
    }
  }
}

void WalkingModule::processPhase(const double &time_unit)
{
  // Update walk parameters
  if (time_ == 0)
  {
    updateTimeParam();
    phase_ = PHASE0;
    if (ctrl_running_ == false)
    {
      if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
      {
        real_running_ = false;
      }
      else
      {
        // set walking param to init
        walking_param_.x_move_amplitude = 0;
        walking_param_.y_move_amplitude = 0;
        walking_param_.angle_move_amplitude = 0;

        previous_x_move_amplitude_ = 0;
      }
    }
  }
  else if (time_ >= (phase1_time_ - time_unit / 2) && time_ < (phase1_time_ + time_unit / 2))  // the position of left foot is the highest.
  {
    updateMovementParam();
    phase_ = PHASE1;
  }
  else if (time_ >= (phase2_time_ - time_unit / 2) && time_ < (phase2_time_ + time_unit / 2))  // middle of double support state
  {
    updateTimeParam();

    time_ = phase2_time_;
    phase_ = PHASE2;
    if (ctrl_running_ == false)
    {
      if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
      {
        real_running_ = false;
      }
      else
      {
        // set walking param to init
        walking_param_.x_move_amplitude = previous_x_move_amplitude_;
        walking_param_.y_move_amplitude = 0;
        walking_param_.angle_move_amplitude = 0;
      }
    }
  }
  else if (time_ >= (phase3_time_ - time_unit / 2) && time_ < (phase3_time_ + time_unit / 2))  // the position of right foot is the highest.
  {
    updateMovementParam();
    phase_ = PHASE3;
  }
}

bool WalkingModule::computeLegAngle(double *leg_angle)
{
  Pose3D swap, right_leg_move, left_leg_move;
  double pelvis_offset_r, pelvis_offset_l;
  double ep[12];

  updatePoseParam();

  // Compute endpoints
  swap.x = wSin(time_, x_swap_period_time_, x_swap_phase_shift_, x_swap_amplitude_, x_swap_amplitude_shift_);
  swap.y = wSin(time_, y_swap_period_time_, y_swap_phase_shift_, y_swap_amplitude_, y_swap_amplitude_shift_);
  swap.z = wSin(time_, z_swap_period_time_, z_swap_phase_shift_, z_swap_amplitude_, z_swap_amplitude_shift_);
  swap.roll = 0.0;
  swap.pitch = 0.0;
  swap.yaw = 0.0;

  if (time_ <= l_ssp_start_time_)
  {
    left_leg_move.x = wSin(l_ssp_start_time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
                           x_move_amplitude_shift_);
    left_leg_move.y = wSin(l_ssp_start_time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
                           y_move_amplitude_shift_);
    left_leg_move.z = wSin(l_ssp_start_time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                           z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(l_ssp_start_time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(l_ssp_start_time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(l_ssp_start_time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(l_ssp_start_time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }
  else if (time_ <= l_ssp_end_time_)
  {
    left_leg_move.x = wSin(time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
                           x_move_amplitude_shift_);
    left_leg_move.y = wSin(time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
                           y_move_amplitude_shift_);
    left_leg_move.z = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                           z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, pelvis_swing_ / 2,
                           pelvis_swing_ / 2);
    pelvis_offset_r = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_,
                           -pelvis_offset_ / 2, -pelvis_offset_ / 2);
  }
  else if (time_ <= r_ssp_start_time_)
  {
    left_leg_move.x = wSin(l_ssp_end_time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
                           x_move_amplitude_shift_);
    left_leg_move.y = wSin(l_ssp_end_time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
                           y_move_amplitude_shift_);
    left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                           z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(l_ssp_end_time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(l_ssp_end_time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(l_ssp_end_time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(l_ssp_end_time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }
  else if (time_ <= r_ssp_end_time_)
  {
    left_leg_move.x = wSin(time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                           x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y = wSin(time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                           y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                           z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, pelvis_offset_ / 2,
                           pelvis_offset_ / 2);
    pelvis_offset_r = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, -pelvis_swing_ / 2,
                           -pelvis_swing_ / 2);
  }
  else
  {
    left_leg_move.x = wSin(r_ssp_end_time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                           x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y = wSin(r_ssp_end_time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                           y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                           z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(r_ssp_end_time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(r_ssp_end_time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(r_ssp_end_time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(r_ssp_end_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(r_ssp_end_time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }

  left_leg_move.roll = 0;
  left_leg_move.pitch = 0;
  right_leg_move.roll = 0;
  right_leg_move.pitch = 0;

  double leg_length = op2_kd_->thigh_length_m_ + op2_kd_->calf_length_m_ + op2_kd_->ankle_length_m_;

  // mm, rad
  ep[0] = swap.x + right_leg_move.x + x_offset_;
  ep[1] = swap.y + right_leg_move.y - y_offset_ / 2;
  ep[2] = swap.z + right_leg_move.z + z_offset_ - leg_length;
  ep[3] = swap.roll + right_leg_move.roll - r_offset_ / 2;
  ep[4] = swap.pitch + right_leg_move.pitch + p_offset_;
  ep[5] = swap.yaw + right_leg_move.yaw - a_offset_ / 2;
  ep[6] = swap.x + left_leg_move.x + x_offset_;
  ep[7] = swap.y + left_leg_move.y + y_offset_ / 2;
  ep[8] = swap.z + left_leg_move.z + z_offset_ - leg_length;
  ep[9] = swap.roll + left_leg_move.roll + r_offset_ / 2;
  ep[10] = swap.pitch + left_leg_move.pitch + p_offset_;
  ep[11] = swap.yaw + left_leg_move.yaw + a_offset_ / 2;

  // Compute body swing
  if (time_ <= l_ssp_end_time_)
  {
    body_swing_y = -ep[7];
    body_swing_z = ep[8];
  }
  else
  {
    body_swing_y = -ep[1];
    body_swing_z = ep[2];
  }
  body_swing_z -= leg_length;

  // right leg
  if (op2_kd_->calcInverseKinematicsForRightLeg(&leg_angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == false)
  {
    printf("IK not Solved EPR : %f %f %f %f %f %f\n", ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]);
    return false;
  }

  if (op2_kd_->calcInverseKinematicsForLeftLeg(&leg_angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == false)
  {
    printf("IK not Solved EPL : %f %f %f %f %f %f\n", ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]);
    return false;
  }

  // Compute dxls angle
  for (int i = 0; i < 12; i++)
  {
    // offset : rad
    double offset = 0;

    if (i == joint_table_["r_hip_roll"])  // R_HIP_ROLL
      offset += op2_kd_->getJointDirection("r_hip_roll") * pelvis_offset_r;
    else if (i == joint_table_["l_hip_roll"])  // L_HIP_ROLL
      offset += op2_kd_->getJointDirection("l_hip_roll") * pelvis_offset_l;
    else if (i == joint_table_["r_hip_pitch"])
      offset -= op2_kd_->getJointDirection("r_hip_pitch") * hit_pitch_offset_;
    else if (i == joint_table_["l_hip_pitch"])  // R_HIP_PITCH or L_HIP_PITCH
      offset -= op2_kd_->getJointDirection("l_hip_pitch") * hit_pitch_offset_;

    leg_angle[i] += offset;
  }

  return true;
}

void WalkingModule::computeArmAngle(double *arm_angle)
{
  // Compute arm swing
  if (x_move_amplitude_ == 0)
  {
    arm_angle[0] = 0;  // Right
    arm_angle[1] = 0;  // Left
  }
  else
  {
    arm_angle[0] = wSin(time_, period_time_, M_PI * 1.5, -x_move_amplitude_ * arm_swing_gain_ * 1000,
                        0) * op2_kd_->getJointDirection("r_sho_pitch") * DEGREE2RADIAN;
    arm_angle[1] = wSin(time_, period_time_, M_PI * 1.5, x_move_amplitude_ * arm_swing_gain_ * 1000,
                        0) * op2_kd_->getJointDirection("l_sho_pitch") * DEGREE2RADIAN;
  }
}

void WalkingModule::sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle)
{
  // adjust balance offset
  if (walking_param_.balance_enable == false)
    return;

  double internal_gain = -0.3;

  balance_angle[joint_table_["r_hip_roll"]] =  op2_kd_->getJointDirection("r_hip_roll") * internal_gain
      * rlGyroErr * walking_param_.balance_hip_roll_gain;  // R_HIP_ROLL
  balance_angle[joint_table_["l_hip_roll"]] =  op2_kd_->getJointDirection("l_hip_roll") * internal_gain
      * rlGyroErr * walking_param_.balance_hip_roll_gain;  // L_HIP_ROLL

  balance_angle[joint_table_["r_knee"]] = - op2_kd_->getJointDirection("r_knee") * internal_gain
      * fbGyroErr * walking_param_.balance_knee_gain;  // R_KNEE
  balance_angle[joint_table_["l_knee"]] = - op2_kd_->getJointDirection("l_knee") * internal_gain
      * fbGyroErr * walking_param_.balance_knee_gain;  // L_KNEE

  balance_angle[joint_table_["r_ank_pitch"]] = - op2_kd_->getJointDirection("r_ank_pitch")
      * internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain;  // R_ANKLE_PITCH
  balance_angle[joint_table_["l_ank_pitch"]] = - op2_kd_->getJointDirection("l_ank_pitch")
      * internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain;  // L_ANKLE_PITCH

  balance_angle[joint_table_["r_ank_roll"]] = - op2_kd_->getJointDirection("r_ank_roll") * internal_gain
      * rlGyroErr * walking_param_.balance_ankle_roll_gain;  // R_ANKLE_ROLL
  balance_angle[joint_table_["l_ank_roll"]] = - op2_kd_->getJointDirection("l_ank_roll") * internal_gain
      * rlGyroErr * walking_param_.balance_ankle_roll_gain;  // L_ANKLE_ROLL

}

void WalkingModule::loadWalkingParam(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // parse movement time
  walking_param_.init_x_offset = doc["x_offset"].as<double>();
  walking_param_.init_y_offset = doc["y_offset"].as<double>();
  walking_param_.init_z_offset = doc["z_offset"].as<double>();
  walking_param_.init_roll_offset = doc["roll_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.init_pitch_offset = doc["pitch_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.init_yaw_offset = doc["yaw_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.hip_pitch_offset = doc["hip_pitch_offset"].as<double>() * DEGREE2RADIAN;
  // time
  walking_param_.period_time = doc["period_time"].as<double>() * 0.001;    // ms -> s
  walking_param_.dsp_ratio = doc["dsp_ratio"].as<double>();
  walking_param_.step_fb_ratio = doc["step_forward_back_ratio"].as<double>();
  // walking
  // walking_param_.x_move_amplitude
  // walking_param_.y_move_amplitude
  walking_param_.z_move_amplitude = doc["foot_height"].as<double>();
  // walking_param_.angle_move_amplitude
  // walking_param_.move_aim_on

  // balance
  // walking_param_.balance_enable
  walking_param_.balance_hip_roll_gain = doc["balance_hip_roll_gain"].as<double>();
  walking_param_.balance_knee_gain = doc["balance_knee_gain"].as<double>();
  walking_param_.balance_ankle_roll_gain = doc["balance_ankle_roll_gain"].as<double>();
  walking_param_.balance_ankle_pitch_gain = doc["balance_ankle_pitch_gain"].as<double>();
  walking_param_.y_swap_amplitude = doc["swing_right_left"].as<double>();
  walking_param_.z_swap_amplitude = doc["swing_top_down"].as<double>();
  walking_param_.pelvis_offset = doc["pelvis_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.arm_swing_gain = doc["arm_swing_gain"].as<double>();

  // gain
  walking_param_.p_gain = doc["p_gain"].as<int>();
  walking_param_.i_gain = doc["i_gain"].as<int>();
  walking_param_.d_gain = doc["d_gain"].as<int>();
}

void WalkingModule::saveWalkingParam(std::string &path)
{
  YAML::Emitter out_emitter;

  out_emitter << YAML::BeginMap;
  out_emitter << YAML::Key << "x_offset" << YAML::Value << walking_param_.init_x_offset;
  out_emitter << YAML::Key << "y_offset" << YAML::Value << walking_param_.init_y_offset;
  out_emitter << YAML::Key << "z_offset" << YAML::Value << walking_param_.init_z_offset;
  out_emitter << YAML::Key << "roll_offset" << YAML::Value << walking_param_.init_roll_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "pitch_offset" << YAML::Value << walking_param_.init_pitch_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "yaw_offset" << YAML::Value << walking_param_.init_yaw_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "hip_pitch_offset" << YAML::Value << walking_param_.hip_pitch_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "period_time" << YAML::Value << walking_param_.period_time * 1000;
  out_emitter << YAML::Key << "dsp_ratio" << YAML::Value << walking_param_.dsp_ratio;
  out_emitter << YAML::Key << "step_forward_back_ratio" << YAML::Value << walking_param_.step_fb_ratio;
  out_emitter << YAML::Key << "foot_height" << YAML::Value << walking_param_.z_move_amplitude;
  out_emitter << YAML::Key << "swing_right_left" << YAML::Value << walking_param_.y_swap_amplitude;
  out_emitter << YAML::Key << "swing_top_down" << YAML::Value << walking_param_.z_swap_amplitude;
  out_emitter << YAML::Key << "pelvis_offset" << YAML::Value << walking_param_.pelvis_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "arm_swing_gain" << YAML::Value << walking_param_.arm_swing_gain;
  out_emitter << YAML::Key << "balance_hip_roll_gain" << YAML::Value << walking_param_.balance_hip_roll_gain;
  out_emitter << YAML::Key << "balance_knee_gain" << YAML::Value << walking_param_.balance_knee_gain;
  out_emitter << YAML::Key << "balance_ankle_roll_gain" << YAML::Value << walking_param_.balance_ankle_roll_gain;
  out_emitter << YAML::Key << "balance_ankle_pitch_gain" << YAML::Value << walking_param_.balance_ankle_pitch_gain;

  out_emitter << YAML::Key << "p_gain" << YAML::Value << walking_param_.p_gain;
  out_emitter << YAML::Key << "i_gain" << YAML::Value << walking_param_.i_gain;
  out_emitter << YAML::Key << "d_gain" << YAML::Value << walking_param_.d_gain;
  out_emitter << YAML::EndMap;

  // output to file
  std::ofstream fout(path.c_str());
  fout << out_emitter.c_str();
}

void WalkingModule::onModuleEnable()
{
  walking_state_ = WalkingEnable;
  ROS_INFO("Walking Enable");
}

void WalkingModule::onModuleDisable()
{
  ROS_INFO("Walking Disable");
  walking_state_ = WalkingDisable;
}

void WalkingModule::iniPoseTraGene(double mov_time)
{
  double smp_time = control_cycle_msec_ * 0.001;
  int all_time_steps = int(mov_time / smp_time) + 1;
  calc_joint_tra_.resize(all_time_steps, result_.size() + 1);

  for (int id = 0; id <= result_.size(); id++)
  {
    double ini_value = goal_position_.coeff(0, id);
    double tar_value = target_position_.coeff(0, id);

    Eigen::MatrixXd tra;

    tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0, smp_time, mov_time);

    calc_joint_tra_.block(0, id, all_time_steps, 1) = tra;
  }

  if(DEBUG)
    std::cout << "Generate Trajecotry : " << mov_time << "s [" << all_time_steps << "]" << std::endl;

  init_pose_count_ = 0;
}
}

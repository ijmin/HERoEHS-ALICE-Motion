/*
 * alice_upper_body_module.cpp
 *
 *  Created on: May 30, 2018
 *      Author: robotemperor
 */
#include "alice_upper_body_module/alice_upper_body_module.h"

using namespace alice_upper_body_module;

void UpperBodyModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&UpperBodyModule::queueThread, this));

  for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
      it != robot->dxls_.end(); it++)
  {
    std::string joint_name = it->first;
    robotis_framework::Dynamixel* dxl_info = it->second;

    joint_name_to_id_[joint_name] = dxl_info->id_;
    joint_id_to_name_[dxl_info->id_] = joint_name;
  }

  // waist yaw
  waist_end_point_.resize(6,8);
  waist_end_point_.fill(0);
  result_rad_waist_.resize(6,1);
  result_rad_waist_.fill(0);

  // head
  head_end_point_.resize(6,8);
  head_end_point_.fill(0);
  result_rad_head_.resize(6,1);
  result_rad_head_.fill(0);

  head_end_point_(4,0) = 20*DEGREE2RADIAN; // pitch 초기값
  head_end_point_(4,1) = 20*DEGREE2RADIAN; //
  end_to_rad_head_->cal_end_point_tra_betta->current_pose = 20*DEGREE2RADIAN;
  end_to_rad_head_->current_pose_change(4,0) = 20*DEGREE2RADIAN;

  //arm
  l_shoulder_pitch_goal = 0;
  r_shoulder_pitch_goal = 0;
  /*
  l_shoulder_roll_goal = 0;
  r_shoulder_roll_goal = 0;
  l_elbow_pitch_goal = 0;
  r_elbow_pitch_goal = 0;
   */
  head_yaw_goal = 0;
  head_pitch_goal = 0;

  l_shoulder_pitch_trj -> initial_pose = 0;
  r_shoulder_pitch_trj -> initial_pose = 0;
  /*
  l_shoulder_roll_trj -> initial_pose = 0;
  r_shoulder_roll_trj -> initial_pose = 0;
  l_elbow_pitch_trj -> initial_pose = -90*DEGREE2RADIAN;
  r_elbow_pitch_trj -> initial_pose = 90*DEGREE2RADIAN;
   */

  head_yaw_trj -> initial_pose = 0;
  head_pitch_trj -> initial_pose = 20*DEGREE2RADIAN;


  l_shoulder_pitch_trj -> current_pose = 0;
  r_shoulder_pitch_trj -> current_pose = 0;
  /*
  l_shoulder_roll_trj -> current_pose = 0;
  r_shoulder_roll_trj -> current_pose = 0;
  l_elbow_pitch_trj -> current_pose = -90*DEGREE2RADIAN;
  r_elbow_pitch_trj -> current_pose = 90*DEGREE2RADIAN;
   */
  head_yaw_trj -> current_pose = 0;
  head_pitch_trj -> current_pose = 0;

  for(int joint_num_= 3; joint_num_< 6 ; joint_num_ ++)  // waist 3, 5번 // head 345 초기화
  {
    waist_end_point_(joint_num_, 7) = 3.0;
    head_end_point_ (joint_num_, 7) = 3.0;
  }


  std::string arm_path = ros::package::getPath("alice_upper_body_module") + "/data/upper_body_arm_"+alice_id_+".yaml";
  parse_motion_data("arm",arm_path);
  std::string head_path = ros::package::getPath("alice_upper_body_module") + "/data/upper_body_head_"+alice_id_+".yaml";
  parse_motion_data("head",head_path);

  ROS_INFO("< -------  Initialize Module : Upper Body Module  [HEAD  && WAIST] !!  ------->");
}
double UpperBodyModule::limitCheck(double calculated_value, double max, double min)
{
  if(calculated_value >= (max*DEGREE2RADIAN))
    return (max*DEGREE2RADIAN);
  else if (calculated_value <= (min*DEGREE2RADIAN))
    return (min*DEGREE2RADIAN);
  else
    return calculated_value;
}
bool UpperBodyModule::isRunning()
{
  return running_;
}
void UpperBodyModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
    std::map<std::string, double> sensors)
{
  if (enable_ == false)
  {
    return;
  }

  //head_motion();
  // 18 / 06 / 15
  if(ball_detected == 0)
  {
    current_x = desired_x;
    current_y = desired_y;
  }
  //

  algorithm_process(command);

  //  printf("-------------------\n\n");
  //  printf("RESULT YAW   ::  %f \n\n",result_rad_head_(3,0)*RADIAN2DEGREE);
  //  printf("RESULT PITCH ::  %f \n\n",result_rad_head_(4,0)*RADIAN2DEGREE);
  //  printf("-------------------\n\n");

  //std::cout << "---------------" << std::endl;
  //std::cout << result_rad_head_ << std::endl;
  head_end_point_(3,1) = limitCheck(head_end_point_(3,1),60,-60);
  head_end_point_(4,1) = limitCheck(head_end_point_(4,1),75,0);

  result_rad_head_  = end_to_rad_head_  -> cal_end_point_to_rad(head_end_point_);


  waist_end_point_(3,1)   = limitCheck(waist_end_point_(3,1),60,-60);

  result_rad_waist_ = end_to_rad_waist_ -> cal_end_point_to_rad(waist_end_point_);

  //result_[joint_id_to_name_[7]]-> goal_position_  =  filter_head->lowPassFilter(temp_head_pitch, temp_pre_pitch, 0.01, 0.008);
  //result_[joint_id_to_name_[8]]-> goal_position_  =  filter_head->lowPassFilter(temp_head_yaw, temp_pre_yaw, 0.01, 0.008);

  result_rad_head_(3,0) =  limitCheck(result_rad_head_(3,0),60,-60);  //yaw
  result_rad_head_(4,0) =  limitCheck(result_rad_head_(4,0),75,0);    //pitch


  if (isnan(result_rad_head_(4,0)) || isnan(result_rad_head_(3,0)))
  {
    printf("NaN 입니다\n");
    ball_detected = 0;
  }
  else
  {
    if(alice_id_ =="2")
    {
      arm_motion();
      result_[joint_id_to_name_[9]] -> goal_position_  = result_rad_waist_ (3,0); // waist yaw
    }


    result_[joint_id_to_name_[1]]-> goal_position_  =  l_shoulder_pitch_goal; // l_shoulder_pitch
    result_[joint_id_to_name_[2]]-> goal_position_  =  r_shoulder_pitch_goal; // r_shoulder_pitch
    //result_[joint_id_to_name_[3]]-> goal_position_  =  l_shoulder_roll_goal; // l_shoulder_roll
    //result_[joint_id_to_name_[4]]-> goal_position_  =  r_shoulder_roll_goal; // r_shoulder_roll
    //result_[joint_id_to_name_[5]]-> goal_position_  =  l_elbow_pitch_goal; // l_elbow_pitch
    //result_[joint_id_to_name_[6]]-> goal_position_  =  r_elbow_pitch_goal; // r_elbow_pitch

    //ROS_INFO("HEAD PITCH :%f   | %d",alice_id_biased_*result_rad_head_(4,0),alice_id_biased_);
    result_[joint_id_to_name_[7]]-> goal_position_  =  result_rad_head_(4,0);
    result_[joint_id_to_name_[8]]-> goal_position_  =  result_rad_head_(3,0);
  }

  /*	temp_pre_roll  = temp_head_roll;
	temp_pre_pitch = temp_head_pitch;
	temp_pre_yaw   = temp_head_yaw;*/

  ball_detected = 0;
  //	logSaveFile();
}
void UpperBodyModule::stop()
{
  return;
}

// algorithm
void UpperBodyModule::updateBalanceParameter()
{
  if(balance_update_ == false)
    return;
  balance_updating_sys_time_sec_ += 0.008;

  if(balance_updating_sys_time_sec_ > balance_updating_duration_sec_)
  {
    balance_updating_sys_time_sec_ = 0;
    balance_update_ = false;
  }
  else
  {
    Eigen::MatrixXd value;
    value.resize(1,8);
    value.fill(0);
    value(0,7) = balance_updating_duration_sec_;
    value(0,1) = x_p_gain;
    pidController_x->kp_ = gain_x_p_adjustment -> fifth_order_traj_gen_one_value(value);
    value(0,1) = y_p_gain;
    pidController_y->kp_ = gain_y_p_adjustment -> fifth_order_traj_gen_one_value(value);
    value(0,1) = x_d_gain;
    pidController_x->kd_ = gain_x_d_adjustment -> fifth_order_traj_gen_one_value(value);
    value(0,1) = y_d_gain;
    pidController_y->kd_ = gain_y_d_adjustment -> fifth_order_traj_gen_one_value(value);

  }
}

void UpperBodyModule::scanning_motion()
{
  ROS_INFO("SCAN MOTION : %d  |  %f",motion_num_scanning,head_scan_time_data_[motion_num_scanning-1]);

  if(motion_num_scanning > head_scan_motion_data_.size()+1)
    return;
  if (current_time_scanning <= head_scan_time_data_[motion_num_scanning-1] )
  {
    waist_end_point_(3,7) = head_scan_time_data_[motion_num_scanning-1];
    head_end_point_(3,7)  = head_scan_time_data_[motion_num_scanning-1];
    head_end_point_(4,7)  = head_scan_time_data_[motion_num_scanning-1];

    waist_end_point_(3, 1) = head_scan_motion_data_[motion_num_scanning][2]*DEGREE2RADIAN;
    head_end_point_(3, 1)  = head_scan_motion_data_[motion_num_scanning][0]*DEGREE2RADIAN;
    head_end_point_(4, 1)  = head_scan_motion_data_[motion_num_scanning][1]*DEGREE2RADIAN;
  }
  else
  {
    motion_num_scanning++;

    if(motion_num_scanning > head_scan_motion_data_.size()+1)
    {
      scan_done_msg.data = true;
      scan_done_pub.publish(scan_done_msg);
      //motion_num_scanning=1;
      // current_time_scanning=0;
    }
    current_time_scanning=0;
  }
  /*	if(leg_check == 1)
		waist_end_point_(3, 1) = 0;*/

  current_time_scanning = current_time_scanning + 0.008;
}
void UpperBodyModule::finding_motion()
{
  ROS_INFO("FIND MOTION : %d  |  %f",motion_num_scanning,head_scan_time_data_[motion_num_scanning-1]);

 if(motion_num_finding > head_find_motion_data_.size())
  {
    motion_num_finding=1;
    current_time_finding=0;
  }
  if (current_time_finding <= head_find_time_data_[motion_num_finding-1] )
  {
    waist_end_point_(3,7) = head_find_time_data_[motion_num_finding-1];
    head_end_point_(3,7)  = head_find_time_data_[motion_num_finding-1];
    head_end_point_(4,7)  = head_find_time_data_[motion_num_finding-1];

    waist_end_point_(3, 1) = head_find_motion_data_[motion_num_finding][2]*DEGREE2RADIAN;
    head_end_point_(3, 1)  = head_find_motion_data_[motion_num_finding][0]*DEGREE2RADIAN;
    head_end_point_(4, 1)  = head_find_motion_data_[motion_num_finding][1]*DEGREE2RADIAN;
  }
  else
  {
    motion_num_finding++;
    if(motion_num_finding > head_find_motion_data_.size())
    {
      motion_num_finding=1;
    }
    current_time_finding=0;
  }

  if(leg_check == 1)
    waist_end_point_(3, 1) = 0;

  current_time_finding = current_time_finding + 0.008;
}

void UpperBodyModule::tracking_function()
{
  //current_x = filter_head->lowPassFilter(current_x, pre_current_x, 0, 0.008);
  //current_y = filter_head->lowPassFilter(current_y, pre_current_y, 0, 0.008);
  //printf("X   current ::  %f  desired :: %f \n",current_x,desired_x);
  //printf("Y   current ::  %f  desired :: %f\n",current_y,desired_y);

  updateBalanceParameter();

  control_angle_yaw_temp   = pidController_x->PID_calculate(desired_x, current_x);
  control_angle_pitch_temp = pidController_y->PID_calculate(desired_y, current_y);

  if(fabs(desired_x - current_x) < 25)
  {
    control_angle_yaw_temp = 0;
  }
  if(fabs(desired_y - current_y) < 15)
  {
    control_angle_pitch_temp = 0;
  }

  //printf("X   control value ::  %f \n", control_angle_yaw_temp);
  //printf("Y   control value ::  %f \n", control_angle_pitch_temp);

  control_angle_yaw   = control_angle_yaw + control_angle_yaw_temp;
  control_angle_pitch = control_angle_pitch - control_angle_pitch_temp;

  control_angle_yaw = limitCheck(control_angle_yaw,60,-60);
  control_angle_pitch = limitCheck(control_angle_pitch,75,0);

  head_end_point_(3, 1)  = control_angle_yaw;
  head_end_point_(4, 1)  = control_angle_pitch;

  //head_end_point_(3, 7)  = 0.8;
  //head_end_point_(4, 7)  = 0.8;

  //printf("***************\n\n");
  //printf("YAW  control value ::  %f \n\n",control_angle_yaw*RADIAN2DEGREE);
  //printf("PITCH control value ::  %f \n\n",control_angle_pitch*RADIAN2DEGREE);
  //printf("**************\n\n");
  //pre_current_x = current_x;
  //pre_current_y = current_y;

}

void UpperBodyModule::algorithm_process(uint8_t command_)
{

  if(command_ == 0)
  {
    waist_end_point_(3,7)  = 3.0;
    head_end_point_(3,7)   = 3.0;
    head_end_point_(4,7)   = 3.0;
    waist_end_point_(3, 1) = 0; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
    //waist_end_point_(4, 1) = 0; // pitch

    head_end_point_(3, 1)  = 0; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
    head_end_point_(4, 1)  = -20*DEGREE2RADIAN; // pitch

    //18 06 15
    current_time_scanning = 0.0;
    motion_num_scanning = 1;
    current_time_finding = 0;
    motion_num_finding = 1;

    control_angle_yaw  = end_to_rad_head_  ->cal_end_point_tra_alpha->current_pose;
    control_angle_pitch = end_to_rad_head_  ->cal_end_point_tra_betta->current_pose;
    //
  }
  else if(command_ == 1)// scanning algorithm
  {
    head_end_point_(3,7) = 2.0;
    head_end_point_(4,7) = 2.0;

    current_time_finding = 0;
    motion_num_finding = 1;


    control_angle_yaw  = end_to_rad_head_  ->cal_end_point_tra_alpha->current_pose;
    control_angle_pitch = end_to_rad_head_  ->cal_end_point_tra_betta->current_pose;

    scanning_motion();
  }
  else if(command_ == 2)// finding algorithm
  {
    ////
    waist_end_point_(3, 7)  = 3.0;
    head_end_point_(3, 7)   = 3.0;
    head_end_point_(4, 7)   = 3.0;

    current_time_scanning = 0.0;
    motion_num_scanning = 1;

    control_angle_yaw  = end_to_rad_head_  ->cal_end_point_tra_alpha->current_pose;
    control_angle_pitch = end_to_rad_head_  ->cal_end_point_tra_betta->current_pose;
    ////
    finding_motion();

  }
  else if(command_ == 3)// tracking algorithm
  {

    head_end_point_(3, 7)   = 0.2;
    head_end_point_(4, 7)   = 0.2;

    waist_end_point_(3, 1)  = 0; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임

    current_time_scanning = 0.0;
    motion_num_scanning = 1;
    current_time_finding = 0;
    motion_num_finding = 1;

    tracking_function();
  }
  else if(command_ == 4)//
  {
    waist_end_point_(3, 7)  = 3.0;
    head_end_point_(3, 7)   = 3.0;
    head_end_point_(4, 7)   = 3.0;

    current_time_scanning = 0.0;
    motion_num_scanning = 1;
    current_time_finding = 0;
    motion_num_finding = 1;

    waist_end_point_(3, 1) = 0; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
    head_end_point_(3, 1)  = 0; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
    head_end_point_(4, 1)  = 75*DEGREE2RADIAN;

    control_angle_yaw   = end_to_rad_head_  ->cal_end_point_tra_alpha->current_pose;
    control_angle_pitch = end_to_rad_head_  ->cal_end_point_tra_betta->current_pose;
  }
  else
    return;

}

void UpperBodyModule::parse_motion_data(const std::string &type, const std::string &path)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path.c_str());

  }catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file!");
    return;
  }

  if(type == "arm")
  {
    motion_joint_data_ = doc["link"].as<std::vector<std::string> >();
    motion_time_data_ = doc["motion_time"].as<std::vector<double> >();
    YAML::Node motion_pose_node = doc["motion"];


    for(YAML::iterator it = motion_pose_node.begin(); it != motion_pose_node.end(); ++it)
    {
      int motion_numb = it->first.as<int>();

      motion_numb_to_joint_pose_data_[motion_numb]=motion_pose_node[motion_numb].as<std::vector<double> >();
    }
  }

  else if(type == "head")
  {
    YAML::Node head_motion_node;
    YAML::Node head_motion_joint_node;
    head_joint_data_ = doc["link"].as<std::vector<std::string> >();

    head_motion_node = doc["find"];
    head_find_time_data_ = head_motion_node["motion_time"].as<std::vector<double> >();
    head_motion_joint_node = head_motion_node["motion"];
    for(YAML::iterator it = head_motion_joint_node.begin(); it != head_motion_joint_node.end(); ++it)
    {
      int head_numb = it->first.as<int>();
      head_find_motion_data_[head_numb]=head_motion_joint_node[head_numb].as<std::vector<double> >();
    }

    head_motion_node = doc["scan"];
    head_scan_time_data_ = head_motion_node["motion_time"].as<std::vector<double> >();
    head_motion_joint_node = head_motion_node["motion"];
    for(YAML::iterator it = head_motion_joint_node.begin(); it != head_motion_joint_node.end(); ++it)
    {
      int head_numb = it->first.as<int>();
      head_scan_motion_data_[head_numb]=head_motion_joint_node[head_numb].as<std::vector<double> >();
    }
  }
}
void UpperBodyModule::arm_motion()
{
  Eigen::MatrixXd value;
  value.resize(1,8);
  value.fill(0);

  if (current_time_arm_motion <= motion_time_data_[motion_num_arm-1] )
  {
    value(0,7) = motion_time_data_[motion_num_arm-1];
    value(0,1) = motion_numb_to_joint_pose_data_[motion_num_arm][0]*DEGREE2RADIAN;
    l_shoulder_pitch_goal = l_shoulder_pitch_trj -> fifth_order_traj_gen_one_value(value);

    value(0,7) = motion_time_data_[motion_num_arm-1];
    value(0,1) = motion_numb_to_joint_pose_data_[motion_num_arm][1]*DEGREE2RADIAN;
    r_shoulder_pitch_goal = r_shoulder_pitch_trj -> fifth_order_traj_gen_one_value(value);

    value(0,7) = motion_time_data_[motion_num_arm-1];
    value(0,1) = motion_numb_to_joint_pose_data_[motion_num_arm][2]*DEGREE2RADIAN;
    l_shoulder_roll_goal = l_shoulder_roll_trj -> fifth_order_traj_gen_one_value(value);

    value(0,7) = motion_time_data_[motion_num_arm-1];
    value(0,1) = motion_numb_to_joint_pose_data_[motion_num_arm][3]*DEGREE2RADIAN;
    r_shoulder_roll_goal = r_shoulder_roll_trj -> fifth_order_traj_gen_one_value(value);

    value(0,7) = motion_time_data_[motion_num_arm-1];
    value(0,1) = motion_numb_to_joint_pose_data_[motion_num_arm][4]*DEGREE2RADIAN;
    l_elbow_pitch_goal = l_elbow_pitch_trj -> fifth_order_traj_gen_one_value(value);

    value(0,7) = motion_time_data_[motion_num_arm-1];
    value(0,1) = motion_numb_to_joint_pose_data_[motion_num_arm][5]*DEGREE2RADIAN;
    r_elbow_pitch_goal = r_elbow_pitch_trj -> fifth_order_traj_gen_one_value(value);

  }
  else
  {
    motion_num_arm++;
    if(motion_num_arm> motion_numb_to_joint_pose_data_.size()+1)
    {
      motion_num_arm=1;
    }
    current_time_arm_motion=0;
  }
  current_time_arm_motion = current_time_arm_motion + 0.008;
}







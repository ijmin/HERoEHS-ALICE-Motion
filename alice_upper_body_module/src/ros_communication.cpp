/*
 * ros_communication.cpp
 *
 *  Created on: May 30, 2018
 *      Author: robotemperor
 */

#include "alice_upper_body_module/alice_upper_body_module.h"

using namespace alice_upper_body_module;

UpperBodyModule::UpperBodyModule()
: control_cycle_msec_(8)
{
	running_ = false;
	gazebo_check = false;
	is_moving_head_ = false;
	is_moving_waist_ = false;

	enable_       = false;
	module_name_  = "upper_body_module";
	control_mode_ = robotis_framework::PositionControl;
	readIDData();
	// Dynamixel initialize ////
	if(alice_id_ == "2")
	{
	  result_["waist_yaw"] = new robotis_framework::DynamixelState(); // joint 9
	}
	result_["head_pitch"]   = new robotis_framework::DynamixelState(); // joint 7
	result_["head_yaw"]   = new robotis_framework::DynamixelState(); // joint 8

	result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
	result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2

  result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3
  result_["r_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 4
  result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5
  result_["r_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 6

	///////////////////////////
	// motion control variables
	waist_kinematics_ = new heroehs_math::KinematicsEulerAngle;
	end_to_rad_waist_ = new heroehs_math::CalRad;

	head_kinematics_  = new heroehs_math::KinematicsEulerAngle;
	head_point_kinematics_ = new heroehs_math::Kinematics;
	end_to_rad_head_  = new heroehs_math::CalRad;

	traj_time_test = 4;
	new_count_ = 1 ;


	end_to_rad_waist_ -> cal_end_point_tra_alpha -> current_time = traj_time_test;
	end_to_rad_waist_ -> cal_end_point_tra_betta -> current_time = 0;
	end_to_rad_waist_ -> cal_end_point_tra_kamma -> current_time = traj_time_test;

	end_to_rad_head_ -> cal_end_point_tra_alpha -> current_time = traj_time_test;
	end_to_rad_head_ -> cal_end_point_tra_betta -> current_time = traj_time_test;
	end_to_rad_head_ -> cal_end_point_tra_kamma -> current_time = traj_time_test;

	end_to_rad_waist_ -> cal_end_point_tra_alpha -> final_time = traj_time_test;
	end_to_rad_waist_ -> cal_end_point_tra_betta -> final_time = 0;
	end_to_rad_waist_ -> cal_end_point_tra_kamma -> final_time = traj_time_test;

	end_to_rad_head_ -> cal_end_point_tra_alpha -> final_time = traj_time_test;
	end_to_rad_head_ -> cal_end_point_tra_betta -> final_time = traj_time_test;
	end_to_rad_head_ -> cal_end_point_tra_kamma -> final_time = traj_time_test;


	filter_head = new control_function::Filter;
	temp_head_roll  = 0;
	temp_head_pitch = 0;
	temp_head_yaw   = 0;
	temp_pre_roll = 0;
	temp_pre_pitch = 0;
	temp_pre_yaw = 0;

	// tracking
	command = 255;
	pidController_x = new control_function::PID_function(0.008,60*DEGREE2RADIAN,-60*DEGREE2RADIAN,0,0,0);
	pidController_y = new control_function::PID_function(0.008,75*DEGREE2RADIAN,0*DEGREE2RADIAN,0,0,0);

	control_angle_yaw = 0;
	control_angle_pitch = 0;
	control_angle_yaw_temp = 0;
	control_angle_pitch_temp = 0;
	pre_current_x = 0;
	pre_current_y = 0;
	frame_x = 1280;
	frame_y = 720;
	int margin_desired_x = 0;
	int margin_desired_y = 5;
	desired_x = (frame_x/2) + margin_desired_x;
	desired_y = (frame_y/2) + margin_desired_y;
	current_x = desired_x;
	current_y = desired_y;

	//balance param
	balance_updating_duration_sec_ = 2.0;
	balance_updating_sys_time_sec_ = 0;
	balance_update_ = false;
	gain_x_p_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_x_d_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_y_p_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_y_d_adjustment = new heroehs_math::FifthOrderTrajectory;
	x_p_gain = 0;
	x_d_gain = 0;
	y_p_gain = 0;
	y_d_gain = 0;

	//motion
	current_time_scanning = 0;
	motion_num_scanning = 1;

	current_time_finding = 0;
	motion_num_finding = 1;

  current_time_checking = 0;
  motion_num_checking = 1;

	current_time_arm_motion = 0;
	motion_num_arm=1;

	arm_motion_run =false;

	l_shoulder_pitch_trj = new heroehs_math::FifthOrderTrajectory;
	r_shoulder_pitch_trj = new heroehs_math::FifthOrderTrajectory;
	l_shoulder_roll_trj  = new heroehs_math::FifthOrderTrajectory;
	r_shoulder_roll_trj  = new heroehs_math::FifthOrderTrajectory;
	l_elbow_pitch_trj    = new heroehs_math::FifthOrderTrajectory;
	r_elbow_pitch_trj    = new heroehs_math::FifthOrderTrajectory;
  head_yaw_trj    = new heroehs_math::FifthOrderTrajectory;
  head_pitch_trj    = new heroehs_math::FifthOrderTrajectory;

	leg_check = 0;
	ball_detected = 0;
	//detected objects
	current_goal_x = 0.0;
	current_goal_y = 0.0;
	current_center_x = 0.0;
	current_center_y = 0.0;
	current_robot_x  = 0.0;
	current_robot_y  = 0.0;
	current_robot_theta = 0.0;

}
UpperBodyModule::~UpperBodyModule()
{
	queue_thread_.join();
}
// ros message communication thread
void UpperBodyModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;
	ros_node.setCallbackQueue(&callback_queue);

	// publish topics
	scan_done_pub = ros_node.advertise<std_msgs::Bool>("/heroehs/alice/scan_done", 1);
	//robot_state_pub = ros_node.advertise<geometry_msgs::Vector3>("/heroehs/alice/robot_state", 1);


	// subscribe topics
	environment_detector_sub = ros_node.subscribe("/heroehs/environment_detector", 5, &UpperBodyModule::environmentDetectorMsgCallback, this);
	//detected_objects_sub = ros_node.subscribe("/heroehs/detected_objects", 5, &UpperBodyModule::detectedObjectsMsgCallback, this);

	head_moving_sub = ros_node.subscribe("/heroehs/alice/head_command", 5, &UpperBodyModule::headMovingMsgCallback, this);
	arm_moving_sub = ros_node.subscribe("/heroehs/alice/arm_command", 5, &UpperBodyModule::armMovingMsgCallback, this);

	// test desired pose
	head_test = ros_node.subscribe("/desired_pose_head", 5, &UpperBodyModule::desiredPoseHeadMsgCallback, this);
	waist_test = ros_node.subscribe("/desired_pose_waist", 5, &UpperBodyModule::desiredPoseWaistMsgCallback, this);

	//test ball
	ball_test_sub = ros_node.subscribe("/ball_test", 5, &UpperBodyModule::ballTestMsgCallback, this);
	ball_param_sub = ros_node.subscribe("/ball_param", 5, &UpperBodyModule::ballTestParamMsgCallback, this);


	//walking status
	walking_module_status_sub = ros_node.subscribe("/heroehs/status", 10, &UpperBodyModule::walkingModuleStatusMsgCallback, this);


	//desired_pose_all_sub = ros_node.subscribe("/desired_pose_all", 5, &UpperBodyModule::desiredPoseAllMsgCallback, this);

	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);

}
// TEST /////////////////////////////////////////////
void UpperBodyModule::walkingModuleStatusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg)  //string
{
	if(!msg->status_msg.compare("Walking_Started"))
	{
		leg_check = 1;
	}
	if(!msg->status_msg.compare("Walking_Finished"))
	{
		leg_check = 0;
	}
}
void UpperBodyModule::desiredPoseWaistMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	waist_end_point_(3, 1) = msg->data[0]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	waist_end_point_(4, 1) = msg->data[1]; // pitch
	waist_end_point_(3, 7) = msg->data[2];
	waist_end_point_(4, 7) = msg->data[3];
	is_moving_waist_ = true;
}
void UpperBodyModule::desiredPoseHeadMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	head_end_point_(3, 1)  = msg->data[0]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	head_end_point_(4, 1)  = msg->data[1]; // pitch
	head_end_point_ (3, 7) = msg->data[2];
	head_end_point_ (4, 7) = msg->data[3];
	is_moving_head_ = true;
}
void UpperBodyModule::environmentDetectorMsgCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg)
{
	for(int i = 0; i < msg->length; i++)
	{
		if(!msg->data[i].name.compare("ball"))
		{
			current_x = msg->data[i].roi.x_offset + msg->data[i].roi.width/2;
			current_y = msg->data[i].roi.y_offset + msg->data[i].roi.height/2;
			ball_detected = 1;
		}
	}
}
void UpperBodyModule::detectedObjectsMsgCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg)
{
	int data_cnt = 0;

	for(int i = 0; i < msg->length; i++)
	{
		if(!msg->data[i].name.compare("goal"))
		{
			data_cnt ++;
			current_goal_x  = msg->data[i].pos.x;
			current_goal_y  = msg->data[i].pos.y;
		}
		if(!msg->data[i].name.compare("center"))
		{
			data_cnt ++;
			current_center_x = msg->data[i].pos.x;
			current_center_y = msg->data[i].pos.y;
		}
	}

	if(data_cnt == 2)
	{
		double theta_center = 0;
		double theta_goal = 0;
		double length_center = 0;
		double length_goal = 0;
		double sin_robot = 0;
		double cos_robot = 0;

		if(current_center_x != 0 && current_goal_x != 0)
		{
			theta_center = fabs(atan2(current_center_y,current_center_x));
			theta_goal   = fabs(atan2(current_goal_y, current_goal_x));
			length_center = sqrt(pow(current_center_x,2) + pow(current_center_y,2));
			length_goal = sqrt(pow(current_goal_x,2) + pow(current_goal_y,2));
			sin_robot = (length_goal * sin(theta_center + theta_goal)) / 4.5;
			cos_robot = sqrt(1-pow(sin_robot,2));

			current_robot_x = length_center*sin_robot;
			current_robot_y = length_center*cos_robot;

			int sign_x = 1;
			int sign_y = 1;

			if((current_goal_y - current_center_y) < 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) < 0) //case 1
			{
				sign_x = 1;
				sign_y = 1;
			}
			else if((current_goal_y - current_center_y) < 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) > 0)
			{
				sign_x = -1;
				sign_y = 1;
			}
			else if((current_goal_y - current_center_y) > 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) > 0)
			{
				sign_x = -1;
				sign_y = -1;
			}
			else if((current_goal_y - current_center_y) > 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) < 0)
			{
				sign_x = 1;
				sign_y = -1;
			}
			current_robot_x = current_robot_x * sign_x;
			current_robot_y = current_robot_y * sign_y;

			if((current_goal_x - current_center_x) != 0)
				current_robot_theta = atan2((current_goal_y - current_center_y), (current_goal_x - current_goal_x));
			else
			{
				if((current_goal_y - current_center_y) > 0)
					current_robot_theta = 90*DEGREE2RADIAN;
				else if((current_goal_y - current_center_y) < 0)
					current_robot_theta = 270*DEGREE2RADIAN;
				else
					current_robot_theta = 0;

			}
		}
		else
		{
			current_center_x = 0.01;
			current_goal_x = 0.01;

			theta_center = fabs(atan2(current_center_y,current_center_x));
			theta_goal   = fabs(atan2(current_goal_y, current_goal_x));
			length_center = sqrt(pow(current_center_x,2) + pow(current_center_y,2));
			length_goal = sqrt(pow(current_goal_x,2) + pow(current_goal_y,2));
			sin_robot = (length_goal * sin(theta_center + theta_goal)) / 4.5;
			cos_robot = sqrt(1-pow(sin_robot,2));

			current_robot_x = length_center*sin_robot;
			current_robot_y = length_center*cos_robot;

			int sign_x = 1;
			int sign_y = 1;

			if((current_goal_y - current_center_y) < 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) < 0) //case 1
			{
				sign_x = 1;
				sign_y = 1;
			}
			else if((current_goal_y - current_center_y) < 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) > 0)
			{
				sign_x = -1;
				sign_y = 1;
			}
			else if((current_goal_y - current_center_y) > 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) > 0)
			{
				sign_x = -1;
				sign_y = -1;
			}
			else if((current_goal_y - current_center_y) > 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) < 0)
			{
				sign_x = 1;
				sign_y = -1;
			}
			current_robot_x = current_robot_x * sign_x;
			current_robot_y = current_robot_y * sign_y;

			if((current_goal_x - current_center_x) != 0)
				current_robot_theta = atan2((current_goal_y - current_center_y), (current_goal_x - current_goal_x));
			else
			{
				if((current_goal_y - current_center_y) > 0)
					current_robot_theta = 90*DEGREE2RADIAN;
				else if((current_goal_y - current_center_y) < 0)
					current_robot_theta = 270*DEGREE2RADIAN;
				else
					current_robot_theta = 0;

			}
		}
	}

	data_cnt = 0;


	robot_state_msg.x = current_robot_x;
	robot_state_msg.y = current_robot_y;
	robot_state_msg.z = current_robot_theta;

	robot_state_pub.publish(robot_state_msg);

}

void UpperBodyModule::armMovingMsgCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg)
{

  if(msg->key == "arm_motion") //arm
  {
    if(msg->value == "1") //on
    {
      arm_motion_run = true;
    }
    else if(msg->value == "0") //off
    {
      arm_motion_run = false;
    }
  }

}
void UpperBodyModule::headMovingMsgCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg)
{

  if(msg->key == "head_tracking") //head tracking
  {
    command = 3;
  }
  else if(msg->key == "head_searching") //head search
  {
    command = 2;
  }
  else if(msg->key == "head_ball_check") //head search
  {
    command = 5;
  }
  else if(msg->key == "head_stop") //head search
  {
    command = 6;
  }
  /*
  else if(msg->key == "arm_motion") //arm
  {
    if(msg->value == "1") //on
    {
      arm_motion_run = true;
    }
    else if(msg->value == "0") //off
    {
      arm_motion_run = false;
    }
  }
  */


  /*command = msg -> data;
	if(command == 1)
	{
		current_time_scanning = 0;
		motion_num_scanning = 1;
	}*/
}
//test
void UpperBodyModule::ballTestMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	current_x = msg->data[0];
	current_y = msg->data[1];

}
void UpperBodyModule::ballTestParamMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	balance_update_ = true;
	balance_updating_duration_sec_ = msg->data[0];
	x_p_gain = msg->data[1];
	x_d_gain = msg->data[2];
	y_p_gain = msg->data[3];
	y_d_gain = msg->data[4];

	/*
	if(x_p_gain == 0)
	  command = 255;
	else
	  command = 3;
  */

	//printf("sec value ::  %f \n",balance_updating_duration_sec_ );
	//printf("P value ::  %f \n",y_p_gain );
	//printf("D P value ::  %f \n",y_d_gain );
}
void UpperBodyModule::readIDData()
{
  ros::NodeHandle nh;
  int alice_id_int  = nh.param<int>("alice_userid",0);

  //ROS_INFO("Base id: %d",alice_id_int);
  std::stringstream alice_id_stream;
  alice_id_stream << alice_id_int;
  alice_id_ = alice_id_stream.str();
}



/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  Christoph Henke (christoph.henke@ima-zlw-ifu.rwth-aachen.de).
 *  Former Author: Sebastian Reuter (sebastian.reuter@ima-zlw-ifu.rwth-aachen.de)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Henke (christoph.henke@ima-zlw-ifu.rwth-aachen.de)
 * Former Author: Sebastian Reuter (sebastian.reuter@ima-zlw-ifu.rwth-aachen.de)
 *********************************************************************/


#ifndef _CAN_DRIVER_CPP_
#define _CAN_DRIVER_CPP_

#include <iostream>
#include <unistd.h>
#include <ros/console.h>
#include <math.h>

// class header file
#include <can_driver.h>

/* Constructor
 * Initializing private member variables
 * Establishing the CAN connection
 */
CanDriver::CanDriver(){
    // get node name
    name_ = ros::this_node::getName().c_str();

	// initialize publishers
	odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
    torsten_state_pub_ = nh.advertise<torsten_msgs::TorstenState>("torsten_state", 50);

	// initialize ROS parameters
    load_parameters();

	// status variables for ramping/velocity reduction modes
	warn_field_mode_1_ = false;
	warn_field_mode_2_ = false;

	// boolean marker for shown move base availability
	move_base_available_ = false;

	// initialization of configuration variables
	loaded_             = false;
	autonomous_mode_    = false;
	error_              = false;
	handling_mode_      = false;
	navigation_mode_    = false;
	bolts_move_up_      = false;
	bolts_move_down_    = false;

	// initialization of input values
	bolts_down_received_    = false;
	bolts_up_received_      = false;
	handling_received_      = false;
	navigation_received_    = false;
	autonomy_received_      = false;
	error_received_         = false;
	pulse_received_         = false;

	// Initialize protective and warning field values
	warning_field_1_host_	=  false;
	warning_field_2_host_	=  false;
	warning_field_1_guest_	=  false;
	warning_field_2_guest_	=  false;
	protective_field_host_	=  false;
	protective_field_guest_	=  false;

    // initialization of state variables
    sound_cmd_running_ = false;
    bolts_cmd_running_ = false;

    // boolean for marking loaded (true) or unloaded state (false)
    load_state_ = false;
    load_config_state_ = false;

    // initialization of further variables
	x_ = 0.0;
	y_ = 0.0;
	yaw_ = 0.0;
	new_odometry_arrived_ = false;

	// initialization of timer variables
	current_time_ = ros::Time::now();
	last_time_ = ros::Time::now();
	last_sound_cmd_time_ = ros::Time::now();
	wait_for_start_up_move_base_ = ros::Time::now();
	wait_timer_move_base_come_up_ = ros::Time::now();

	// initialization of heart-beat bit
	pulse_ = false;

	// instantiation of can bus object (used to send/receive process data objects (PDOs))
	can_bus_ = new PeakCANUSB();

	// initialization of ros service clients
	srv_bolts_up_ = nh.serviceClient<torsten_driver::boltsState>("torsten_driver_node/bolts_up");
	srv_bolts_down_ = nh.serviceClient<torsten_driver::boltsState>("torsten_driver_node/bolts_down");

	// initialization of bolts state publisher
	pub_bolts_up_ 		= nh.advertise<std_msgs::String>("/torsten_driver/bolts_up_state", 10);
	pub_bolts_down_ 	= nh.advertise<std_msgs::String>("/torsten_driver/bolts_down_state", 10);

	// establish CAN bus connection
	while(true){
		if( can_bus_->init_connection() ){
			ROS_INFO("%s: CAN connection established", name_.c_str());
			break;
		} else{
			ROS_WARN("%s: CAN connection not established - retrying connection initialization", name_.c_str());
		}
		// sleep for a second
		sleep(1);
	}
}

/* Destructor
 *
 */
CanDriver::~CanDriver(){
	close_connection();
}

/* Method for closing the CAN connection
 *
 */
void
CanDriver::close_connection(){
	can_bus_->close_connection();
	ROS_INFO("%s: CAN connection closed", name_.c_str());
}

/* Method for loading ROS parameters
 *
 */
void
CanDriver::load_parameters(){
    /*
     * TODO: This parameter validation and setting should be
     * done in a separate configuration class structure
     */
    // load ros parameters and validate them

    if (nh.hasParam("/torsten_driver_node/declare_dead_duration")){
        nh.getParam("/torsten_driver_node/declare_dead_duration", cfg_declare_dead_duration_);
        ROS_INFO("%s: Assigned declare_dead_duration: %f", name_.c_str(), cfg_declare_dead_duration_);
    }
    else{
        cfg_declare_dead_duration_ = 1.0;
        ROS_INFO("%s: didn't find param 'declare_dead_duration' took default value :%f", name_.c_str(), cfg_declare_dead_duration_);
    }

    if (nh.hasParam("/torsten_driver_node/continuous_angular_factor")){
        nh.getParam("/torsten_driver_node/continuous_angular_factor", cfg_continuous_angular_factor_);
        ROS_INFO("%s: Assigned continuous_angular_factor: %f", name_.c_str(), cfg_continuous_angular_factor_);
    }
    else{
        cfg_continuous_angular_factor_ = 1.192054143;
        ROS_INFO("%s: didn't find param 'continuous_angular_factor' took default value :%f", name_.c_str(), cfg_continuous_angular_factor_);
    }

    if (nh.hasParam("/torsten_driver_node/print_odom_values_to_debug")){
        nh.getParam("/torsten_driver_node/print_odom_values_to_debug", cfg_print_odom_values_to_debug_);
        ROS_INFO("%s: Assigned print_odom_values_to_debug: %d", name_.c_str(), cfg_print_odom_values_to_debug_);
    }
    else{
        cfg_print_odom_values_to_debug_ = false;
        ROS_INFO("%s: didn't find param 'print_odom_values_to_debug' took default value :%d", name_.c_str(), cfg_print_odom_values_to_debug_);
    }

    /*
     * This parameter defines if the warn fields of the safety laser scanners
     * are used or not. Using the warn fields enables the robot to reduce it's velocity
     * through an reliable incoming signal when obstacles are in the near field
     */
    if (nh.hasParam("/torsten_driver_node/cfg_use_warn_fields")){
        nh.getParam("/torsten_driver_node/cfg_use_warn_fields", cfg_use_warn_fields_);
        ROS_INFO("%s: Assigned cfg_use_warn_fields: %d", name_.c_str(), cfg_use_warn_fields_);
    }
    else{
        cfg_use_warn_fields_ = true;
        ROS_INFO("%s: didn't find param 'cfg_use_warn_fields' took default value :%d", name_.c_str(), cfg_use_warn_fields_);
    }
}

/* Callback for moving the bolts up
 * @param req true for moving the bolts up
 * @param res true if bolts are moved up
 */
bool
CanDriver::moveBoltsUpCB(torsten_driver::setBolts::Request &req, torsten_driver::setBolts::Response &res)
{
	// if a bolts up service call was received - move the bolts up
	if ((bool) req.data)
	{
		setHandling(true);
		setLoaded(false);
		bolts_cmd_running_ = true;
		res.done = true;
		return true;
	// else perform a reset
	} else {
		setHandling(false);
		setLoaded(false);
		bolts_cmd_running_ = false;
		res.done = true;
		return true;
	}
}

/* Callback for moving the bolts down
 * @param req true for moving the bolts down
 * @param res true if bolts are moved down
 */
bool
CanDriver::moveBoltsDownCB(torsten_driver::setBolts::Request &req, torsten_driver::setBolts::Response &res)
{
	// if a bolts down service call was received - move the bolts down
	if ((bool) req.data)
	{
		setLoaded(true);
		setHandling(false);
		bolts_cmd_running_ = true;
		res.done = true;
		return true;
	// else perform a reset
	} else {
		setHandling(false);
		setLoaded(false);
		bolts_cmd_running_ = false;
		res.done = true;
		return true;
	}
}

/* Callback for playing sounds
 * @param req string of which sound has to be played
 * @param res true if sound file was played successfully
 */
bool
CanDriver::playSoundCB(torsten_driver::setSound::Request &req, torsten_driver::setSound::Response &res)
{
	if (strcmp(req.sound.c_str(), "start_moving") == 0)
	{
		setAutonomousMode(true);
		sound_cmd_running_ = true;
		last_sound_cmd_time_ = ros::Time::now();
		res.done = true;
		return true;
	} else if (strcmp(req.sound.c_str(), "order_completed") == 0) {
		setError(true);
		sound_cmd_running_ = true;
		last_sound_cmd_time_ = ros::Time::now();
		res.done = true;
		return true;
	} else if (strcmp(req.sound.c_str(), "system_error") == 0) {
		setNavigationMode(true);
		sound_cmd_running_ = true;
		last_sound_cmd_time_ = ros::Time::now();
		res.done = true;
		return true;
	} else if (strcmp(req.sound.c_str(), "platform_pick_up") == 0) {
		setError(true);
		setAutonomousMode(true);
		sound_cmd_running_ = true;
		last_sound_cmd_time_ = ros::Time::now();
		res.done = true;
		return true;
	} else if (strcmp(req.sound.c_str(), "engine_start") == 0) {
		setNavigationMode(true);
		setAutonomousMode(true);
		sound_cmd_running_ = true;
		last_sound_cmd_time_ = ros::Time::now();
		res.done = true;
		return true;
	} else {
		setAutonomousMode(false);
		setNavigationMode(false);
		setError(false);
		sound_cmd_running_ = false;
		last_sound_cmd_time_ = ros::Time::now();
		res.done = false;
		return false;
	}
}

/* Callback method for subscription of cmd_vel
 * @param cmd_vel commanded velocity message
 *
 * received commands will be stored in member variable
 */
void
CanDriver::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &cmd_vel){

	cmd_vel_.linear.x = cmd_vel->linear.x;
	cmd_vel_.linear.y = cmd_vel->linear.y;
	cmd_vel_.linear.z = cmd_vel->linear.z;

	cmd_vel_.angular.x = cmd_vel->angular.x;
	cmd_vel_.angular.y = cmd_vel->angular.y;
	cmd_vel_.angular.z = cmd_vel->angular.z;

	time_last_cmd_vel_ = ros::Time::now();
}

/* Method for publishing odometry messages
 * @param e ros timer event
 *
 */
void
CanDriver::publish_odom(const ros::TimerEvent& e){

	// publish only if new CAN message received
	if(new_odometry_arrived_){

		float vx = odom_.linear.x;
		float vy = odom_.linear.y;
		float v_yaw = odom_.angular.z * cfg_continuous_angular_factor_;

		// store current time the velocity has been received - for integration calculations
		current_time_ = ros::Time::now();

		// integrate current velocities to odometry
		double dt = (current_time_ - last_time_).toSec();
		double delta_x = (vx * cos(yaw_) - vy * sin(yaw_)) * dt;
		double delta_y = (vx * sin(yaw_) + vy * cos(yaw_)) * dt;
		double delta_yaw = v_yaw * dt;

		// add the integrated odometry to the current odometry
		x_ += delta_x;
		y_ += delta_y;
		yaw_ += delta_yaw;

		// instantiate new quaternion message for orientation calculations
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw_);

		// publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time_;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x_;
		odom_trans.transform.translation.y = y_;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		// send the transform
		odom_broadcaster_.sendTransform(odom_trans);

		// instantiate new odometry message
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time_;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";

		// set the position
		odom.pose.pose.position.x = x_;
		odom.pose.pose.position.y = y_;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		// set the velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = v_yaw;

		// publish the current odometry
		odom_pub_.publish(odom);

		// store last time integration for later calculations
		last_time_ = current_time_;

		// reset the message received marker
		new_odometry_arrived_ = false;

		/* for debugging purposes the current odometry can be published
		 * into the logger.
		 * the cfg parameter can be set before starting the node
		 */
		if(cfg_print_odom_values_to_debug_){
			float yaw_angle_ = (yaw_ * 180 / M_PI);
			ROS_INFO("%s: odom is x=%.3f \t y=%.3f \t yaw=%.3f", name_.c_str(),  x_, y_, yaw_angle_);
		}
	}
}

/* Method for sending CAN messages
 * @param e ros timer event
 */
void
CanDriver::can_send_cmd_vel(const ros::TimerEvent& e){

	// Instantiate new CAN message
	CANMessage_Cmd_Vel msg;

	/* if a bolts up/down cmd is currently running, check the state and
	 * reset the command to idle state if done
	 */
	if (bolts_cmd_running_)
	{
		if (isLoaded() && !isInHandlingMode() && bolts_down_received_) {
			setLoaded(false);
			setHandling(false);
			bolts_cmd_running_ = false;

			// generate new boltsState message to inform of the new bolts state
			torsten_driver::boltsState bolts_state_down_srv;
			bolts_state_down_srv.request.data = true;

			if (srv_bolts_down_.call(bolts_state_down_srv))
			{
				ROS_INFO("%s: Bolts are moved down - sending srv to torsten_driver/bolts_down", name_.c_str());
			} else {
				ROS_INFO("%s: Bolts are moved down - sending srv to torsten_driver/bolts_down - service call failed", name_.c_str());
			}

			/* Publish the current bolts state
			 * TODO: Embedding these functionalities into a designated
			 * robot state topic
			 */
			std_msgs::String msg;
			msg.data = "down";
			pub_bolts_down_.publish(msg);

		} else if (!isLoaded() && isInHandlingMode() && bolts_up_received_) {
			setLoaded(false);
			setHandling(false);
			bolts_cmd_running_ = false;

			// generate new boltsState message to inform of the new bolts state
			torsten_driver::boltsState bolts_state_up_srv;
			bolts_state_up_srv.request.data = true;

			if (srv_bolts_up_.call(bolts_state_up_srv))
			{
				ROS_INFO("%s: Bolts are moved up - sending srv to torsten_driver/bolts_up", name_.c_str());
			} else {
				ROS_INFO("%s: Bolts are moved up - sending srv to torsten_driver/bolts_up - service call failed", name_.c_str());
			}

			/* Publish the current bolts state
			 * TODO: Embedding these functionalities into a designated
			 * robot state topic
			 */
			std_msgs::String msg;
			msg.data = "up";
			pub_bolts_up_.publish(msg);
		}
	}

	// if sound cmd is longer than duration running, perform a reset
	if (sound_cmd_running_ && (ros::Time::now() - last_sound_cmd_time_ > ros::Duration(4.0)))
	{
		setAutonomousMode(false);
		setNavigationMode(false);
		setError(false);

		// reset sound cmd running marker
		sound_cmd_running_ = false;
	}

	// converting rotational velocity from radiants to degrees
	float v_angular_z_in_degree = (cmd_vel_.angular.z * 180 / M_PI);

	// Generate message to publish the actual send vel command - for debugging purposes
	geometry_msgs::Twist cmd_vel;
	cmd_vel = cmd_vel_;

	// check if the last cmd_vel message is not too old
	if( (ros::Time::now() - time_last_cmd_vel_) < ros::Duration(cfg_declare_dead_duration_)){
		msg.set_msg( cmd_vel_.linear.x,	cmd_vel_.linear.y, v_angular_z_in_degree,
				isLoaded(),
				isInHandlingMode(),
				isInNavigationMode(),
				isInAutonomousMode(),
				isError(),
				Pulse()
				);
	} else {
		msg.set_msg( 0.0, 0.0, 0.0,
				isLoaded(),
				isInHandlingMode(),
				isInNavigationMode(),
				isInAutonomousMode(),
				isError(),
				Pulse()
				);
	}

	// sending CAN message
	can_bus_->send_msg(msg);

	// flip heart-beat bit
	setPulse(!Pulse());
}

/* Reading CAN input data
 * data is separated by message ID
 */
void
CanDriver::can_read_odom(const ros::TimerEvent& e){

	// instantiate CAN input message
	CANMessage* msg = new CANMessage;

	// read messages
	if( can_bus_->receive_msg((*msg))){

		// read message of specific ID
		if( msg->getID() == 0x190){

			// cast input message into odometry message type
			CANMessage_Odometry* odom_msg = static_cast<CANMessage_Odometry*>(msg);

			/* copy input current velocity into member variable
			 * input velocities in x and y are received in unit [m/s]
			 * input angular velocity is received in unit [rad/s]
			 */
			odom_.linear.x 	= odom_msg->get_vel_x();
			odom_.linear.y 	= odom_msg->get_vel_y();
			odom_.angular.z = (odom_msg->get_vel_yaw() * M_PI / 180);

			// set the odometry received marker
			new_odometry_arrived_ = true;

			// copy protective field and warning field states
			warning_field_1_host_ = odom_msg->get_warning_field_1_host();
			warning_field_2_host_ = odom_msg->get_warning_field_2_host();
			warning_field_1_guest_ = odom_msg->get_warning_field_1_guest();
			warning_field_2_guest_ = odom_msg->get_warning_field_2_guest();
			protective_field_host_ = odom_msg->get_protective_field_host();
			protective_field_guest_ = odom_msg->get_protective_field_guest();

			// copy state values into member variables
			bolts_down_received_    = odom_msg->get_bolts_down();
			bolts_up_received_      = odom_msg->get_bolts_up();
			handling_received_      = odom_msg->get_handling();
			navigation_received_    = odom_msg->get_navigation();
			autonomy_received_      = odom_msg->get_autonomy();
			error_received_         = odom_msg->get_error();
			pulse_received_         = odom_msg->get_pulse();

			// change the loading state of the vehicle - loaded (true) or unloaded state (false)
			/* if no concrete bolts state is received (prob. while moving up or down)
			 * then assume a loaded state
			 */
			if (bolts_down_received_) {
				load_state_ = false;
			} else {
				load_state_ = true;
			}
		}
	}
}

/* Method for evaluating the safety (warning and protective) fields
 * Reduces speed if warning or protective field injured
 * @param e ros timer event
 */
void
CanDriver::scan_field_safety_evaluation(const ros::TimerEvent& e)
{
	/*
	 * TODO: Implement this more elegant to enable
	 * an appropriate start up check of the move base.
	 * Currently the additional waiting time is necessary
	 * because the parameter settings would stop the move base
	 * within the initialization process
	 */
	ros::Duration timeout_startup(20.0);

	/*
	 * TODO: Use the cfg parameter in the node implementation
	 * to disable the start of the timer instance
	 */

	if (cfg_use_warn_fields_) {
		if(ros::service::exists("/move_base/TebLocalPlannerROS/set_parameters", false) &&
				(ros::Time::now() - wait_timer_move_base_come_up_) > timeout_startup)
		{
			// set the velocity reduction if one warn fields is injured
			if(!warn_field_mode_1_ && !(warning_field_1_host_ && warning_field_2_host_ &&
					warning_field_1_guest_ && warning_field_2_guest_ &&
					protective_field_host_ && protective_field_guest_))
			{
				dynamic_reconfigure::ReconfigureRequest srv_req;
				dynamic_reconfigure::ReconfigureResponse srv_resp;
				dynamic_reconfigure::DoubleParameter vx_value;
				dynamic_reconfigure::DoubleParameter vy_value;
				dynamic_reconfigure::DoubleParameter theta_value;
				dynamic_reconfigure::Config conf;

				// set the reduced max_vel_x for warn_field_mode_1
				vx_value.name = "max_vel_x";
				vx_value.value = 0.3;
				conf.doubles.push_back(vx_value);

				// set the reduced max_vel_y for warn_field_mode_1
				vy_value.name = "max_vel_y";
				vy_value.value = 0.1;
				conf.doubles.push_back(vy_value);

				// set the reduced max_vel_theta for warn_field_mode_1
				theta_value.name = "max_vel_theta";
				theta_value.value = 0.1;
				conf.doubles.push_back(theta_value);

				srv_req.config = conf;

				if (ros::service::call("/move_base/TebLocalPlannerROS/set_parameters", srv_req, srv_resp)) {
					ROS_INFO("%s: call to set TebLocalPlannerROS parameters succeeded", name_.c_str());
					// Set the warn_field_mode_1_ marker
					warn_field_mode_1_ = true;
				} else {
					ROS_INFO("%s: call to set TebLocalPlannerROS parameters failed", name_.c_str());
				}
			}

			// reset the velocity reduction if all warn fields are free
			if (warn_field_mode_1_ && (warning_field_1_host_ && warning_field_2_host_ &&
					warning_field_1_guest_ && warning_field_2_guest_ &&
					protective_field_host_ && protective_field_guest_))
			{
				dynamic_reconfigure::ReconfigureRequest srv_req;
				dynamic_reconfigure::ReconfigureResponse srv_resp;
				dynamic_reconfigure::DoubleParameter vx_value;
				dynamic_reconfigure::DoubleParameter vy_value;
				dynamic_reconfigure::DoubleParameter theta_value;
				dynamic_reconfigure::Config conf;

				// reset the reduced max_vel_x for warn_field_mode_1
				vx_value.name = "max_vel_x";
				vx_value.value = 0.6;
				conf.doubles.push_back(vx_value);

				// reset the reduced max_vel_y for warn_field_mode_1
				vy_value.name = "max_vel_y";
				vy_value.value = 0.2;
				conf.doubles.push_back(vy_value);

				// reset the reduced max_vel_theta for warn_field_mode_1
				theta_value.name = "max_vel_theta";
				theta_value.value = 0.3;
				conf.doubles.push_back(theta_value);

				srv_req.config = conf;

				if (ros::service::call("/move_base/TebLocalPlannerROS/set_parameters", srv_req, srv_resp)) {
					ROS_INFO("%s: call to reset TebLocalPlannerROS parameters succeeded", name_.c_str());
					// Reset the warn_field_mode_1_ marker
					warn_field_mode_1_ = false;
				} else {
					ROS_INFO("%s: call to reset TebLocalPlannerROS parameters failed", name_.c_str());
				}
			}

			// TODO: Add ramping functionalities if mode_1 or mode_2 is called

		} else {
			ros::Duration timeout(2.0);
			if ((ros::Time::now() - wait_for_start_up_move_base_) > timeout)
			{
				ROS_INFO("%s: Waiting for service /move_base/TebLocalPlannerROS/set_parameters to come up", name_.c_str());
				wait_for_start_up_move_base_ = ros::Time::now();
			}
		}
	}
}

/* Publishing torsten state information
 *
 */
void
CanDriver::publish_torsten_state(const ros::TimerEvent& e) {
    torsten_msgs::TorstenState msg;

    msg.loaded =                    isLoaded();
    msg.handling_mode =             isInHandlingMode();
    msg.navigation_mode =           isInNavigationMode();
    msg.autonomous_mode =           isInAutonomousMode();
    msg.error =                     isError();
    msg.pulse =                     Pulse();
    msg.bolts_move_up =             isBoltsMovedUp();
    msg.bolts_move_down =           isBoltsMovedDown();
    msg.bolts_down_received =       bolts_down_received_;
    msg.bolts_up_received =         bolts_up_received_;
    msg.handling_received =         handling_received_;
    msg.navigation_received =       navigation_received_;
    msg.autonomy_received =         autonomy_received_;
    msg.error_received =            error_received_;
    msg.pulse_received =            pulse_received_;
    msg.warning_field_1_host =      warning_field_1_host_;
    msg.warning_field_2_host =      warning_field_2_host_;
    msg.warning_field_1_guest =     warning_field_1_guest_;
    msg.warning_field_2_guest =     warning_field_2_guest_;
    msg.protective_field_host =     protective_field_host_;
    msg.protective_field_guest =    protective_field_guest_;

    torsten_state_pub_.publish(msg);
}

/* Getters and Setters
 *
 */

bool
CanDriver::isBoltsMovedUp() const {
	return bolts_up_received_;
}

bool
CanDriver::isBoltsMovedDown() const {
	return bolts_down_received_;
}

bool
CanDriver::isInAutonomousMode() const {
	return autonomous_mode_;
}

void
CanDriver::setAutonomousMode(bool autonomous) {
	autonomous_mode_ = autonomous;
}

bool
CanDriver::isError() const {
		return error_;
}

void CanDriver::setError(bool error) {
	error_ = error;
}

bool CanDriver::isInHandlingMode() const {
	return handling_mode_;
}

void CanDriver::setHandling(bool handling) {
	handling_mode_ = handling;
}

bool
CanDriver::isLoaded() const {
	return loaded_;
}

void
CanDriver::setLoaded(bool loaded) {
	loaded_ = loaded;
}

bool CanDriver::isInNavigationMode() const {
	return navigation_mode_;
}

void
CanDriver::setNavigationMode(bool navigation) {
	navigation_mode_ = navigation;
}

bool
CanDriver::Pulse() const {
	return pulse_;
}

void
CanDriver::setPulse(bool pulse) {
	pulse_ = pulse;
}


/* Service callback to set registers
 *
 */

bool
CanDriver::setLoadedsrv(torsten_driver::setBit::Request &req, torsten_driver::setBit::Response &res) {
	loaded_ = (bool)req.data;
    return true;
}

bool
CanDriver::setInHandlingModesrv(torsten_driver::setBit::Request &req, torsten_driver::setBit::Response &res) {
     handling_mode_ = (bool)req.data;
     return true;
}

bool
CanDriver::setInNavigationModesrv(torsten_driver::setBit::Request &req, torsten_driver::setBit::Response &res) {
    navigation_mode_ = (bool)req.data;
    return true;
}

bool
CanDriver::setInAutonomousModesrv(torsten_driver::setBit::Request &req, torsten_driver::setBit::Response &res) {
    autonomous_mode_ = (bool)req.data;
    return true;
}

bool
CanDriver::setIsErrorsrv(torsten_driver::setBit::Request &req, torsten_driver::setBit::Response &res) {
	error_ = (bool)req.data;
    return true;
}

#endif /* TORSTEN_PACKAGES_TORSTEN_DRIVER_SRC_TORSTEN_CAN_DRIVER_CPP_ */

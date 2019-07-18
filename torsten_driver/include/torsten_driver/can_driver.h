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

#ifndef _CAN_DRIVER_H_
#define _CAN_DRIVER_H_

#include <vector>
#include <string>

#include <ros/ros.h>

// tf library
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// ros msgs
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

// torsten msgs
#include <torsten_msgs/TorstenState.h>

// CAN interfaces and message header
#include <CANInterface.h>
#include <CANMessage.h>
#include <peak_can_usb.h>
#include <msgs/CANMessage_CMD_VEL.h>
#include <msgs/CANMessage_ODOMETRY.h>

// service header
#include <torsten_driver/setBit.h>
#include <torsten_driver/setBolts.h>
#include <torsten_driver/setSound.h>
#include <torsten_driver/boltsState.h>

// dynamic reconfigure
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

// move base client
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

  class CanDriver{

    public:
	  // typedefs
	  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

	  // Constructor
	  CanDriver();

	  // Destructor
	  ~CanDriver();

	  ros::NodeHandle nh;

	  // subscriber of /cmd_vel msgs
	  void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &cmd_vel);

      // subscriber of /cmd_vel msgs
      void load_parameters();

	  // subscriber of /sound_cmd msgs
	  void sound_cmd_callback(const std_msgs::String &sound_cmd);

	  // subscriber of /odom msgs
	  void publish_odom(const ros::TimerEvent& e);

      // publish torsten state
      void publish_torsten_state(const ros::TimerEvent& e);

	  // services
      bool setLoadedsrv(torsten_driver::setBit::Request &req, torsten_driver::setBit::Response &res);
      bool setInHandlingModesrv(torsten_driver::setBit::Request &req, torsten_driver::setBit::Response &res);
      bool setInNavigationModesrv(torsten_driver::setBit::Request &req, torsten_driver::setBit::Response &res);
      bool setInAutonomousModesrv(torsten_driver::setBit::Request &req, torsten_driver::setBit::Response &res);
      bool setIsErrorsrv(torsten_driver::setBit::Request &req, torsten_driver::setBit::Response &res);

      // service callbacks
      bool moveBoltsUpCB(torsten_driver::setBolts::Request &req, torsten_driver::setBolts::Response &res);
      bool moveBoltsDownCB(torsten_driver::setBolts::Request &req, torsten_driver::setBolts::Response &res);
      bool playSoundCB(torsten_driver::setSound::Request &req, torsten_driver::setSound::Response &res);


	  // CAN INTERFACE - send cmd_vel to CAN and receive odom from CAN
	  void can_send_cmd_vel(const ros::TimerEvent& e);
	  void can_read_odom(const ros::TimerEvent& e);

	  // method for reducing speed if warning field is activated
	  void scan_field_safety_evaluation(const ros::TimerEvent& e);

	  // method for closing the CAN connection
	  void close_connection();

	  // getter and setter
	  bool isBoltsMovedUp() const;
	  bool isBoltsMovedDown() const;
	  bool isInAutonomousMode() const;
	  void setAutonomousMode(bool autonomous);
	  bool isError() const;
	  void setError(bool error);
	  bool isInHandlingMode() const;
	  void setHandling(bool handling);
	  bool isLoaded() const;
	  void setLoaded(bool loaded);
	  bool isInNavigationMode() const;
	  void setNavigationMode(bool navigation);
	  bool Pulse() const;
	  void setPulse(bool pulse);

    private:

	  // used for name spaces and info, error logs
	  std::string name_;

	  // publisher
	  ros::Publisher odom_pub_;
      ros::Publisher torsten_state_pub_;

	  // CAN interface variable
	  CANInterface* can_bus_;

	  // timer elements
	  ros::Time	time_last_cmd_vel_;
	  ros::Time time_last_sound_cmd_;
	  ros::Time current_time_;
	  ros::Time last_time_;
	  ros::Time last_bolts_cmd_time_;
	  ros::Time last_sound_cmd_time_;

	  // config variables
	  bool cfg_print_odom_values_to_debug_;
	  double cfg_declare_dead_duration_;
	  double cfg_continuous_angular_factor_;

	  // marker variables
	  bool new_odometry_arrived_;

	  // messages
	  geometry_msgs::Twist cmd_vel_;
	  geometry_msgs::Twist odom_;

	  // declaring the used sound command
	  std::string sound_cmd_;

	  // tf transform broadcaster - broadcasting current odom frame to tf-tree
	  tf::TransformBroadcaster odom_broadcaster_;

	  // variables for service calls - ToDo: Replace this stuff with actionserver for bolts and sounds
	  bool sound_cmd_running_;
	  bool bolts_cmd_running_;

	  // current odometry values (units [m] and [rad])
	  double x_;
	  double y_;
	  double yaw_;

	  // service clients to mark bolts state
	  ros::ServiceClient srv_bolts_up_;
	  ros::ServiceClient srv_bolts_down_;

	  // same as publisher -ToDo: Replace this with the services
	  ros::Publisher pub_bolts_up_;
	  ros::Publisher pub_bolts_down_;

	  // booleans for marking loaded (true) or unloaded state (false) - and for the current configuration
	  bool load_state_;
	  bool load_config_state_;

	  // using the warn field velocity reduction
	  bool cfg_use_warn_fields_;

	  // status variables for ramping/velocity reduction modes
	  bool warn_field_mode_1_, warn_field_mode_2_;

	  // waiting for move base start up time
	  ros::Time wait_for_start_up_move_base_;
	  ros::Time wait_timer_move_base_come_up_;

	  // boolean marker for shown move base availability
	  bool move_base_available_;

	  // status bits from RWTH Controller
	  bool loaded_, handling_mode_, navigation_mode_, autonomous_mode_, error_, pulse_, bolts_move_up_, bolts_move_down_;

	  // status bits from SEW PLC
	  bool bolts_down_received_, bolts_up_received_, handling_received_, navigation_received_, autonomy_received_, error_received_, pulse_received_;

	  // status bits for protective and warning fields (SICK safety scanner)
  	  bool warning_field_1_host_, warning_field_2_host_, warning_field_1_guest_, warning_field_2_guest_, protective_field_host_, protective_field_guest_;
  };


#endif /* TORSTEN_PACKAGES_TORSTEN_DRIVER_INCLUDE_TORSTEN_CAN_DRIVER_H_ */

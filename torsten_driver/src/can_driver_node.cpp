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

// include can specific header files
#include <can_driver.cpp>

int main(int argc, char **argv)
{
	// initialization
	ros::init(argc, argv, "torsten_can_driver");

	// creating CanDriver instance
	CanDriver tcd;

	// subscribes cmd_vel messages
	ros::Subscriber cmd_vel_sub = tcd.nh.subscribe("cmd_vel", 20, &CanDriver::cmd_vel_callback, &tcd);

	// sends last cmd_vel msg over CAN with 20ms delay
	ros::Timer timer_send_cmd_vel = 	tcd.nh.createTimer(ros::Duration(0.001),  &CanDriver::can_send_cmd_vel, &tcd);

	// polls for new odometry info on CAN-bus
	ros::Timer timer_read_odom = 		tcd.nh.createTimer(ros::Duration(0.001),  &CanDriver::can_read_odom, &tcd);

	// publish odometry info in ROS
	ros::Timer timer_publish_odom = 	tcd.nh.createTimer(ros::Duration(0.001),  &CanDriver::publish_odom, &tcd);

    // publish torsten_state
    ros::Timer timer_torsten_state = 	tcd.nh.createTimer(ros::Duration(0.001),  &CanDriver::publish_torsten_state, &tcd);

	// reduces maximum allowed velocity if warning field is activated
	ros::Timer timer_reduce_speed_safety = 	tcd.nh.createTimer(ros::Duration(0.01),  &CanDriver::scan_field_safety_evaluation, &tcd);

	// standard service server
	ros::ServiceServer driver_server_setLoaded              = tcd.nh.advertiseService(ros::this_node::getName() + "/setLoaded", &CanDriver::setLoadedsrv, &tcd);
	ros::ServiceServer driver_server_setInHandlingMode      = tcd.nh.advertiseService(ros::this_node::getName() + "/setInHandlingMode", &CanDriver::setInHandlingModesrv, &tcd);
	ros::ServiceServer driver_server_setInNavigationMode    = tcd.nh.advertiseService(ros::this_node::getName() + "/setInNavigationMode", &CanDriver::setInNavigationModesrv, &tcd);
	ros::ServiceServer driver_server_setInAutonomousMode    = tcd.nh.advertiseService(ros::this_node::getName() + "/setInAutonomousMode", &CanDriver::setInAutonomousModesrv, &tcd);
	ros::ServiceServer driver_server_setIsError             = tcd.nh.advertiseService(ros::this_node::getName() + "/setIsError", &CanDriver::setIsErrorsrv, &tcd);

	// special service server
	ros::ServiceServer move_bolts_up_server    = tcd.nh.advertiseService(ros::this_node::getName() + "/move_bolts_up", &CanDriver::moveBoltsUpCB, &tcd);
	ros::ServiceServer move_bolts_down_server  = tcd.nh.advertiseService(ros::this_node::getName() + "/move_bolts_down", &CanDriver::moveBoltsDownCB, &tcd);
	ros::ServiceServer sound_cmd_server        = tcd.nh.advertiseService(ros::this_node::getName() + "/play_sound", &CanDriver::playSoundCB, &tcd);

	// perform ros spin while running
	while( tcd.nh.ok() ){
		ros::spin();
	}

	return 0;
}

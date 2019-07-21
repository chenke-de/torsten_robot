/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  Christoph Henke (christoph.henke@ima-ifu.rwth-aachen.de).
 *  Sebastian Reuter (sebastian.reuter@ima-ifu.rwth-aachen.de)
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
 * Author: Christoph Henke (christoph.henke@ima-ifu.rwth-aachen.de)
 * Author: Sebastian Reuter (sebastian.reuter@ima-ifu.rwth-aachen.de)
 *********************************************************************/

#include <torsten_driver.cpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "torsten_driver_node");

    TorstenDriver tcd;

	// subscriber cmd_vel messages
	ros::Subscriber cmd_vel_sub = tcd.nh.subscribe("cmd_vel", 20, &TorstenDriver::cmd_vel_callback, &tcd);

	// send CAN communication
	ros::Timer timer_send_cmd_vel = 	tcd.nh.createTimer(ros::Duration(0.001),  &TorstenDriver::can_send_cmd_vel, &tcd);

	// read CAN communication
	ros::Timer timer_read_odom = 		tcd.nh.createTimer(ros::Duration(0.001),  &TorstenDriver::can_read_odom, &tcd);

	// publisher odometry
	ros::Timer timer_publish_odom = 	tcd.nh.createTimer(ros::Duration(0.001),  &TorstenDriver::publish_odom, &tcd);

    // publisher torsten_state
    ros::Timer timer_torsten_state = 	tcd.nh.createTimer(ros::Duration(0.001),  &TorstenDriver::publish_torsten_state, &tcd);

	// reduces maximum allowed velocity if warning field is activated
	ros::Timer timer_reduce_speed_safety = 	tcd.nh.createTimer(ros::Duration(0.01),  &TorstenDriver::scan_field_safety_evaluation, &tcd);

	// standard service server
	ros::ServiceServer driver_server_setLoaded              = tcd.nh.advertiseService(ros::this_node::getName() + "/setLoaded", &TorstenDriver::setLoadedsrv, &tcd);
	ros::ServiceServer driver_server_setInHandlingMode      = tcd.nh.advertiseService(ros::this_node::getName() + "/setInHandlingMode", &TorstenDriver::setInHandlingModesrv, &tcd);
	ros::ServiceServer driver_server_setInNavigationMode    = tcd.nh.advertiseService(ros::this_node::getName() + "/setInNavigationMode", &TorstenDriver::setInNavigationModesrv, &tcd);
	ros::ServiceServer driver_server_setInAutonomousMode    = tcd.nh.advertiseService(ros::this_node::getName() + "/setInAutonomousMode", &TorstenDriver::setInAutonomousModesrv, &tcd);
	ros::ServiceServer driver_server_setIsError             = tcd.nh.advertiseService(ros::this_node::getName() + "/setIsError", &TorstenDriver::setIsErrorsrv, &tcd);

	// special service server
	ros::ServiceServer move_bolts_up_server    = tcd.nh.advertiseService(ros::this_node::getName() + "/move_bolts_up", &TorstenDriver::moveBoltsUpCB, &tcd);
	ros::ServiceServer move_bolts_down_server  = tcd.nh.advertiseService(ros::this_node::getName() + "/move_bolts_down", &TorstenDriver::moveBoltsDownCB, &tcd);
	ros::ServiceServer sound_cmd_server        = tcd.nh.advertiseService(ros::this_node::getName() + "/play_sound", &TorstenDriver::playSoundCB, &tcd);

	while( tcd.nh.ok() ){
		ros::spin();
	}

	return 0;
}

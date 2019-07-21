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

#ifndef _CANMESSAGE_ODOMETRY_H_
#define _CANMESSAGE_ODOMETRY_H_

#include <bitset>


class CANMessage_Odometry : public CANMessage {


public:
	CANMessage_Odometry(){
		setID(0x190);
		setLength(8);
		setType(0x00);
	};

	~CANMessage_Odometry(){

	};

	void set_msg(float vx, float vy, float v_yaw, bool bolts_down, bool bolts_up, bool handling, bool navigation, bool autonomy, bool error, bool pulse){
		set_velocity(vx, vy, v_yaw);
		set_cfg_bits(bolts_down, bolts_up, handling, navigation, autonomy, error, pulse);
	};

private:

	void set_velocity(float vx, float vy, float v_yaw){

		// converts (float) [m/s] into (short) [mm/s]
		short short_vx = (short) (10000*vx);
		short short_vy = (short) (10000*vy);
		short short_v_yaw = (short) (100*v_yaw);

		// converts (short) into (BYTE)
		BYTE b_vx[2];
		b_vx[0] = short_vx & 0xff;				// high
		b_vx[1] = (short_vx >> 8) & 0xff;		// low

		BYTE b_vy[2];
		b_vy[0] = short_vy & 0xff;				// high
		b_vy[1] = (short_vy >> 8) & 0xff;		// low

		BYTE b_v_yaw[2];
		b_v_yaw[0] = short_v_yaw & 0xff;		// high
		b_v_yaw[1] = (short_v_yaw >> 8) & 0xff;	// low

		// set values in BYTE-array of message
		setAt(b_vx[0], 0);
		setAt(b_vx[1], 1);

		setAt(b_vy[0], 2);
		setAt(b_vy[1], 3);

		setAt(b_v_yaw[0], 4);
		setAt(b_v_yaw[1], 5);
	}

	void set_cfg_bits(bool bolts_down, bool bolts_up, bool handling, bool navigation, bool autonomy, bool error, bool pulse){

		// initialize BYTE with ZEROs
		BYTE cfg = 0x00;

		// set the bits to "HIGH" if specified
		if(bolts_down)
			cfg |= (1<<6);  // set the 6th bit = shift 3 times left

		if(bolts_up)
			cfg |= (1<<5);  // set the 5th bit = shift 3 times left

		if(handling)
			cfg |= (1<<4);	// set the 4th bit = shift 3 times left

		if(navigation)
			cfg |= (1<<3);	// set the 3rd bit = shift 4 times left

		if(autonomy)
			cfg |= (1<<2);	// set the 2nd bit = shift 5 times left

		if(error)
			cfg |= (1<<1);	// set the 1st bit = shift 6 times left

		if(pulse)
			cfg |= (1<<0);	// set the ZERO bit = shift 7 times left

		setAt(cfg, 7);
	}


	/*
	 * converts unsigned integer of bit-size 16 into signed integer
	 *
	 * param unsigned integer of bit-size 16
	 *
	 * reads first bit representing SIGN (-/+)
	 * if negative - inverts and substracts 1
	 * if positive - take value as is
	 *
	 * returns converted Integer value
	 */
	int get_int_from_twos_complement(uint16_t smallInt){

		const int negative = (smallInt & (1 << 15)) != 0;
		int nativeInt;

		if (negative)
		  nativeInt = smallInt | ~((1 << 16) - 1);
		else
		  nativeInt = smallInt;

		return nativeInt;
	}

public:
	void print_values(){

	    uint16_t vx1 = 0;
	    vx1 = msg_bDat_[0] | msg_bDat_[1] << 8;

	    uint16_t vy1 = 0;
	    vy1 = msg_bDat_[2] | msg_bDat_[3] << 8;

	    uint16_t v_yaw1 = 0;
	    v_yaw1 = msg_bDat_[4] | msg_bDat_[5] << 8;


	    int vx = get_int_from_twos_complement(vx1);
	    int vy = get_int_from_twos_complement(vy1);
	    int v_yaw = get_int_from_twos_complement(v_yaw1);

		// converts (float) [m/s] into (short) [0,1 mm/s]
	    float d_vx = ((float) vx)/10000;
	    float d_vy = ((float) vy)/10000;
		// converts (float) [deg/s] into (short) [0,01 deg/s]
	    float d_v_yaw = ((float) v_yaw)/100;

		std::cout << "id=" <<
				std::hex << getID() <<
				std::dec << " len=" << getLength() <<
							" vx=" << d_vx << "[m/s] vy=" << d_vy << "[m/s] v_yaw=" << d_v_yaw << "[deg/s] cfg=" <<
				std::bitset<8>(msg_bDat_[7]) << std::endl;

	}

	/*
	 * get yaw velocity
	 *
	 * extracts BYTEs from BYTE-Array into unsigned int with bit-size 16
	 * converts into 2's-complement representation into signed integer
	 * converts integer into float in [m/s]
	 *
	 * returns yaw-velocity in [m/s]
	 */
	float get_vel_yaw() {

	    uint16_t v_yaw_2s = 0;
	    v_yaw_2s = msg_bDat_[4] | msg_bDat_[5] << 8;
	    int v_yaw = get_int_from_twos_complement(v_yaw_2s);
	    float d_v_yaw = ((float) v_yaw)/100;

		return d_v_yaw;
	}

	/*
	 * get x velocity
	 *
	 * extracts BYTEs from BYTE-Array into unsigned int with bit-size 16
	 * converts into 2's-complement representation into signed integer
	 * converts integer into float in [m/s]
	 *
	 * returns velocity in x-direction [m/s]
	 */
	float get_vel_x() {

	    uint16_t v_x_2s = 0;
	    v_x_2s = msg_bDat_[0] | msg_bDat_[1] << 8;
	    int v_x = get_int_from_twos_complement(v_x_2s);
	    float d_v_x = ((float) v_x)/10000;

		return d_v_x;
	}

	/*
	 * get y velocity
	 *
	 * extracts BYTEs from BYTE-Array into unsigned int with bit-size 16
	 * converts into 2's-complement representation into signed integer
	 * converts integer into float in [m/s]
	 *
	 * returns velocity in y-direction [m/s]
	 */
	float get_vel_y() {

	    uint16_t v_y_2s = 0;
	    v_y_2s = msg_bDat_[2] | msg_bDat_[3] << 8;
	    int v_y = get_int_from_twos_complement(v_y_2s);
	    float d_v_y = ((float) v_y)/10000;

		return d_v_y;
	}


	/*
	* get protective field - host
	*/
	bool get_protective_field_guest() {
		   return (msg_bDat_[6] & (1 << 4)) != 0;
	}


	/*
	* get protective field - host
	*/
	bool get_protective_field_host() {
		   return (msg_bDat_[6] & (1 << 3)) != 0;
	}

	/*
	* get warning field 2 - guest
	*/
	bool get_warning_field_2_guest() {
		   return (msg_bDat_[6] & (1 << 2)) != 0;
	}

	/*
	* get warning field 1 - guest
	*/
	bool get_warning_field_1_guest() {
		   return (msg_bDat_[6] & (1 << 1)) != 0;
	}

	/*
	* get warning field 2 - host
	*/
	bool get_warning_field_2_host() {
		   return (msg_bDat_[6] & (1 << 0)) != 0;
	}

	/*
    * get warning field 1 - host
    */
    bool get_warning_field_1_host() {
    	   return (msg_bDat_[7] & (1 << 7)) != 0;
    }

    /*
    * get bolts_down
    */
    bool get_bolts_down() {
    	   return (msg_bDat_[7] & (1 << 6)) != 0;
    }

    /*
    * get bolts_up
    */
    bool get_bolts_up() {
    	return (msg_bDat_[7] & (1 << 5)) != 0;
    }

	/*
	* get bolts_down
	*/
	bool get_handling() {
		return (msg_bDat_[7] & (1 << 4)) != 0;
    }

	/*
	* get bolts_up
	*/
	bool get_navigation() {
		return (msg_bDat_[7] & (1 << 3)) != 0;
	}

	/*
	* get bolts_down
	*/
	bool get_autonomy() {
		return (msg_bDat_[7] & (1 << 2)) != 0;
	}

	/*
	* get bolts_up
	*/
	bool get_error() {
		return (msg_bDat_[7] & (1 << 1)) != 0;
	}

	/*
	* get bolts_up
	*/
	bool get_pulse() {
		return (msg_bDat_[7] & (1 << 0)) != 0;
	}
};

#endif

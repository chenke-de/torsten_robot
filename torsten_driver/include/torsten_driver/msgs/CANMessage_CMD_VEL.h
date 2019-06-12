#ifndef _CANMESSAGE_CMD_VEL_H_
#define _CANMESSAGE_CMD_VEL_H_

#include <bitset>

class CANMessage_Cmd_Vel : public CANMessage {


public:
	CANMessage_Cmd_Vel(){
		setID(0x210);
		setLength(8);
		setType(0x00);
	};

	~CANMessage_Cmd_Vel(){

	};

	/*
	 * \param vx - Traversal Velocitx in x-direction [m/sec]
	 * \param vy - Traversal Velocitx in y-direction [m/sec]
	 * \param v_yaw - Rotational Velocitx  [deg/sec]
	 * \param load - is the vehicle loaded [true] or unloaded [false]
	 * \param handling - ?
	 * \param navigation - ?
	 * \param autonomy - is the vehicle in autonomous mode [true] or not [false]
	 * \param error - is the vehicle in an error state [true] or not [false]
	 * \param pulse - ?
	 */
	void set_msg(float vx, float vy, float v_yaw, bool load, bool handling, bool navigation, bool autonomy, bool error, bool pulse){

		set_velocity(vx, vy, v_yaw);
		set_cfg_bits(load, handling, navigation, autonomy, error, pulse);

	};


	// TODO - safe velocity !!!

private:

	/* Casts float (into short and then into BYTE) and sets corresponding BYTEs of msg
	 *
	 * \param vx - lat vel in x-direction [m/s]
	 * \param vy - lat vel in y-direction [m/s]
	 * \param v_yaw - rotational velocity [deg/s]
	 */
	void set_velocity(float vx, float vy, float v_yaw){

		// converts (float) [m/s] into (short) [0,1 mm/s]
		short short_vx = (short) (10000*vx);
		short short_vy = (short) (10000*vy);
		// converts (float) [deg/s] into (short) [0,01 deg/s]
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

	void set_cfg_bits(bool load, bool handling, bool navigation, bool autonomy, bool error, bool pulse){

		// initialize BYTE with ZEROs
		BYTE cfg = 0x00;

		// set the bits to "HIGH" if specified
		if(load)
			cfg |= (1<<5);	// set the 5th bit = shift 2 times left

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

	uint16_t get_twos_complement(int value){

		return ( ~value +1 );	// negate all values and add one to get two's complement
	}

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

};

#endif

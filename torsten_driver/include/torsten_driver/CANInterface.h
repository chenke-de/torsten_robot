/*
 * CANInterface.h
 *
 *  Created on: 08.09.2016
 *      Author: reuter
 */

#ifndef CANINTERFACE_H_
#define CANINTERFACE_H_

#define CAN_BAUDRATE_1M 	0x0
#define CAN_BAUDRATE_500K	0x2
#define CAN_BAUDRATE_250K 	0x4
#define CAN_BAUDRATE_125K	0x6
#define CAN_BAUDRATE_50K	0x9
#define CAN_BAUDRATE_20K	0xB
#define CAN_BAUDRATE_10K	0xD

#include "CANMessage.h"

class CANInterface {
public:
	virtual ~CANInterface(){};

	virtual bool init_connection() = 0;

	virtual bool close_connection() = 0;

	virtual bool send_msg(CANMessage msg) = 0;

	virtual bool receive_msg(CANMessage& msg) = 0;


};

#endif /* CANINTERFACE_H_ */



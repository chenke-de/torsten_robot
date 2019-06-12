/*
 * CANPeakUSB.h
 *
 *  Created on: 08.09.2016
 *      Author: reuter
 */

#ifndef SRC_CANPEAKUSB_H_
#define SRC_CANPEAKUSB_H_
#define linux __linux__		// 'linux'-flag is expected by PCANBasic.h for including Windows BYTE etc definitions

#include <stdio.h>
#include <vector>

#include "CANInterface.h"
#include "PCANBasic.h"

#include <unistd.h>


class CANPeakUSB : public CANInterface{

private:

	std::vector<TPCANMsg> CANMsgQ_;
	TPCANMsg CANMsg_;
	TPCANStatus CANStatus_;
	TPCANHandle CANHandle_ = PCAN_USBBUS1;
	TPCANBaudrate CANBaudrate_ = PCAN_BAUD_500K;



public:
	CANPeakUSB(){

	};

	~CANPeakUSB(){

	};

	/*
	 * Initializes a CAN connection
	 * - uses global parameters
	 * -- TPCANHandle CANHandle_ = PCAN_USBBUS1 - specification which CAN Channel to be used
	 * -- TPCANBaudrate CANBaudrate_ = PCAN_BAUD_500K - specification of Baudrate to be used
	 * returns [true] for a successful connection [false] otherwise
	 */
	bool init_connection(){

		 TPCANStatus status;

		 // initialize connection
		 status = CAN_Initialize(
				 	 	 	 CANHandle_,		// TPCANHandle Channel
							 CANBaudrate_,		// TPCANBaudrate Baud rate
							 0,					// TPCANType HwType
							 0,					// DWORD IOPort
							 0					// Word Interrupt
		 );

		// return status
		return (status == PCAN_ERROR_OK);
	};


	/*
	 * Closes a CAN connection
	 * - uses global parameters
	 * -- TPCANHandle CANHandle_ = PCAN_USBBUS1 - specification which CAN Channel to be used
	 * -- TPCANBaudrate CANBaudrate_ = PCAN_BAUD_500K - specification of Baudrate to be used
	 * 	 returns [true] for a successfully closed connection [false] otherwise
	 */
	bool close_connection(){

		TPCANStatus status;

		// close connection
		status = CAN_Uninitialize( CANHandle_ );		// TPCANHandle Channel

		// return status
		return (status == PCAN_ERROR_OK);
	};


	/*
	 * Sends a msg over the can channel
	 * param msg - msg to be send
	 * returns [true] for successful delivery of a message [false] otherwise
	 */
	bool send_msg(CANMessage msg){

		TPCANStatus status;
		TPCANMsg CANMsg;

		CANMsg.ID = msg.getID();
		CANMsg.MSGTYPE = (BYTE) msg.getType();
		CANMsg.LEN = msg.getLength();
        for(int i=0; i<8; i++)
                CANMsg.DATA[i] = msg.getAt(i);

		status = CAN_Write(CANHandle_, &CANMsg);


		// return status
		return (status == PCAN_ERROR_OK);
	};

	/*
	 * Receive a msg from a channel
	 * param msg - reference to msg where the received msg will be saved in
	 * returns [true] for successful reception of a message [false] otherwise
	 */
	bool receive_msg(CANMessage& msg){

		TPCANStatus status;
		TPCANMsg CANMsg;


		/* TODO
		 *
		 * THE FOLLOWING CODE IS BLOCKING THE APPLICATION !!!
		 * SWITCH TO EVENT BASED READING
		 *
		 */
		// read Message
		while( (status = CAN_Read( CANHandle_, &CANMsg, NULL )) == PCAN_ERROR_QRCVEMPTY );

		msg.setID(CANMsg.ID);
		msg.setType(CANMsg.MSGTYPE);
		msg.setLength(CANMsg.LEN);
        for(int i=0; i<8; i++)
                msg.setAt( CANMsg.DATA[i], i );

		// return status
		return (status == PCAN_ERROR_OK);
	};

	bool reset(){
		return (CAN_Reset(CANHandle_) == PCAN_ERROR_OK);
	}
};

#endif /* SRC_CANPEAKUSB_H_ */

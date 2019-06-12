/*
 * CANPeakUSB.h
 *
 *  Created on: 08.09.2016
 *      Author: reuter
 */

#ifndef SRC_PEAKCANUSB_H_
#define SRC_PEAKCANUSB_H_

#include <stdio.h>
#include <vector>

#include "CANInterface.h"

#include <libpcan.h>
#include <fcntl.h>

#include <unistd.h>


class PeakCANUSB : public CANInterface{

private:

	HANDLE CANHandle_;
	int CANBaudrate_ = CAN_BAUD_500K;
	int CANMsgType_ = CAN_INIT_TYPE_ST;


public:
	PeakCANUSB(){

	};

	~PeakCANUSB(){

	};

	/*
	 * Initializes a CAN connection
	 * - uses global parameters
	 * -- TPCANHandle CANHandle_ = PCAN_USBBUS1 - specification which CAN Channel to be used
	 * -- TPCANBaudrate CANBaudrate_ = PCAN_BAUD_500K - specification of Baudrate to be used
	 * returns [true] for a successful connection [false] otherwise
	 */
	bool init_connection(){

		 int status;

		 CANHandle_ = LINUX_CAN_Open("/dev/pcan32", O_RDWR);


		 // initialize connection
		 status = CAN_Init(
				 	 	 	 CANHandle_,		// HANDLE for device "/dev/pcan"
							 CANBaudrate_,		// (int) Baud rate
							 CANMsgType_		// (int) Message Type [Standard or Extended]
		 );

		// return status
		return (status == CAN_ERR_OK);
	};


	/*
	 * Closes a CAN connection
	 * - uses global parameters
	 * -- TPCANHandle CANHandle_ = PCAN_USBBUS1 - specification which CAN Channel to be used
	 * -- TPCANBaudrate CANBaudrate_ = PCAN_BAUD_500K - specification of Baudrate to be used
	 * 	 returns [true] for a successfully closed connection [false] otherwise
	 */
	bool close_connection(){

		int status;

		// close connection
		status = CAN_Close(CANHandle_);		// Handle for device

		// return status
		return (status == CAN_ERR_OK);
	};


	/*
	 * Sends a msg over the can channel
	 * param msg - msg to be send
	 * returns [true] for successful delivery of a message [false] otherwise
	 */
	bool send_msg(CANMessage msg){

		int status;
		TPCANMsg CANMsg;

		CANMsg.ID = msg.getID();
		CANMsg.MSGTYPE = (BYTE) msg.getType();
		CANMsg.LEN = msg.getLength();
        for(int i=0; i<8; i++)
                CANMsg.DATA[i] = msg.getAt(i);

		status = LINUX_CAN_Write_Timeout(CANHandle_, &CANMsg, 0);


		// return status
		return (status == CAN_ERR_OK);
	};

	/*
	 * Receive a msg from a channel
	 * param msg - reference to msg where the received msg will be saved in
	 * returns [true] for successful reception of a message [false] otherwise
	 */
	bool receive_msg(CANMessage& msg){

		int status;
		TPCANRdMsg CANRdMsg;

		// read Message
		status = LINUX_CAN_Read_Timeout( CANHandle_, &CANRdMsg, 0);

		if( status == CAN_ERR_OK ){	// everything is fine

			msg.setID(CANRdMsg.Msg.ID);
			msg.setType(CANRdMsg.Msg.MSGTYPE);
			msg.setLength(CANRdMsg.Msg.LEN);
	        for(int i=0; i<8; i++){
	                msg.setAt( CANRdMsg.Msg.DATA[i], i );
			}
			return true;
		}
		else if( status == CAN_ERR_QRCVEMPTY ){	// Msg Queue is empty
			//		std::cout << "PeakCanUsb - Receive-queue empty on read" << std::endl;
		}
		else if(status == CAN_ERR_QOVERRUN){ // Msg Queue is full
			std::cout << "PeakCanUsb - Receive-que overrun" << std::endl;
		}

		return false;
	};

};

#endif /* SRC_PEAKCANUSB_H_ */

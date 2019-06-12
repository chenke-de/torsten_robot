#ifndef _CANMESSAGE_H
#define _CANMESSAGE_H

#include <iostream>
#include <stdio.h>
#include <bitset>
#include <iomanip>

/**
 * Defenition of a CAN message.
 */
class CANMessage
{
public:
	typedef unsigned char BYTE;

private:
	int msg_ID_;
	int msg_Len_;
	int msg_Type_;

protected:
	/** A CAN message consists of eight bytes. */
	BYTE msg_bDat_[8];

public:
	/**
	 * Default constructor.
	 */
	CANMessage()
	{
		msg_ID_ 	= 	0x00;
		msg_Len_ 	= 	8;
		msg_Type_ 	=   0x00;	// MSGTYPE_STANDARD
	}

	~CANMessage(){

	}

	/**
	 * Gets the bytes of the telegram.
	 */
	void get(BYTE* pData0, BYTE* pData1, BYTE* pData2, BYTE* pData3, BYTE* pData4, BYTE* pData5, BYTE* pData6, BYTE* pData7)
	{
		*pData0 = msg_bDat_[0];
		*pData1 = msg_bDat_[1];
		*pData2 = msg_bDat_[2];
		*pData3 = msg_bDat_[3];
		*pData4 = msg_bDat_[4];
		*pData5 = msg_bDat_[5];
		*pData6 = msg_bDat_[6];
		*pData7 = msg_bDat_[7];
	}

	/**
	 * Returns a spezific byte of the telegram.
	 * @param Pos number of the byte.
	 */
	int getAt(int Pos)
	{
		return msg_bDat_[Pos];
	}

	/**
	 * Prints the telegram.
	 */
	void print()
	{

//		// prints last BYTE bit-wise
//		std::bitset<8> mybits = msg_bDat_[7];
//		std::string mystring =	mybits.to_string<char,std::string::traits_type,std::string::allocator_type>();
//
//        printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %s\n",
//                (int)msg_ID_, (int)msg_Len_,
//                (int)msg_bDat_[0], (int)msg_bDat_[1],
//                (int)msg_bDat_[2], (int)msg_bDat_[3],
//                (int)msg_bDat_[4], (int)msg_bDat_[5],
//                (int)msg_bDat_[6], mystring.c_str() );

        printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
                (int)msg_ID_, (int)msg_Len_,
                (int)msg_bDat_[0], (int)msg_bDat_[1],
                (int)msg_bDat_[2], (int)msg_bDat_[3],
                (int)msg_bDat_[4], (int)msg_bDat_[5],
                (int)msg_bDat_[6], (int)msg_bDat_[7] );
	}


	/**
	 * Get the identifier stored in this message structure.
	 * @return the message identifier.
	 */
	int getID()
	{
		return msg_ID_;
	}

	/**
	 * Get the message length set within this data structure.
	 * @return The message length in the range [0..8].
	 */
	int getLength()
	{
		return msg_Len_;
	}

	/**
	 * Get the message type. By default, the type is 0x00.
	 * @return The message type.
	 */
	int getType()
	{
		return msg_Type_;
	}

	/**
	 * Sets the bytes to the telegram.
	 */
	void set(BYTE Data0=0, BYTE Data1=0, BYTE Data2=0, BYTE Data3=0, BYTE Data4=0, BYTE Data5=0, BYTE Data6=0, BYTE Data7=0)
	{
		msg_bDat_[0] = Data0;
		msg_bDat_[1] = Data1;
		msg_bDat_[2] = Data2;
		msg_bDat_[3] = Data3;
		msg_bDat_[4] = Data4;
		msg_bDat_[5] = Data5;
		msg_bDat_[6] = Data6;
		msg_bDat_[7] = Data7;
	}

	/**
	 * Set the byte at the given position.
	 */
	void setAt(BYTE data, int Pos)
	{
		msg_bDat_[Pos] = data;
	}

	/**
	 * Set the message identifier within this message structure.
	 * @param id The message identifier. Its value must be in the range [0..2047], i.e.
	 * 29-bit identifiers are not supported here.
	 */
	void setID(int id)
	{
		if( (0 <= id) && (id <= 2047) )
			msg_ID_ = id;
	}

	/**
	 * Set the message length within this message structure.
	 * @param len The message length. Its value must be in the range [0..8].
	 */
	void setLength(int len)
	{
		if( (0 <= len) && (len <= 8) )
			msg_Len_ = len;
	}

	/**
	 * Set the message type. By default, the type is 0x00.
	 * @param type The message type.
	 */
	void setType(int type)
	{
		msg_Type_ = type;
	}


};
//-----------------------------------------------
#endif

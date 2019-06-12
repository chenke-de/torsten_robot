//  pcanwrite.cpp
//
//  ~~~~~~~~~~~~
//
//  PCANBasic Example: Simple Write
//
//  ~~~~~~~~~~~~
//
//  ------------------------------------------------------------------
//  Author : Thomas Haber (thomas@toem.de)
//  Last change: 18.06.2010
//
//  Language: C++
//  ------------------------------------------------------------------
//
//  Copyright (C) 1999-2010  PEAK-System Technik GmbH, Darmstadt
//  more Info at http://www.peak-system.com
//  ------------------------------------------------------------------
//
// linux@peak-system.com
// www.peak-system.com
//
//  ------------------------------------------------------------------
//  History:
//  07-11-2013 Stephane Grosjean
//  - Move DWORD definition from "unsigned long" to "__u32" to run on 64-bits
//    Kernel
//  - Change initital bitrate from 250K to 500K (default pcan driver bitrate)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//

#include <stdio.h>
#include <unistd.h>
#include <asm/types.h>

/*
#define DWORD  __u32
#define WORD   unsigned short
#define BYTE   unsigned char
#define LPSTR  char*
*/
#include "../pcanbasic/PCANBasic.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Main entry-point for this application. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="argc">	The argc. </param>
/// <param name="argv">	[in,out] If non-null, the argv. </param>
///
/// <returns>	. </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <pthread.h>

#include <unistd.h>

#define PCAN_DEVICE             PCAN_USBBUS1

/*
int main(int argc, char* argv[]){
  pthread_t f1_thread; 
  void* _main(void*);
  pthread_create(&f1_thread,NULL,_main,NULL);
  pthread_join(f1_thread,NULL);
}
*/
//void* _main(void*)
int main(int argc, char* argv[])
{
        TPCANMsg Message;
        TPCANStatus Status;
        unsigned long ulIndex = 0;

        Status = CAN_Initialize(PCAN_DEVICE, PCAN_BAUD_500K, 0, 0, 0);
        printf("Initialize CAN: %x\n",(int)Status);

        Message.ID = 0x210;
        Message.LEN = 8;
        Message.MSGTYPE = PCAN_MESSAGE_STANDARD;
        Message.DATA[0]=0;

        bool even = false;

        while(1){
            while ((Status=CAN_Write(PCAN_DEVICE,&Message)) == PCAN_ERROR_OK) {

                even = !even;

                Message.DATA[0]= 0xd4;
                Message.DATA[1]= 0xef;
                Message.DATA[2]= 0x00;
                Message.DATA[3]= 0x00;
                Message.DATA[4]= 0x00;
                Message.DATA[5]= 0x00;
                Message.DATA[6]= 0x00;
                if(even){
                        Message.DATA[7]= 0x00;
                }   
                else{
                        Message.DATA[7]= 0x01;
                }

                ulIndex++;

                printf("Sending  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
                        (int)Message.ID, (int)Message.LEN,
                        (int)Message.DATA[0], (int)Message.DATA[1],
                        (int)Message.DATA[2], (int)Message.DATA[3],
                        (int)Message.DATA[4], (int)Message.DATA[5],
                        (int)Message.DATA[6], (int)Message.DATA[7]);

                usleep(20000);

            }
        }

        printf("STATUS %i\n", (int)Status);

        return 0;
}



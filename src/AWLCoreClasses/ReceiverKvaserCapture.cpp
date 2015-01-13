/* ReceiverKvaserCapture.cpp */
/*
	 Some parts Copyright 2014 Aerostar R&D Canada Inc.

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

/*
 * Some portions:
**                         Copyright 1996-98 by KVASER AB            
**                   P.O Box 4076 SE-51104 KINNAHULT, SWEDEN
**             E-mail: staff@kvaser.se   WWW: http://www.kvaser.se
**
** This software is furnished under a license and may be used and copied
** only in accordance with the terms of such license.
**
*/

#include <string>
#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#endif

#include <canlib.h> // Kvaser CAN library

#include "DebugPrintf.h"


#include "DetectionStruct.h"
#include "ReceiverCANCapture.h"

#include "ReceiverKvaserCapture.h"

using namespace std;
using namespace awl;

const int receiveTimeOutInMillisec = 500;  // Default is 1000. As AWL refresh rate is 100Hz, this should not exceed 10ms
const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm ports every repoenPortDelayMillisec, 
										   // To see if the system reconnects

const int defaultKvaserBitRate = canBITRATE_1M;

ReceiverKvaserCapture::ReceiverKvaserCapture(int receiverID, int inReceiverChannelQty, const int &inKvaserChannel, 
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, const AlgorithmSet &inParametersAlgos):
ReceiverCANCapture(receiverID, inReceiverChannelQty, inFrameRate, inChannelMask, inMessageMask, inRangeOffset,  inRegistersFPGA, inRegistersADC, inRegistersGPIO, inParametersAlgos),
canChannelID(inKvaserChannel),
kvaserHandle(-1),
closeCANReentryCount(0)

{

}


ReceiverKvaserCapture::ReceiverKvaserCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCANCapture(receiverID, propTree),
canChannelID(-1),
kvaserHandle(-1),
closeCANReentryCount(0)

{
	// Read the configuration from the configuration file
	ReadConfigFromPropTree(propTree);
	ReadRegistersFromPropTree(propTree);
}

ReceiverKvaserCapture::~ReceiverKvaserCapture()
{
	CloseDebugFile(debugFile);
	EndDistanceLog();
	Stop(); // Stop the thread
}
bool  ReceiverKvaserCapture::OpenCANPort()

{
	canStatus status = canOK;

	// What do we do if the port is already opened?
	CloseCANPort();

	reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);


	// Initialize the KVaser CAN Library.  This can be called multiple times.
	canInitializeLibrary();

	//
    // Open a handle to the CAN circuit. Specifying
    // canOPEN_EXCLUSIVE ensures we get a circuit that no one else
    // is using.
    //
	int canChannelCount = -1;
	status = canGetNumberOfChannels(&canChannelCount);
    if (status  < 0) 
	{
        CheckStatus("canOpenChannel", (canStatus)kvaserHandle);
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
		return(false);
	}

	kvaserHandle = canOpenChannel(canChannelID, canOPEN_EXCLUSIVE);
//	kvaserHandle = canOpenChannel(canChannelID, canOPEN_ACCEPT_VIRTUAL | canOPEN_REQUIRE_INIT_ACCESS);


    if (kvaserHandle < 0) 
	{
        CheckStatus("canOpenChannel", (canStatus)kvaserHandle);
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
		return(false);
	}

    //
    // Using our new shiny handle, we specify the baud rate
    // The bit layout is in depth discussed in most CAN
    // controller data sheets, and on the web at
    // http://www.kvaser.se.
    //
    status = canSetBusParams(kvaserHandle, defaultKvaserBitRate, 0, 0, 0, 0, 0);
    if (status < 0) 
	{
       CheckStatus("canSetBusParam", status);
       reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
	   return(false);
    }

    //
    // Then we start the ball rolling.
    // 
    status = canBusOn(kvaserHandle);
    if (status < 0) 
	{
       CheckStatus("canBusOn", status);
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
		return(false);
    }
 

	bFrameInvalidated = false;
	return(true);
}

bool  ReceiverKvaserCapture::CloseCANPort()

{
	canStatus status;

	if (closeCANReentryCount > 0) return(false);

	closeCANReentryCount++;

	//
    // That's all for today!
    //

    // These two are not really necessary but for the completeness...
	if (kvaserHandle >= 0)
	{
		status = canBusOff(kvaserHandle);
		if (status < 0) 
		{
			CheckStatus("canBusOff", status);
		}

		status = canClose(kvaserHandle);
		if (status < 0) 
		{
			CheckStatus("canClose", status);
		}
	}

	kvaserHandle = -1;

	// Unload the library. This will force reenumeration of the CAN devices on the next canInitializeLibrary()
	canUnloadLibrary();


	reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
    closeCANReentryCount--;
	return(true);
}

//
// Check a status code and issue an error message if the code
// isn't canOK.
//
void ReceiverKvaserCapture::CheckStatus(char* id, canStatus status)
{
    char buf[50];
    if (status != canOK) {
        buf[0] = '\0';
        canGetErrorText(status, buf, sizeof(buf));
        DebugFilePrintf(debugFile, "%s: failed, stat=%d (%s)\n", id, (int)status, buf);
    }
}



typedef struct  {
    long id;        // Message id
    unsigned long timestamp; // timestamp in milliseconds	
    unsigned int  canFlags;     // [extended_id|1][RTR:1][reserver:6]
    unsigned int  len;       // Frame size (0.8)
    uint8_t       data[8]; // Databytes 0..7
} KvaserCanMessage;

void ReceiverKvaserCapture::DoOneThreadIteration()

{
	canStatus status = canOK;

	if (!WasStopped())
    {
		AWLCANMessage msg;
		KvaserCanMessage inMessage;
		float distance;

		if (kvaserHandle >= 0) 
		{

			status = canRead(kvaserHandle, &inMessage.id, inMessage.data, &inMessage.len, &inMessage.canFlags, &inMessage.timestamp);
			if (status != canOK && status != canERR_NOMSG) 
			{
				CheckStatus("canRead", status);
			}

			if (status == canOK)
			{
				reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);

				msg.id = (unsigned long) inMessage.id;
				msg.timestamp = (unsigned long) inMessage.timestamp;
				msg.len = (int) inMessage.len;
				for (int i = 0; i < 8; i++)
				{
					msg.data[i] = inMessage.data[i]; 
				}

				ParseMessage(msg);
			}
			// Could not read. Time out after a certain delay and try to reopen.
			else if (status == canERR_NOMSG) 
			{
				if (boost::posix_time::microsec_clock::local_time() > reconnectTime)
				{
					DebugFilePrintf(debugFile,  "Reconnecting CAN Port"); 
					if (OpenCANPort())
					{
						WriteCurrentDateTime();
						ReceiverCapture::SetMessageFilters();
					}
				}
			}
#if 0
			// canRead returned an error.  
			// This means we have an error on the port.
			// Try to repoen the port
			else 
			{
				DebugFilePrintf(debugFile,  "Error on canRead.  Resetting CAN Port"); 
				if (OpenCANPort())
				{
					WriteCurrentDateTime();
					ReceiverCapture::SetMessageFilters();
				}
			}
#else
			// canRead returned an error.  
			// This means we have an error on the port.
			// Let the port reconnect after time-out naturally
			else 
			{
				DebugFilePrintf(debugFile,  "Error on canRead.  Resetting CAN Port"); 
				CloseCANPort();
				reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
			}
#endif
		} // if (kvaserHandle > 0)
		else //(KVaserHandle <= 0)
		{
			// Port is not opened.  Try to repoen after a certain delay.
			if (boost::posix_time::microsec_clock::local_time() > reconnectTime)
			{
				DebugFilePrintf(debugFile,  "Reconnecting CAN Port"); 
				if (OpenCANPort())
				{
					WriteCurrentDateTime();
					ReceiverCapture::SetMessageFilters();
				}
			}
		}

	} // if  (!WasStoppped)
}

bool ReceiverKvaserCapture::WriteMessage(const AWLCANMessage &inMsg)
{
	std::string outResponse;
	
	if (kvaserHandle < 0) return(false);

    canStatus status = canWrite(kvaserHandle, inMsg.id, (void *) &inMsg.data, 8, 0);
	if (status < 0)  
	{
		CheckStatus("canWrite", status);
	}

	return(true);
}


bool ReceiverKvaserCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
		ReceiverCANCapture::ReadConfigFromPropTree(propTree);

		char receiverKeyString[32];
		sprintf(receiverKeyString, "config.receivers.receiver%d", receiverID);
		std::string receiverKey = receiverKeyString;

		boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
		// Communication parameters
		canChannelID  =  receiverNode.get<int>("kvaserChannel");

		return(true);
}


/* ReceiverPosixTTYCapture.cpp */
/*
	Copyright 2014, 2015 Phantom Intelligence Inc.

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

#include <stdio.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include <string>
#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp> 
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#endif

#include "DebugPrintf.h"
#include "BlockingReader.h"

#include "DetectionStruct.h"
#include "ReceiverCANCapture.h"
#include "ReceiverPosixTTYCapture.h"


using namespace std;
using namespace awl;

const int receiveTimeOutInMillisec = 500;  // Default is 1000. As AWL refresh rate is 100Hz, this should not exceed 10ms
const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm fds every repoenPortDelayMillisec, 
										   // To see if the system reconnects


ReceiverPosixTTYCapture::ReceiverPosixTTYCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround, 
	                   const std::string &inTtyName, const ReceiverCANCapture::eReceiverCANRate inCANBitRate,
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, const AlgorithmSet &inParametersAlgos,
					   const AlgorithmSet &inParametersTrackers) :
ReceiverCANCapture(receiverID, inReceiverChannelQty, inReceiverColumns, inReceiverRows, inLineWrapAround, 
                   canRate1Mbps, inFrameRate, inChannelMask, inMessageMask, inRangeOffset,  inRegistersFPGA, inRegistersADC, inRegistersGPIO, 
				   inParametersAlgos, inParametersTrackers),
fd(-1),
closeCANReentryCount(0)
{
	// Update settings from application
	ttyName = inTtyName;

}


ReceiverPosixTTYCapture::ReceiverPosixTTYCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCANCapture(receiverID, propTree),
fd(-1),
closeCANReentryCount(0)

{
	// Read the configuration from the configuration file
	ReadConfigFromPropTree(propTree);
	ReadRegistersFromPropTree(propTree);
}

ReceiverPosixTTYCapture::~ReceiverPosixTTYCapture()
{
	CloseDebugFile(debugFile);
	EndDistanceLog();
	Stop(); // Stop the thread
}

bool  ReceiverPosixTTYCapture::OpenCANPort()
{
	int ret;

	if (fd < 0) 
	{
		CloseCANPort();
	}

	fd = open(ttyName.c_str(), O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		perror("TTY open");
		goto posixtty_exit;
	}

	printf("UDP Using port %s\n", ttyName.c_str());

	return true;

posixtty_exit:
	{
		fd = -1;

		std::string sErr = " Cannot open PosixTTY port ";
		sErr += ttyName;
		fprintf(stderr, sErr.c_str());
		
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
		return(false);
	}


	bFrameInvalidated = false;
}

bool  ReceiverPosixTTYCapture::CloseCANPort()

{
	if (closeCANReentryCount > 0) return(false);

	closeCANReentryCount++;

		if (fd < 0) close(fd);
		fd = -1;
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
	    closeCANReentryCount--;
		return(true);
}


void ReceiverPosixTTYCapture::DoOneThreadIteration()

{
	uint8_t buffer[256];
	uint32_t *buf32;
	size_t size = sizeof(buffer);
	int ret;

	buf32 = (uint32_t*)buffer;

	AWLCANMessage msg;

	if (fd >= 0)
	{
		ret = read(fd, (void*)buffer, size);

		if (ret < 0)
		{
			//perror("CAN read");
		}
		else
		{
			//	if (buffer.can_id == MSG_CONTROL_GROUP) {
			//process_cmd(awl, buffer.can_id, buffer.data, buffer.can_dlc);
			msg.id  = buf32[0];
			msg.len = ret - sizeof(uint32_t);
			for (int i = 0; i < 8 && i < msg.len; i ++) {
				msg.data[i] = buffer[sizeof(uint32_t)+i];
			}
			ParseMessage(msg);
		}
	}
}

bool ReceiverPosixTTYCapture::WriteMessage(const AWLCANMessage &inMsg)
{
	uint8_t buffer[256];
	uint32_t* buf32;
	size_t size = sizeof(buffer);
	size_t offset = 0;
	int ret;

	if (fd < 0) return(false);

	if (inMsg.id != 80) {
		buf32 = (uint32_t*)buffer;
		buf32[0] = inMsg.id;
		offset = sizeof(uint32_t);
	}

	for (int i = 0; i < 8 && i < inMsg.len; i++) {
		buffer[offset + i] = inMsg.data[i];
	}

	ret = write(fd, (char*)buffer, offset + inMsg.len);

	return(true);
}



bool ReceiverPosixTTYCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
	ReceiverCANCapture::ReadConfigFromPropTree(propTree);

	char receiverKeyString[32];
	sprintf(receiverKeyString, "config.receivers.receiver%d", receiverID);
	std::string receiverKey = receiverKeyString;

	boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
	// Communication parameters
	ttyName =  receiverNode.get<std::string>("posixTTYName");

	return(true);
}


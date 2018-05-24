/* ReceiverSocketCANCapture.cpp */
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
#include <linux/can.h>

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
#include "ReceiverSocketCANCapture.h"


using namespace std;
using namespace awl;

const int receiveTimeOutInMillisec = 500;  // Default is 1000. As AWL refresh rate is 100Hz, this should not exceed 10ms
const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm fds every repoenPortDelayMillisec, 
										   // To see if the system reconnects


ReceiverSocketCANCapture::ReceiverSocketCANCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround, 
	                   const std::string &inDevicePath, const ReceiverCANCapture::eReceiverCANRate inCANBitRate,
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, const AlgorithmSet &inParametersAlgos,
					   const AlgorithmSet &inParametersTrackers) :
ReceiverCANCapture(receiverID, inReceiverChannelQty, inReceiverColumns, inReceiverRows, inLineWrapAround, 
                   inCANBitRate, inFrameRate, inChannelMask, inMessageMask, inRangeOffset,  inRegistersFPGA, inRegistersADC, inRegistersGPIO, 
				   inParametersAlgos, inParametersTrackers),
fd(-1),
closeCANReentryCount(0)
{
	// Update settings from application
	sCANDevicePath = inDevicePath;


	ConvertSocketCANCANBitRateCode();
}


ReceiverSocketCANCapture::ReceiverSocketCANCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCANCapture(receiverID, propTree),
fd(-1),
closeCANReentryCount(0)

{
	// Read the configuration from the configuration file
	ReadConfigFromPropTree(propTree);
	ReadRegistersFromPropTree(propTree);

	// Default values that are not in the configuration file anymore
	ConvertSocketCANCANBitRateCode();
}

ReceiverSocketCANCapture::~ReceiverSocketCANCapture()
{
	CloseDebugFile(debugFile);
	EndDistanceLog();
	Stop(); // Stop the thread
}
bool  ReceiverSocketCANCapture::OpenCANPort()

{
	struct ifreq ifr;
	int ret;
	sockaddr_can can_addr;

	if (fd < 0) 
	{
		CloseCANPort();
	}


	fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (fd < 0) {
		goto socketcan_exit;
	}

	ret = fcntl(fd, F_GETFL, 0);
	if (ret >= 0) {
		ret = fcntl(fd, F_SETFL, ret | O_NONBLOCK);
	}

	can_addr.can_family = AF_CAN;

	strcpy(ifr.ifr_name, sCANDevicePath.c_str());
	ret = ioctl(fd, SIOCGIFINDEX, &ifr);
	if (ret < 0) {
		perror("CAN SIOCGIFINDEX");
		close(fd);
		fd = -1;
		goto socketcan_exit;
	}
	can_addr.can_ifindex = ifr.ifr_ifindex;
	if (!can_addr.can_ifindex) {
		perror("CAN invalid interface");
		close(fd);
		fd = -1;
		goto socketcan_exit;
	}

	printf("CAN Using interface %s\n", sCANDevicePath.c_str());

	ret = bind(fd, (struct sockaddr*)&can_addr, sizeof(can_addr));
	if (ret < 0) {
		perror("CAN bind");
		close(fd);
		fd = -1;
		goto socketcan_exit;
	}

	return true;

socketcan_exit:
	{
		fd = -1;

		std::string sErr = " Cannot open SocketCAN device ";
		sErr += sCANDevicePath;
		fprintf(stderr, sErr.c_str());
		
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
		return(false);
	}


	bFrameInvalidated = false;
}

bool  ReceiverSocketCANCapture::CloseCANPort()

{
	if (closeCANReentryCount > 0) return(false);

	closeCANReentryCount++;

		if (fd < 0) close(fd);
		fd = -1;
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
	    closeCANReentryCount--;
		return(true);
}


void ReceiverSocketCANCapture::DoOneThreadIteration()

{
	struct can_frame cf;
	size_t size = sizeof(cf);
	int ret;

	AWLCANMessage msg;

	if (fd >= 0)
	{
		ret = read(fd, &cf, size);
		if (ret < 0)
		{
			//perror("CAN read");
		}
		else
		{
			//	if (cf.can_id == MSG_CONTROL_GROUP) {
			//process_cmd(awl, cf.can_id, cf.data, cf.can_dlc);
			msg.id  = cf.can_id;
			msg.len = cf.can_dlc;
			for (int i = 0; i < 8 && i < msg.len; i ++) {
				msg.data[i] = cf.data[i];
			}
			if (0 && cf.can_id > 0 && cf.can_dlc > 0) {	
				printf ("CAN id %03x", cf.can_id);
				for (int i = 0; i < 8 && i < cf.can_dlc; i++) {
					printf (" %02x", cf.data[i]);
				}
				printf ("\n");
			}
		

			ParseMessage(msg);
		}
	}
}

bool ReceiverSocketCANCapture::WriteMessage(const AWLCANMessage &inMsg)
{
	struct can_frame cf;
	size_t size = sizeof(cf);
	int ret;

	if (fd < 0) return(false);

	cf.can_id = inMsg.id;
	cf.can_dlc = inMsg.len;

	for (int i = 0; i < 8 && i < inMsg.len; i++) 
	{
		cf.data[i] = inMsg.data[i];
	}

	ret = write(fd, &cf, sizeof(cf));
	
	return(true);
}



bool ReceiverSocketCANCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
	ReceiverCANCapture::ReadConfigFromPropTree(propTree);

	char receiverKeyString[32];
	sprintf(receiverKeyString, "config.receivers.receiver%d", receiverID);
	std::string receiverKey = receiverKeyString;

	boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
	// Communication parameters
	sCANDevicePath =  receiverNode.get<std::string>("socketCANdevice");

	return(true);
}

void ReceiverSocketCANCapture::ConvertSocketCANCANBitRateCode()
{
	switch (canRate)
	{		
	case canRate1Mbps: 
		{
			canBitRate = 1000000;
		}
		break;

	case canRate500kbps:
		{
			canBitRate = 500000;
		}
		break;

	case canRate250kbps:
		{
			canBitRate = 250000;
		}
		break;

	case canRate125kbps:
		{
			canBitRate = 125000;
		}
		break;

	case canRate100kbps:
		{
			canBitRate = 100000;
		}
		break;

	case canRate50kbps:
		{
			canBitRate = 50000;
		}
		break;

	case canRate10kps:
		{
			canBitRate = 10000;
		}
		break;

	default:
		{
			canBitRate = 1000000;  // Default is 1Mbps
		}
		break;
	}
}


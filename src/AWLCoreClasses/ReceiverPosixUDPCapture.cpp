/* ReceiverPosixUDPCapture.cpp */
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
#include "ReceiverPosixUDPCapture.h"


using namespace std;
using namespace awl;

const int receiveTimeOutInMillisec = 500;  // Default is 1000. As AWL refresh rate is 100Hz, this should not exceed 10ms
const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm fds every repoenPortDelayMillisec, 
										   // To see if the system reconnects

const size_t bufferSize = 5192;

ReceiverPosixUDPCapture::ReceiverPosixUDPCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround, 
	                   const std::string &inAddress, int inUDPPort, const ReceiverCANCapture::eReceiverCANRate inCANBitRate,
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
	serverAddress = inAddress;
	serverUDPPort = inUDPPort;
	remoteAddress = 0;
	localAddress = 0;
}


ReceiverPosixUDPCapture::ReceiverPosixUDPCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCANCapture(receiverID, propTree),
fd(-1),
closeCANReentryCount(0)

{
	// Read the configuration from the configuration file
	ReadConfigFromPropTree(propTree);
	ReadRegistersFromPropTree(propTree);
	for (int i = 0; i < numBuffers; i++) {
		buffers[i] = (uint8_t *)malloc(bufferSize);
	}
	currentBuffer = 0;
}

ReceiverPosixUDPCapture::~ReceiverPosixUDPCapture()
{
	for (int i = 0; i < numBuffers; i++) {
		if (buffers[i]) {
			free(buffers[i]);
			buffers[i] = 0;
		}
	}
	CloseDebugFile(debugFile);
	EndDistanceLog();
	Stop(); // Stop the thread
}

bool  ReceiverPosixUDPCapture::OpenCANPort()
{
	int ret;
	sockaddr_in *udp_srv_addr;
	sockaddr_in *udp_cli_addr;

	udp_srv_addr = new sockaddr_in;
	udp_cli_addr = new sockaddr_in;

	if (fd < 0) 
	{
		CloseCANPort();
	}

	fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fd < 0) {
		perror("UDP socket");
		goto posixudp_exit;
	}

	ret = fcntl(fd, F_GETFL, 0);
	if (ret >= 0) {
		ret = fcntl(fd, F_SETFL, ret | O_NONBLOCK);
	}

	memset(udp_srv_addr, 0, sizeof(sockaddr_in));
	udp_srv_addr->sin_family = AF_INET;
	udp_srv_addr->sin_addr.s_addr = inet_addr(serverAddress.c_str());
	udp_srv_addr->sin_port = htons(serverUDPPort);

	memset(udp_cli_addr, 0, sizeof(sockaddr_in));
	udp_cli_addr->sin_family = AF_INET;
	udp_cli_addr->sin_addr.s_addr = htonl(INADDR_ANY);
	udp_cli_addr->sin_port = htons(0);

	printf("UDP Using interface %s\n", serverAddress.c_str());

	ret = bind(fd, (struct sockaddr*)udp_cli_addr, sizeof(sockaddr_in));
	if (ret < 0) {
		perror("UDP bind");
		close(fd);
		fd = -1;
		goto posixudp_exit;
	}

	remoteAddress = udp_srv_addr;
	localAddress = udp_cli_addr;

	return true;

posixudp_exit:
	{
		fd = -1;

		std::string sErr = " Cannot open PosixUDP for address ";
		sErr += serverAddress;
		fprintf(stderr, sErr.c_str());
		
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
		return(false);
	}


	bFrameInvalidated = false;
}

bool  ReceiverPosixUDPCapture::CloseCANPort()

{
	if (closeCANReentryCount > 0) return(false);

	closeCANReentryCount++;

		if (fd < 0) close(fd);
		fd = -1;
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
	    closeCANReentryCount--;
		return(true);
}

uint8_t *ReceiverPosixUDPCapture::GetCurrentBuffer()
{
	return buffers[currentBuffer];
}

uint8_t *ReceiverPosixUDPCapture::GetNextBuffer()
{
	currentBuffer = (currentBuffer + 1) % numBuffers;
	return buffers[currentBuffer];
}


void ReceiverPosixUDPCapture::DoOneThreadIteration()

{
	uint8_t *buffer;
	uint32_t *buf32;
	size_t size;
	int ret;

	buffer = GetNextBuffer();
	buf32 = (uint32_t*)buffer;
	size = bufferSize;

	AWLCANMessage msg;

	if (fd >= 0)
	{
		ret = recvfrom(fd, (void*)buffer, size, 0, 0, 0);

		if (ret < 0)
		{
			//perror("UDP read");
		}
		else
		{
			msg.id  = buf32[0];
			if (msg.id < 0x60) {
				msg.len = ret - sizeof(uint32_t);
				for (int i = 0; i < 8 && i < msg.len; i ++) {
					msg.data[i] = buffer[sizeof(uint32_t)+i];
				}
				ParseMessage(msg);
			} else {
				ProcessRaw(rawFromPosixUDP, buffer, ret);
				//printf("raw %02x \n", buffer[0]);
			}
		}
	}
}

bool ReceiverPosixUDPCapture::WriteMessage(const AWLCANMessage &inMsg)
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

	ret = sendto(fd, (char*)buffer, offset + inMsg.len, 0,
		(struct sockaddr *)remoteAddress, sizeof(sockaddr_in));

	return(true);
}



bool ReceiverPosixUDPCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
	ReceiverCANCapture::ReadConfigFromPropTree(propTree);

	char receiverKeyString[32];
	sprintf(receiverKeyString, "config.receivers.receiver%d", receiverID);
	std::string receiverKey = receiverKeyString;

	boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
	// Communication parameters
	serverAddress =  receiverNode.get<std::string>("posixUDPServerAddress");

	serverUDPPort =  receiverNode.get<int>("posixUDPServerPort");

	return(true);
}


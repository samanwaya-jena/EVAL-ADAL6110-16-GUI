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
const size_t header_size = 12;
const uint8_t tty_header[header_size] = {0xb0, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x80, 0xff, 0x7f};

uint8_t buffer[4096];

ReceiverPosixTTYCapture::ReceiverPosixTTYCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround, 
	                   const std::string &inTtyName, const ReceiverCANCapture::eReceiverCANRate inCANBitRate,
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, const AlgorithmSet &inParametersAlgos,
					   const AlgorithmSet &inParametersTrackers) :
ReceiverCANCapture(receiverID, inReceiverChannelQty, inReceiverColumns, inReceiverRows, inLineWrapAround, 
                   canRate1Mbps, inFrameRate, inChannelMask, inMessageMask, inRangeOffset,  inRegistersFPGA, inRegistersADC, inRegistersGPIO, 
				   inParametersAlgos, inParametersTrackers),
fd(-1), synced(false), synced_state(0), payload_size(0), payload_read(0), max_pixel(0),
closeCANReentryCount(0)
{
	// Update settings from application
	ttyName = inTtyName;
	printf("Using TTY %s\n", ttyName.c_str());
}


ReceiverPosixTTYCapture::ReceiverPosixTTYCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCANCapture(receiverID, propTree),
fd(-1), synced(false), synced_state(0), payload_size(0), payload_read(0), max_pixel(0),
closeCANReentryCount(0)

{
	// Read the configuration from the configuration file
	ReadConfigFromPropTree(propTree);
	ReadRegistersFromPropTree(propTree);
	printf("Using TTY %s\n", ttyName.c_str());
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

	if (fd >= 0) 
	{
		CloseCANPort();
	}

	fd = open(ttyName.c_str(), O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		perror("TTY open");
		goto posixtty_exit;
	}

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

		if (fd >= 0) close(fd);
		fd = -1;
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
	    closeCANReentryCount--;
		return(true);
}

void ReceiverPosixTTYCapture::DoOneThreadIteration()

{
	size_t size = sizeof(buffer);
	int ret;
	static int fid = 0;


	if (synced) {
		//printf("TTY F\n");
		ProcessFrame();
	} else {
		//printf("TTY S\n");
		Sync();
	}
}

void ReceiverPosixTTYCapture::ProcessFrame()
{
	ssize_t ret;
	static int fid = 0;
	//off_t pos;
	static int max_pixel = 0;

	AWLCANMessage msg;

	payload_read = 0;

	while (payload_read < payload_size) {

		if (fd >= 0)
		{
			//pos = lseek (fd, 0, SEEK_CUR);
			ret = read(fd, buffer + header_size + payload_read, payload_size - payload_read);
			if (ret < 0) {
				if (errno != EAGAIN && errno != EWOULDBLOCK) {
					perror("TTY read");
					CloseCANPort();
					OpenCANPort();
				}
			} else if (ret > 0) {
				payload_read += ret;

				if (payload_read == payload_size) {
					uint16_t* data16 = (uint16_t*)msg.data;

					if (pixel == max_pixel) {
						msg.id = 0x0a;
						msg.len = 8;
						data16[0] = fid;
						data16[1] = 0x200;
						ParseMessage(msg);

						msg.id = 0x0b;
						msg.len = 8;
						data16[0] = fid;
						data16[2] = 0x200;
						ParseMessage(msg);

						msg.id = 0x09;
						msg.len = 8;
						data16[0] = fid;
						ParseMessage(msg);

						fid ++;
					}

					ProcessRaw(rawFromPosixTTY, buffer, payload_size + header_size);
					synced = false;
					synced_state = 0;
					/*
					printf ("Raw @%d %d ", pos, payload_size);
					for (int i = 0; i < payload_size/2; i++) {
						printf ("%04x ", ((uint16_t*)(buffer + header_size))[i]);
					}
					pos = lseek (fd, 0, SEEK_CUR);
					printf (" @%d \n", pos);
					*/
				}
			} else {
				CloseCANPort();
				OpenCANPort();
			}
		}
	}
}

void ReceiverPosixTTYCapture::Sync()
{
	//static int last_pixel = 0;
	//int count;
	//
	//off_t pos;
	ssize_t ret;

	//pos = lseek (fd, 0, SEEK_CUR);
	//printf ("TTY Sync @%d ", pos);
	if (synced) return; 

	while (synced_state < header_size) {
		//printf("%d ", synced_state);
		if (fd >= 0)
		{
			ret = read(fd, (void *)(buffer + synced_state), 1);
			if (ret < 0) {
				if (errno != EAGAIN && errno != EWOULDBLOCK) {
					perror("TTY read");
					CloseCANPort();
					OpenCANPort();
				}
			} else if (ret > 0) {
				if (synced_state == 3) pixel = buffer[synced_state] << 8 | buffer[synced_state - 1];
				if (synced_state == 5) timestamp = buffer[synced_state] << 8 | buffer[synced_state - 1]; 
				if (synced_state == 7) payload_size = (buffer[synced_state] << 8 | buffer[synced_state - 1]) * 2;

				if (pixel > max_pixel) max_pixel = pixel;

				if (tty_header[synced_state] == 0x01) { // 0x01 stands for don't care
					synced_state ++;
				} else {
					if (buffer[synced_state] == tty_header [synced_state]) {
						synced_state ++;
					} else {
						synced_state = 0;
					}
				}
			} else {
				CloseCANPort();
				OpenCANPort();
			}
		}
	}
	//pos = lseek (fd, 0, SEEK_CUR);
	//printf (" @%d %d \n", pos, ret);
	synced = true;
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


/* ReceiverLibUSB2Capture.cpp */
/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the SensorCoreClasses library of the
** LiDAR Sensor Toolkit.
**
** $PHANTOM_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding a valid commercial license granted by Phantom Intelligence
** may use this file in  accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the terms
** contained in a written agreement between you and Phantom Intelligence.
** For licensing terms and conditions contact directly
** Phantom Intelligence using the contact informaton supplied above.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.LGPL3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License  version 3 or any later version approved by
** Phantom Intelligence. The licenses are as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $PHANTOM_END_LICENSE$
**
****************************************************************************/


#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#endif //WIN32

#include <stdio.h>
#include <string.h>

#ifndef WIN32
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#endif //WIN32

#include <string>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#endif

#include "DebugPrintf.h"
//#include "BlockingReader.h"

#include "DetectionStruct.h"
#include "ReceiverCANCapture.h"

#include "ReceiverLibUSB2Capture.h"


SENSORCORE_USE_NAMESPACE

//const int receiveTimeOutInMillisec = 500; 
const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm fds every repoenPortDelayMillisec, 


const size_t receiveBufferSize = 212;
										   // To see if the system reconnects

ReceiverLibUSB2Capture::ReceiverLibUSB2Capture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCANCapture(receiverID, propTree),
closeCANReentryCount(0),
handle(NULL),
swap_handle(NULL)
{
  reconnectTime = boost::posix_time::microsec_clock::local_time();
  m_pFile = NULL;
  // Read the configuration from the configuration file
  ReadConfigFromPropTree(propTree);
  ReadRegistersFromPropTree(propTree);
}

ReceiverLibUSB2Capture::~ReceiverLibUSB2Capture()
{
}


int ReceiverLibUSB2Capture::ReadBytes(uint8_t * pData, int num)
{
  int received;

  int ret = libusb_bulk_transfer((libusb_device_handle *)handle, usbEndPointIn, (unsigned char *)pData, num, &received, usbTimeOut);
  if (ret)
  {
	
	  return 0;
  }
  
  if (num != received)
  {
	  return (received); // Just to have some breakpoint
  }

   return received;
}

int ReceiverLibUSB2Capture::WriteBytes(uint8_t * pData, int num)
{
  int transferred;

  int ret = libusb_bulk_transfer((libusb_device_handle *)handle, usbEndPointOut, (unsigned char *)pData, num, &transferred, usbTimeOut);
  if (ret || (transferred != num))
    return -1;

  return transferred;
}

void * ReceiverLibUSB2Capture::GetHandle(void) 
{
        return handle;
}

void ReceiverLibUSB2Capture::SetHandle(void *h)
{
        swap_handle = h;
}

void  ReceiverLibUSB2Capture::Go()

{
        assert(!mThread);

		if (OpenCANPort())
		{
			WriteCurrentDateTime();
			SetMessageFilters(receiverStatus.demandedFrameRate, receiverStatus.voxelMask, receiverStatus.messageMask);
		}


        mWorkerRunning = true;
        startTime = boost::posix_time::microsec_clock::local_time();

          mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverLibUSB2Capture::DoThreadLoop, this)));

#ifdef _WINDOWS_
        // Set the priority under windows.  This is the most critical display thread 
        // for user interaction


         HANDLE th = mThread->native_handle();
         SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
         //SetThreadPriority(th, THREAD_PRIORITY_ABOVE_NORMAL);
#endif
}


void ReceiverLibUSB2Capture::Stop()
{
  ReceiverCapture::Stop();
}

bool ReceiverLibUSB2Capture::SendSoftwareReset()
{
  if (!handle)
    return false;

  ReceiverCANMessage msg;
  msg.id = RECEIVERCANMSG_ID_COMMANDMESSAGE;
  msg.len = RECEIVERCANMSG_LEN;
  msg.data[0] = RECEIVERCANMSG_ID_CMD_SET_PARAMETER;
  msg.data[1] = RECEIVERCANMSG_ID_CMD_PARAM_ADC_REGISTER;
  msg.data[2] = 0x00;
  msg.data[3] = 0x00;
  msg.data[4] = 0x00;
  msg.data[5] = 0x00;
  msg.data[6] = 0x00;
  msg.data[7] = 0x00;

  return WriteMessage(msg);
}


void ReceiverLibUSB2Capture::DoOneThreadIteration()

{
	uint8_t buffer[receiveBufferSize];
	int bytesRead;
	bool sent;

	ReceiverCANMessage msg, poll;
	poll.id = RECEIVERCANMSG_ID_POLL;
	if (swap_handle) {
		handle = swap_handle;
		swap_handle = NULL;
	}
	if (handle)
	{   
		
		//ret = recvfrom(fd, (char*)buffer, size, 0, 0, 0);
		
		//DebugFilePrintf("Test");
		sent = WriteMessage(poll);
		bytesRead = ReadBytes((uint8_t*)buffer, sizeof(buffer));

		if (bytesRead <= 0)
		{
			//perror("UDP read");
			CloseCANPort();
		}
		else
		{
			msg.id = buffer[0];
			msg.len = buffer[9];
			//printf ("UDP %08x\n", msg.id);
			if (msg.id < 0x60) {
				msg.len = ((uint8_t)bytesRead) - (int) sizeof(uint32_t);
				for (int i = 0; i < 8 && i < msg.len; i++) {
					msg.data[i] = buffer[10 + i];
				}
				ParseMessage(msg);
			}
			else {
				ProcessRaw(buffer);
			}
		}
			}
	else {
		if (boost::posix_time::microsec_clock::local_time() > reconnectTime) {
			if (OpenCANPort()) {
				WriteCurrentDateTime();
				SetMessageFilters(receiverStatus.demandedFrameRate, receiverStatus.voxelMask, receiverStatus.messageMask);	
			}
		}
	}
}


bool  ReceiverLibUSB2Capture::OpenCANPort()
{
  int ret = 0;

  reconnectTime = boost::posix_time::microsec_clock::local_time() + boost::posix_time::milliseconds(reopenPortDelaylMillisec);

  ret = libusb_init(&context);

  if (ret)
    return false;

	// discover devices
	libusb_device **list;
	libusb_device_descriptor descriptor;
	ssize_t cnt = libusb_get_device_list(NULL, &list);
	ssize_t i = 0;
	int err = 0;
	if (cnt < 0)
    		perror("libusb_get_device_list");

	int matches = 0;

	for (i = 0; i < cnt; i++) {
    	libusb_device *device = list[i];
    	err = libusb_get_device_descriptor(device, &descriptor);
		if (!err && descriptor.idVendor == usbVendorId && descriptor.idProduct == usbProductId) {
			matches ++;
		}
	}

	//if (matches != (receiverID + 1)) return false;
	if (matches < (receiverID + 1)) return false;

	for (i = 0; i < cnt; i++) {
    	libusb_device *device = list[i];
    	err = libusb_get_device_descriptor(device, &descriptor);
		if (!err && descriptor.idVendor == usbVendorId && descriptor.idProduct == usbProductId) {
			libusb_device_handle* h;
			err = libusb_open(device, &h);
			if (err) {
				perror("libusb_open_no_access");
			}
			else {
				handle = h;
				//Claim Interface 0 from the device
				ret = libusb_claim_interface((libusb_device_handle*)handle, 0);

				if (!ret) {
					fprintf(stderr, "%ld %ld usb_claim_interface %ld %p succeeded\n", receiverID, matches, i, handle);
				}
				else {
					fprintf(stderr, "%ld %ld usb_claim_interface %ld %p error %d\n", receiverID, matches, i, handle, ret);
					libusb_close((libusb_device_handle*)handle);
					handle = NULL;
				}
			}
		}
		if (handle) break;
	}

//printf("out %p\n", handle);
	if (handle)
	{
		int received = 0;

		boost::mutex::scoped_lock rawLock(m_Mutex);

		// DGG: Clear any outstanding data left in the USB buffers
		do
		{
			char tmp[256];
			ret = libusb_bulk_transfer((libusb_device_handle*)handle, usbEndPointIn, (unsigned char*)tmp, sizeof(tmp), &received, 100);
		} while (ret == 0 && received > 0);

		rawLock.unlock();

		return true;
	}
	else {  // if (!handle)
		return false;
	}
}


bool ReceiverLibUSB2Capture::SetMessageFilters(ReceiverFrameRate frameRate, VoxelMask voxelMask, MessageMask messageMask)

{
	ReceiverCANMessage message;
	message.id = RECEIVERCANMSG_ID_COMMANDMESSAGE;       // Message id: RECEIVERCANMSG_ID_COMMANDMESSAGE- Command message
	message.len = RECEIVERCANMSG_LEN;       // Frame size (0.8)

	bool bMessageOk = true;

	if (bMessageOk)
	{
		// Set Frame Rate
		bMessageOk = SetFPGARegister(0x13, frameRate);
	}

	if (bMessageOk)
	{
		message.data[0] = RECEIVERCANMSG_ID_CMD_TRANSMIT_COOKED;   // Transmit_cooked enable flags

		*(int16_t*)&message.data[1] = voxelMask.wordData;
		message.data[3] = 1;  // Rate Decimation.
		message.data[4] = 0;  // Reserved
		message.data[5] = 0;  // Reserved
		message.data[6] = 0;  // Reserved
		message.data[7] = 0;  // Reserved

		bMessageOk = WriteMessage(message);
	}

	if (bMessageOk)
	{
		message.data[0] = RECEIVERCANMSG_ID_CMD_TRANSMIT_RAW;   // Transmit_raw enable flags

		if (messageMask.bitFieldData.raw)
		{
			*(int16_t*)&message.data[1] = voxelMask.wordData;
		}
		else 
		{
			*(int16_t*)&message.data[1] = 0;
		}
		message.data[3] = 1;  // Rate Decimation.
		message.data[4] = 0;  // Reserved
		message.data[5] = 0;  // Reserved
		message.data[6] = 0;  // Reserved
		message.data[7] = 0;  // Reserved
		bMessageOk = WriteMessage(message);
	}

	if (bMessageOk)
	{
		// Acquisition enable
		bMessageOk = SetFPGARegister(0x10, 1);
	}

	// The message has no confirmation built in
	return(bMessageOk);
}
bool  ReceiverLibUSB2Capture::CloseCANPort()
{
  boost::mutex::scoped_lock rawLock(m_Mutex);

  if (handle)
  {
    libusb_release_interface((libusb_device_handle *)handle, 0);

    libusb_close((libusb_device_handle *)handle);

    handle = NULL;
  }

	//libusb_exit(NULL);

	reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);

	return(true);
}

bool ReceiverLibUSB2Capture::WriteMessage(const ReceiverCANMessage &inMsg)
{
	uint8_t buffer[256];
	uint32_t* buf32;
	size_t offset = 0;
	int ret;
	static int timestamp = 0;

	buf32 = (uint32_t*)buffer;

	if (!handle) return(false);

	buf32[0] = inMsg.id;
	offset += sizeof(uint32_t);

	buf32[1] = timestamp++;
	offset += sizeof(uint32_t);

	buffer[offset] = 0; //flags
	offset++;

	buffer[offset] = 8; //len
	offset++;

	for (int i = 0; i < 8; i++) {
		buffer[offset + i] = inMsg.data[i];
	}
	offset = offset + 8;
	buffer[offset] = 0; //pad1
	offset++;
	buffer[offset] = 0; //pad2
	offset++;
	//printf("len :  %d\n", offset);

  	ret = WriteBytes((uint8_t*)buffer, offset);
//	ret = sendto(fd, (char*)buffer, offset + inMsg.len, 0,
//		(struct sockaddr *)remoteAddress, sizeof(sockaddr_in));

	return(true);
}

static int channelToColumnArray[] = {
  14,
  12,
  10,
  8,
  6,
  4,
  2,
  0,
  1,
  3,
  5,
  7,
  9,
  11,
  13,
  15
};

static int columnToChannelArray[] = {
7,
8,
6,
9,
5,
10,
4,
11,
3,
12,
2,
13,
1,
14,
0
};

CellID ReceiverLibUSB2Capture::GetCellIDFromChannel(int inChannelID)
{
	int row = 0;
	int column = channelToColumnArray[inChannelID];
	return (CellID(column, row));
}

int ReceiverLibUSB2Capture::GetChannelIDFromCell(CellID inCellID)
{
	for (int i = 0; i < receiverVoxelQty; i++)
	{
		if (channelToColumnArray[i] == inCellID.column) return (i);
	}
	return (0); 
}

void ReceiverLibUSB2Capture::ProcessRaw(uint8_t* rawData)
{
	uint16_t* rawData16 = (uint16_t*)rawData;
	int voxelIndex = rawData16[1] & 0xFF;
	CellID cellID(voxelIndex, 0);
	int msg_id = rawData[0];
	size_t sampleOffset = 12;
	size_t sampleDrop = 0;
	size_t sampleSize = 2;
	bool sampleSigned = true;
	bool transmit = false;

	if (msg_id != 0xb0) return;
	if (voxelIndex >= maxRawBufferCount) return;

	if (!rawBuffers[voxelIndex])
	{
		rawBuffers[voxelIndex] = new uint8_t[maxRawBufferSize];
	}
	rawBufferCount++;

	memcpy(rawBuffers[voxelIndex], rawData, receiveBufferSize);
	sampleCount = (receiveBufferSize / 2) - sampleOffset;
	transmit = true;
	
	if (voxelIndex > max_voxel) max_voxel = voxelIndex;
	if (voxelIndex == max_voxel) transmit = true;


	//printf("ascan %02x %02x %d %d %d\n", msg_id, max_msg_id, channel, size, sampleCount);

	if (transmit)
	{

		boost::mutex::scoped_lock rawLock(GetMutex());

		AScan::Ptr aScan = currentFrame->MakeUniqueAScan(currentFrame->aScans, receiverID, cellID, GetChannelIDFromCell(cellID));
		aScan->samples = rawBuffers[voxelIndex];
		aScan->sampleSize = sampleSize;
		aScan->sampleOffset = sampleOffset;
		aScan->sampleCount = sampleCount - sampleDrop;
		aScan->sampleSigned = sampleSigned;
		//printf("transmit ascan %d %d\n", aScan->channelID, aScan->sampleCount);

		rawLock.unlock();
	}
}

bool ReceiverLibUSB2Capture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
	ReceiverCANCapture::ReadConfigFromPropTree(propTree);

	std::string receiverKey = std::string("config.receivers.receiver") + std::to_string(receiverID);;

	boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
	// Communication parameters

	usbVendorId =  receiverNode.get<int>("libUsbVendorId", 1419);
	usbProductId =  receiverNode.get<int>("libUsbProductId",80);
	usbEndPointIn = (unsigned char)receiverNode.get<int>("libUsbEndPointIn", 129);
	usbEndPointOut = (unsigned char)receiverNode.get<int>("libUsbEndPointOut", 2);
	usbTimeOut =  receiverNode.get<int>("libUsbTimeOut", 1000);

	return(true);
}

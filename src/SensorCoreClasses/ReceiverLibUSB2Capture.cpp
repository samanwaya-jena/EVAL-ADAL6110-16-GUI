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
#include "SensorSettings.h"


SENSORCORE_USE_NAMESPACE

//const int receiveTimeOutInMillisec = 500; 
const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm fds every repoenPortDelayMillisec, 


const size_t receiveBufferSize = 212;

// Data charactereistics of raw buffer
const size_t sampleQty = 100;
const size_t sampleSize = 2;
const size_t sampleOffset = 12;
const size_t sampleDrop = 0;
const bool sampleSigned = true;

const uint16_t productIDRegister = 0x00;
const uint16_t manufactureDateRegister = 0x01;
const uint16_t uniqueIDRegister = 0x02;
const uint16_t acquisitionRegister = 0x10;
const uint16_t frameRateRegister = 0x13;


// To see if the system reconnects

ReceiverLibUSB2Capture::ReceiverLibUSB2Capture(int receiverID, boost::property_tree::ptree& propTree) :
	ReceiverCANCapture(receiverID, propTree),
	closeCANReentryCount(0),
	handle(NULL),
	swap_handle(NULL)
{

	// Prepare the static pollMessage;
	pollMsg.id = RECEIVERCANMSG_ID_POLL;
	pollMsg.len = RECEIVERCANMSG_LEN;

	reconnectTime = boost::posix_time::microsec_clock::local_time();
	m_pFile = NULL;
	// Read the configuration from the configuration file
	ReadConfigFromPropTree(propTree);
	ReadRegistersFromPropTree(propTree);
}

ReceiverLibUSB2Capture::~ReceiverLibUSB2Capture()
{
}


int ReceiverLibUSB2Capture::ReadBytes(uint8_t* pData, int num)
{
	return ReadBytes(pData, num, usbTimeOut);
}

int ReceiverLibUSB2Capture::ReadBytes(uint8_t* pData, int num, int timeOut)
{
	int received;

	int ret = libusb_bulk_transfer((libusb_device_handle*)handle, usbEndPointIn, (unsigned char*)pData, num, &received, timeOut);
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

int ReceiverLibUSB2Capture::WriteBytes(uint8_t* pData, int num)
{
	int transferred;

	int ret = libusb_bulk_transfer((libusb_device_handle*)handle, usbEndPointOut, (unsigned char*)pData, num, &transferred, usbTimeOut);
	if (ret || (transferred != num))
		return -1;

	return transferred;
}

void* ReceiverLibUSB2Capture::GetHandle(void)
{
	return handle;
}

void ReceiverLibUSB2Capture::SetHandle(void* h)
{
	swap_handle = h;
}

void  ReceiverLibUSB2Capture::Go()

{
	assert(!mThread);

	mWorkerRunning = true;
	startTime = boost::posix_time::microsec_clock::local_time();

	// Reconnect time is now.  This will force reconnect on first iteration of DoThreadLoop()
	reconnectTime = boost::posix_time::microsec_clock::local_time();

	mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverLibUSB2Capture::DoThreadLoop, this)));

#ifdef _WINDOWS_
	// Set the threadpriority under windows.  This is the most critical display thread 
	// for user interaction

	// We have found that setting to THREAD_PRIORITY_TIME_CRITICAL is slighly less performant.  May reduce performance of LiBUSB.
	HANDLE th = mThread->native_handle();
	SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
#endif
}


void ReceiverLibUSB2Capture::Stop()
{
	ReceiverCapture::Stop();
}

bool ReceiverLibUSB2Capture::IsConnected() 
{ 
	bool hasHandle = handle != NULL;
	return(hasHandle);
}


void ReceiverLibUSB2Capture::DoOneThreadIteration()

{
	uint8_t buffer[receiveBufferSize];
	int bytesRead;
	bool sent;

	if (swap_handle) {
		handle = swap_handle;
		swap_handle = NULL;
	}
	if (handle)
	{
		sent = WriteMessage(pollMsg);

		ReceiverCANMessage* msgPtr = (ReceiverCANMessage*)buffer;

#ifdef _WINDOWS_
		// Under Windows, do not wait for other input.
		bytesRead = ReadBytes((uint8_t*)buffer, sizeof(buffer),0);
#else
		// Use time-out defined in XML file
		bytesRead = ReadBytes((uint8_t*)buffer, sizeof(buffer));
#endif


		if (bytesRead < sizeof(ReceiverCANMessage))
		{
			CloseCANPort();
		}
		else
		{
			if (msgPtr->id < 0xB0)
			{
				ParseMessage(*msgPtr);
			}
			else
			{
				ProcessRaw(buffer);
			}
		}
	}

	else { // Handle = NULL
		if (boost::posix_time::microsec_clock::local_time() > reconnectTime) {
			if (OpenCANPort()) {
				QueryProductID();
				QueryUniqueID();
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
	libusb_device** list;
	libusb_device_descriptor descriptor;
	ssize_t cnt = libusb_get_device_list(NULL, &list);
	ssize_t i = 0;
	int err = 0;
	if (cnt < 0)
		perror("libusb_get_device_list");

	int matches = 0;

	for (i = 0; i < cnt; i++) {
		libusb_device* device = list[i];
		err = libusb_get_device_descriptor(device, &descriptor);
		if (!err && descriptor.idVendor == usbVendorId && descriptor.idProduct == usbProductId) {
			matches++;
		}
	}

	//if (matches != (receiverID + 1)) return false;
	if (matches < (receiverID + 1)) return false;

	for (i = 0; i < cnt; i++) {
		libusb_device* device = list[i];
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
		// force stop on acquisition. Unit may have been running for a while and 
		// has data in queue we may need to get rid of. 
		SetFPGARegister(acquisitionRegister, 0);
		// Wait for the acquisition to stop
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));

		FlushMessages();

		// Restart the acquisition
		SetFPGARegister(acquisitionRegister, 1);
		return(true);
	}
	else {  // if (!handle)
		ClearAllRegisters();
		return false;
	}
}

bool  ReceiverLibUSB2Capture::CloseCANPort()
{
	boost::mutex::scoped_lock rawLock(m_Mutex);

	if (handle)
	{
		libusb_release_interface((libusb_device_handle*)handle, 0);

		libusb_close((libusb_device_handle*)handle);

		handle = NULL;
		ClearAllRegisters();
	}

	//libusb_exit(NULL);

	reconnectTime = boost::posix_time::microsec_clock::local_time() + boost::posix_time::milliseconds(reopenPortDelaylMillisec);

	return(true);
}

bool ReceiverLibUSB2Capture::FlushMessages()
{
	int bytesRead;

	uint8_t buffer[receiveBufferSize];

	if (!handle) return(false);

	//  Clear any outstanding data left in the USB buffers
	do
	{
		bytesRead = ReadBytes((uint8_t*)buffer, sizeof(buffer));
	} while (bytesRead > 0);

	// Wait for a complete reset of the unit, in case this is where we start the unit
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	// Clear messages outstanding in the sensor's internal queues, but do not process them.
	// End only when there is no more datam, or we have hit a QUEU_EMPTY message.
	ReceiverCANMessage inMsg;
	bool bEmpty = false;

	do
	{
		WriteMessage(pollMsg);
		bytesRead = ReadBytes((uint8_t*)buffer, sizeof(buffer));
		if (bytesRead <= sizeof(ReceiverCANMessage)) bEmpty = true;
		else {
			inMsg.id = buffer[0];
			inMsg.len = buffer[9];
			if (inMsg.id == RECEIVERCANMSG_ID_COMMANDMESSAGE && buffer[10] == RECEIVERCANMSG_ID_CMD_RESPONSE_QUEUE_EMPTY)
			{
				bEmpty = true;
			}
		}

	} while (!bEmpty);

	// Messages have been flushed. Reset the sensor status for comm errors.
	receiverStatus.receiverError.byteData = 0;

	return true;

}

bool ReceiverLibUSB2Capture::SetMessageFilters(ReceiverFrameRate frameRate, VoxelMask voxelMask, MessageMask messageMask)

{
	ReceiverCANMessage message;
	message.id = RECEIVERCANMSG_ID_COMMANDMESSAGE;       // Message id: RECEIVERCANMSG_ID_COMMANDMESSAGE- Command message
	message.len = RECEIVERCANMSG_LEN;       // Frame size (0.8)

	bool bMessageOk = true;

	// Stop the acquisition while we set the operating parameters
	bMessageOk = SetFPGARegister(acquisitionRegister, 0);
	// Wait for the acquisition to stop
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	if (bMessageOk)
	{
		// Set Frame Rate
		bMessageOk = SetFPGARegister(frameRateRegister, frameRate);
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


	// Restart the acquisition
	if (bMessageOk)
	{
		bMessageOk = SetFPGARegister(acquisitionRegister, 1);
	}


	// The message has no confirmation built in
	return(bMessageOk);
}



bool ReceiverLibUSB2Capture::WriteMessage(const ReceiverCANMessage& inMsg)
{
#if 1
	WriteBytes((uint8_t *)&inMsg, sizeof(ReceiverCANMessage));
#else
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
#endif
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

	// Log the data, if logging is enabled.
	if (SensorSettings::GetGlobalSettings()->bWriteLogFile)
	{
		boost::mutex::scoped_lock rawLock(m_Mutex);

		if (SensorSettings::GetGlobalSettings()->bWriteLogFile) // Check again for file handle now that we have the mutex
		{
				LogWaveform(rawData);
		}
		rawLock.unlock();
	}
}

bool ReceiverLibUSB2Capture::LogWaveform(uint8_t* rawData)
{
	uint16_t* rawData16 = (uint16_t*)rawData;
	int voxelIndex = rawData16[1] & 0xFF;
	CellID cellID(voxelIndex, 0);
	int msg_id = rawData[0];

	if (msg_id != 0xb0) return false;

	logFileMutex.lock();

	if (!logFilePtr)
	{
		logFileMutex.unlock();
		return false;
	}

	std::string theWaveString(", ,Wave, ,");
	theWaveString += std::to_string(receiverID);
	theWaveString += std::string(",Channel,");
	theWaveString += std::to_string(cellID.column) + "," + std::to_string(cellID.row) + ",";
	theWaveString += std::to_string(GetChannelIDFromCell(cellID)) + ", ,";

	short* pData = (short*)&rawData[sampleOffset];
	for (size_t i = 0; i < sampleQty; i++)
	{
		if (i == (sampleQty - 1))
			theWaveString += std::to_string(*pData++);
		else
			theWaveString += std::to_string(*pData++) + ",";
	}
	LogFilePrintf(*logFilePtr, theWaveString.c_str());
	
	logFileMutex.unlock();
}

bool ReceiverLibUSB2Capture::ReadConfigFromPropTree(boost::property_tree::ptree& propTree)
{
	ReceiverCANCapture::ReadConfigFromPropTree(propTree);

	std::string receiverKey = std::string("config.receivers.receiver") + std::to_string(receiverID);;

	boost::property_tree::ptree& receiverNode = propTree.get_child(receiverKey);
	// Communication parameters

	usbVendorId = receiverNode.get<int>("libUsbVendorId", 1419);
	usbProductId = receiverNode.get<int>("libUsbProductId", 80);
	usbEndPointIn = (unsigned char)receiverNode.get<int>("libUsbEndPointIn", 129);
	usbEndPointOut = (unsigned char)receiverNode.get<int>("libUsbEndPointOut", 2);
	usbTimeOut = receiverNode.get<int>("libUsbTimeOut", 1000);

	return(true);
}


bool ReceiverLibUSB2Capture::QueryUniqueID()
{
	// Basic LibUSB device does not have UniqueID
	return(QueryFPGARegister(uniqueIDRegister));
}

bool ReceiverLibUSB2Capture::QueryProductID()
{
	// Basic LiBUsb device does not have ProductID emmbeded in firmware 
	return(QueryFPGARegister(productIDRegister));
}

uint32_t ReceiverLibUSB2Capture::GetUniqueID()
{
	if (registersFPGA[uniqueIDRegister].pendingUpdates == updateStatusPendingUpdate)
	{
		// If the contents of the register is not refreshed yet (a query is under way), return (undefined);
		return(0);
	}

	// A value of zero indicates that serial No is not available
	return(registersFPGA[uniqueIDRegister].value);
}

uint32_t ReceiverLibUSB2Capture::GetProductID()
{
	if (registersFPGA[productIDRegister].pendingUpdates == updateStatusPendingUpdate)
	{
		// If the contents of the register is not refreshed yet (a query is under way), return (undefined);
		return(0);
	}

	// A value of zero indicates that serial No is not available
	return(registersFPGA[productIDRegister].value);
}

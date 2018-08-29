/* ReceiverLibUSBCapture.cpp */
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
//#include <sys/ioctl.h>
//#include <unistd.h>
//#include <fcntl.h>

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
#include "ReceiverLibUSBCapture.h"


using namespace std;
using namespace awl;

const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm fds every repoenPortDelayMillisec, 
										   // To see if the system reconnects

#define MAX_POLL_CAN_MESSAGES 32


ReceiverLibUSBCapture::ReceiverLibUSBCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround, 
	                   const ReceiverCANCapture::eReceiverCANRate inCANBitRate,
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, const AlgorithmSet &inParametersAlgos,
					   const AlgorithmSet &inParametersTrackers) :
ReceiverCANCapture(receiverID, inReceiverChannelQty, inReceiverColumns, inReceiverRows, inLineWrapAround, 
                   canRate1Mbps, inFrameRate, inChannelMask, inMessageMask, inRangeOffset,  inRegistersFPGA, inRegistersADC, inRegistersGPIO, 
				   inParametersAlgos, inParametersTrackers),
handle(NULL)
{
  reconnectTime = boost::posix_time::microsec_clock::local_time();
}


ReceiverLibUSBCapture::ReceiverLibUSBCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCANCapture(receiverID, propTree),
handle(NULL)
{
  reconnectTime = boost::posix_time::microsec_clock::local_time();

  // Read the configuration from the configuration file
	ReadConfigFromPropTree(propTree);
	ReadRegistersFromPropTree(propTree);
}

ReceiverLibUSBCapture::~ReceiverLibUSBCapture()
{
	CloseDebugFile(debugFile);
	EndDistanceLog();
	Stop(); // Stop the thread
}


void ReceiverLibUSBCapture::Go()
{
  assert(!mThread);

  mWorkerRunning = true;

  startTime = boost::posix_time::microsec_clock::local_time();

  mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverLibUSBCapture::DoThreadLoop, this)));

#ifdef _WINDOWS_
  // Set the priority under windows.  This is the most critical display thread 
  // for user interaction
  HANDLE th = mThread->native_handle();
  SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
  //   SetThreadPriority(th, THREAD_PRIORITY_ABOVE_NORMAL);
#endif
}

void ReceiverLibUSBCapture::Stop()
{
  ReceiverCapture::Stop();
}

bool  ReceiverLibUSBCapture::OpenCANPort()
{
  int ret = 0;

  reconnectTime = boost::posix_time::microsec_clock::local_time() + boost::posix_time::milliseconds(reopenPortDelaylMillisec);

  ret = libusb_init(&context);

  if (ret)
    return false;

  //libusb_set_debug(context, 3);

    //Open Device with VendorID and ProductID
	handle = libusb_open_device_with_vid_pid(context, usbVendorId, usbProductId);
	if (!handle) {
		perror("device not found");
		return false;
	}

	//Claim Interface 0 from the device
  ret = libusb_claim_interface(handle, 0);

	if (ret) {
		fprintf(stderr, "usb_claim_interface error %d\n", ret);
		return false;
	}

  if (handle)
  {
    int transferred = 0;
    int received = 0;

    boost::mutex::scoped_lock rawLock(mMutexUSB);

    // DGG: Clear any outstanding data left in the USB buffers
    do 
    {
      char tmp[256];
      ret = libusb_bulk_transfer(handle, usbEndPointIn, (unsigned char *)tmp, sizeof(tmp), &received, 100);
    }
    while (ret == 0 && received > 0);

    AWLCANMessage msg;
    msg.id = AWLCANMSG_ID_LIDARQUERY;

    AWLCANMessage resp;

    ret = libusb_bulk_transfer(handle, usbEndPointOut, (unsigned char *)&msg, sizeof(msg), &transferred, usbTimeOut);
    if (ret || (transferred != sizeof(AWLCANMessage)))
      return false;

    ret = libusb_bulk_transfer(handle, usbEndPointIn, (unsigned char *)&resp, sizeof(resp), &received, usbTimeOut);
    if (ret || (received != sizeof(AWLCANMessage)))
      return false;

    return true;
  }

  return false;
}

bool  ReceiverLibUSBCapture::CloseCANPort()
{
  boost::mutex::scoped_lock rawLock(mMutexUSB);

  if (handle)
  {
    int ret = libusb_release_interface(handle, 0);

    libusb_close(handle);
    handle = NULL;
  }

	libusb_exit(NULL);

	reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);

	return(true);
}

bool ReceiverLibUSBCapture::LidarQuery(DWORD * pdwCount, DWORD * pdwReadPending)
{
  int ret = 0;
  int transferred = 0;
  int received = 0;

  boost::mutex::scoped_lock rawLock(mMutexUSB);

  if (!handle)
    return false;

  AWLCANMessage msg;
  msg.id = AWLCANMSG_ID_LIDARQUERY;

  AWLCANMessage resp;

  ret = libusb_bulk_transfer(handle, usbEndPointOut, (unsigned char *)&msg, sizeof(msg), &transferred, usbTimeOut);
  if (ret || (transferred != sizeof(AWLCANMessage)))
    return false;

  ret = libusb_bulk_transfer(handle, usbEndPointIn, (unsigned char *)&resp, sizeof(resp), &received, usbTimeOut);
  if (ret || (received != sizeof(AWLCANMessage)))
    return false;

  *pdwCount = *((uint32_t*)&resp.data[0]);
  *pdwReadPending = *((uint32_t*)&resp.data[4]);

  return true;
}


bool ReceiverLibUSBCapture::ReadDataFromUSB(char * ptr, int uiCount, DWORD dwCount)
{
  int transferred = 0;
  int received = 0;
  int ret = 0;

  boost::mutex::scoped_lock rawLock(mMutexUSB);

  if (!handle)
    return false;

  AWLCANMessage msg;
  msg.id = AWLCANMSG_ID_GETDATA;
  msg.data[0] = (unsigned char) dwCount;

  ret = libusb_bulk_transfer(handle, usbEndPointOut, (unsigned char *)&msg, sizeof(msg), &transferred, usbTimeOut);
  if (ret || (transferred != sizeof(AWLCANMessage)))
    return false;

  ret = libusb_bulk_transfer(handle, usbEndPointIn, (unsigned char *)ptr, uiCount, &received, usbTimeOut);
  if (ret || (received != uiCount))
    return false;

  return true;
}

#include "algo.h"

typedef struct
{
  short AcqFifo[GUARDIAN_NUM_CHANNEL * GUARDIAN_SAMPLING_LENGTH];
} tDataFifo;

tDataFifo dataFifo[8];

bool ReceiverLibUSBCapture::DoOneLoop()
{
  DWORD dwCount = 0;
  DWORD dwReadPending = 0;

  bool bRet = LidarQuery(&dwCount, &dwReadPending);

  if (!bRet)
    return false;

  if (dwCount)
  {
    bRet = ReadDataFromUSB((char*)dataFifo, dwCount * sizeof(tDataFifo), dwCount);

    ProcessRaw(rawFromLibUSB, (uint8_t*)dataFifo->AcqFifo, GUARDIAN_NUM_CHANNEL * GUARDIAN_SAMPLING_LENGTH * sizeof(short));
  }

  if (dwReadPending)
  {
    bRet = PollMessages(dwReadPending);
  }

  if (!dwCount && !dwReadPending)
  {
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }

  if (!bRet)
    return false;

  return bRet;
}

void ReceiverLibUSBCapture::DoOneThreadIteration()
{
  if (handle)
  {
    bool bRet = DoOneLoop();

    if (!bRet)
    {
      CloseCANPort();
    }
  }
  else
  {
    if (boost::posix_time::microsec_clock::local_time() > reconnectTime)
    {
      if (OpenCANPort())
      {
        WriteCurrentDateTime();
        SetMessageFilters(receiverStatus.frameRate, receiverStatus.channelMask, receiverStatus.messageMask);
        // Update all the info (eventually) from the status of the machine
        QueryAlgorithm();
        QueryTracker();
      }
    }
  }
}

void ReceiverLibUSBCapture::DoThreadLoop()
{
  while (!WasStopped())
  {
    DoOneThreadIteration();
  } // while (!WasStoppped)
}

bool ReceiverLibUSBCapture::PollMessages(DWORD dwNumMsg)
{
  AWLCANMessage canReq;
  AWLCANMessage canResp[MAX_POLL_CAN_MESSAGES];
  int transferred = 0;
  int received = 0;
  int ret;

  boost::mutex::scoped_lock rawLock(mMutexUSB);

  if (!handle)
    return false;

  if (dwNumMsg > MAX_POLL_CAN_MESSAGES)
    dwNumMsg = MAX_POLL_CAN_MESSAGES;

  AWLCANMessage msg;
  msg.id = AWLCANMSG_ID_POLLMESSAGES;
  msg.data[0] = (unsigned char) dwNumMsg;

  ret = libusb_bulk_transfer(handle, usbEndPointOut, (unsigned char *)&msg, sizeof(AWLCANMessage), &transferred, usbTimeOut);
  if (ret || (transferred != sizeof(AWLCANMessage)))
    return false;

  ret = libusb_bulk_transfer(handle, usbEndPointIn, (unsigned char *)&canResp, dwNumMsg * sizeof(AWLCANMessage), &received, usbTimeOut);
  if (ret || (received != dwNumMsg * sizeof(AWLCANMessage)))
    return false;

  for (DWORD i=0; i<dwNumMsg; i++)
  {
    if (canResp[i].id)
      ParseMessage(canResp[i]);
  }

  return true;
}

bool ReceiverLibUSBCapture::WriteMessage(const AWLCANMessage &inMsg)
{
  AWLCANMessage canResp;
  int transferred = 0;
  int received = 0;
  int ret;

  LARGE_INTEGER t1, t2, t3;
  double elapsedTime1, elapsedTime2, elapsedTime3;

  if (!handle)
    return false;

  boost::mutex::scoped_lock rawLock(mMutexUSB);

  ret = libusb_bulk_transfer(handle, usbEndPointOut, (unsigned char *)&inMsg, sizeof(AWLCANMessage), &transferred, usbTimeOut);
  if (ret || (transferred != sizeof(AWLCANMessage)))
    return false;

  ret = libusb_bulk_transfer(handle, usbEndPointIn, (unsigned char *)&canResp, sizeof(AWLCANMessage), &received, usbTimeOut);
  if (ret || (received != sizeof(AWLCANMessage)))
    return false;

  if (canResp.id)
    ParseMessage(canResp);

	return false;
}

bool ReceiverLibUSBCapture::SendSoftwareReset()
{
  if (!handle)
    return false;

  AWLCANMessage msg;
  msg.id = AWLCANMSG_ID_COMMANDMESSAGE;
  msg.len = AWLCANMSG_LEN;
  msg.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
  msg.data[1] = AWLCANMSG_ID_CMD_PARAM_AWL_REGISTER;
  msg.data[2] = (uint8_t) (997 >> 0);
  msg.data[3] = (uint8_t) (997 >> 8);
  msg.data[4] = 0x00;
  msg.data[5] = 0x00;
  msg.data[6] = 0x00;
  msg.data[7] = 0x00;

  return WriteMessage(msg);
}

bool ReceiverLibUSBCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
	ReceiverCANCapture::ReadConfigFromPropTree(propTree);

	char receiverKeyString[32];
	sprintf(receiverKeyString, "config.receivers.receiver%d", receiverID);
	std::string receiverKey = receiverKeyString;

	boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
	// Communication parameters

	usbVendorId =  receiverNode.get<int>("libUsbVendorId");
	usbProductId =  receiverNode.get<int>("libUsbProductId");
	usbEndPointIn =  receiverNode.get<int>("libUsbEndPointIn");
	usbEndPointOut =  receiverNode.get<int>("libUsbEndPointOut");
	usbTimeOut =  receiverNode.get<int>("libUsbTimeOut");

	return(true);
}

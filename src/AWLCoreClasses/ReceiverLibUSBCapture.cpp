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

const int receiveTimeOutInMillisec = 500;  // Default is 1000. As AWL refresh rate is 100Hz, this should not exceed 10ms
const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm fds every repoenPortDelayMillisec, 
										   // To see if the system reconnects




ReceiverLibUSBCapture::ReceiverLibUSBCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround, 
	                   const ReceiverCANCapture::eReceiverCANRate inCANBitRate,
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, const AlgorithmSet &inParametersAlgos,
					   const AlgorithmSet &inParametersTrackers) :
ReceiverCANCapture(receiverID, inReceiverChannelQty, inReceiverColumns, inReceiverRows, inLineWrapAround, 
                   canRate1Mbps, inFrameRate, inChannelMask, inMessageMask, inRangeOffset,  inRegistersFPGA, inRegistersADC, inRegistersGPIO, 
				   inParametersAlgos, inParametersTrackers),
closeCANReentryCount(0),
handle(NULL)
{

	/*
	#define USB_VENDOR_ID	    0x0483
	#define USB_PRODUCT_ID	    0xFFFF 
	#define USB_ENDPOINT_IN	    (LIBUSB_ENDPOINT_IN  | 1) 
	#define USB_ENDPOINT_OUT	(LIBUSB_ENDPOINT_OUT | 2) 
	#define USB_TIMEOUT	3000 
	*/
}


ReceiverLibUSBCapture::ReceiverLibUSBCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCANCapture(receiverID, propTree),
closeCANReentryCount(0),
handle(NULL)
{
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

#ifdef _WIN32
LARGE_INTEGER frequency;
#endif

bool  ReceiverLibUSBCapture::OpenCANPort()
{
  int ret = 0;

#ifdef _WIN32
  QueryPerformanceFrequency(&frequency);
#endif

	ret = libusb_init(&context);

  libusb_set_debug(context, 3);

    //Open Device with VendorID and ProductID
	handle = libusb_open_device_with_vid_pid(context, usbVendorId, usbProductId);
	if (!handle) {
		perror("device not found");
		return 1;
	}

	//Claim Interface 0 from the device
  ret = libusb_claim_interface(handle, 0);

	if (ret < 0) {
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
      ret = libusb_bulk_transfer(handle, usbEndPointIn, (unsigned char *)tmp, sizeof(tmp), &received, 1);

      //if (received)
      //{
      //  char str[256];
      //  sprintf(str, "Init %d --> %d (%d) \n", ret, tmp[0], received);
      //  OutputDebugStringA(str);
      //}
    }
    while (ret == 0 && received > 0);

    AWLCANMessage msg;
    msg.id = AWLCANMSG_ID_LIDARQUERY;

    AWLCANMessage resp;

    ret = libusb_bulk_transfer(handle, usbEndPointOut, (unsigned char *)&msg, sizeof(msg), &transferred, 0);

    ret = libusb_bulk_transfer(handle, usbEndPointIn, (unsigned char *)&resp, sizeof(resp), &received, 0);

    if (transferred != sizeof(msg))
      transferred = 0;
    if (received != sizeof(resp))
      received = 0;

    ret = 0;
  }

  return (ret == 0);
}

bool  ReceiverLibUSBCapture::CloseCANPort()
{
	if (closeCANReentryCount > 0) return(false);

  boost::mutex::scoped_lock rawLock(mMutexUSB);

	closeCANReentryCount++;

	libusb_close(handle);
	libusb_exit(NULL);

	handle = NULL;

	reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
	    closeCANReentryCount--;
	return(true);
}

int ReceiverLibUSBCapture::LidarQuery(DWORD * pdwCount, DWORD * pdwReadPending)
{
  int ret = 0;
  int transferred = 0;
  int received = 0;

  boost::mutex::scoped_lock rawLock(mMutexUSB);

  if (!handle) return 1;

  AWLCANMessage msg;
  msg.id = AWLCANMSG_ID_LIDARQUERY;

  AWLCANMessage resp;

  ret = libusb_bulk_transfer(handle, usbEndPointOut, (unsigned char *)&msg, sizeof(msg), &transferred, 0);

  ret = libusb_bulk_transfer(handle, usbEndPointIn, (unsigned char *)&resp, sizeof(resp), &received, 0);

  if (transferred != sizeof(msg))
    transferred = 0;
  if (received != sizeof(resp))
    received = 0;

  if (received == sizeof(resp))
  {
    *pdwCount = *((uint32_t*)&resp.data[0]);
    *pdwReadPending = *((uint32_t*)&resp.data[4]);
  }
  else
    received = 0;

  return ret;
}


int ReceiverLibUSBCapture::ReadDataFromUSB(char * ptr, int uiCount, DWORD dwCount)
{
  int transferred = 0;
  int received = 0;
  int ret = 0;

  boost::mutex::scoped_lock rawLock(mMutexUSB);

  if (!handle) return 1;

  AWLCANMessage msg;
  msg.id = AWLCANMSG_ID_GETDATA;
  msg.data[0] = (unsigned char) dwCount;

  ret = libusb_bulk_transfer(handle, usbEndPointOut, (unsigned char *)&msg, sizeof(msg), &transferred, 0);

  ret = libusb_bulk_transfer(handle, usbEndPointIn, (unsigned char *)ptr, uiCount, &received, 0);

  if (transferred != sizeof(msg))
    transferred = 0;
  if (received != uiCount)
    received = 0;

  return received;
}

#include "algo.h"

typedef struct
{
  short AcqFifo[GUARDIAN_NUM_CHANNEL * GUARDIAN_SAMPLING_LENGTH];
} tDataFifo;

tDataFifo dataFifo[8];

void ReceiverLibUSBCapture::DoOneThreadIteration()
{
	if (handle)
  {
    DWORD dwCount = 0;
    DWORD dwReadPending = 0;

    LidarQuery(&dwCount, &dwReadPending);

    if (dwCount)
    {
      ReadDataFromUSB((char*)dataFifo, dwCount * sizeof(tDataFifo), dwCount);

      ProcessRaw(rawFromLibUSB, (uint8_t*)dataFifo->AcqFifo, 1600 * 2);
    }

    if (dwReadPending)
    {
      PollMessages(dwReadPending);
    }

    if (!dwCount && !dwReadPending)
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
  }
}

LARGE_INTEGER t0;

bool ReceiverLibUSBCapture::PollMessages(DWORD dwNumMsg)
{
  AWLCANMessage canReq;
  AWLCANMessage canResp[32];
  int transferred = 0;
  int received = 0;
  int ret;

  LARGE_INTEGER t1, t2, t3;
  double elapsedTime1, elapsedTime2, elapsedTime3;

  boost::mutex::scoped_lock rawLock(mMutexUSB);

  if (!handle)
    return false;

  if (dwNumMsg > 32)
    dwNumMsg = 32;

  AWLCANMessage msg;
  msg.id = AWLCANMSG_ID_POLLMESSAGES;
  msg.data[0] = (unsigned char) dwNumMsg;

#ifdef _WIN32
  QueryPerformanceCounter(&t1);
#endif

  ret = libusb_bulk_transfer(handle, usbEndPointOut, (unsigned char *)&msg, sizeof(AWLCANMessage), &transferred, 0);

#ifdef _WIN32
  QueryPerformanceCounter(&t2);
#endif

  ret = libusb_bulk_transfer(handle, usbEndPointIn, (unsigned char *)&canResp, dwNumMsg * sizeof(AWLCANMessage), &received, 0);

#ifdef _WIN32
  QueryPerformanceCounter(&t3);
  elapsedTime1 = 1000.0 * (t1.QuadPart - t0.QuadPart) / frequency.QuadPart;
  elapsedTime2 = 1000.0 * (t2.QuadPart - t1.QuadPart) / frequency.QuadPart;
  elapsedTime3 = 1000.0 * (t3.QuadPart - t2.QuadPart) / frequency.QuadPart;

  t0 = t1;
#endif

  if (transferred != sizeof(AWLCANMessage))
    transferred = 0;
  if (received != dwNumMsg * sizeof(AWLCANMessage))
    received = 0;

  for (DWORD i=0; i<dwNumMsg; i++)
  {
    if (canResp[i].id)
      ParseMessage(canResp[i]);
  }


  //char str[256];
  //sprintf(str, "%0.3f PollMsg(%d) %0.3f / %0.3f \n", elapsedTime1, dwNumMsg, elapsedTime2, elapsedTime3);
  //OutputDebugStringA(str);

  return(false);
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

#ifdef _WIN32
  QueryPerformanceCounter(&t1);
#endif

  ret = libusb_bulk_transfer(handle, usbEndPointOut, (unsigned char *)&inMsg, sizeof(AWLCANMessage), &transferred, 0);

#ifdef _WIN32
  QueryPerformanceCounter(&t2);
#endif

  ret = libusb_bulk_transfer(handle, usbEndPointIn, (unsigned char *)&canResp, sizeof(AWLCANMessage), &received, 0);

#ifdef _WIN32
  QueryPerformanceCounter(&t3);
  elapsedTime1 = 1000.0 * (t1.QuadPart - t0.QuadPart) / frequency.QuadPart;
  elapsedTime2 = 1000.0 * (t2.QuadPart - t1.QuadPart) / frequency.QuadPart;
  elapsedTime3 = 1000.0 * (t3.QuadPart - t2.QuadPart) / frequency.QuadPart;

  t0 = t1;
#endif

  if (transferred != sizeof(AWLCANMessage))
    transferred = 0;
  if (received != sizeof(AWLCANMessage))
    received = 0;

  if (canResp.id)
    ParseMessage(canResp);


  //char str[256];
  //sprintf(str, "%0.3f WriteMsg %0.3f / %0.3f \n", elapsedTime1, elapsedTime2, elapsedTime3);
  //OutputDebugStringA(str);

#if 0
	switch(ret){
		case 0:
			printf("send %d bytes to device\n", transferred);
			return true;
		case LIBUSB_ERROR_TIMEOUT:
			printf("ERROR in bulk write: %d Timeout\n", ret);
			break;
		case LIBUSB_ERROR_PIPE:
			printf("ERROR in bulk write: %d Pipe\n", ret);
			break;
		case LIBUSB_ERROR_OVERFLOW:
			printf("ERROR in bulk write: %d Overflow\n", ret);
			break;
		case LIBUSB_ERROR_NO_DEVICE:
			printf("ERROR in bulk write: %d No Device\n", ret);
			break;
		default:
			printf("ERROR in bulk write: %d\n", ret);
			break;
	}
#endif

	return(false);
}

bool ReceiverLibUSBCapture::SendSoftwareReset()
{
  if (!handle)
    return false;

  AWLCANMessage msg;
  msg.id = AWLCANMSG_ID_COMMANDMESSAGE;
  msg.len = AWLCANMSG_LEN;
  msg.data[0] = 0xC0;   // Set Parameter
  msg.data[1] = 0x03; // AWL_Register  
  msg.data[2] = 997 >> 0;
  msg.data[3] = 997 >> 8;
  msg.data[4] = 0x00;
  msg.data[5] = 0x00;
  msg.data[6] = 0x00;
  msg.data[7] = 0x00;

  WriteMessage(msg);

  return(true);
}

bool ReceiverLibUSBCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
	ReceiverCANCapture::ReadConfigFromPropTree(propTree);

	char receiverKeyString[32];
	sprintf(receiverKeyString, "config.receivers.receiver%d", receiverID);
	std::string receiverKey = receiverKeyString;

	boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
	// Communication parameters

	/*
	#define USB_VENDOR_ID	    0x0483
	#define USB_PRODUCT_ID	    0xFFFF 
	#define USB_ENDPOINT_IN	    (LIBUSB_ENDPOINT_IN  | 1) 
	#define USB_ENDPOINT_OUT	(LIBUSB_ENDPOINT_OUT | 2) 
	#define USB_TIMEOUT	3000 
	*/
	usbVendorId =  receiverNode.get<int>("libUsbVendorId");
	usbProductId =  receiverNode.get<int>("libUsbProductId");
	usbEndPointIn =  receiverNode.get<int>("libUsbEndPointIn");
	usbEndPointOut =  receiverNode.get<int>("libUsbEndPointOut");
	usbTimeOut =  receiverNode.get<int>("libUsbTimeOut");

	return(true);
}

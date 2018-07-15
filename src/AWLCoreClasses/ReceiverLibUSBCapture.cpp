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
closeCANReentryCount(0)
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
closeCANReentryCount(0)

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

bool  ReceiverLibUSBCapture::OpenCANPort()
{

	libusb_init(&context);
	libusb_set_debug(context, 3);

    //Open Device with VendorID and ProductID
	handle = libusb_open_device_with_vid_pid(context, usbVendorId, usbProductId);
	if (!handle) {
		perror("device not found");
		return 1;
	}

	int r = 1;
	//Claim Interface 0 from the device
    r = libusb_claim_interface(handle, 0);
	if (r < 0) {
		fprintf(stderr, "usb_claim_interface error %d\n", r);
		return 2;
	}
	printf("Interface claimed\n");

return true;

posixudp_exit:
	{
		handle = NULL;

		std::string sErr = " Cannot open LibUSB";
		fprintf(stderr, sErr.c_str());
		
		reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
		return(false);
	}


	bFrameInvalidated = false;
}

bool  ReceiverLibUSBCapture::CloseCANPort()

{
	if (closeCANReentryCount > 0) return(false);

	closeCANReentryCount++;

	libusb_close(handle);
	libusb_exit(NULL);

	handle = NULL;

	reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);
	    closeCANReentryCount--;
	return(true);
}


void ReceiverLibUSBCapture::DoOneThreadIteration()

{
	uint8_t buffer[256];
	uint32_t *buf32;
	size_t size = sizeof(buffer);
	int ret;

	buf32 = (uint32_t*)buffer;

	AWLCANMessage msg;
	int nread;

	if (handle) {

		ret = libusb_bulk_transfer(handle, usbEndPointIn, buffer, size, &nread, usbTimeOut);
		if (ret){
			//printf("ERROR in bulk read: %d\n", ret);
		}
		else{
			//printf("%d receive %d bytes from device: %s\n", ++counter, nread, receiveBuf);
			//printf("%s", receiveBuf);  //Use this for benchmarking purposes
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

bool ReceiverLibUSBCapture::WriteMessage(const AWLCANMessage &inMsg)
{
	uint8_t buffer[256];
	uint32_t* buf32;
	size_t size = sizeof(buffer);
	size_t offset = 0;
	int ret;
	int n;

	if (handle < 0) return(false);

	if (inMsg.id != 80) {
		buf32 = (uint32_t*)buffer;
		buf32[0] = inMsg.id;
		offset = sizeof(uint32_t);
	}

	for (int i = 0; i < 8 && i < inMsg.len; i++) {
		buffer[offset + i] = inMsg.data[i];
	}

	size = offset + inMsg.len;

	ret = libusb_bulk_transfer(handle, usbEndPointOut, buffer, size, &n, usbTimeOut);

	switch(ret){
		case 0:
			printf("send %d bytes to device\n", n);
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
	return(false);
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


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
#include <string>
#include "ReceiverLibUSBCapture.h"


using namespace std;
using namespace awl;

const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm fds every repoenPortDelayMillisec, 
										   // To see if the system reconnects


ReceiverLibUSBCapture::ReceiverLibUSBCapture(int receiverID, boost::property_tree::ptree &propTree):
  ReceiverPolledCapture(receiverID, propTree),
context(NULL)
{
  // Read the configuration from the configuration file
  ReadConfigFromPropTree(propTree);
  ReadRegistersFromPropTree(propTree);
}

ReceiverLibUSBCapture::~ReceiverLibUSBCapture()
{
}

int testCount = 0;
int ReceiverLibUSBCapture::ReadBytes(uint8_t * pData, int num)
{
  int received;

  int ret = libusb_bulk_transfer((libusb_device_handle *)handle, usbEndPointIn, (unsigned char *)pData, num, &received, usbTimeOut);
  if (ret)
  {
	  testCount = 0;
	  return 0;
  }
  
  if (num != received)
  {
	  return (received); // Just to have some breakpoint
  }

  testCount++;
  return received;
}

int ReceiverLibUSBCapture::WriteBytes(uint8_t * pData, int num)
{
  int transferred;

  int ret = libusb_bulk_transfer((libusb_device_handle *)handle, usbEndPointOut, (unsigned char *)pData, num, &transferred, usbTimeOut);
  if (ret || (transferred != num))
    return -1;

  return transferred;
}

bool  ReceiverLibUSBCapture::OpenCANPort()
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
		int transferred = 0;
		int received = 0;

		boost::mutex::scoped_lock rawLock(m_Mutex);

		// DGG: Clear any outstanding data left in the USB buffers
		do
		{
			char tmp[256];
			ret = libusb_bulk_transfer((libusb_device_handle*)handle, usbEndPointIn, (unsigned char*)tmp, sizeof(tmp), &received, 100);
		} while (ret == 0 && received > 0);

		AWLCANMessage msg;
		msg.id = AWLCANMSG_ID_LIDARQUERY;

		AWLCANMessage resp;

		transferred = WriteBytes((uint8_t*)& msg, sizeof(msg));
		if (transferred != sizeof(AWLCANMessage))
			return false;

		received = ReadBytes((uint8_t*)& resp, sizeof(resp));
		if (received != sizeof(AWLCANMessage))
			return false;

		rawLock.unlock();

		SendSoftwareReset();

		m_FrameRate = 0;

		return true;
	}
	else {  // if (!handle)
		return false;
	}
}

bool  ReceiverLibUSBCapture::CloseCANPort()
{
  boost::mutex::scoped_lock rawLock(m_Mutex);

  if (handle)
  {
    int ret = libusb_release_interface((libusb_device_handle *)handle, 0);

    libusb_close((libusb_device_handle *)handle);

    handle = NULL;
  }

	//libusb_exit(NULL);

	reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);

	return(true);
}

bool ReceiverLibUSBCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
	ReceiverPolledCapture::ReadConfigFromPropTree(propTree);

	char receiverKeyString[32];
	sprintf(receiverKeyString, "config.receivers.receiver%d", receiverID);
	std::string receiverKey = receiverKeyString;

	boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
	// Communication parameters

	usbVendorId =  receiverNode.get<int>("libUsbVendorId", 1419);
	usbProductId =  receiverNode.get<int>("libUsbProductId",80);
	usbEndPointIn =  receiverNode.get<int>("libUsbEndPointIn", 129);
	usbEndPointOut =  receiverNode.get<int>("libUsbEndPointOut", 2);
	usbTimeOut =  receiverNode.get<int>("libUsbTimeOut", 1000);


	return(true);
}

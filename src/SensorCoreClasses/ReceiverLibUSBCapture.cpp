/* ReceiverLibUSBCapture.cpp */
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
#include <stdio.h>
#include <string>
#include "SensorCoreClassesGlobal.h"
#include "ReceiverLibUSBCapture.h"


SENSORCORE_USE_NAMESPACE

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
					fprintf(stderr, "%ld %d usb_claim_interface %ld %p succeeded\n", (long) receiverID, matches, (long)i, handle);
				}
				else {
					fprintf(stderr, "%ld %d usb_claim_interface %ld %p error %d\n", (long)receiverID, matches, (long) i, handle, ret);
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

		ReceiverCANMessage msg;
		msg.id = RECEIVERCANMSG_ID_LIDARQUERY;

		ReceiverCANMessage resp;

		transferred = WriteBytes((uint8_t*)& msg, sizeof(msg));
		if (transferred != sizeof(ReceiverCANMessage))
			return false;

		received = ReadBytes((uint8_t*)& resp, sizeof(resp));
		if (received != sizeof(ReceiverCANMessage))
			return false;

		rawLock.unlock();

		SendSoftwareReset();

		m_calculatedFrameRate = 0;

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
    libusb_release_interface((libusb_device_handle *)handle, 0);

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

	std::string receiverKey = std::string("config.receivers.receiver") + std::to_string(receiverID);

	boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
	// Communication parameters

	usbVendorId =  receiverNode.get<int>("libUsbVendorId", 1419);
	usbProductId =  receiverNode.get<int>("libUsbProductId",80);
	usbEndPointIn =  (unsigned char) receiverNode.get<int>("libUsbEndPointIn", 129);
	usbEndPointOut =  (unsigned char) receiverNode.get<int>("libUsbEndPointOut", 2);
	usbTimeOut =  receiverNode.get<int>("libUsbTimeOut", 1000);


	return(true);
}

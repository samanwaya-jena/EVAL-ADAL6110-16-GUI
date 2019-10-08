#ifndef AWL_RECEIVER_LIBUSB_CAPTURE_H
#define AWL_RECEIVER_LIBUSB_CAPTURE_H

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

#include <libusb.h>

#include "ReceiverPolledCapture.h"


using namespace std;

namespace awl
{

/** \brief ReceiverLibUSBCapture class is a specialized implementation of the ReceiverCapture
   *        used to acquire data from AWL LIDAR modules	through the LibUSB CAN bus interface.
  */
class ReceiverLibUSBCapture: public ReceiverPolledCapture
{
// Public types
public:

// public Methods
public:

	ReceiverLibUSBCapture(int receiverID,  boost::property_tree::ptree  &propTree);

	/** \brief ReceiverLibUSBCapture Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~ReceiverLibUSBCapture();

protected:

	/** \brief Open the CAN port
	  * \returns true if the port is successfully opened, false otherwise.
	  * \remarks Once the port is successfully opened, use the "reader" pointer to access the can data.
	  *          If opening the port fails, reader is set to NULL.
	  */
	virtual bool OpenCANPort();

	/** \brief Closes the CAN port and associated objects.
	  * \returns true if the port is successfully closed, false otherwise.
	  */
	virtual bool CloseCANPort();

	/** \brief Return True if the device is connected (that is if the communications driver has established connection.
	* \return true if connected, false otherwise.
	*/
	virtual bool IsConnected() { return (handle != NULL); }

	/** \brief Reads the configuration proerties from the configuration file
	  * \param[in] propTree the boost propertyTree created from reading the configuration file.
	  * \returns Returns true otherwise.
	  * \throws  Throws boost error on read of the property keys.
      */
	virtual bool ReadConfigFromPropTree( boost::property_tree::ptree &propTree);

  virtual int ReadBytes(uint8_t * pData, int num);
  virtual int WriteBytes(uint8_t * pData, int num);

// Protected variables
protected:
		libusb_context *context;

		int usbVendorId;
		int usbProductId;
		unsigned char usbEndPointIn;
		unsigned char  usbEndPointOut;
		int usbTimeOut;


		void * swap_handle;

};

} // namespace AWL

#endif //AWL_RECEIVER_LIBUSB_CAPTURE_H

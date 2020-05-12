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
#ifndef SENSORCORE_RECEIVER_LIBUSB_CAPTURE_H
#define SENSORCORE_RECEIVER_LIBUSB_CAPTURE_H

#ifdef WIN32
        #include <libusb.h>
#endif
#ifdef __linux__
        #include <libusb-1.0/libusb.h>
#endif

#include "SensorCoreClassesGlobal.h"
#include "ReceiverPolledCapture.h"

SENSORCORE_BEGIN_NAMESPACE


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

	/** \Brief Get the device serial number
*   Value of 0 indicates that the data is not available.
*/
	virtual uint32_t GetProductID();

	/** \Brief send a message to get the device serial number
	 *   Value of 0 indicates that the data is not available.
	*/

	virtual uint32_t GetUniqueID();


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

	/** \brief Read num bytes from the USB port, into the memory pointed to by pData, using configured TimeOut
	  * \param[in] pData pointer to the read data
	  * \param[in] num quantity  of bytes to read
	  * \return quantity of bytes read.
	 */
	virtual int ReadBytes(uint8_t * pData, int num);

	/** \brief Write num bytes to the USB port, from  the memory pointed to by pData, using configured time out.
	  * \param[in] pData pointer to the data to be written
	  * \param[in] num quantity  of bytes to write
	  * \return quantity of bytes actually written.
	 */
	virtual int WriteBytes(uint8_t * pData, int num);

	/** \Brief send a message to get the device serial number
	  *  Message is asynchonous.  Result will not be available immediately.
      */
	virtual bool QueryUniqueID();

	/** \Brief send a messahge to get the device type
	  *  Message is asynchonous.  Result will not be available immediately.
      */
	 virtual bool QueryProductID();

// Protected variables
protected:
	/** \brief LibUSB context info */
	libusb_context* context;

	/** \brief usbVendorID used to search for the device.  Defined in config file */
	int usbVendorId;
	/** \brief usbPrductID used to search for the device.  Defined in config file */
	int usbProductId;
	/** \brief usbEndPointIn used to search for the device.  Defined in config file */
	unsigned char usbEndPointIn;
	/** \brief usbEndPointOut used to search for the device.  Defined in config file */
	unsigned char  usbEndPointOut;

	/** \brief default time out in read wand write operation to USB port.  Defined in config file */
	int usbTimeOut;

	/** \brief Temporary variable used to hold USB port handle.  Used when we force reassignment of USB port order */
	void* swap_handle;

};

SENSORCORE_END_NAMESPACE

#endif //SENSORCORE_RECEIVER_LIBUSB_CAPTURE_H

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
#ifndef SENSORCORE_RECEIVER_LIBUSB2_CAPTURE_H
#define SENSORCORE_RECEIVER_LIBUSB2_CAPTURE_H

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

#include <stdint.h>

#include "Publisher.h"
#include "ThreadedWorker.h"
#include "DetectionStruct.h"
#include "ReceiverCANCapture.h"

#ifdef WIN32
        #include <libusb.h>
#endif
#ifdef __linux__
        #include <libusb-1.0/libusb.h>
#endif


SENSORCORE_BEGIN_NAMESPACE

/** \brief ReceiverLibUSB2Capture class is a specialized implementation of the ReceiverCapture
   *        used to acquire data from Wagner LIDAR modules	through the LibUSB CAN bus interface.
  */
class ReceiverLibUSB2Capture: public ReceiverCANCapture
{
// Public types
public:
	typedef boost::shared_ptr<ReceiverLibUSB2Capture> Ptr;
	typedef boost::shared_ptr<ReceiverLibUSB2Capture> ConstPtr;

// public Methods
public:

	/** \brief ReceiverLibUSB2Capture constructor from a configuration file information.
 	    * \param[in] inReceiverID  unique receiverID
	    * \param[in] propTree propertyTree that contains teh confoguration file information.
      */

	ReceiverLibUSB2Capture(int receiverID,  boost::property_tree::ptree  &propTree);

	/** \brief ReceiverPosixUDPCapture Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~ReceiverLibUSB2Capture();

	virtual void Go();
	virtual void Stop();

	/** \brief Return true if connexion with the device has been etablished.
	  *        In a LibUSB 2 device, connexion is establihed when port has been acquired and initial device information queries have been answered.
      * \return True if device connexion is established.

      */
	virtual bool IsConnected();

	/** \Brief Get the device serial number
 *   Value of 0 indicates that the data is not available.
*/
	virtual uint32_t GetProductID();

	/** \Brief send a message to get the device serial number
	 *   Value of 0 indicates that the data is not available.
	*/

	virtual uint32_t GetUniqueID();

protected:

	/** \brief Do one iteration of the thread loop.
      */
	virtual void DoOneThreadIteration();

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

	virtual bool WriteMessage(const ReceiverCANMessage &inMsg);

	/** \brief Returns the CellID, given a channelID
   * \param[in] channelID the input "channel"
   * \return ChannelID, indicating unique pixel position in the detector array
   * \remarks For the ReceiverPolledCapture, channels are out of order on receive for raw data messages.
   */
	virtual CellID GetCellIDFromChannel(int inChannelID);

	/** \brief Returns the CellID, given a channelID
	* \param[in] channelID the input "channel"
	* \return ChannelID, indicating unique pixel position in the detector array
	* \remarks For the ReceiverPolledCapture, channels are out of order on receive for raw data messages.
	*/

	virtual int GetChannelIDFromCell(CellID inCellID);

	/** \brief Process Raw Data messages from no-CAN devices
	   * \param[in] rawData pointer to the raw data memeory block
	 */

	virtual void ProcessRaw(uint8_t* rawData);


	/** \brief Reads the configuration proerties from the configuration file
	  * \param[in] propTree the boost propertyTree created from reading the configuration file.
	  * \returns Returns true otherwise.
	  * \throws  Throws boost error on read of the property keys.
      */
	virtual bool ReadConfigFromPropTree( boost::property_tree::ptree &propTree);
  	virtual int ReadBytes(uint8_t * pData, int num);
	virtual int ReadBytes(uint8_t* pData, int num, int timeOut);
  	virtual int WriteBytes(uint8_t * pData, int num);
 	uint8_t *GetCurrentBuffer(void);
	uint8_t *GetNextBuffer(void);

  	void * GetHandle(void);
  	void SetHandle(void *);

	/** \brief Changes the controls of which messages are sent from AWL to the client to reflect provided settings
	* \param[in] frameRate new frame rate for the system. A value of 0 means no change
	* \param[in] voxelMask mask for the analyzed voxels.
	* \param[in] messageMask mask identifies which groups of target/distance/intensity messages are transmitted over CAN.
	* \return true if success.  false on error.
	*/
	bool SetMessageFilters(ReceiverFrameRate frameRate, VoxelMask voxelMask, MessageMask messageMask);

	/** \Brief send a message to get the device serial number
	  *  Message is asynchonous.  Result will not be available immediately.
	*/
	virtual bool QueryUniqueID();

	/** \Brief send a messahge to get the device type
	  *  Message is asynchonous.  Result will not be available immediately.
	*/
	virtual bool QueryProductID();


	/** \Brief Flush theUSB buffers and Pending message Queues in sensor.
	*   Messages are not processed.
	*   This resolves issues when the application is shutdown in the middle of a
	*   USB transfer from the device, and buffer queues on both sides have pending data.
	*/
	bool FlushMessages();


// Protected variables
protected:
		libusb_context *context;

		int usbVendorId;
		int usbProductId;
		unsigned char usbEndPointIn;
		unsigned char  usbEndPointOut;
		int usbTimeOut;


        void*handle;
        void *swap_handle;

        /** \brief Time-out without an input message after which we try to recomnnect the serial port. */
        boost::posix_time::ptime reconnectTime;

		/** \brief counter in the close() call, used to avoid reentry iduring thread close */
		int closeCANReentryCount;
		               /** \brief CAN file descriptor.*/
		boost::mutex m_Mutex;

   		FILE * m_pFile;


		static const int numBuffers = 4;
		uint8_t *buffers[numBuffers];
		int currentBuffer;
}; // class USB2

SENSORCORE_END_NAMESPACE

#endif //SENSORCORE_RECEIVER_LIBUSB2_CAPTURE_H

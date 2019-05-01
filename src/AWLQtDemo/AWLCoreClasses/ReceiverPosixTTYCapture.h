#ifndef AWL_RECEIVER_POSIXTTY_CAPTURE_H
#define AWL_RECEIVER_POSIXTTY_CAPTURE_H

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

#ifndef Q_MOC_RUN
#include <boost/asio.hpp> 
#include <boost/asio/serial_port.hpp> 
#endif

#include "Publisher.h"
#include "ThreadedWorker.h"
#include "DetectionStruct.h"
#include "ReceiverCANCapture.h"

using namespace std;

namespace awl
{

/** \brief TheReceiverCANCapture class is a specialized implementation of the ReceiverCapture
   *        used to acquire data from AWL LIDAR modules	through the PosixTTY CAN bus interface.
  * \author Jean-Yves Deschênes
  */
class ReceiverPosixTTYCapture: public ReceiverCANCapture
{
// Public types
public:
	typedef boost::shared_ptr<ReceiverPosixTTYCapture> Ptr;
    typedef boost::shared_ptr<ReceiverPosixTTYCapture> ConstPtr;

// public Methods
public:
	/** \brief ReceiverPosixTTYCapture constructor.
 	    * \param[in] inReceiverID  unique receiverID
	    * \param[in] inReceiverChannelQty number of channels in the receiver
		* \param[in] inSerialPort name of the serial port for the receiver
		* \param[in] inReceiverColumns number of columns in receiver array
		* \param[in] inReceiverRows number of rows  in the receiver array
		* \param[in] inLineWrapAround "distance" coded between rows in the original communications protocol for arrayed sensors
		* \param[in] inCANRate bit rate of the Receiver Unit (in kBPS).  See ReceiverCANCapture for allowable values (default should be 1MBps).
	    * \param[in] inFrameRate frameRate of the receiver
	    * \param[in] inChannelMask  channelMask indicating which channels are activated in the receiver
	    * \param[in] inMessageMask mask of the messages that are enabled in the communications protocol
	    * \param[in] inRangeOffset rangeOffset that corresponds to a calibration error in the sensor.
		*                          Will automatically be added to any range received.
		* \param[in] inRegistersFPGA default description of the FPGA registers
		* \param[in] inRegistersADC default description of the ADC registers
		* \param[in] inRegistersGPIO default description of the GPIO registers
        * \param[in] inParametersAlgos default description if the algorithm parameters
 		* \param[in] inParametersTrackers default description of the Tracker parameters
      */

	ReceiverPosixTTYCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround, 
		               const std::string &inTtyName, const ReceiverCANCapture::eReceiverCANRate inCANBitRate,
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, 
					   const AlgorithmSet &inParametersAlgos,
					   const AlgorithmSet &inParametersTrackers);

	/** \brief ReceiverPosixTTYCapture constructor from a configuration file information.
 	    * \param[in] inReceiverID  unique receiverID
	    * \param[in] propTree propertyTree that contains teh confoguration file information.
      */

	ReceiverPosixTTYCapture(int receiverID,  boost::property_tree::ptree  &propTree);


	/** \brief ReceiverPosixTTYCapture Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~ReceiverPosixTTYCapture();

public:
// Protected methods
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

	/** \brief Synchronous write of a CAN message in the stream 
 	  * \param[in] outString  Message to send
	  * \return true iof the function was successful. false otherwise.
      */
	virtual bool WriteMessage(const AWLCANMessage &inMsg);

	/** \brief Reads the configuration proerties from the configuration file
	  * \param[in] propTree the boost propertyTree created from reading the configuration file.
	  * \returns Returns true otherwise.
	  * \throws  Throws boost error on read of the property keys.
      */
	virtual bool ReadConfigFromPropTree( boost::property_tree::ptree &propTree);

	void Sync();
	void ProcessFrame();

// Protected variables
protected:
	    	/** \brief TTY Name. */ 
		std::string ttyName;

		bool synced;
		int synced_state;

		int pixel;
		int max_pixel;
		int timestamp;
		size_t payload_size;
		size_t payload_read;

	    	/** \brief TTY file descriptor.*/
		int fd;

		uint8_t buffer[4096];

		/** \brief Time-out without an input message after which we try to recomnnect the serial port. */
		boost::posix_time::ptime reconnectTime;

		/** \brief counter in the close() call, used to avoid reentry iduring thread close */
		int closeCANReentryCount;

};

} // namespace AWL

#endif //AWL_RECEIVER_POSIXTTY_CAPTURE_H

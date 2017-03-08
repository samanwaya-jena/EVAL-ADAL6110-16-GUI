#ifndef AWL_RECEIVER_EASYSYNC_CAPTURE_H
#define AWL_RECEIVER_EASYSYNC_CAPTURE_H

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

#include "BlockingReader.h"

#include "Publisher.h"
#include "ThreadedWorker.h"
#include "DetectionStruct.h"
#include "ReceiverCANCapture.h"

using namespace std;

namespace awl
{

/** \brief TheReceiverCANCapture class is a specialized implementation of the ReceiverCapture
   *        used to acquire data from AWL LIDAR modules	through the EasySync CAN bus interface.
  * \author Jean-Yves Deschênes
  */
class ReceiverEasySyncCapture: public ReceiverCANCapture
{
// Public types
public:
	typedef boost::shared_ptr<ReceiverEasySyncCapture> Ptr;
    typedef boost::shared_ptr<ReceiverEasySyncCapture> ConstPtr;

// public Methods
public:
	/** \brief ReceiverEasySyncCapture constructor.
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
      */

	ReceiverEasySyncCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround, 
		               const std::string &inSerialPort, const ReceiverCANCapture::eReceiverCANRate inCANBitRate,
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, const AlgorithmSet &inParametersAlgos);

	/** \brief ReceiverEasySyncCapture constructor from a configuration file information.
 	    * \param[in] inReceiverID  unique receiverID
	    * \param[in] propTree propertyTree that contains teh confoguration file information.
      */

	ReceiverEasySyncCapture(int receiverID,  boost::property_tree::ptree  &propTree);


	/** \brief ReceiverEasySyncCapture Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~ReceiverEasySyncCapture();

public:
// Protected methods
protected:

	/** \brief Do one iteration of the thread loop.
      */
	virtual void DoOneThreadIteration();

	/** \brief Process one line of CAN data from serial port and interpret it into a CANmgs structure.
	  * \param[in] inResponse  The string that has to be decoded.
	  * \param[out] outMsg  The formatted CAN message.  Is invalid when false is returned.
	  * \return true if the inResponse line contains a structured CAN message, false otherwise.
      */
	bool ParseLine(std::string inResponse, AWLCANMessage &outMsg);

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

	/** \brief Synchronous write of a sting in the stream 
 	  * \param[in] inString  Message to send
      */
	virtual void WriteString(std::string inString);

	/** \brief Synchronous write of a CAN message in the stream 
 	  * \param[in] outString  Message to send
	  * \return true iof the function was successful. false otherwise.
      */
	virtual bool WriteMessage(const AWLCANMessage &inMsg);


	/** \brief Get a byte in hex format from the text response string supplied by the EasySync and convert it into
	  *        a uint8_t value.  Used in parsing the EasySync response lines.
	  * \param[in] inResponse  EasySync response string that corresponds to a CAN message.
	  * \param[out] outByte  interpreted value of the substring.
	  * \param[in] startIndex  Index of the substring that we want to parse within the string.
	  * \param[in] len length of the substring to be interpreted.  Should be 1 or 2.  
	  *                 Default is 1. Values exceeding 2 are limited to 2 characters.
 	  * \return true if the function was successful (the substring is a valid hex number) . False otherwise.
     */
	bool GetDataByte(std::string &inResponse, uint8_t &outByte, int startIndex, int len = 1);

	/** \brief Parse a message ID (which is an unsigned long in hex format) from the text response string supplied 
	  *        by the EasySync and convert it into a an unsigned_long value.
	  *        Used in parsing the EasySync response lines.
	  * \param[in] inResponse  EasySync response string that corresponds to a CAN message.
	  * \param[out] outID  interpreded messageID extracted from the substring.
	  * \param[in] startIndex  Index of the substring that we want to parse within the string.
 	  * \return true if the function was successful (the substring is a valid hex number) . False otherwise.
     */
	bool GetStandardID(std::string &inResponse,  unsigned long &outID, int startIndex);

	/** \brief Reads the configuration proerties from the configuration file
	  * \param[in] propTree the boost propertyTree created from reading the configuration file.
	  * \returns Returns true otherwise.
	  * \throws  Throws boost error on read of the property keys.
      */
	virtual bool ReadConfigFromPropTree( boost::property_tree::ptree &propTree);

	/** \brief Converts the ReceiverCANCapture canRate into a EasySync-specific rate string, stored in the canBitRateCode member.
      */
	void ConvertEasySyncCANBitRateCode();
// Protected variables
protected:
	    /** \brief Operating system identification string for the communications port (ex: "COM16") */
		std::string sCommPort;
	    /** \brief Commmunications rate, in baud.*/
		long serialPortRate;

		/** \brief BOOST i/o service object for the serial port. */
		boost::asio::io_service io;
		/** \brief BOOST serial port status and description. */
		boost::asio::serial_port *port;
		/** \brief A blocking reader for this port that will time out a read after 500 milliseconds. */
		blocking_reader *reader; 

		/** \brief Response string acquires from the EasySync CAN to Serial Converter. */
		std::string responseString;

		
		/** \brief Time-out without an input message after which we try to recomnnect the serial port. */
		boost::posix_time::ptime reconnectTime;

		/** \brief counter in the closeCanPort() call, used to avoid reentry iduring thread close */
		int closeCANReentryCount;

		/** \brief EasySync CAN BitRate string.  Default should be "S8" for 1MBPS (see EasySync "Set CAN Channel Timing – simple" command). */
		std::string sCANBitRateCode;
};

} // namespace AWL

#endif //AWL_RECEIVER_EASYSYNC_CAPTURE_H
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

#ifndef SENSORCORE_RECEIVER_CAN_CAPTURE_H
#define SENSORCORE_RECEIVER_CAN_CAPTURE_H

#include <stdint.h>

#include "SensorCoreClassesGlobal.h"
#include "Publisher.h"
#include "ThreadedWorker.h"
#include "DetectionStruct.h"
#include "ReceiverCapture.h"
#include "ReceiverCANMessageDef.h"

#define GLOBAL_PARAMETERS_ID 0

//#define FORCE_FRAME_RESYNC_PATCH 1

SENSORCORE_BEGIN_NAMESPACE


/** \brief TheReceiverCANCapture class is a virtual base class that is a specialized implementation of the ReceiverCapture
   *        It implements the mechanics for acquisition of CAN data messages.
   *
   *        Derived classes that wish to implement a CAN interface must define methods for:
   *        OpenCANPort(), CloseCANPort, WriteMessage(), DoOneThreadIteration() and ReadConfigFromPropTree().
   *
   *        To parse incoming messages:
   *            In DoOneThreadIteration(), once a CAN Message has been read and Formatted into a ReceiverCANMessage,
   *            the derived classes can call ParseMessage() to continue the interpretation of the messages.
   *            In DoOneThreadIteration(), derived classes should also handle the disconnection / reconnection of the
   *            CAN port in a robust manner.
   *
   *        To Write outgoing messages:
   *            Write the appropriate WriteMessage() to send an ReceiverCANMessage to the CAN device.
   *
   *        Use the ReceiverEasySyncCapture class as a reference for deriving CAN-Based Receivers.
  * \author Jean-Yves Deschênes
  */
class ReceiverCANCapture: public ReceiverCapture
{
// Public types
public:
	typedef boost::shared_ptr<ReceiverCANCapture> Ptr;
    typedef boost::shared_ptr<ReceiverCANCapture> ConstPtr;

	/**Allowed CAN Rates for the ReceiverCANCapture device. Preferred rate is canRate1Mbps*/
	typedef enum eReceiverCANRate 
	{	
		canRate1Mbps = 1000,
		canRate500kbps = 500,
		canRate250kbps = 250,
		canRate125kbps = 125,
		canRate100kbps = 100,
		canRate50kbps = 50,
		canRate10kps = 10
	}
	eReceiverCANRate;

// public Methods
public:
	/** \brief ReceiverCANCapture constructor.
 	    * \param[in] inReceiverID  unique receiverID
	    * \param[in] inReceiverVoxelQty number of voxels in the receiver
	    * \param[in] inCANRate CAN Communitcations baud rate of the receiver
		* \param[in] inReceiverColumns number of columns in receiver array
		* \param[in] inReceiverRows number of rows  in the receiver array
		* \param[in] inLineWrapAround "distance" coded between rows in the original communications protocol for arrayed sensors
		* \param[in] inDemandedFrameRate frameRate of the receiver
	    * \param[in] inVoxelMask  voxelMask indicating which voxels are activated in the receiver
	    * \param[in] inMessageMask mask of the messages that are enabled in the communications protocol
	    * \param[in] inRangeOffset rangeOffset that corresponds to a calibration error in the sensor.
		*                          Will automatically be added to any range received.
		* \param[in] inRegistersFPGA default de .cription of the FPGA registers
		* \param[in] inRegistersADC default description of the ADC registers
		* \param[in] inRegistersGPIO default description of the GPIO registers
        * \param[in] inParametersAlgos default description of the algorithm parameters
		* \param[in] inParametersTrackers default description of the Tracker parameters
      */
	ReceiverCANCapture(int receiverID, int inReceiverVoxelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround, 
					   eReceiverCANRate inCANRate, ReceiverFrameRate inDemandedFrameRate, VoxelMask &inVoxelMask, MessageMask &inMessageMask, float inRangeOffset,
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, 
					   const AlgorithmSet &inParametersAlgos,
					   const AlgorithmSet &inParametersTrackers);

	/** \brief ReceiverCANCapture constructor from a configuration file information.
 	    * \param[in] inReceiverID  unique receiverID
	    * \param[in] propTree propertyTree that contains teh confoguration file information.
      */
	ReceiverCANCapture(int receiverID,  boost::property_tree::ptree  &propTree);


	/** \brief ReceiverCANCapture Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~ReceiverCANCapture();

	/** \brief Start the lidar Data Projection  thread
      */
	virtual void  Go(); 

	/** \brief Stop the lidar data projection thread
      */
	virtual void  Stop(); 

public:
	/** \brief Parse the CAN messages and call the appropriate processing function 
 	    * \param[in] inMsg  CAN message contents
      */
	void ParseMessage(ReceiverCANMessage &inMsg);




		/** \brief Sets the playback filename at the receiver device level.
      * \param[in] inPlaybackFileName the name for the playback file.
      * \return true if success.  false on error
     */
	virtual bool SetPlaybackFileName(std::string inPlaybackFileName);


	/** \brief Sets the record filename at the receiver device level.
      * \param[in] inRecordFileName the name for the playback file.
      * \return true if success.  false on error
     */
	virtual bool SetRecordFileName(std::string inRecordFileName);

	/** \brief Starts the playback of the file specified in the last SetPlaybackFileName() call.
      * \param[in] frameRate playback frame rate. Ignored on some implementations of AWL.
      * \param[in] voxelMask mask for the voxels that will be played back. that an empty voxelMask is equivalent to StopPlayback().
      * \return true if success.  false on error
 	  * \remarks status of playback is updated in the receiverStatus member.
	  * \remarks File is recorded locally on SD Card.
     */
	virtual bool StartPlayback(ReceiverFrameRate frameRate, VoxelMask voxelMask);

	/** \brief Starts the record of a file whose name was set using the last SetRecordFileName() call. 
      * \param[in] frameRate recording frame rate. Ignored on some implementations of AWL (in this case, default frame rate is used).
      * \param[in] voxelMask mask for the recorded voxels. that an empty voxelMask is equivalent to StopRecord().
      * \return true if success.  false on error
	  * \remarks status of record is updated in the receiverStatus member.
	  * \remarks File is recorded locally on SD Card.
     */
	virtual bool StartRecord(ReceiverFrameRate frameRate, VoxelMask voxelMask);

	/** \brief Stops any current playback of a file. 
      * \return true if success.  false on error
 	  * \remarks status of playback is updated in the receiverStatus member.
  	  * \remarks After playback is interrupted, the unit should return to the default acquisition mode.
    */
	virtual bool StopPlayback();
	

	/** \brief Stops any current recording. 
      * \return true if success.  false on error
  	  * \remarks status of recording is updated in the receiverStatus member.
 	  * \remarks After recording, the unit should return to the default acquisition mode.
    */
	virtual bool StopRecord();

	/** \brief Starts the internal calibration of the system. 
      * \param[in] frameQty number of frames on which calibration is calculated
      * \param[in] beta beta parameter for the calibration
      * \param[in] voxelMask mask for the recorded voxelss. that an empty voxelMask is equivalent to StopRecord().
      * \return true if success.  false on error
	  * \remarks Calibration file is recorded locally on SD Card.
     */
	virtual bool StartCalibration(uint8_t frameQty, float beta, VoxelMask voxelMask);


	/** \brief Issues the command to set the current algorithm in the sensor.
	  *\param[in] algorigthmID  ID of the selected algorithm.
	* \return true if success.  false on error.
	*/
		
	virtual bool SetAlgorithm(uint16_t algorithmID);


	virtual bool SetTracker(uint16_t trackerID);

	/** \brief Sets an internal FPGA register to the value sent as argument. 
	  *\param[in] registerAddress Adrress of the register to change.
	  *\param[in] registerValue Value to put into register.
	* \return true if success.  false on error.
	*/
		
	virtual bool SetFPGARegister(uint16_t registerAddress, uint32_t registerValue);

	/** \brief Sets an ADC register to the value sent as argument. 
	  *\param[in] registerAddress Adrress of the register to change.
	  *\param[in] registerValue Value to put into register.
	* \return true if success.  false on error.
	*/
	virtual bool SetADCRegister(uint16_t registerAddress, uint32_t registerValue);

	/** \brief Sets an internal GPIO register to the value sent as argument. 
	  *\param[in] registerAddress Adrress of the register to change.
	  *\param[in] registerValue Value to put into register (values accepted are 0-1).
	* \return true if success.  false on error.
	*/
		
	virtual bool SetGPIORegister(uint16_t registerAddress, uint32_t registerValue);

	/** \brief Sets algorithm parameters to the value sent as argument. 
	  *\param[in] algoID ID of the detection algo affected by the change.
	  *\param[in] registerAddress Adrress of the parameter to change.
	  *\param[in] registerValue Value to put into register (values accepted are 0-1).
	* \return true if success.  false on error.
	*/
		
	virtual bool SetAlgoParameter(int algoID, uint16_t registerAddress, uint32_t registerValue);

	/** \brief Sets global  algorithm parameters to the value sent as argument. 
	  *\param[in] registerAddress Adrress of the parameter to change.
	  *\param[in] registerValue Value to put into register (values accepted are 0-1).
	* \return true if success.  false on error.
	*/
		
	virtual bool SetGlobalAlgoParameter(uint16_t registerAddress, uint32_t registerValue);

	/** \brief Sets tracker parameters to the value sent as argument.
	*\param[in] trackerID of the detection algo affected by the change.
	*\param[in] registerAddress Adrress of the parameter to change.
	*\param[in] registerValue Value to put into register (values accepted are 0-1).
	* \return true if success.  false on error.
	*/

	virtual bool SetTrackerParameter(int trackerID, uint16_t registerAddress, uint32_t registerValue);

	/** \brief Changes the controls of which messages are sent from AWL to the client to reflect provided settings
    * \param[in] frameRate new frame rate for the system. A value of 0 means no change
    * \param[in] voxelMask mask for the analyzed voxels.
    * \param[in] messageMask mask identifies which groups of target/distance/intensity messages are transmitted over CAN.
	* \return true if success.  false on error.
	*/
		
	virtual bool SetMessageFilters(ReceiverFrameRate frameRate, VoxelMask voxelMask, MessageMask messageMask);


	/** \brief Issues an asynchronous query command to get the current algorithm.
	* \return true if success.  false on error.
	*/
	virtual bool QueryAlgorithm();

	/** \brief Issues an asynchronous query command to get the current tracker.
	* \return true if success.  false on error.
	*/
	virtual bool QueryTracker();


	/** \brief Send an asynchronous query command for an internal FPGA register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the receiverStatus member and in globalSettings. 
		*/
	virtual bool QueryFPGARegister(uint16_t registerAddress);

	/** \brief Send an asynchronous query command for an ADC register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the receiverStatus member and in globalSettings. 
		*/
	virtual bool QueryADCRegister(uint16_t registerAddress);

	/** \brief Send an asynchronous query command for a GPIO register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the receiverStatus member and in the globalSettings. 
		*/
	virtual bool QueryGPIORegister(uint16_t registerAddress);

	/** \brief Send an asynchronous query command for an algorithm parameter. 
		  *\param[in] algoID ID of the detection algo for which we want to query.
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the receiverStatus member and in the globalSettings. 
		*/
	virtual bool QueryAlgoParameter(int algoID, uint16_t registerAddress);

		/** \brief Send an asynchronous query command for a global algorithm parameter. 
		  *\param[in] algoID ID of the detection algo for which we want to query.
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the receiverStatus member and in the globalSettings. 
		*/
	virtual bool QueryGlobalAlgoParameter(uint16_t registerAddress);

	/** \brief Send an asynchronous query command for a tracker parameter.
	*\param[in] trackerID ID of the tracker algo for which we want to query.
	*\param[in] registerAddress Adrress of the register to query.
	* \return true if success.  false on error.
	* \remarks On reception of the answer to query the register address and value will be
	*          placed in the a trackerParameters registerSet.
	*/
	virtual bool QueryTrackerParameter(int trackerID, uint16_t registerAddress);

	/** \Brief Get the device serial number
     *   Value of 0 indicates that the data is not available.
     */
	virtual uint32_t GetProductID();

	/** \Brief send a message to get the device serial number
	 *   Value of 0 indicates that the data is not available.
	 */
	virtual uint32_t GetUniqueID();


// Protected methods
protected:

	/** \brief Return the lidar data rendering thread status
      * \return true if the lidar data rendering thread is stoppped.
      */
	virtual void  DoThreadLoop();

	/** \brief Do one iteration of the thread loop.
	  *        Acquire CAN Data until ParseMessage() can be called with a CAN Message.
	  *        Try to manage automatic connection/reconnection of the CAN communications in the loop.
      */
	virtual void DoOneThreadIteration() = 0;

	/** \brief Read the sensor status message (001)
 	    * \param[in] inMsg  CAN message contents
      */
	void ParseSensorStatus(ReceiverCANMessage &inMsg);

	/** \brief Read the sensor boot status message (002)
 	    * \param[in] inMsg  CAN message contents
      */
	void ParseSensorBoot(ReceiverCANMessage &inMsg);

	/** \brief Read the distance readings from CAN messages (20-26 30-36)
 	    * \param[in] inMsg   CAN message contents
      */
	void ParseChannelDistance(ReceiverCANMessage &inMsg);


	/** \brief Read the intensity readings from CAN messages (40-46 50-56)
 	    * \param[in] inMsg   CAN message contents
     */
	void ParseChannelIntensity(ReceiverCANMessage &inMsg);

	/** \brief Read the voxel distance and intensity readings from CAN messages (60)
	* \param[in] inMsg   CAN message contents
	* \remarks This message is sent in place of independent distance and intensity messages on sensors made after AWL
	*/
	void ParseChannelDistanceAndIntensity(ReceiverCANMessage &inMsg);

	/** \brief Read the obstacle track description message (10). If required, creste the corresponding Track object
 	    * \param[in] inMsg   CAN message contents
     */
	void ParseObstacleTrack(ReceiverCANMessage &inMsg);

	/** \brief Read the obstacle speed/velocity  message (11). If required, creste the corresponding Track object
 	    * \param[in] inMsg   CAN message contents
     */
	void ParseObstacleVelocity(ReceiverCANMessage &inMsg);

	/** \brief Read the obstacle size/intensity message (12). If required, creste the corresponding Track object
 	    * \param[in] inMsg   CAN message contents
     */
	void ParseObstacleSize(ReceiverCANMessage &inMsg);

	/** \brief Read the obstacle angular position message (31). If required, creste the corresponding Track object
 	    * \param[in] inMsg   CAN message contents
     */
	void ParseObstacleAngularPosition(ReceiverCANMessage &inMsg);


	/** \brief Parse control messages (80) and dispatch to appropriate handling method 
 	    * \param[in] inMsg   CAN message contents
     */	
	void ParseControlMessage(ReceiverCANMessage &inMsg);

	/** \brief Process the debug / parameter / set_parameter message (0xC0)
 	    * \param[in] inMsg   CAN message contents
		* \ remarks	This message is normally a master message.  It is ignored. 
     */	
	void ParseParameterSet(ReceiverCANMessage &inMsg);

	/** \brief Process the debug / parameter / Query message (0xC1)
 	    * \param[in] inMsg   CAN message contents
		* \ remarks	This message is normally a master message.  It is ignored. 
     */	
	void ParseParameterQuery(ReceiverCANMessage &inMsg);

	/** \brief Process the debug / parameter / response message (0xC2)
 	    * \param[in] inMsg   CAN message contents
     */	
	void ParseParameterResponse(ReceiverCANMessage &inMsg);

	/** \brief Process the debug / parameter / error message (0xC3)
 	    * \param[in] inMsg   CAN message contents
		* \remarks  The error flag is set in the receiverStatus.
     */	
	void ParseParameterError(ReceiverCANMessage &inMsg);
#ifdef FORCE_FRAME_RESYNC_PATCH
	/** \brief Check for potential end of frame, even in absence of the End of Frame message (Message 9)
	* \param[in] inMsg   CAN message contents
	* \remarks  This is a "patch" intended to correct a known bug in AWL, where CAN frames may "overwrite" each other, 
	* \         causing the end of frame message not to be received.
	*/
	void ForceFrameResync(ReceiverCANMessage &inMsg);
#endif //FORCE_FRAME_RESYNC_PATCH

	void ParseParameterAlgoSelectResponse(ReceiverCANMessage &inMsg);
	void ParseParameterAlgoParameterResponse(ReceiverCANMessage &inMsg);
	void ParseParameterTrackerSelectResponse(ReceiverCANMessage &inMsg);
	void ParseParameterTrackerParameterResponse(ReceiverCANMessage &inMsg);
	void ParseParameterFPGARegisterResponse(ReceiverCANMessage &inMsg);
	void ParseParameterBiasResponse(ReceiverCANMessage &inMsg);
	void ParseParameterADCRegisterResponse(ReceiverCANMessage &inMsg);
	void ParseParameterPresetResponse(ReceiverCANMessage &inMsg);
	void ParseParameterGlobalParameterResponse(ReceiverCANMessage &inMsg);
	void ParseParameterGPIORegisterResponse(ReceiverCANMessage &inMsg);
	void ParseParameterDateTimeResponse(ReceiverCANMessage &inMsg);
	void ParseParameterRecordResponse(ReceiverCANMessage &inMsg);
	void ParseParameterPlaybackResponse(ReceiverCANMessage &inMsg);
	void ParseParameterSensorSpecificResponse(ReceiverCANMessage& inMsg);

	void ParseParameterAlgoSelectError(ReceiverCANMessage &inMsg);
	void ParseParameterAlgoParameterError(ReceiverCANMessage &inMsg);
	void ParseParameterTrackerSelectError(ReceiverCANMessage &inMsg);
	void ParseParameterTrackerParameterError(ReceiverCANMessage &inMsg);
	void ParseParameterFPGARegisterError(ReceiverCANMessage &inMsg);
	void ParseParameterBiasError(ReceiverCANMessage &inMsg);
	void ParseParameterADCRegisterError(ReceiverCANMessage &inMsg);
	void ParseParameterPresetError(ReceiverCANMessage &inMsg);
	void ParseParameterGlobalParameterError(ReceiverCANMessage &inMsg);
	void ParseParameterGPIORegisterError(ReceiverCANMessage &inMsg);
	void ParseParameterDateTimeError(ReceiverCANMessage &inMsg);
	void ParseParameterRecordError(ReceiverCANMessage &inMsg);
	void ParseParameterPlaybackError(ReceiverCANMessage &inMsg);
	void ParseParameterSensorSpecificError(ReceiverCANMessage& inMsg);

	/** \brief Open the CAN port
	  * \returns true if the port is successfully opened, false otherwise.
	  * \remarks Once the port is successfully opened, use the "reader" pointer to access the can data.
	  *          If opening the port fails, reader is set to NULL.
	  */
	virtual bool OpenCANPort()  = 0;


	/** \brief Closes the CAN port and associated objects.
	  * \returns true if the port is successfully closed, false otherwise.
	  */
	virtual bool CloseCANPort() = 0;

	/** \brief Synchronous write of a CAN message in the stream 
 	  * \param[in] outString  Message to send
	  * \return true iof the function was successful. false otherwise.
      */
	virtual bool WriteMessage(const ReceiverCANMessage &inMsg) = 0;

	/** \brief Put the current date and time to the CAN port
 	  * \return true iof the function was successful. false otherwise.
     */
	bool WriteCurrentDateTime();

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

	/** \Brief send a message to get the device serial number
	*  Message is asynchonous.  Result will not be available immediately.
	 */
	virtual bool QueryUniqueID();

	/** \Brief send a messahge to get the device type
	  *  Message is asynchonous.  Result will not be available immediately.
	*/
	virtual bool QueryProductID();


	/** \brief Reads the configuration proerties from the configuration file
	  * \param[in] propTree the boost propertyTree created from reading the configuration file.
	  * \returns Returns true otherwise.
	  * \throws  Throws boost error on read of the property keys.
      */
	virtual bool ReadConfigFromPropTree( boost::property_tree::ptree &propTree);

	/** \brief Reads the description of registers (FPGA, ADC and GPIO) and controllable algorithm parameters 
	  *        from the configuration file.
	  * \param[in] propTree the boost propertyTree created from reading the configuration file.
	  * \returns Returns false in case of a read of the property tree, when all the register description is absent or not found.  Returns true otherwise.
	  * \throws  Throws boost error on read of the property keys orther than the root key.
      */
	virtual bool ReadRegistersFromPropTree( boost::property_tree::ptree &propTree);

// Protected variables
protected:
	    /** \brief All dates received from the CAN devices are sent as offsets from 1900. Add the yearOffset to convert to
		  *        POSIX-Compliant years. This may change depending on device versions*/
		uint16_t yearOffset;		   

		/** \brief All months received from the CAN devices are sent starting from 0. 
		  *        POSIX starts at 1. Add the monthOffset to convert to
		  *        POSIX-Compliant months. This may change depending on device versions*/
		uint16_t monthOffset;			


		/** \brief counter in the closeCanPort() call, used to avoid reentry iduring thread close */
		int closeCANReentryCount;

		/** \brief CAN Rate
		 */
		eReceiverCANRate canRate;


		/** \briefmaximum number of rawData buffers for non-CAN devices transmitting raw Data */
		static const size_t maxRawBufferCount = 1024;
		/** \brief size of a rawData Buffer for non-CAN devices transmitting raw Data */
		static const size_t maxRawBufferSize = 16384;

		/** \brief buffers used to acquire rawData form non-CAN devices */
		uint8_t * rawBuffers[maxRawBufferCount];

		/** \brief number of RawData buffers acqually in use */
		size_t rawBufferCount;

		/** \brief buffers used to acquire rawData form non-CAN devices */
		size_t sampleCount;

		/** \brief higest voxelID for voxels received.  This is used to determine end of frame on non-CAN devices  */
		int max_voxel;

#ifdef FORCE_FRAME_RESYNC_PATCH
		/** \brief Channel Mask variable used to determine if frames are out of order in PATCH ForceFrameResync*/
		VoxelMask lastVoxelMask;
		/** \brief Clast received Channel ID used to determine if frames are out of order in PATCH ForceFrameResync*/
		uint16_t lastChannelID;
#endif //FORCE_FRAME_RESYNC_PATCH

};

SENSORCORE_END_NAMESPACE

#endif // SNSORCORE_RECEIVER_CAN_CAPTURE

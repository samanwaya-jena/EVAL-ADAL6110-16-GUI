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

#ifndef SENSORCORE_RECEIVER_CAPTURE_H
#define SENSORCORE_RECEIVER_CAPTURE_H

#include <stdint.h>

#include <fstream>

#ifndef Q_MOC_RUN

#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/container/vector.hpp>
#include <boost/property_tree/ptree.hpp>

#endif

#include "SensorCoreClassesGlobal.h"
#include "Publisher.h"
#include "ThreadedWorker.h"

#include "DetectionStruct.h"

const int maxReceiverFrames = 100;

SENSORCORE_BEGIN_NAMESPACE


 /** \brief MessageMask struct describes receiver message groups that can be toggled on/off 
  *        for customized operations or to preserve bandwidth  
  *        and communications
  * \author Jean-Yves Desch�nes
  */
typedef union 
{
	uint8_t byteData;
	struct  {
		bool raw					: 1;
		bool obstacle				: 1;
		bool distance_1_4			: 1;
		bool distance_5_8			: 1;
		bool intensity_1_4			: 1;
		bool intensity_5_8			: 1;
		bool distance_intensity			: 1;
		bool obstacle_compact			: 1;
	} bitFieldData;
} MessageMask;

/** \brief FrameRate in frames per second.
* \author Jean-Yves Desch�nes
  */
typedef uint16_t ReceiverFrameRate;


/** \brief ReceiverStatus struct describes receiver operation status  
  * \author Jean-Yves Desch�nes
  */

typedef struct ReceiverStatus
{
public:

	/** \brief Set to true each time a new status is updated.
	  *        Should be set to false by the reader thread.
      */
	bool  bUpdated;


	/** \brief System temperature.
      */
	float temperature;

	/** \brief Core voltage.
      */
	float voltage;

	/** \brief Current demanded frame rate. 
      */
	ReceiverFrameRate demandedFrameRate;

	/** \brief Current frame rate in sensor. 
     */
	ReceiverFrameRate obtainedFrameRate;


	/** \brief Hardware error flags.
      */
	union 
	{
		uint8_t byteData;
		struct bitFieldData {
			bool emitter0 : 1;
			bool emitter1 : 1;
			bool receiver : 1;
			bool dsp		 : 1;
			bool memory	 : 1;
		} bitFieldData;
	} hardwareError;

	/** \brief Receiver voxel error flags.  Indicates an error condition on a specific detector.
      */

	union 
	{
		uint8_t byteData;
		struct  {
			bool voxel0	: 1;
			bool voxel1	: 1;
			bool voxel2	: 1;
			bool voxel3	: 1;
			bool voxel4	: 1;
			bool voxel5	: 1;
			bool voxel6	: 1;
			bool monitor		: 1;
		} bitFieldData;
	} receiverError;


	/** \brief Operation errors.
      */
	union  {
		 uint8_t byteData;
		struct  {
			bool selfTest	: 1;
			bool	shutdown	: 1;
			bool	sensorBlocked : 1;
			bool reducedPerformance : 1;
			bool saturation : 1;
		} bitFieldData;
	} status;
	
	/** \brief Firmware version information.
      */

	struct {
		 uint8_t major;
		 uint8_t minor;
	} version;

	/** \brief Boot sequence checksum validation.
      */
	union  {
		 uint8_t byteData;
		struct  {
			bool mainChecksum : 1;
			bool auxChecksum  : 1;
		} bitFieldData;
	} bootChecksumError;

	/** \brief Boot sequence hardware self-tests.
      */
	union  {
		 uint8_t byteData;
		struct  {
			bool emitter0	: 1;
			bool emitter1	: 1;
			bool receiver	: 1;
			bool dsp			: 1;
			bool memory		: 1;
			bool	checksum	: 1;
		} bitFieldData;
	} bootSelfTest;

	/** \brief Set to true when the sensor is in record mode.
      */
	bool bInRecord;

	/** \brief Set to true when the sensor is in playback mode.
      */
	bool bInPlayback;

	/** \brief Current playback file name.
      */
	std::string sPlaybackFileName;

	/** \brief Current record file name.
      */

	std::string sRecordFileName;

	/** \brief Communications protocol command error.  
	  * 0 means no error. Otherwise, the Command type that generated the error code is indicated.
	  * \remarks Details on the error can be found in the communications log file.
      */
	uint8_t lastCommandError;

	/** \brief Last FPGA Register Address Read.  Used to update UI */
	uint16_t fpgaRegisterAddressRead;
	/** \brief Last FPGA Register Value Read.  Used to update UI */
	uint32_t fpgaRegisterValueRead;

	/** \brief Last ADC Register Address Read.  Used to update UI */
	uint16_t adcRegisterAddressRead;

	/** \brief Last ADC Register Value Read.  Used to update UI */
	uint32_t adcRegisterValueRead;

	/** \brief Last GPIO Register Address Read.  Used to update UI */
	uint16_t gpioRegisterAddressRead;

	/** \brief Last GPIO Register ValueRead.  Used to update UI */
	uint32_t gpioRegisterValueRead;

	/** \brief Current Algo.  */
	uint16_t currentAlgo;
	/** \brief True if current algo is pending an update following user interaction.  */
	uint16_t currentAlgoPendingUpdates;

	/** \brief Current Tracking Algo.  */
	uint16_t currentTracker;
	/** \brief True if current Tracker is pending an update following user interaction.  */
	uint16_t currentTrackerPendingUpdates;

	/** \brief Current SNR.  */
	float    signalToNoiseFloor;
	
	/** \brief Voxel mask sent to Receiver to filter messages.  */
	VoxelMask	voxelMask;
	/** \brief MessageMask mask sent to Receiver to filter messages.  */
	MessageMask	messageMask;
}
ReceiverStatus;

/** \brief State indicators used to follow the register update status.
 *         Register update is asyncronous.
*/
typedef enum eUpdateStatus
{
	/**Register value is up to date*/
	updateStatusUpToDate = 0, // 
	/**Register value just has been queried.  Waiting for sensor response*/
	updateStatusPendingUpdate = 1,   
	/**Register value has been updated in memory, but not refreshed at the UI level.*/
	updateStatusPendingVisual = 2  
}
eUpdateStatus;

/** \brief RegisterSetting is an internal representation of an AWL register for an internal device
*/
typedef struct RegisterSetting 
{
  /** \brief Flag indicating whether the register is an advanced register
  */
  bool bAdvanced;

	/** \brief String used to describe the index. 
	  *        This string corresponds to the index identifier in the device, typically used in the user manual.
	 * \remarks Can be alphanumeric or numeric or hex string (ex: "R0", "0x1A", "09")..
	*/
	std::string sIndex; 

	/** \brief String used to describe the register function. 
	*/
	std::string sDescription;

	/** \brief Physical address of the register. 
	 * \remarks On some devices, the physical address may be different than the register index. (Ex:multibyte registers).
	*/
	uint16_t address;

	/** \brief Current value of the register. 
	*/
	uint32_t value;

	/** \brief Count of the messages demanding a change in the value, that have not been acknowledged by the 
	  *        AWL moldule. A non-zero count indicates that the value may not have been received and reflected in the module yet. 
	*/
	eUpdateStatus pendingUpdates;
}
RegisterSetting;

/** \brief A RegisterSet is a container of related RegisterSettings
*    \remarks For example,  a typical AWL receiver provides controls for FPGA registers, ADC registers and GPIO registers.
*             Each of these devices is represented in a RegisterSet.
*/
typedef boost::container::vector<RegisterSetting> RegisterSet;


/** \brief The AlgoParamType describles the type of Algorithm parameters, which are either  are either float or int values.
*/
typedef enum  {
	/**Parameter is an  integer*/
	eAlgoParamInt = 0,  
	/**Parameter is a floating point value*/
 	eAlgoParamFloat = 1
}
AlgoParamType;

/** \brief The AlgorithmParameter describes a parameter value for the internal AWL algorithms.
*/
typedef struct AlgorithmParameter 
{
	/** \brief String used to describe the parameter index. 
	 * \remarks Should be numeric - formatted for display purposes.
	*/
	std::string sIndex;

	/** \brief Identifier of the parameter for the algorithm. 
	 * \remarks Parameteres are proprietary and are not further described in documentation.
	*/
	uint16_t address;

	/** \brief Short description string opf the parameter. 
	 * \remarks Parameteres are proprietary and are not further described in documentation.
	*/
	std::string sDescription;

	/** \brief Indication of the storage type of the parameter. Can be  eAlgoParamInt or eAlgoParamFloat.
	*/
	AlgoParamType paramType;

	/** \brief Storage area for parameters of whose type is eAlgoParamInt.
	*/
	uint32_t intValue;

	/** \brief Storage area for parameters of whose type is eAlgoParamFloat.
	*/
	float floatValue;

	/** \brief Count of the messages demanding a change in the value, that have not been acknowledged by the 
	  *        AWL moldule. A non-zero count indicates that the value may not have been received and reflected in the module yet. 
	*/
	eUpdateStatus pendingUpdates;
}
AlgorithmParameter;

/** \brief An AlgorithmParameterVector is a container of related AlgorithmParameters
*/
typedef boost::container::vector<AlgorithmParameter> AlgorithmParameterVector;

/** \brief An AlgorithmDescription describes a detection Algorithm and all of its controllable parameters.
*/
typedef struct AlgorithmDescription
{
	/** \brief Algorithm Name descriptive string. Used for display and documentation purposes only.
	*/
	std::string sAlgoName;

	/** \brief Internal identifier of the algorithm.
	*/
	uint16_t	algoID;

	/** \brief Description of all controllable parameters for that algorithm.
	*/
	AlgorithmParameterVector parameters;
}
AlgorithmDescription;

/** \brief An AlgorithmSet is a container describing all the available detection algorithm and their associated parameters.
*    \remarks For experimental purposes, the AWL modules may host different detection algorithms, which can be selected through
*             the communications protocol.
*             The performance of each algorithm can be "tweaked" by altering the operation paramneters.
*             Beware, as units may "store" the parameter changes, which will become the default operating parameters
*             at boot time.  Those changes may not be reflected in the configuration files.
*/
typedef struct AlgorithmSet
{
	/** \brief Default displayed algo in the user interface.
	*/
	int defaultAlgo;

	/** \brief Container of all the Algorithm descriptions.
	*/
	boost::container::vector<AlgorithmDescription> algorithms;
}
AlgorithmSet;


/** \brief ReceiverCapture class is an abstract class for all classes used to acquire data from physical LIDAR units.
  *        The ReceiverCapture acquires LIDAR sensor data in SensorFrames.
  *        It buffers up a few frames to facilitate processing afterwards.
  *        The ReceiverCapture also manages a "local" copy of the status indicators of the LIDAR units.
  *        The ReceiverCapture also stores a "local"  copy of all Registers and Algorithm parameters of the LIDAR unit
  *        that can be controlled via its communications protocols.
  *        Finally, the ReceiverCapture manages optional "logging" of the track and distance data into a local log file on the PC
  * 
  *        Children classes will implement actual physical units based on their specific communications protocols and
  *        software extensions.
  *
  * \author Jean-Yves Desch�nes
  */
class ReceiverCapture: public ThreadedWorker, public Publisher
{
// Public types and constants
public:
	
	/** \brief Maximum number of frames kept in frame buffer*/
	static const int maximumSensorFrames;  

	typedef boost::shared_ptr<ReceiverCapture> Ptr;
    typedef boost::shared_ptr<ReceiverCapture> ConstPtr;
	typedef boost::container::vector<ReceiverCapture::Ptr> List;
	typedef ReceiverCapture::List *ListPtr;

	// public Methods
public:
	/** \brief ReceiverCapture constructor from user supplied values.
 	    * \param[in] inReceiverID  unique receiverID
	    * \param[in] inReceiverVoxelQty number of channels in the receiver
	    * \param[in] inReceiverColumns number of columns in receiver array
		* \param[in] inReceiverRows number of rows  in the receiver array
		* \param[in] inLineWrapAround "distance" coded between rows in the original communications protocol for arrayed sensors
		* \param[in] inDemandedFrameRate demanded frameRate for the receiver
	    * \param[in] inVoxelMask  voxelMask indicating which channels are activated in the receiver
	    * \param[in] inMessageMask mask of the messages that are enabled in the communications protocol
	    * \param[in] inRangeOffset rangeOffset that corresponds to a calibration error in the sensor.
		*                          Will automatically be added to any range received.
		* \param[in] inRegistersFPGA default description of the FPGA registers
		* \param[in] inRegistersADC default description of the ADC registers
		* \param[in] inRegistersGPIO default description of the GPIO registers
        * \param[in] inParametersAlgos default description if the algorithm parameters
		* \param[in] inParametersTrackers default description of the Tracker parameters
		*/
	ReceiverCapture(int receiverID, int inReceiverVoxelQty, int inReceiverColumns, int inReceiverRows,  float inLineWrapAround,
						ReceiverFrameRate inDemandedFrameRate, VoxelMask &inVoxelMask, MessageMask &inMessageMask, float inRangeOffset,
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, 
					   const AlgorithmSet &inParametersAlgos,
					   const AlgorithmSet &inParametersTrackers);

	/** \brief ReceiverCapture constructor from a configuration file information.
 	    * \param[in] inReceiverID  unique receiverID
	    * \param[in] propTree propertyTree that contains teh confoguration file information.
      */
	ReceiverCapture(int receiverID, boost::property_tree::ptree &propTree);

	/** \brief ReceiverCapture Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~ReceiverCapture();

	/** \brief Start the lidar Data Projection  thread
      */
	virtual void  Go(); 

	/** \brief Return the receiver ID.
      * \return receiverID used for other system structures.
      */
	int  GetReceiverID() {return receiverID;};

	/** \brief Return the number of frames acquired
      * \return int indicating the number of frames.
      */
	int  GetFrameQty();

	/** \Brief Get the device serial number
	 *   Value of 0 indicates that the data is not available.
	 */
	virtual uint32_t GetProductID() = 0;

	/** \Brief send a message to get the device serial number
	 *   Value of 0 indicates that the data is not available.
	 */
	virtual uint32_t GetUniqueID() = 0;

	/** \brief Return the frame rate requested by the application to the sensor
	  * \return frame rate requested by the application to the sensor. Corresponds to configuration file settings.
	  */
	ReceiverFrameRate  GetDemandedFrameRate() { return receiverStatus.demandedFrameRate; };

	/** \brief Return the frame rate returned by the sensor to the application
	  * \return frame rate returned by the sensor to the application. Corresponds to sensor settings.
	  */
	ReceiverFrameRate  GetObtainedFrameRate() { return receiverStatus.obtainedFrameRate; };

	/** \brief Return the frame rate computed by the application based on number of frames processed per second.
	  * \return frame rate computed by the application based on number of frames processed per second.
	  */
	ReceiverFrameRate  GetCalculatedFrameRate();

	/** \brief Return true if connexion with the device has been etablished.
      * \return True if device connexion is established.
	  */
  virtual bool IsConnected() { return true; }

	/** \brief Return the number of receiver voxels used for video projection
      * \return int indicating the number of channels.
      */
	virtual int GetVoxelQty() {return receiverVoxelQty;};


	/** \brief copy the frame identified by frameID to to a local copy (thread-safe)
     * \param[in] inFrameID frame identificator of the requiested frame
	   \param[out] outSensorFrame ChannelFram structure to which the data is copied.
	   \param[in] inSubscriberID subscriber info used to manage the update information and thread locking.
     * \return True if channel data is copied successfully. False if frame corresponding to inFrameID or channel data not found
     */
	//bool ReceiverCapture::CopyReceiverFrame(FrameID inFrameID, SensorFrame::Ptr &outSensorFrame, Publisher::SubscriberID inSubscriberID);
	// Linux
	bool CopyReceiverFrame(FrameID inFrameID, SensorFrame::Ptr &outSensorFrame, Publisher::SubscriberID inSubscriberID);

	/** \brief copy the raw detection data identified with a frameID to to a local copy (thread-safe)
     * \param[in] inFrameID frame identificator of the requiested frame
	   \param[out] outChannelFrame ChannelFram structure to which the data is copied.
	   \param[in] inSubscriberID subscriber info used to manage the update information and thread locking.
     * \return True if channel data is copied successfully. False if frame corresponding to inFrameID or channel data not found
     */
	virtual bool CopyReceiverRawDetections(FrameID inFrameID,  Detection::Vector &outDetections, Publisher::SubscriberID inSubscriberID);

	/** \brief copy the A-SCan waveform data identified with a frameID to to a local copy (thread-safe)
	 * \param[in] inFrameID frame identificator of the requiested frame
	   \param[out] outAScans AScan:::Vector to which the A-Scans are copied.
	   \param[in] inSubscriberID subscriber info used to manage the update information and thread locking.
	 * \return True if A-Scan data is copied successfully. False if frame corresponding to inFrameID or A-Scan data not found
	 */
	virtual bool CopyReceiverAScans(FrameID inFrameID,  AScan::Vector &outAScans, Publisher::SubscriberID inSubscriberID);

	/** \brief copy the voxel status informationidentified with a frameID to to a local copy (thread-safe)
	   \param[out] outStatus ReceiverStatus structure to which the data is copied.
     * \return True if status data is copied successfully. 
     */
	virtual bool CopyReceiverStatusData(ReceiverStatus &outStatus);

	/** \brief Return the current frame identification number for informational purposes 
     * \return Current frame identification number.
	    \remark Note that the frameID corresponds to the "incomplete" frame currently being assembled.
		        For the last complete frame, useGetLastFrameID();
     */
	virtual FrameID GetFrameID() { return(frameID); };

	/** \brief Return the  frame identification number for the frame located at inFrameIndex 
      * \param[in] inFrameIndex index of the requiested frame
    * \return Current frame identification number.
	    \remark Note that the frameID corresponds to the "incomplete" frame currently being assembled.
		        For the last complete frame, useGetLastFrameID();
     */
	virtual FrameID GetFrameID(uint16_t inFrameIndex);

	/** \brief Return theframe identification number of the last complete frame assembled 
     * \return Last complete frame identification number.
	    */
	virtual FrameID GetLastFrameID() {return(acquisitionSequence->GetLastFrameID());};

	/** \brief Return the time elapsed, in milliseconds, since the start of thread.
	    \return time in milliseconds since tthe start of thread.
     */
	Timestamp GetElapsed();

	/** \brief Add an offet to the distance measurements.  This is different from the sensor depth, which is distance from bumper.
      * \param[in] inMeasurementOffset measurementOffset introduced by detection algorithm
      * \remark measurement offset is an offset in distance from sensor caused by the nature of algorithm used.
      */
	void SetMeasurementOffset(float inMeasurementOffset);

	/** \brief Get the measurement offset  in meters.
      * \param[out] outMeasurementOffset measurementOffset introduced by detection algorithm.
      */
	void GetMeasurementOffset(float &outMeasurementOffset);

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
	virtual bool StartPlayback(ReceiverFrameRate /*frameRate*/, VoxelMask /*voxelMask*/);

	/** \brief Starts the record of a file whose name was set using the last SetRecordFileName() call. 
      * \param[in] frameRate recording frame rate. Ignored on some implementations of AWL (in this case, default frame rate is used).
      * \param[in] voxelMask mask for the recorded voxels. that an empty voxelMask is equivalent to StopRecord().
      * \return true if success.  false on error
	  * \remarks status of record is updated in the receiverStatus member.
	  * \remarks File is recorded locally on SD Card.
     */
	virtual bool StartRecord(ReceiverFrameRate /*frameRate*/, VoxelMask /*voxelMask*/);

	/** \brief Stops any current playback of a file. 
      * \return true if success.  false on error
 	  * \remarks status of playback is updated in the receiverStatus member.
  	  * \remarks After playback is interrupted, the unit should return to the default acquisition mode.
    */
	virtual bool StopPlayback();

	/** \brief Starts the internal calibration of the system. 
      * \param[in] frameQty number of frames on which calibration is calculated
      * \param[in] beta beta parameter for the calibration
      * \param[in] voxelMask mask for the recorded voxels. that an empty voxelMask is equivalent to StopRecord().
      * \return true if success.  false on error
	  * \remarks Calibration file is recorded locally on SD Card.
     */
	virtual bool StartCalibration(uint8_t frameQty, float beta, VoxelMask voxelMask) = 0;


	/** \brief Stops any current recording. 
      * \return true if success.  false on error
  	  * \remarks status of recording is updated in the receiverStatus member.
 	  * \remarks After recording, the unit should return to the default acquisition mode.
    */
	virtual bool StopRecord();

	/** \brief Starts the logging of distance data in a local log file. 
      * \return true if success.  false on error
     */
	virtual bool BeginDistanceLog();

	/** \brief Ends the logging of distance data in a local log file. 
      * \return true if success.  false on error
     */
	virtual bool EndDistanceLog();


	/** \brief Issues the command to set the current algorithm in the sensor.
	  *\param[in] algorigthmID  ID of the selected algorithm.
	* \return true if success.  false on error.
	*/
	virtual bool SetAlgorithm(uint16_t algorithmID) = 0;

	/** \brief Issues the command to set the current tracker in the sensor.
	*\param[in] trackerID  ID of the selected tracker.
	* \return true if success.  false on error.
	*/
	virtual bool SetTracker(uint16_t trackerID) = 0;

	/** \brief Sets an FPGA register to the value sent as argument.
	  *\param[in] registerAddress Adrress of the register to change.
	  *\param[in] registerValue Value to put into register.
	* \return true if success.  false on error.
	*/
	virtual bool SetFPGARegister(uint16_t registerAddress, uint32_t registerValue) = 0;

	/** \brief Sets an ADC register to the value sent as argument. 
	  *\param[in] registerAddress Adrress of the register to change.
	  *\param[in] registerValue Value to put into register.
	* \return true if success.  false on error.
	*/
	virtual bool SetADCRegister(uint16_t registerAddress, uint32_t registerValue) = 0;

	/** \brief Sets an internal GPIO register to the value sent as argument. 
	  *\param[in] registerAddress Adrress of the register to change.
	  *\param[in] registerValue Value to put into register (values accepted are 0-1).
	* \return true if success.  false on error.
	*/
	virtual bool SetGPIORegister(uint16_t registerAddress, uint32_t registerValue) = 0;

	/** \brief Sets algorithm parameters to the value sent as argument. 
	  *\param[in] algoID ID of the detection algo affected by the change.
	  *\param[in] registerAddress Adrress of the parameter to change.
	  *\param[in] registerValue Value to put into register (values accepted are 0-1).
	* \return true if success.  false on error.
	*/
	virtual bool SetAlgoParameter(int algoID, uint16_t registerAddress, uint32_t registerValue) = 0;

	/** \brief Sets global  algorithm parameters to the value sent as argument. 
	  *\param[in] registerAddress Adrress of the parameter to change.
	  *\param[in] registerValue Value to put into register.
	* \return true if success.  false on error.
	*/
	virtual bool SetGlobalAlgoParameter(uint16_t registerAddress, uint32_t registerValue) = 0;


	/** \brief Sets Tracker parameters to the value sent as argument.
	*\param[in] tracker ID of the Tracker affected by the change.
	*\param[in] registerAddress Adrress of the parameter to change.
	*\param[in] registerValue Value to put into register .
	* \return true if success.  false on error.
	*/
	virtual bool SetTrackerParameter(int trackerID, uint16_t registerAddress, uint32_t registerValue) = 0;

	/** \brief Changes the controls of which messages are sent from AWL to the client to reflect provided settings
    * \param[in] frameRate new frame rate for the system. A value of 0 means no change
    * \param[in] voxelMask mask for the analyzed voxels.
    * \param[in] messageMask mask identifies which groups of target/distance/intensity messages are transmitted over CAN.
	* \return true if success.  false on error.
	*/
	virtual bool SetMessageFilters(ReceiverFrameRate /*frameRate*/, VoxelMask /*voxelMask*/, MessageMask /*messageMask*/) = 0;



	
	/** \brief  an asynchronous query command to get the current tracker.
	* \return true if success.  false on error.
	*/
	virtual bool QueryTracker() = 0;

	/** \brief Send an asynchronous query command for an internal FPGA register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the FPGA registerSet. 
	*/

	virtual bool QueryFPGARegister(uint16_t registerAddress) = 0;

	/** \brief Send an asynchronous query command for an ADC register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the ADC registerSet. 
		*/
	virtual bool QueryADCRegister(uint16_t registerAddress) = 0;

	/** \brief Send an asynchronous query command for a GPIO register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the GPIO registerSet. 
		*/
	virtual bool QueryGPIORegister(uint16_t registerAddress) = 0;

	/** \brief Send an asynchronous query command for an algorithm parameter. 
		  *\param[in] algoID ID of the detection algo for which we want to query.
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the algorithmParameters registerSet. 
		*/
	virtual bool QueryAlgoParameter(int algoID, uint16_t registerAddress) = 0;

		/** \brief Send an asynchronous query command for a global algorithm parameter. 
		  *\param[in] algoID ID of the detection algo for which we want to query.
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the algorithmParameters registerSet. 
		*/
	virtual bool QueryGlobalAlgoParameter(uint16_t registerAddress) = 0;

	/** \brief Send an asynchronous query command for a tracker parameter.
	*\param[in] trackerID ID of the tracker algo for which we want to query.
	*\param[in] registerAddress Adrress of the register to query.
	* \return true if success.  false on error.
	* \remarks On reception of the answer to query the register address and value will be
	*          placed in the a trackerParameters registerSet.
	*/
	virtual bool QueryTrackerParameter(int trackerID, uint16_t registerAddress) = 0;

	/** \brief Returns pointer to the Algorithm Description for the Algorithm parameter set that
	has the  algoID
	* \param[in] algorithmSet in which we want to find the specified algorith,
	* \param[in] algoID an algorithm for which we want the parameter description.
	* \return pointer to the found AlgorithmDescription in the algo Set. NULL if no algorithm matches that algoID.
	*/
	AlgorithmDescription * FindAlgoDescriptionByID(AlgorithmSet &inAlgoSet, int inAlgoID);

	/** \brief Returns the CellID, given a channelID 
	* \param[in] channelID the input "channel"
	* \return ChannelID, indicating unique pixel position in the detector array
	* \remarks Simple receptors will ahave a one to one relationship between channelID and CellID 
	*          (in that case channelID is equivalent to the voxelIndex).
	*          Some receivers have out of order cell adressing. cellID may not correspond to teh physical position of the pixel.
	*          Basic ReceiverCapture assumes  linear relationship matrix using rowQty and columnQty.
	*/
	virtual CellID GetCellIDFromChannel(int inChannelID);

	/** \brief Returns the CellID, given a channelID
	* \param[in] channelID the input "channel"
	* \return ChannelID, indicating unique pixel position in the detector array
	* \remarks Simple receptors qill ahave a one to one relationship between channelID and CellID.
	*          (in that case channelID is equivalent to the voxelIndex).
	*          Some receivers have out of order cell adressing. cellID may not correspond to teh physical position of the pixel.
	*          Basic ReceiverCapture assumes  linear relationship matrix using rowQty and columnQty.
	*/
	virtual int GetChannelIDFromCell(CellID inCellID);


	// public variables
public:

	/** \brief Unique receiver ID. Corresponds to the index of receiver in receiverArray
	*/
	int receiverID;

	/** \brief String indentifying the unique receiver type.
	*/
	std::string sReceiverType;

	/** \brief String indentifying the receiver parameter set (dependent ont firmware version).
	*/
	std::string sReceiverRegisterSet;

	/** \brief String indentifying the receiver optical Configuration.
	*/
	std::string sReceiverVoxelGeometry;

	/** \brief Number of receiver voxels on the sensor
      */
	int receiverVoxelQty;

	/** \brief Number of rows in receiver array
	*/
	int receiverRowQty;

	/** \brief Number of columns in receiver array
	*/
	int receiverColumnQty;

	/** \brief line WrapAround is used as distance between rows in communications protocol
	    \remark A negative value means that line wrap around is not used.
	*/
	float lineWrapAround;

	/** \brief Current frame ID being built.
	    \remark Note that the frameID corresponds to the "incomplete" frame currently being assembled.
		        For the last complete frame, use acquisitionSequence->GetLastFrameID();
      */
	volatile uint32_t frameID;

	/** \brief Structure holding the frame data accumulation */
	AcquisitionSequence::Ptr acquisitionSequence;
		
	/** \brief Receiver status information 
      */
	ReceiverStatus	receiverStatus;

	/** \brief FPGA Registers description 
	 *  \remarks The FPGA register set should be initialized with a representation of the FPGA registers of the receiver model used.
	 *           Usually, these are extracted from a config file.  If the register set is not initialized, some of the communication
	 *           functions related to FPGA parameters will not transmit or interpret FPGA specific messages from the Receiver.
	 *           However, they should not fail and can be called safely.
      */
	RegisterSet registersFPGA;
	std::string registersFPGALabel;

	/** \brief ADC Registers description
	 *  \remarks The ADC register set should be initialized with a representation of the ADC registers of the receiver model used.
	 *           Usually, these are extracted from a config file.  If the register set is not initialized, some of the communication
	 *           functions related to ADC parameters will not transmit or interpret ADC specific messages from the Receiver.
	 *           However, they should not fail and can be called safely.
      */	
	RegisterSet registersADC;
	std::string registersADCLabel;

	/** \brief GPIO Registers description 
	 *  \remarks The GPIO register set should be initialized with a representation of the GPIO registers of the receiver model used.
	 *           Usually, these are extracted from a config file.  If the register set is not initialized, some of the communication
	 *           functions related to GPIO parameters will not transmit or interpret GPIO specific messages from the Receiver.
	 *           However, they should not fail and can be called safely.
      */
	RegisterSet registersGPIO;
	
	/** \brief Algorithm parameters  description
	 *  \remarks Algorithms index start at 1. Algorithm 0 (GLOBAL_PARAMETERS_ID) is global parameters.
	 *  \remarks The Algorithm set should be initialized with a representation of the algorithms parameters of the receiver model used.
	 *           Usually, these are extracted from a config file.  If the algorithm set is not initialized, some of the communication
	 *           functions related to algorithm parameters will not transmit or interpret algo messages from the Receiver.
	 *           However, they should not fail and can be called safely.
	 */
	AlgorithmSet parametersAlgos;

	/** \brief Tracking parameters  description
	*  \remarks Tracking parameters index start at 0. 
	*  \remarks The Tracking set should be initialized with a representation of the tracking parameters of the receiver model used.
	*           Usually, these are extracted from a config file.  If the tracking set is not initialized, some of the communication
	*           functions related to tracking parameters will not transmit or interpret tracking messages from the Receiver.
	*           However, they should not fail and can be called safely.
	*/
	AlgorithmSet parametersTrackers;

	/** \brief Reference distance used when logging data.
	*/
	float targetHintDistance;

	/** \brief Reference angle used when logging data.
	*/
	float targetHintAngle;

// Protected methods
protected:
	/** \brief Return the lidar data rendering thread status
      * \return true if the lidar data rendering thread is stoppped.
      */
	virtual void  DoThreadLoop();
	
	
	/** \brief Mark the  current frame as invalid after a parse error of the CAN contents.
	           This will prevent the frame from being stored and processed.
      */
	void InvalidateFrame();

	/** \brief Once all distances have been acquired in the current frame,
	  *        push that frame into the frame buffer.
	  *         Make sure the frameBuffer does not exceed the maximum number of frames
	  * 		by removing the oldest frames, if necessary.
	  *         Then, create a new "current frame" for processing.
	  *         This should be canned only once, when all frame messages are received.
	  *         Currently, it is invoked on reception of message 36 (las distance from last channel)
      */
	virtual void ProcessCompletedFrame();

	/** \brief Timestamp all the Tracks of the sourceFrame with the timeStamp of the sourceFrame      */
	void TimestampTracks(SensorFrame::Ptr sourceFrame);

	/** \brief Timestamp all the detections of the sourceFrame with the timeStamp of the sourceFrameFrame      */
	void TimestampDetections(SensorFrame::Ptr sourceFrame);

	/** \brief Write all the tracks in the sourceFrameframe to the log file
	  * \param[in] sourceFrame  frame that contains the tracks to log
      */
	virtual void LogTracks(SensorFrame::Ptr sourceFrame);

	/** \brief Write all the distances in the sourceFrame to the log file
	  * \param[in] sourceFrame  frame that contains the tracks to log
      */
	virtual void LogDistances(SensorFrame::Ptr sourceFrame);

	/** \brief Do one iteration of the thread loop.
      */
	virtual void DoOneThreadIteration();

	/** \brief Initialize status variables.
      */
	virtual void InitStatus();

	/** \brief Return the index of the RegisterSetting for the object that
	           has the address specified.
    * \param[in] inRegisterSet the registerSet that we want to search into
	* \param[in] inAddress the register address that we are searching for.
	* \return "index" of the found object in the list (this is NOT the sIndex field). -1 if no registers match that address.

      */
	int FindRegisterByAddress(const RegisterSet &inRegisterSet, uint16_t inAddress);

	/** \brief Set all registers in the registerSet to 0, and pendingUpdates status to updateStatusUpToDate;
	  * \param[in] inRegisterSet the registerSet that we want to clear into
	*/

	void ClearRegisterSet(RegisterSet& inRegisterSet);

	/** \brief Clear All RegisterSets in the receiver, by calling ClearRegisterSet for each of them.
	*		   RegisterSets should be cleared when a unit is disconnected.	
	* \param[in] inRegisterSet the registerSet that we want to clear into
	*/
	void ClearAllRegisters();

	/** \brief Returns pointer to the Algorithm parameter for the parameter that
	           has the address specified
	  * \param[in] algoID an algorithm for which we want the parameter description.
	 * \param[in] inAddress the parameter address
	* \return pointer to the found parameter in the list. NULL if no parameters match that address.

      */
	AlgorithmParameter *FindAlgoParamByAddress(int inAlgoID, uint16_t inAddress);

	/** \brief Returns pointer to the Tracker parameter for the parameter that
	has the address specified
	* \param[in] trackerID a tracker for which we want the parameter description.
	* \param[in] inAddress the parameter address
	* \return pointer to the found parameter in the list. NULL if no parameters match that address.

	*/
	AlgorithmParameter *FindTrackerParamByAddress(int inTrackerID, uint16_t inAddress);

	/** \brief Reads the configuration proerties from the configuration file
	  * \param[in] propTree the boost propertyTree created from reading the configuration file.
	  * \returns Returns true otherwise.
	  * \throws  Throws boost error on read of the property keys.
      */
	virtual bool ReadConfigFromPropTree( boost::property_tree::ptree &propTree);

	/** \brief Reads the optical Geometry proerties from the configuration file
	* \param[in] propTree the boost propertyTree created from reading the configuration file.
	* \returns Returns true otherwise.
	* \throws  Throws boost error on read of the Geometry property keys.
	*/
	virtual bool ReadGeometryFromPropTree(boost::property_tree::ptree &propTree);


	/** \brief Reads the description of registers (FPGA, ADC and GPIO) and controllable algorithm parameters 
	  *        from the configuration file.
	  * \param[in] propTree the boost propertyTree created from reading the configuration file.
	  * \returns Returns false in case of a read of the property tree, when all the register description is absent or not found.  Returns true otherwise.
	  * \throws  Throws boost error on read of the property keys orther than the root key.
      */
	virtual bool ReadRegistersFromPropTree( boost::property_tree::ptree & /*propTree*/);

	/** \Brief send a message to get the device serial number
     *  Message is asynchonous.  Result will not be available immediately.
    */
	virtual bool QueryUniqueID() = 0;

	/** \Brief send a messahge to get the device type
	  *  Message is asynchonous.  Result will not be available immediately.
	*/
	virtual bool QueryProductID() = 0;

// Protected variables
protected:

	/** \brief Marker indicating that current frame as invalid after a parse error of the CAN contents.
	           This will prevent the frame from being stored and processed.
			   Reset after call of ProcessCompletedFrame();
     */
	bool bFrameInvalidated;

	/** \brief Time at the start of thread
	 * \remark Initialized on object creation and evertytime the Go() method is called.
	*/
	boost::posix_time::ptime startTime;

	/** \brief  measurement offset from sensor, introduced by detection algorithm */
	float  measurementOffset;

	/** \brief  Pointer to the current frame information during frame acquisition */
	SensorFrame::Ptr currentFrame;

	/** \brief  debug file. */
	std::ofstream debugFile;

	/** \brief  Log file. */
<<<<<<< HEAD:src/AWLQtDemo/AWLCoreClasses/ReceiverCapture.h
	ofstream *logFilePtr;
	boost::mutex logFileMutex;
=======
	std::ofstream *logFilePtr;
>>>>>>> 8032a407dc2ccbc58f53138121c5585d46d33401:src/SensorCoreClasses/ReceiverCapture.h

	/** \brief  Mutex t insure thread-safe wi=riting to the log file. */
	boost::mutex logFileMutex;


  protected:
	  /** \brief  Frame rate as calculated from processed incoming messages. */
	  ReceiverFrameRate m_calculatedFrameRate;
	  /** \brief  Frames processed used in internal frame rate calculation. */
	  int m_nbrCompletedFrame;
	  /** \brief  Time stamp used in internal frame rate calculation. */
	  Timestamp m_frameRateMS;
};


SENSORCORE_END_NAMESPACE

#endif // SENSORCORE_RECEIVER_CAPTURE
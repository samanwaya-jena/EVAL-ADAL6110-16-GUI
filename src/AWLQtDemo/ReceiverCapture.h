#ifndef AWL_RECEIVER_CAPTURE_H
#define AWL_RECEIVER_CAPTURE_H

#include <stdint.h>
#include <fstream>

#ifndef Q_MOC_RUN
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/container/vector.hpp>
#endif

#include "Publisher.h"
#include "ThreadedWorker.h"
#include "Tracker.h"
#include "AWLSettings.h"

using namespace std;
const int maxReceiverFrames = 100;

namespace awl
{

/** \brief ChannelMask struct describes receiverchannel bit mask used in most data structures
  *        and communications
  * \author Jean-Yves Deschênes
  */

typedef union 
{
	uint8_t byteData;
	struct  {
		bool channel0	: 1;
		bool channel1	: 1;
		bool channel2	: 1;
		bool channel3	: 1;
		bool channel4	: 1;
		bool channel5	: 1;
		bool channel6	: 1;
		bool unused		: 1;
	} bitFieldData;

} ChannelMask;

 /** \brief MessageMask struct describes receiver message groups that can be toggled on/off 
  *        for customized operations or to preserve bandwidth  
  *        and communications
  * \author Jean-Yves Deschênes
  */

typedef union 
{
	uint8_t byteData;
	struct  {
		bool unused_0		: 1;
		bool obstacle		: 1;
		bool distance_1_4	: 1;
		bool distance_5_8	: 1;
		bool intensity_1_4	: 1;
		bool intensity_5_8	: 1;
		bool unused_6		: 1;
		bool unused_7		: 1;
	} bitFieldData;
} MessageMask;

/** \brief ReceiverStatus struct describes receiver operation status  
  * \author Jean-Yves Deschênes
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

	/** \brief Current frame rate. May not be reported accurately during playback.
      */
	int frameRate;

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

	/** \brief Receiver channel error flags.  Indicates an error condition on a specific detector.
      */

	union 
	{
		uint8_t byteData;
		struct  {
			bool channel0	: 1;
			bool channel1	: 1;
			bool channel2	: 1;
			bool channel3	: 1;
			bool channel4	: 1;
			bool channel5	: 1;
			bool channel6	: 1;
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

	uint16_t fpgaRegisterAddressRead;
	uint32_t fpgaRegisterValueRead;
	uint16_t adcRegisterAddressRead;
	uint32_t adcRegisterValueRead;
	uint16_t gpioRegisterAddressRead;
	uint32_t gpioRegisterValueRead;

	uint16_t currentAlgo;
	uint16_t currentAlgoPendingUpdates;
	
	ChannelMask	channelMask;
	MessageMask	messageMask;
}
ReceiverStatus;



/** \brief ReceiverCapture class is an abstract class for all classes used to acquire data from the physical LIDAR unit.
  *        The receiver capture buffers up a few frames to facilitate processing afterwards.
  *        The receiver capture manages optional "logging" of the track and distance data into a local log file on the PC
  * \author Jean-Yves Deschênes
  */
class ReceiverCapture: public ThreadedWorker, public Publisher
{
// Public types and constants
public:
	
	static const int maximumSensorFrames;  // Maximum number of frames kept in frame buffer
	typedef boost::shared_ptr<ReceiverCapture> Ptr;
    typedef boost::shared_ptr<ReceiverCapture> ConstPtr;
	typedef boost::container::vector<ReceiverCapture::Ptr> List;
	typedef ReceiverCapture::List *ListPtr;

	typedef enum  
	{
		eInjectRamp = 0,
		eInjectNoisy = 1,
		eInjectSlowMove = 2,
		eInjectConstant = 3
	}
	InjectType;
	
	// public Methods
public:

	ReceiverCapture(int inReceiverID) { receiverID = inReceiverID;};

	/** \brief ReceiverCapture constructor.  Builds an empty sequence.
	    * \param[in] inSequenceID  unique sequence ID (for documentation)
	    * \param[in] inReceiverChannelQty index of the required channel
    */

	ReceiverCapture(int inReceiverID, int inSequenceID, int inReceiverChannelQty);

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

	/** \brief Return the number of receiver channels used for video projection
      * \return int indicating the number of channels.
      */
	virtual int GetChannelQty() {return receiverChannelQty;};

	/** \brief Return the minimum distance for detection
      * \return float indicating the minimum distance.
      */
	virtual float GetMinDistance() {return minDistance;};
#if 0
	/** \brief Return the maxiimum distance for detection
      * \return float indicating the maximimum distance.
      */
	virtual float GetMaxDistance() {return maxDistance;};
#endif
	/** \brief Return the maxiimum distance for detection
      * \param[in] channelIndex index of the channel for which we want the maximum distance
      * \return float indicating the maximimum distance.
      */
	virtual float GetMaxDistance(int channelIndex);

	/** \brief Return the minimum distance for detection
      * \param[in] inMinDistance the new minimum distance
      * \return float indicating the minimum distance.
      */
	virtual float SetMinDistance(float inMinDistance);
#if 0
	/** \brief Return the maxiimum distance for detection
      * \param[in] inMaxDistance the new maximum distance
      * \return float indicating the maximimum distance.
      */
	virtual float SetMaxDistance(float inMinDistance);
#endif
	/** \brief Set the maxiimum distance for detection
      * \param[in] channelIndex index of the channel for which we want the maximum distance
      * \param[in] inMaxDistance the new maximum distance
      * \return float indicating the maximimum distance.
	  * \note  This affects the AWLSettings::channelsConfig[channelIndex].maxDistance
      */
	virtual float SetMaxDistance(int channelIndex, float inMaxDistance);

	/** \brief copy the channel data identified with a frameID to to a local copy (thread-safe)
     * \param[in] inFrameID frame identificator of the requiested frame
     * \param[in] inChannelID index of the required channel
	   \param[out] outChannelFrame ChannelFram structure to which the data is copied.
	   \param[in] inSubscriberID subscriber info used to manage the update information and thread locking.
     * \return True if channel data is copied successfully. False if frame corresponding to inFrameID or channel data not found
     */
	virtual bool CopyReceiverChannelData(uint32_t inFrameID, int inChannelID, ChannelFrame::Ptr &outChannelFrame, Publisher::SubscriberID inSubscriberID);

	/** \brief copy the channel status informationidentified with a frameID to to a local copy (thread-safe)
     * \param[in] inFrameID frame identificator of the requiested frame
     * \param[in] inChannelID index of the required channel
	   \param[out] outChannelFrame ChannelFram structure to which the data is copied.
	   \param[in] inSubscriberID subscriber info used to manage the update information and thread locking.
     * \return True if channel data is copied successfully. False if frame corresponding to inFrameID or channel data not found
     */
	virtual bool CopyReceiverStatusData(ReceiverStatus &outStatus, Publisher::SubscriberID inSubscriberID);

	/** \brief Return the current frame identification number for informational purposes 
     * \return Current frame identification number.
	    \remark Note that the frameID corresponds to the "incomplete" frame currently being assembled.
		        For the last complete frame, useGetLastFrameID();
     */
	virtual uint32_t GetFrameID() { return(frameID); };

	/** \brief Return the  frame identification number for the frame located at inFrameIndex 
      * \param[in] inFrameIndex index of the requiested frame
    * \return Current frame identification number.
	    \remark Note that the frameID corresponds to the "incomplete" frame currently being assembled.
		        For the last complete frame, useGetLastFrameID();
     */
	virtual uint32_t GetFrameID(int inFrameIndex);

	/** \brief Return theframe identification number of the last complete frame assembled 
     * \return Last complete frame identification number.
	    */
	virtual uint32_t GetLastFrameID() {return(acquisitionSequence->GetLastFrameID());};

	/** \brief Sets the current frame as the frameID for all user interface snapshots
	  * \return Current frame identification number.
	    \return frameID of the common snapshotFrame.
		\remark This is provided as a user interface utility. As many threaded objects use the 
		        frame information for display asynchonously, this allows some degree of "pacing" to
				insure information is coherent along all users of the frame information.
     */
	virtual uint32_t SnapSnapshotFrameID();

	/** \brief Get the common frameID for user interface snapshots
	    \return frameID of the common snapshotFrame.
		\remark This is provided as a user interface utility. As many threaded objects use the 
		        frame information for display asynchonously, this allows some degree of "pacing" to
				insure information is coherent along all users of the frame information.
     */
	 virtual uint32_t GetSnapshotFrameID() { return (snapshotFrameID);};

	/** \brief Return the time elapsed, in milliseconds, since the start of thread.
	    \return time in milliseconds since tthe start of thread.
     */
	 double GetElapsed();

	/** \brief Add an offet to the distance measurements.  This is different from the sensor depth, which is distance from bumper.
      * \param[in] inMeasurementOffset measurementOffset introduced by detection algorithm
      * \remark measurement offset is an offset in distance from sensor caused by the nature of algorithm used.
      */
	void SetMeasurementOffset(double inMeasurementOffset);

	/** \brief Get the measurement offset  in meters.
      * \param[out] outMeasurementOffset measurementOffset introduced by detection algorithm.
      */
	void GetMeasurementOffset(double &outMeasurementOffset);

	/** \brief Enable or Disable simulation data injection.  (Injects a ramp).
      * \param[in] inSimulatedEnabled simulated data is injected if true. Injection is disabled if false.
      * \remark measurement offset is an offset in distance from sensor caused by the nature of algorithm used.
      */
	void EnableSimulationData(bool inSimulationEnabled) { bSimulatedDataEnabled = inSimulationEnabled;};

	/** \brief Get the status of the simulation data injection.
      * \return Boolean is true if simulated data injection is enable, false otherwise.
      */
	bool IsSimulatedDataEnabled() {return (bSimulatedDataEnabled);};

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
      * \param[in] channelMask mask for the channels that will be played back. that an empty channelMask is equivalent to StopPlayback().
      * \return true if success.  false on error
 	  * \remarks status of playback is updated in the receiverStatus member.
	  * \remarks File is recorded locally on SD Card.
     */
	virtual bool StartPlayback(uint8_t frameRate, ChannelMask channelMask);

	/** \brief Starts the record of a file whose name was set using the last SetRecordFileName() call. 
      * \param[in] frameRate recording frame rate. Ignored on some implementations of AWL (in this case, default frame rate is used).
      * \param[in] channelMask mask for the recorded channels. that an empty channelMask is equivalent to StopRecord().
      * \return true if success.  false on error
	  * \remarks status of record is updated in the receiverStatus member.
	  * \remarks File is recorded locally on SD Card.
     */
	virtual bool StartRecord(uint8_t frameRate, ChannelMask channelMask);

	/** \brief Stops any current playback of a file. 
      * \return true if success.  false on error
 	  * \remarks status of playback is updated in the receiverStatus member.
  	  * \remarks After playback is interrupted, the unit should return to the default acquisition mode.
    */
	virtual bool StopPlayback();

	/** \brief Starts the internal calibration of the system. 
      * \param[in] frameQty number of frames on which calibration is calculated
      * \param[in] beta beta parameter for the calibration
      * \param[in] channelMask mask for the recorded channels. that an empty channelMask is equivalent to StopRecord().
      * \return true if success.  false on error
	  * \remarks Calibration file is recorded locally on SD Card.
     */
	virtual bool StartCalibration(uint8_t frameQty, float beta, ChannelMask channelMask) = 0;


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


	/** \brief Issues the command to set the current algorithm in the FPGA.
	  *\param[in] algorigthmID  ID of the selected algorithm.
	* \return true if success.  false on error.
	*/
		
	virtual bool SetAlgorithm(uint16_t algorithmID) = 0;

	/** \brief Sets an internal FPGA register to the value sent as argument. 
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
	  *\param[in] registerValue Value to put into register (values accepted are 0-1).
	* \return true if success.  false on error.
	*/
		
	virtual bool SetGlobalAlgoParameter(uint16_t registerAddress, uint32_t registerValue) = 0;

	/** \brief Changes the controls of which messages are sent from AWL to the client to reflect the current global setting.
	* \return true if success.  false on error.
	* \remarks uses the AWLSettings::global settings variables to call  SetMessageFilters(frameRate, channelMask, messageMask)
	*/
		
	virtual bool SetMessageFilters();

	/** \brief Changes the controls of which messages are sent from AWL to the client to reflect provided settings
    * \param[in] frameRate new frame rate for the system. A value of 0 means no change
    * \param[in] channelMask mask for the analyzed channels.
    * \param[in] messageMask mask identifies which groups of target/distance/intensity messages are transmitted over CAN.
	* \return true if success.  false on error.
	*/
		
	virtual bool SetMessageFilters(uint8_t frameRate, ChannelMask channelMask, MessageMask messageMask);


	/** \  an asynchronous query command to get the current algorithm.
	* \return true if success.  false on error.
	*/
	virtual bool QueryAlgorithm() = 0;

	/** \brief Send an asynchronous query command for an internal FPGA register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the receiverStatus member and in globalSettings. 
		*/
	virtual bool QueryFPGARegister(uint16_t registerAddress) = 0;

	/** \brief Send an asynchronous query command for an ADC register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the receiverStatus member and in globalSettings. 
		*/
	virtual bool QueryADCRegister(uint16_t registerAddress) = 0;

	/** \brief Send an asynchronous query command for a GPIO register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the receiverStatus member and in the globalSettings. 
		*/
	virtual bool QueryGPIORegister(uint16_t registerAddress) = 0;

	/** \brief Send an asynchronous query command for an algorithm parameter. 
		  *\param[in] algoID ID of the detection algo for which we want to query.
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the receiverStatus member and in the globalSettings. 
		*/
	virtual bool QueryAlgoParameter(int algoID, uint16_t registerAddress) = 0;

		/** \brief Send an asynchronous query command for a global algorithm parameter. 
		  *\param[in] algoID ID of the detection algo for which we want to query.
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the receiverStatus member and in the globalSettings. 
		*/
	virtual bool QueryGlobalAlgoParameter(uint16_t registerAddress) = 0;

	// public variables
public:

	/** \brief Unique receiver ID. Corresponds to the index of receiver in receiverArray
	*/
	int receiverID;

	/** \brief Number of receiver channels on the sensor
      */
	int receiverChannelQty;

	/** \brief Current frame ID being built.
	    \remark Note that the frameID corresponds to the "incomplete" frame currently being assembled.
		        For the last complete frame, use acquisitionSequence->GetLastFrameID();
      */
	volatile uint32_t frameID;

	/** \brief Snapshot frame Index
      */
	volatile uint32_t snapshotFrameID;

	/** \brief Structure holding the frame data accumulation */
	AcquisitionSequence::Ptr acquisitionSequence;
		
	/** \brief Receiver status information 
      */
	ReceiverStatus	receiverStatus;

// Protected methods
protected:
	/** \brief Return the lidar data rendering thread status
      * \return true if the lidar data rendering thread is stoppped.
      */
	virtual void  DoThreadLoop();
	
	/** \brief Once all distances have been acquired in the current frame,
	  *        push that frame into the frame buffer.
	  *         Make sure the frameBuffer does not exceed the maximum number of frames
	  * 		by removing the oldest frames, if necessary.
	  *         Then, create a new "current frame" for processing.
	  *         This should be canned only once, when all frame messages are received.
	  *         Currently, it is invoked on reception of message 36 (las distance from last channel)
      */
	virtual void ProcessCompletedFrame();

	/** \brief Write all the tracks in the sourceFrameframe to the log file
 	  * \param[in] logFile		file to which the log is injected
	  * \param[in] sourceFrame  frame that contains the tracks to log
      */
	virtual void LogTracks(ofstream &logFile, SensorFrame::Ptr sourceFrame);

	/** \brief Write all the distances in the sourceFrame to the log file
 	  * \param[in] logFile		file to which the log is injected
	  * \param[in] sourceFrame  frame that contains the tracks to log
      */
	virtual void LogDistances(ofstream &logFile, SensorFrame::Ptr sourceFrame);

	/** \brief Inject distance information in the channel,  using ramp simulation
 	    * \param[in] channel   channel in whhich data is injected
      */
	void FakeChannelDistanceRamp(int channel);

	/** \brief Inject distance information in the channel,  using noisy data simulation
 	    * \param[in] channel   channel in whhich data is injected
      */
	void FakeChannelDistanceNoisy(int channel);

	/** \brief Inject distance information in the channel,  usingslow move simulation
 	    * \param[in] channel   channel in which data is injected
      */
	void FakeChannelDistanceSlowMove(int channel);

	/** \brief Inject  a constant distance information in the channel
 	    * \param[in] channel   channel in which data is injected
      */
	void FakeChannelDistanceConstant(int channel);

	/** \brief Inject distance and velocity information in the trackList, for the channel,
	   *        usingslow move simulation
 	    * \param[in] channel   channel for which data is injected
      */
	void FakeChannelTrackSlowMove(int channel);


	/** \brief Do one iteration of the thread loop.
      */
	virtual void DoOneThreadIteration();

	/** \brief Initialize status variables.
      */
	virtual void InitStatus();


// Protected variables
protected:
	/** \brief Minimum distance for objects.  All messages with distance < minDistance are eliminated.
		\default 3.0;
	*/

	float minDistance;

	/** \brief Maximum distance for objects for each channel .  
	           All messages with distance > maxDistance are eliminated.
		\default 50.0 meters
	*/

	boost::container::vector<float> maxDistances;

	/** \brief Time at the start of thread
	 * \remark Initialized on object creation and evertytime the Go() method is called.
	*/
	boost::posix_time::ptime startTime;

	/** \brief  measurement offset from sensor, introduced by detection algorithm */
	double  measurementOffset;

	/** \brief  controls the injection of simulated data.  Injection is enabled when true */
	bool  bSimulatedDataEnabled;

	/** \brief  scaling error (multiplication) introduced by clock speed variations in configurations, should be 1 by default */
	float  distanceScale;

	/** \brief  defines the type of data injected.  */
	InjectType injectType;

	/** \brief  controls demo features such as simulatedFeedback.  demo features are on when true */
	bool bEnableDemo;

	
	/** \brief  Pointer to the current frame information during frame acquisition */
	SensorFrame::Ptr currentFrame;

	/** \brief  Time tracker for the last frame used to calculate pacing delays in simulations. */
	double lastElapsed;

	/** \brief  debug file. */
	ofstream debugFile;

	/** \brief  Log file. */
	ofstream logFile;

	
	/** \brief  distance used in simulations - distance injection code */
	double lastDistance;
	TrackID trackIDGenerator;
	int lastTransition;
	int transitionDirection;
	int directionPacing; /* Every 3 seconds, we move from left to right */
	int nextElapsedDirection;

	float distanceIncrement;
	int distancePacing; /* 12 ms per move at 0.1m means we do 40m in 5 seconds */
	int nextElapsedDistance;
};


} // namespace AWL

#endif
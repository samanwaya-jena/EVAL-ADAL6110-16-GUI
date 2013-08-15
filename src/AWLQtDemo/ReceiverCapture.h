#ifndef AWL_RECEIVER_CAPTURE_H
#define AWL_RECEIVER_CAPTURE_H

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include <stdint.h>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#include "Subscription.h"
#include "Tracker.h"
using namespace std;
using namespace pcl;

const int maxReceiverFrames = 100;

namespace awl
{

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
}
ReceiverStatus;



/** \brief Threaded ReceiverCapture class is used to acquire data from the physical LIDAR unit.
  *        The receiver capture buffers up a few frames to faciulitae processing afterwards.
  * \author Jean-Yves Deschênes
  */
class ReceiverCapture
{
// Public types
public:
	typedef boost::shared_ptr<ReceiverCapture> Ptr;
    typedef boost::shared_ptr<ReceiverCapture> ConstPtr;

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

// public Methods
public:

	ReceiverCapture() {};

	/** \brief ReceiverCapture constructor.  Builds an empty sequence.
	    * \param[in] inSequenceID  unique sequence ID (for documentation)
	    * \param[in] inReceiverChannelQty index of the required channel
 	    * \param[in] inDetectionsPerChannel number of detections per channel
      */

	ReceiverCapture(int inSequenceID, int inReceiverChannelQty, int inDetectionsPerChannel);

	/** \brief ReceiverCapture constructor.  Builds a sequence from file .
	    * \param[in] inSequenceID  unique sequence ID (for documentation)
	    * \param[in] inReceiverChannelQty index of the required channel
 	    * \param[in] inDetectionsPerChannel number of detections per channel
		* \param[in] inFileName file with the sequence information
      */
	ReceiverCapture(int inSequenceID, int inReceiverChannelQty, int inDetectionsPerChannel, std::string inFileName);

	/** \brief ReceiverCapture Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~ReceiverCapture();

	/** \brief Process the command line arguments related to the capture.
	  *        Arguments are :  --minRange%f  the minimum detection range.
	  *                         --maxRange%f the maximumDetection range.
	  */
	virtual void ProcessCommandLineArguments(int argc, char** argv);


	/** \brief Start the lidar Data Projection  thread
      */
	virtual void  Go(bool inIsThreaded = false); 

	/** \brief Stop the lidar data projection thread
      */
	virtual void  Stop(); 

	/** \brief Return the video acquisition thread status
      * \return true if the video acquisition thread is stoppped.
      */
	virtual bool  WasStopped();

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

	/** \brief Return the maxiimum distance for detection
      * \return float indicating the maximimum distance.
      */
	virtual float GetMaxDistance() {return maxDistance;};

	/** \brief Return the number of receiver channels used for video projection
      * \return int indicating the number of channels.
      */
	virtual int GetDetectionQtyPerChannel() {return detectionsPerChannel;};

	/** \brief copy the current channel data to to a local copy (thread-safe)
     * \param[in] inChannelID index of the required channel
	   \param[out] outChannelFrame ChannelFram structure to which the data is copied.
	   \param[in] inSubscriberID subscriber info used to manage the update information and thread locking.
     * \return frameID of the current sensorFrame from which the data is taken.
     */
	virtual uint32_t CopyCurrentReceiverChannelData(int inChannelID, ChannelFrame::Ptr &outChannelFrame, Subscription::SubscriberID inSubscriberID);

	/** \brief copy the channel data identified with a frameID to to a local copy (thread-safe)
     * \param[in] inFrameID frame identificator of the requiested frame
     * \param[in] inChannelID index of the required channel
	   \param[out] outChannelFrame ChannelFram structure to which the data is copied.
	   \param[in] inSubscriberID subscriber info used to manage the update information and thread locking.
     * \return True if channel data is copied successfully. False if frame corresponding to inFrameID or channel data not found
     */
	virtual bool CopyReceiverChannelData(uint32_t inFrameID, int inChannelID, ChannelFrame::Ptr &outChannelFrame, Subscription::SubscriberID inSubscriberID);

	/** \brief Return a copy of the detection identified by its indexes
     * \param[in] inFrameIndex index of the requiested frame
     * \param[in] inChannelID index of the required channel
     * \param[in] inDetectionIndex index of the detection
	 * \param[out] outChannelFrame ChannelFrame structure to which the data is copied.
	 * \param[in] inSubscriberID subscriber info used to manage the update information and thread locking.
     * \return True if detection is copied successfully. False if frame corresponding to inFrameIndex or channel data not found
     */
	bool ReceiverCapture::GetDetection(uint32_t inFrameIndex, int inChannelID, int inDetectionIndex, Detection::Ptr &outDetection, Subscription::SubscriberID inSubscriberID);

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

	/** \brief Return the timeStamp of the specified frame 
      * \param[in] inFrameID frame identificator of the requiested frame
    * \return timeStamp (elapsed in milliseconds) of specified frame.  Negative value if frame not found
     */
	virtual double GetFrameTime(uint32_t inFrameID);

	/** \brief Return the timeStamp of the frame located at index.
      * \param[in] inFrameIndex index of the requiested frame
    * \return timeStamp (elapsed in milliseconds) of specified frame.  Negative value if frame not found
     */
	virtual double GetFrameTimeAtIndex(int inFrameIndex);

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
      * \param[in] inMeasurementOffset sensor depth, in meters (normally negative)
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
	virtual bool StartPlayback(uint8_t frameRate, ReceiverCapture::ChannelMask channelMask);

	/** \brief Starts the record of a file whose name was set using the last SetRecordFileName() call. 
      * \param[in] frameRate recording frame rate. Ignored on some implementations of AWL (in this case, default frame rate is used).
      * \param[in] channelMask mask for the recorded channels. that an empty channelMask is equivalent to StopRecord().
      * \return true if success.  false on error
	  * \remarks status of record is updated in the receiverStatus member.
	  * \remarks File is recorded locally on SD Card.
     */
	virtual bool StartRecord(uint8_t frameRate, ReceiverCapture::ChannelMask channelMask);

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
	virtual bool StartCalibration(uint8_t frameQty, float beta, ReceiverCapture::ChannelMask channelMask);


	/** \brief Stops any current recording. 
      * \return true if success.  false on error
  	  * \remarks status of recording is updated in the receiverStatus member.
 	  * \remarks After recording, the unit should return to the default acquisition mode.
    */
	virtual bool StopRecord();

	
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

	/** \brief Send an asynchronous query command for an internal FPGA register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the receiverStatus member. 
		*/
	virtual bool QueryFPGARegister(uint16_t registerAddress);

	/** \brief Send an asynchronous query command for an ADC register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks On reception of the answer to query the register address and value will be
	  *          placed in the receiverStatus member. 
		*/
	virtual bool QueryADCRegister(uint16_t registerAddress);

	// public variables
public:
	/** \brief Number of receiver channels on the sensor
      */
	int receiverChannelQty;

	/** \brief Maximum number of detections per channel per frame
      */
	int detectionsPerChannel;

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

	/** \brief A public subscription checkpoint infrastructure for the output 
	  * of the frame data.
      */
	Subscription::Ptr currentReceiverCaptureSubscriptions;	

		
	/** \brief Receiver status information 
      */
	ReceiverStatus	receiverStatus;


	/** \brief Do one iteration of the thread loop.
      */
	virtual void DoThreadIteration();



// Protected methods
protected:

	
	void FakeChannelDistance(int channel);

	/** \brief Return the lidar data rendering thread status
      * \return true if the lidar data rendering thread is stoppped.
      */
	void  DoThreadLoop();

	/** \brief Do one iteration of the thread loop.
      */
	virtual void DoOneThreadIteration();

	/** \brief Initialize status variables.
      */
	virtual void InitStatus();


// Protected variables
protected:
	protected:
	
    /** \brief Local flag indicating the termination of thread. */
	volatile bool mStopRequested;

    /** \brief Video acquisition thread . */
    boost::shared_ptr<boost::thread> mThread;
	
	/** \brief Data sharing mutex. */
    boost::mutex mMutex;

	/** \brief Indicates if the object is threaded, or if it is fed using the internal 
	  *        loop.
	  *        This is determined at run-time on the Go() call.
	*/
	bool bIsThreaded;

	/** \brief Minimum distance for objects.  All messages with distance < minDistance are eliminated.
		\default 3.0;
	*/

	float minDistance;

	/** \brief Maximum distance for objects.  All messages with distance > maxDistance are eliminated.
		\default 50.0
	*/

	float maxDistance;

	/** \brief Time at the start of thread
	 * \remark Initialized on object creation and evertytime the Go() method is called.
	*/
	boost::posix_time::ptime startTime;

	/** \brief  measurement offset from sensor, introduced by detection algorithm */
	double  measurementOffset;

	/** \brief  controls the injection of simulated data.  Injection is enabled when true */
	bool  bSimulatedDataEnabled;
};


} // namespace AWL

#endif
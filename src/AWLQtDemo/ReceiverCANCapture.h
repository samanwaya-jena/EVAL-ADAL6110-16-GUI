#ifndef AWL_RECEIVER_CAN_CAPTURE_H
#define AWL_RECEIVER_CAN_CAPTURE_H

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include <stdint.h>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>

#include <boost/asio.hpp> 
#include <boost/asio/serial_port.hpp> 
#endif

#include "BlockingReader.h"

#include "Subscription.h"
#include "Tracker.h"
#include "ReceiverCapture.h"
using namespace std;
using namespace pcl;

namespace awl
{

const int maximumSensorFrames = 100;
// CAN Frame
typedef struct {
    unsigned long id;        // Message id
    unsigned long timestamp; // timestamp in milliseconds	
    unsigned char flags;     // [extended_id|1][RTR:1][reserver:6]
    unsigned char len;       // Frame size (0.8)
    unsigned char data[8]; // Databytes 0..7
} AWLCANMessage;


/** \brief Threaded ReceiverCANCapture class is used to acquire data  from LIDA
   *		through CAN bus.
  *        The receiver file capture buffers up a few frames to faciulitae processing afterwards.
  * \author Jean-Yves Deschênes
  */
class ReceiverCANCapture: public ReceiverCapture
{
// Public types
public:
	typedef boost::shared_ptr<ReceiverCANCapture> Ptr;
    typedef boost::shared_ptr<ReceiverCANCapture> ConstPtr;

	typedef enum eCANRates
	{
		rate10Kbps = 0,
		rate20Kbps = 1,
		rate50Kbps = 2,
		rate100Kbps = 3,
		rate125Kbps = 4,
		rate250Kbps = 5,
		rate500Kbps = 6,
		rate800Kbps = 7,
		rate1000Kbps = 8
	};

// public Methods
public:

	/** \brief ReceiverCANCapture constructor.
 	    * \param[in] inSequenceID  unique sequence ID (for documentation)
	    * \param[in] inReceiverChannelQty index of the required channel
 	    * \param[in] inDetectionsPerChannel number of detections per channel
      */

	ReceiverCANCapture(int inSequenceID, int inReceiverChannelQty, int inDetectionsPerChannels, int argc, char** argv);

	/** \brief ReceiverCANCapture Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~ReceiverCANCapture();

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

	// public variables
public:

	/** \brief Do one iteration of the thread loop.
      */
	virtual void DoThreadIteration();

	/** \brief Parse the CAN messages and call the appropriate processing function 
 	    * \param[in] inMsg  CAN message contents
      */
	void ParseMessage(AWLCANMessage &inMsg);

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
	

	/** \brief Stops any current recording. 
      * \return true if success.  false on error
  	  * \remarks status of recording is updated in the receiverStatus member.
 	  * \remarks After recording, the unit should return to the default acquisition mode.
    */
	virtual bool StopRecord();

	/** \brief Starts the internal calibration of the system. 
      * \param[in] frameQty number of frames on which calibration is calculated
      * \param[in] beta beta parameter for the calibration
      * \param[in] channelMask mask for the recorded channels. that an empty channelMask is equivalent to StopRecord().
      * \return true if success.  false on error
	  * \remarks Calibration file is recorded locally on SD Card.
     */
	virtual bool StartCalibration(uint8_t frameQty, float beta, ReceiverCapture::ChannelMask channelMask);

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


// Protected methods
protected:

	/** \brief Process the command line arguments related to the CAN port.
	  *        Arguments are :  --bitRate%s  the bitrate command to the canPort, according to EasySync documents.
	  *                         ("S2" = 50Kbps, "S8" = 1Mbps)
	  *                         --comPort%s the com port identifier (default is "COM16");.
	  */
	virtual void ProcessCommandLineArguments(int argc, char** argv);

	/** \brief Return the lidar data rendering thread status
      * \return true if the lidar data rendering thread is stoppped.
      */
	void  DoThreadLoop();

	/** \brief Do one iteration of the thread loop.
      */
	virtual void DoOneThreadIteration();

	/** \brief Process one line of CAN data from serial port and interpret it into a CANmgs structure.
	  * \param[in] inResponse  The string that has to be decoded.
	  * \param[out] outMsg  The formatted CAN message.  Is invalid when false is returned.
	  * \return true if the inResponse line contains a structured CAN message, false otherwise.
      */
	bool ParseLine(std::string inResponse, AWLCANMessage &outMsg);

	/** \brief Read the sensor status message (001)
 	    * \param[in] inMsg  CAN message contents
      */
	void ParseSensorStatus(AWLCANMessage &inMsg);

	/** \brief Read the sensor boot status message (002)
 	    * \param[in] inMsg  CAN message contents
      */
	void ParseSensorBoot(AWLCANMessage &inMsg);

	/** \brief Read the distance readings from CAN messages (20-26 30-36)
 	    * \param[in] inMsg   CAN message contents
      */
	void ParseChannelDistance(AWLCANMessage &inMsg);

	
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

	/** \brief Read the intensity readings from CAN messages (40-46 50-56)
 	    * \param[in] inMsg   CAN message contents
     */
	void ParseChannelIntensity(AWLCANMessage &inMsg);

	/** \brief Once all distances have been acquired in the current frame,
	  *        push that frame into the frame buffer.
	  *         Make sure the frameBuffer does not exceed the maximum number of frames
	  * 		by removing the oldest frames, if necessary.
	  *         Then, create a new "current frame" for processing.
	  *         This should be canned only once, when all frame messages are received.
	  *         Currently, it is invoked on reception of message 36 (las distance from last channel)
      */
	void ProcessCompletedFrame();

	
	/** \brief Parse control messages (80) and dispatch to appropriate handling method 
 	    * \param[in] inMsg   CAN message contents
     */	
	void ParseControlMessage(AWLCANMessage &inMsg);

	/** \brief Process the debug / parameter / set_parameter message (0xC0)
 	    * \param[in] inMsg   CAN message contents
		* \ remarks	This message is normally a master message.  It is ignored. 
     */	
	void ParseParameterSet(AWLCANMessage &inMsg);

	/** \brief Process the debug / parameter / Query message (0xC1)
 	    * \param[in] inMsg   CAN message contents
		* \ remarks	This message is normally a master message.  It is ignored. 
     */	
	void ParseParameterQuery(AWLCANMessage &inMsg);

	/** \brief Process the debug / parameter / response message (0xC2)
 	    * \param[in] inMsg   CAN message contents
     */	
	void ParseParameterResponse(AWLCANMessage &inMsg);

	/** \brief Process the debug / parameter / error message (0xC3)
 	    * \param[in] inMsg   CAN message contents
		* \remarks  The error flag is set in the receiverStatus.
     */	
	void ParseParameterError(AWLCANMessage &inMsg);


	void ParseParameterAlgoSelectResponse(AWLCANMessage &inMsg);
	void ParseParameterAlgoParameterResponse(AWLCANMessage &inMsg);
	void ParseParameterFPGARegisterResponse(AWLCANMessage &inMsg);
	void ParseParameterBiasResponse(AWLCANMessage &inMsg);
	void ParseParameterADCRegisterResponse(AWLCANMessage &inMsg);
	void ParseParameterPresetResponse(AWLCANMessage &inMsg);
	void ParseParameterHistogramResponse(AWLCANMessage &inMsg);
	void ParseParameterDateTimeResponse(AWLCANMessage &inMsg);
	void ParseParameterRecordResponse(AWLCANMessage &inMsg);
	void ParseParameterPlaybackResponse(AWLCANMessage &inMsg);

	void ParseParameterAlgoSelectError(AWLCANMessage &inMsg);
	void ParseParameterAlgoParameterError(AWLCANMessage &inMsg);
	void ParseParameterFPGARegisterError(AWLCANMessage &inMsg);
	void ParseParameterBiasError(AWLCANMessage &inMsg);
	void ParseParameterADCRegisterError(AWLCANMessage &inMsg);
	void ParseParameterPresetError(AWLCANMessage &inMsg);
	void ParseParameterHistogramError(AWLCANMessage &inMsg);
	void ParseParameterDateTimeError(AWLCANMessage &inMsg);
	void ParseParameterRecordError(AWLCANMessage &inMsg);
	void ParseParameterPlaybackError(AWLCANMessage &inMsg);

	/** \brief Open the CAN port
	  * \returns true if the port is successfully opened, false otherwise.
	  * \remarks Once the port is successfully opened, use the "reader" pointer to access the can data.
	  *          If opening the port fails, reader is set to NULL.
	  */
	bool OpenCANPort();


	/** \brief Closes the CAN port and associated objects.
	  * \returns true if the port is successfully closed, false otherwise.
	  */
	bool CloseCANPort();

	/** \brief Synchronous write of a sting in the stream 
 	  * \param[in] inString  Message to send
      */
	void WriteString(std::string inString);

	/** \brief Synchronous write of a CAN message in the stream 
 	  * \param[in] outString  Message to send
	  * \return true iof the function was successful. false otherwise.
      */
	bool WriteMessage(const AWLCANMessage &inMsg);

	/** \brief Put the current date and time to the CAN port
 	  * \return true iof the function was successful. false otherwise.
     */
	bool WriteCurrentDateTime();

// Protected variables
protected:
		std::string	sBitRate;
		std::string sCommPort;
		long serialPortRate;
		uint16_t yearOffset;		   // All CAN Dates are offset from 1900
		uint16_t monthOffset;			// All CAN months start at 0.  Posix starts aty 1.

		SensorFrame::Ptr currentFrame;

		// This trick will be used to determine when we have a complete frame.
		// we assume that all messages are inorder and that a frame is complete on message 36
		unsigned long lastMessageID;

		boost::asio::io_service io;
		boost::asio::serial_port *port;
		std::string responseString;
		// A blocking reader for this port that 
		// will time out a read after 500 milliseconds.
		blocking_reader *reader; 

};

} // namespace AWL

#endif
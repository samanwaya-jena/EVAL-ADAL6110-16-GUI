#ifndef AWL_TRACKER_H
#define AWL_TRACKER_H


#include <stdint.h>
#include <iostream>
#include <fstream>
#include <string>
#include <queue>

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif


using namespace std;

namespace awl
{

#if 0
static const unsigned long __nan[2] = {0xffffffff, 0x7fffffff};
#define NAN (*(const float *) __nan)
#else
const float NAN = std::numeric_limits<float>::quiet_NaN ();
#define isNAN(val) (val == NAN)
#endif


typedef uint16_t TrackID;

class SensorFrame;
class ChannelFrame;
class Detection;

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

/** \brief The Detection class corresponds to a single detectio returned by the receiver.
           It holds the distance for the detection, its intensity, threat level
*/

class Detection 
{

public:
	typedef boost::shared_ptr<Detection> Ptr;
    typedef boost::shared_ptr<Detection> ConstPtr;
	typedef std::vector<Detection::Ptr> Vector;

	typedef enum ThreatLevel {
		eThreatNone = 0,  // No threat level assigned
		eThreatLow = 1,   // target detected, threat level low
		eThreatWarn = 2,  // Target may have collision course, but not acertained yet
		eThreatCritical = 3, // target is within collision range
	}
	ThreatLevel;

public:
	
	Detection(int inChannelID, int inDetectionID);
	Detection(int inChannelID, int inDetectionID, float inDistance, float inIntensity, float inVelocity, 
	float inTimeStamp, float inFirstTimeStamp, TrackID inTrackID, ThreatLevel inThreatLevel = eThreatNone);
	
	Detection::Detection(int inChannelID, int inDetectionID,  ifstream &inTrackFile);

	/** \brief Verify if there is any data in the detection.
	  *        a detection with distance = 0 is invalid.  it should not be stored.
      * \return bool true if the detection is valid.  False otherwise.
      */
	bool IsValid();
	int	GetDetectionID() {return(detectionID);}
	int	GetChannelID() {return(channelID);}

public:
	/** \brief channel ID of the channel where detection origins from */
	int	  channelID;

	/** \brief index of the detection within the channel */
	int	  detectionID;

	/** \brief distance, in meters */
	float distance;

	/** \brief intensity ratio (0-1 on twelve bits) */
	float intensity;

	/** \brief Velocity, in m/s */
	float velocity;

	/** \brief acceleration, in m/s squared */
	float acceleration;

	/** \brief Time to collision, in seconds */
	float timeToCollision;

	/** \brief Required acceleration to zero speed, in m/s2seconds */
	float decelerationToStop;

	/** \brief Track Positive detection probability estimate */
	float probability;

	/** \brief Timestamp, in frames */
	float timeStamp;

	/** \brief First time stamp at which the target was acquired. */
	float firstTimeStamp;

	/** \brief Track ID. */
	TrackID	trackID;

	/** \brief Threat level associated to detection */
	ThreatLevel	threatLevel;
};

/** \brief A track corresponds to an obstacle tracking based on a singl;e or multiple detections,
           and gives information on the evolution of the detected obstacle across multiple frames
*/

class Track
{
public:
	typedef boost::shared_ptr<Track> Ptr;
    typedef boost::shared_ptr<Track> ConstPtr;
	typedef std::vector<Track::Ptr> Vector;

	Track(int trackID);

	bool IsValid();

	int	GetTrackID() {return(trackID);}

	bool Contains(Detection::Ptr & inDetectionPtr);
	bool Contains(int channelID);

	// A track is built from 2 message sections.  Make sure both parts are entered before a track is completed.
	bool IsComplete() { return (part1Entered && part2Entered); };

public:
	/** \brief Track ID. */
	TrackID	trackID;

	/** \brief distance, in meters */
	float distance;

	/** \brief Velocity, in m/s.  Positive velocity means target is moving away from sensor */
	float velocity;

	/** \brief acceleration, in m/s squared */
	float acceleration;

	/** \brief Time to collision, in seconds */
	float timeToCollision;

	/** \brief Required acceleration to zero speed, in m/s2seconds */
	float decelerationToStop;

	/** \brief Track Positive detection probability estimate */

	float probability;

	/** \brief Timestamp, in frames */
	float timeStamp;

	/** \brief First time stamp at which the target was acquired. */
	float firstTimeStamp;

	/** \brief Threat level associated to detection */
	Detection::ThreatLevel	threatLevel;

	/** \brief Channels in which detections were made for the track **/
	uint8_t channels;

	Detection::Vector detections; 

	// A track is built from 2 message sections.  Make sure both parts are entered before a track is completed.

	bool part1Entered;
	bool part2Entered;
protected:
};

/** \brief The ChannelFrame class holds the multiple detections associated to one channel, in one single time frame.
*/

class ChannelFrame
{
public:
	typedef boost::shared_ptr<ChannelFrame> Ptr;
    typedef boost::shared_ptr<ChannelFrame> ConstPtr;
	typedef std::vector<ChannelFrame::Ptr> Vector;
public:
	ChannelFrame(int channelID);
	ChannelFrame(int channelID, int inDetectionQty, ifstream &inTrackFile);
	virtual ~ChannelFrame() {};


	int GetChannelID() { return(channelID);}

	bool FindDetection(int inDetectionID, Detection::Ptr &outDetection);

public:
	int channelID;
	// Timestamp im milliseconds, elapsed from start of thread.
	double timeStamp;

	Detection::Vector detections;
};


/** \brief The SensorFrame class holds all channel detections (channelFrames),
		corresponding to a unique time frame.
*/
class SensorFrame
{
public:
	typedef boost::shared_ptr<SensorFrame> Ptr;
    typedef boost::shared_ptr<SensorFrame> ConstPtr;
	typedef std::queue<SensorFrame::Ptr> Queue;
public:
	SensorFrame(uint32_t inFrameID);
	SensorFrame(uint32_t inFrameID, int inChannelQty, int inDetectionQty);
	SensorFrame(int inChannelQty, int inDetectionQty, ifstream &inTrackFile);
	virtual ~SensorFrame() {};

	uint32_t	GetFrameID() {return(frameID);}
	

	Detection::Ptr MakeUniqueDetection(int channelID, int detectionID);
public:
	uint32_t frameID;
	ChannelFrame::Vector channelFrames;
	Track::Vector tracks; 

	// Timestamp im milliseconds, elapsed from start of thread.
	double timeStamp;

//	ChannelFrame &operator[](int channelIndex) {return(*(channelFrames[channelIndex]));}
};

/** \brief The AcquisitionSequence is a list of all the information acquired from the receiver,
		across multiple time frames.
*/
class AcquisitionSequence
{
public:
	typedef boost::shared_ptr<AcquisitionSequence> Ptr;
    typedef boost::shared_ptr<AcquisitionSequence> ConstPtr;

	typedef	enum TrackingMode 
	{
		eTrackSingleChannel,
		eTrackAllChannels
	};


public:
	AcquisitionSequence(int inSequenceID);
	AcquisitionSequence(int inSequenceID, int inChannelQty, int inDetectionQty);
	AcquisitionSequence(int inSequenceID, int inChannelQty, int inDetectionQty, ifstream &inTrackFile);
	AcquisitionSequence(int inSequenceID, int inChannelQty, int inDetectionQty, std::string inFileName);

	void ReadFile(std::string inFileName, int inChannelQty, int inDetectionQty);
	void ReadFile(ifstream &inTrackFile, int inChannelQty, int inDetectionQty);

//	SensorFrame &operator[](int frameIndex) {return(*(sensorFrames[frameIndex]));}

	virtual ~AcquisitionSequence() {};
	int	GetID() {return(sequenceID);}
	
	/** \brief Get the frameID of the last complete frame
	*/
	uint32_t	GetLastFrameID();

	void Clear();

	uint32_t AllocateFrameID();

	bool FindTrack(SensorFrame::Ptr currentFrame,TrackID trackID, Track::Ptr &outTrack);
	Track::Ptr MakeUniqueTrack(SensorFrame::Ptr currentFrame,TrackID trackID);

	bool FindSensorFrame(uint32_t frameID, SensorFrame::Ptr &outSensorFrame);
	ChannelFrame::Ptr & GetChannelAtIndex(int frameIndex, int channelIndex);

	int FindFrameIndex(uint32_t frameID);
	int GetLastFrameIndex();

	// Build detections from the current track set.
	void BuildDetectionsFromTracks(SensorFrame::Ptr currentFrame);
protected:
	void UpdateTrackInfo(SensorFrame::Ptr currentFrame);

public: 
	int sequenceID;
	int channelQty;
	int detectionQty;
	SensorFrame::Queue sensorFrames;
	
	std::string infoLine;

	uint32_t frameID;
};

} // namespace awl
#endif // AWL_TRACKER_H
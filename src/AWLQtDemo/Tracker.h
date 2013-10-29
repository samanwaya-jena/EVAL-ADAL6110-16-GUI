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


#define TRACK_TRACKER

using namespace std;

namespace awl
{
typedef uint16_t TrackID;

class SensorFrame;
class ChannelFrame;
class Detection;

/** \brief The Detection class corresponds to a single detectio returned by the receiver.
           It holds the distance for the detection, its intensity, threat level
*/

class Detection 
{

public:
	typedef boost::shared_ptr<Detection> Ptr;
    typedef boost::shared_ptr<Detection> ConstPtr;

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
	Track(TrackID inTrackID,
		float inDistance, float inVelocity, float inTimeStamp, float inFirstTimeStamp, 
		   Detection::ThreatLevel inThreatLevel = Detection::eThreatNone);
	bool IsValid();

	int	GetTrackID() {return(trackID);}

	bool Contains(Detection::Ptr & inDetectionPtr);
	bool Contains(int channelID);



	/** \brief Return true if track probability level is above probability threshold.
     * \return bool true if track probability level is above probability threshold..  False otherwise.
      */
	bool IsProbable();

public:
	/** \brief Track ID. */
	TrackID	trackID;

	/** \brief distance, in meters */
	float distance;

	/** \brief Velocity, in m/s */
	float velocity;
	
	/** \brief Timestamp, in frames */
	float timeStamp;

	/** \brief First time stamp at which the target was acquired. */
	float firstTimeStamp;

	/** \brief Threat level associated to detection */
	Detection::ThreatLevel	threatLevel;

	/** \brief Track Positive detection probability estimate */

	float probability;

	std::vector<Detection::Ptr> detections; 
protected:

};

/** \brief The ChannelFrame class holds the multiple detections associated to one channel, in one single time frame.
*/

class ChannelFrame
{
public:
	typedef boost::shared_ptr<ChannelFrame> Ptr;
    typedef boost::shared_ptr<ChannelFrame> ConstPtr;

public:
	ChannelFrame(int channelID);
	ChannelFrame(int channelID, int inDetectionQty);
	ChannelFrame(int channelID, int inDetectionQty, ifstream &inTrackFile);
	virtual ~ChannelFrame() {};


	int GetChannelID() { return(channelID);}
public:
	int channelID;
	// Timestamp im milliseconds, elapsed from start of thread.
	double timeStamp;

	std::vector<Detection::Ptr> detections;
};


/** \brief The SensorFrame class holds all channel detections (channelFrames),
		corresponding to a unique time frame.
*/
class SensorFrame
{
public:
	typedef boost::shared_ptr<SensorFrame> Ptr;
    typedef boost::shared_ptr<SensorFrame> ConstPtr;
public:
	SensorFrame(uint32_t inFrameID);
	SensorFrame(uint32_t inFrameID, int inChannelQty, int inDetectionQty);
	SensorFrame(int inChannelQty, int inDetectionQty, ifstream &inTrackFile);
	virtual ~SensorFrame() {};

	uint32_t	GetFrameID() {return(frameID);}
public:
	uint32_t frameID;
	std::vector<ChannelFrame::Ptr> channelFrames;

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

	TrackingMode GetTrackingMode();
	TrackingMode SetTrackingMode(AcquisitionSequence::TrackingMode inTrackingMode);

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

	bool FindSensorFrame(uint32_t frameID, SensorFrame::Ptr &outSensorFrame);
	ChannelFrame::Ptr & GetChannelAtIndex(int frameIndex, int channelIndex);
	Detection::Ptr & GetDetectionAtIndex(int frameIndex, int channelIndex, int detectionIndex);

	int FindFrameIndex(uint32_t frameID);
	int GetLastFrameIndex();


	void AcquisitionSequence::BuildTracks(double inTimeStamp);
	Track::Vector &GetTracks() {return(tracks);};

public: 
	int sequenceID;
	int channelQty;
	int detectionQty;
	std::queue<SensorFrame::Ptr> sensorFrames;
	
	std::string infoLine;

	uint32_t frameID;

protected:
#ifdef TRACK_TRACKER

	void UpdateTracks(double inTimeStamp);
	void CleanTracks(double inTimeStamp);
	int  FitDetectionToTrack(double inTimeStamp, Detection::Ptr & detection);
	Track::Ptr & CreateTrack(double inTimeStamp, Track::Vector &trackVector, Detection::Ptr & detection);

protected:
	double distanceDamping; //(0.6);  // damping to compute the last position of the track. Range 0-1.
	double speedDamping ;// (0.6);	// damping to compute the last velocity of the track	
	double trackTimeOut; //(0.5);		// Track inactivity time out (seconds) 
	double distanceThreshold; //(0.5); // Maximum error to correlate, in meters
	double minimumDistance; //(1.0); // Minimum distance at which we consider an obstacle valid.

	Track::Vector tracks; 

	TrackingMode	trackingMode;

#endif
};



} // namespace awl
#endif // AWL_TRACKER_H
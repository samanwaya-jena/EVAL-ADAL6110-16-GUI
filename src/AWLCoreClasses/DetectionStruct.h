#ifndef AWL_DETECTION_STRUCT_H
#define AWL_DETECTION_STRUCT_H
/*
	Copyright 2014 Aerostar R&D Canada Inc.

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
#include <string>
#include <queue>
#include <boost/shared_ptr.hpp>
#include <boost/container/vector.hpp>
#include "CoordinateSystem.h"


using namespace std;

namespace awl
{

const float NAN = std::numeric_limits<float>::quiet_NaN ();
#define isNAN(val) (val == NAN)

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
	typedef boost::container::vector<Detection::Ptr> Vector;

	typedef enum ThreatLevel {
		eThreatNone = 0,  // No threat level assigned
		eThreatLow = 1,   // target detected, threat level low
		eThreatWarn = 2,  // Target may have collision course, but not acertained yet
		eThreatCritical = 3, // target is within collision range
	}
	ThreatLevel;

public:
	Detection();
	Detection(int inReceiverID, int inChannelID, int inDetectionID);
	Detection(int inReceiverID, int inChannelID, int inDetectionID, float inDistance, float inIntensity, float inVelocity, 
	          float inTimeStamp, float inFirstTimeStamp, TrackID inTrackID, ThreatLevel inThreatLevel = eThreatNone);
	
	int	GetReceiverID() {return(receiverID);}	
	int	GetDetectionID() {return(detectionID);}
	int	GetChannelID() {return(channelID);}

public:
	/** \brief sensor ID of the sensor where detection origins from */
	int	  receiverID;

	/** \brief channel ID of the channel where detection origins from */
	int	  channelID;

	/** \brief index of the detection within the channel */
	int	  detectionID;

	/** \brief distance, in meters */
	float distance;

	/** \brief intensity ratio exprtessed as SNR */
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

	/** \brief Track ID from which the detection was generated, if that is the case. */
	TrackID	trackID;

	/** \brief Coordinates of detection relative to sensor */
	CartesianCoord relativeToSensorCart;
	SphericalCoord	   relativeToSensorSpherical;

	/** \brief Coordinates of detection relative to vehicule bumper */
	CartesianCoord relativeToVehicleCart;
	SphericalCoord	   relativeToVehicleSpherical;

	/** \brief Coordinates of detection relative to world */
	CartesianCoord relativeToWorldCart;
	SphericalCoord	   relativeToWorldSpherical;

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
	typedef boost::container::vector<Track::Ptr> Vector;

	Track(int trackID);

	int	GetTrackID() {return(trackID);}

	// A track is built from 4 distinct CAN message sections.  Make sure all parts are entered before a track is completed.
	bool IsComplete() { return (part1Entered && part2Entered && part3Entered && part4Entered); };

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

	/** \bried Track relative intensity, expressed as SNR */
	float intensity;

	/** \brief Timestamp, in frames */
	float timeStamp;

	/** \brief First time stamp at which the target was acquired. */
	float firstTimeStamp;

	/** \brief Threat level associated to detection */
	Detection::ThreatLevel	threatLevel;

	/** \brief Channels in which detections were made for the track **/
	uint8_t channels;

	// A track is built from up to 4 message sections (in CAN).  Make sure all parts are entered before a track is completed.

	bool part1Entered;
	bool part2Entered;
	bool part3Entered;
	bool part4Entered;
protected:
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
	SensorFrame(int inReceiverID, uint32_t inFrameID, int inChannelQty);
	virtual ~SensorFrame() {};

	int GetReceiverID() { return(receiverID);}
	uint32_t	GetFrameID() {return(frameID);}


	Detection::Ptr MakeUniqueDetection(Detection::Vector &detectionVector, int channelID, int detectionID);
	bool FindDetection(Detection::Vector &detectionVector, int inChannelID, int inDetectionID, Detection::Ptr &outDetection);

public:
	int receiverID;
	uint32_t frameID;
	int channelQty;

	Detection::Vector rawDetections;		// Raw detections as acquired from device

	Track::Vector tracks; 

	// Timestamp im milliseconds, elapsed from start of thread.
	double timeStamp;

};

/** \brief The AcquisitionSequence is a list of all the information acquired from the receiver,
		across multiple time frames.
*/
class AcquisitionSequence
{
public:
	typedef boost::shared_ptr<AcquisitionSequence> Ptr;
    typedef boost::shared_ptr<AcquisitionSequence> ConstPtr;


public:
	AcquisitionSequence();

//	SensorFrame &operator[](int frameIndex) {return(*(sensorFrames[frameIndex]));}

	virtual ~AcquisitionSequence() {};
	
	/** \brief Get the frameID of the last complete frame
	*/
	uint32_t	GetLastFrameID();

	uint32_t AllocateFrameID();

	bool FindTrack(SensorFrame::Ptr currentFrame,TrackID trackID, Track::Ptr &outTrack);
	Track::Ptr MakeUniqueTrack(SensorFrame::Ptr currentFrame,TrackID trackID);

	bool FindSensorFrame(uint32_t frameID, SensorFrame::Ptr &outSensorFrame);

public: 
	// Queue of the stored sensor frames.
	// The acquisitionSequence stores multiple frames, in support of asynchonous operations,
	// as well as to support tracking algorithms
	SensorFrame::Queue sensorFrames;
	
	// Frame ID counter.  Each SensorFrame within an acquisition sequence has a unique frame ID.
	uint32_t frameID;
};



} // namespace awl
#endif // AWL_DETECTION_STRUCT_H
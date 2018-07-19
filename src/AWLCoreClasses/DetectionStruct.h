#ifndef AWL_DETECTION_STRUCT_H
#define AWL_DETECTION_STRUCT_H
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
#include <string>
#include <queue>
#include <boost/shared_ptr.hpp>
#include <boost/container/vector.hpp>
#include "CoordinateSystem.h"


using namespace std;

namespace awl
{

//const float NAN = std::numeric_limits<float>::quiet_NaN ();
#define isNAN(val) (val == NAN)

typedef uint16_t TrackID;
typedef uint32_t FrameID;

class AlertCondition;
class Detection;
class AScan;
class Track;
class SensorFrame;
class ChannelFrame;

/** \brief ChannelMask struct describes receiverchannel bit mask used in most data structures
*        and communications
* \author Jean-Yves Deschênes
*/

typedef union
{
	uint8_t byteData;
	struct  {
		bool channel0 : 1;
		bool channel1 : 1;
		bool channel2 : 1;
		bool channel3 : 1;
		bool channel4 : 1;
		bool channel5 : 1;
		bool channel6 : 1;
		bool channel7 : 1;
	} bitFieldData;

} ChannelMask, AlertChannelMask;


/** \brief The Alert class defines alert conditions.
*/

class AlertCondition {
public: 
	typedef boost::shared_ptr<AlertCondition> Ptr;
	typedef boost::shared_ptr<AlertCondition> ConstPtr;
	typedef boost::container::vector<AlertCondition::Ptr> Vector;

	typedef enum ThreatLevel {
		eThreatNone = 0,  // No threat level assigned
		eThreatLow = 1,   // target detected, threat level low
		eThreatWarn = 2,  // Target may have collision course, but not acertained yet
		eThreatCritical = 3 // target is within collision range
	}
	ThreatLevel;

	typedef enum AlertType {
		eAlertInvalid = 0,
		eAlertDistanceWithin = 1, // Alert based on distance  within range specified
		eAlertDistanceOutside = 2, // Alert based on distance  outside range specified
		eAlertSpeed = 3, // Alert based on speed range;
		eAlertAcceleration = 4,    // Alert based on accel range;
		eAlertDecelerationToStop = 5, // Alert based on deceleration to stop
		eAlertTTC = 6		// Alert based on time to collision;
	}
	AlertType;

public:
	AlertCondition() {};
	AlertCondition(AlertCondition::AlertType inAlertType, int inReceiverID, ChannelMask inChannelMask, float inMinRange, float inMaxRange, ThreatLevel inThreatLevel = eThreatNone);
	AlertCondition(AlertCondition &sourceCondition);

	static AlertCondition::ThreatLevel FindDetectionThreat(boost::shared_ptr<Detection> detection);
	static AlertCondition::ThreatLevel FindTrackThreat(int inReceiverID, boost::shared_ptr<Track> track);

	int	GetReceiverID() { return(receiverID); }
	AlertChannelMask	GetChannelMask() { return(alertChannelMask); }
	float GetMinRange() { return (minRange); }
	float GetMaxRange() { return (maxRange); }
	ThreatLevel GetThreatLevel (){ return(threatLevel); }

	static void Store(AlertCondition::Ptr storedCondition) { globalAlertsVector.push_back(storedCondition); }

	AlertCondition::Vector GetAlertConditions() { return (globalAlertsVector); }
public:

	/** \brief Type of condition for the alert */
	AlertType alertType;

	/** \brief sensor ID of the sensor where detection origins from */
	int	  receiverID;

	/** \brief channelMask of the channels where detection origins from */
	AlertChannelMask	  alertChannelMask;

	/** \brief minimum range for triggerting of the alert. Nature depends on AlertType */
	float	  minRange;

	/** \brief maximum range for triggerting of the alert. Nature depends on AlertType */
	float	  maxRange;

	ThreatLevel threatLevel;


protected:
	static AlertCondition::Vector globalAlertsVector;
};

/** \brief The Detection class corresponds to a single detectio returned by the receiver.
           It holds the distance for the detection, its intensity, threat level
*/

class Detection 
{
	friend class AlertCondition;

public:
	typedef boost::shared_ptr<Detection> Ptr;
    typedef boost::shared_ptr<Detection> ConstPtr;
	typedef boost::container::vector<Detection::Ptr> Vector;

public:
	Detection();
	Detection(int inReceiverID, int inChannelID, int inDetectionID);
	Detection(int inReceiverID, int inChannelID, int inDetectionID, float inDistance, float inIntensity, float inVelocity, 
	          float inTimeStamp, float inFirstTimeStamp, TrackID inTrackID, AlertCondition::ThreatLevel inThreatLevel = AlertCondition::eThreatNone);
	
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
	AlertCondition::ThreatLevel	threatLevel;
};

enum RawProvider{
	rawFromPosixUDP,
	rawFromPosixTTY,
	rawFromLibUSB
}; 

class AScan
{
public:
	typedef boost::shared_ptr<AScan> Ptr;
    typedef boost::shared_ptr<AScan> ConstPtr;
	typedef boost::container::vector<AScan::Ptr> Vector;
public:
	AScan(int inReceiverID, int inChannelID, int inAScanID)
	{
		receiverID = inReceiverID;
		channelID = inChannelID;
		aScanID = inAScanID;
	}

	int	GetChannelID() {return(channelID);}
public:
	int receiverID;
	/** \brief channel ID of the channel where detection origins from */
	int channelID;
	int aScanID;

	RawProvider rawProvider;

	size_t sampleOffset;

	size_t sampleCount;

	size_t sampleSize;

	bool sampleSigned;

	void *samples;


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

	// A track is built from 2 distinct CAN message sections.  Make sure all parts are entered before a track is completed.
	// Note: (Before 2018-02-18, it used to be 4 parts, but messages of part 3 and 4 were deprecated.
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

	/** \bried Track relative intensity, expressed as SNR */
	float intensity;

	/** \brief Timestamp, in frames */
	float timeStamp;

	/** \brief First time stamp at which the target was acquired. */
	float firstTimeStamp;

	/** \brief Threat level associated to detection */
	AlertCondition::ThreatLevel	threatLevel;

	/** \brief Channels in which detections were made for the track **/
	ChannelMask trackChannels;

	/** \brief Main channel in which detections is made for the track **/
	uint16_t trackMainChannel;


	// A track is built from up to 4 message sections (in CAN).  Make sure all parts are entered before a track is completed.

	bool part1Entered;
	bool part2Entered;
	bool part3Entered;  // Not required anymore, message was deprecated.
	bool part4Entered;  // Note required anymore, message was deprecated.
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
	typedef std::deque<SensorFrame::Ptr> Queue;
public:
	SensorFrame(int inReceiverID, FrameID inFrameID, int inChannelQty);
	virtual ~SensorFrame() {};

	int GetReceiverID() { return(receiverID);}
	FrameID	GetFrameID() {return(frameID);}


	Detection::Ptr MakeUniqueDetection(Detection::Vector &detectionVector, int channelID, int detectionID);
	bool FindDetection(Detection::Vector &detectionVector, int inChannelID, int inDetectionID, Detection::Ptr &outDetection);

	AScan::Ptr MakeUniqueAScan(AScan::Vector &detectionVector, int channelID, int detectionID);
	bool FindAScan(AScan::Vector &detectionVector, int inChannelID, int inAScanID, AScan::Ptr &outAScan);

public:
	int receiverID;
	FrameID frameID;
	int channelQty;

	Detection::Vector rawDetections;		// Raw detections as acquired from device
	AScan::Vector aScans;

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
	FrameID	GetLastFrameID();

	FrameID AllocateFrameID();

	bool FindTrack(SensorFrame::Ptr currentFrame,TrackID trackID, Track::Ptr &outTrack);
	Track::Ptr MakeUniqueTrack(SensorFrame::Ptr currentFrame,TrackID trackID);

	bool FindSensorFrame(FrameID frameID, SensorFrame::Ptr &outSensorFrame);

public: 
	// Queue of the stored sensor frames.
	// The acquisitionSequence stores multiple frames, in support of asynchonous operations,
	// as well as to support tracking algorithms
	SensorFrame::Queue sensorFrames;
	
	// Frame ID counter.  Each SensorFrame within an acquisition sequence has a unique frame ID.
	FrameID frameID;
};



} // namespace awl
#endif // AWL_DETECTION_STRUCT_H

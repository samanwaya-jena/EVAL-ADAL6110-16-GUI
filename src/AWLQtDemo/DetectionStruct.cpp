

#include <stdint.h>
#include <string>
#define _USE_MATH_DEFINES 1  // Makes sure we have access to all math constants, like M_PI
#include <math.h>

#include "CoordinateSystem.h"
#include "DetectionStruct.h"

using namespace std;
using namespace awl;


#if 0

	/** \brief Predict the time to collision (distance = 0) between sensor and obstacle,  
	  *        given current distance and speed, assuming constant deceleration.
 	  * \param[in] currentDistance to obstacle, in meters
 	  * \param[in] relativeSpeed relative speed between sensor and obstacle, in meters/sec
	  * \return Predicted time to collision, in seconds.
	  * \remarks This corresponds to TTC1 measure defined in:
	  *          Yizhen Zhang, Erik K. Antonsson and Karl Grote: A New Threat Assessment Measure for Collision Avoidance Systems
      */

float PredictTimeToCollisionConstant (float currentDistance, float relativeSpeed)
{
	// Time to collision assuming constant distance. In m/s
	return(currentDistance / relativeSpeed);
}



	/** \brief Predict the relative distance of an obstacle after a certain time delay,  
	  *        given current distance speed and  deceleration.
 	  * \param[in] currentDistance to obstacle, in meters
 	  * \param[in] relativeSpeed relative speed between sensor and obstacle, in meters/sec
 	  * \param[in] currentAcceleration current acceleration between sensor and obstacle, in meters/sec2
	  * \param[in] timeDelay timeDelay (in sec) for which updated position is calculated
	  * \return Predicted relative distance of the obstacle, , in m/s2.
	  * \remarks  arguments use acceleration, not deceleration! Positive acceleration means object is moving away.
      */

float PredictDistance(float currentDistance, float relativeSpeed, float acceleration, float time)
	
{
	if (isNAN(acceleration)) acceleration = 0;

	// Distance of vehicle at "time"
	float at2 = (acceleration * time * time);
	float predictedDistance = currentDistance + (relativeSpeed * time) + (0.5 * at2);

	return (predictedDistance);

}


	/** \brief Calculate the required accceleration to get to zero speed at the specified distance, 
	  *        given currentSpeed and current deceleration.
 	  * \param[in] currentDistance to obstacle, in meters
 	  * \param[in] relativeSpeed relative speed between sensor and obstacle, in meters/sec
 	  * \param[in] currentDeceleration current deceleration between sensor and obstacle, in meters/sec2
	  * \return Current required acceleration, in m/s2. Should be a negative value for objects movig towards sensor.
	  * \remarks  arguments use acceleration, not deceleration! Positive acceleration means object is moving away.
      */
float CalculateAccelerationToStop(float currentDistance, float relativeSpeed, float currentAcceleration)

{
	if (isNAN(currentAcceleration)) currentAcceleration = 0;
	
	float acceleration = (relativeSpeed * relativeSpeed) / currentDistance;
	if (relativeSpeed < 0) 
	{
		acceleration *= -1.0;
	}

	acceleration -= currentAcceleration;

	return(acceleration);
}


float PredictTTC(float distance, float speed, float acceleration, float time)  // Value for acceleration should be negative when 
{
	if (isNAN(acceleration)) acceleration = 0;

	// TTC at "time".  
	float at2 = (acceleration * time * time);
	float ttc = time + ((1/(2*speed)) * at2);

	return(ttc);
}

#endif

AcquisitionSequence::AcquisitionSequence():
frameID(1)

{
}

uint32_t AcquisitionSequence::AllocateFrameID()

{
	return(frameID++);
}

uint32_t	AcquisitionSequence::GetLastFrameID()

{
	if (sensorFrames.size() <= 0) return(0);
	
	return(sensorFrames.back()->GetFrameID());

}

#if 0
bool AcquisitionSequence::CompleteTrackInfo(SensorFrame::Ptr currentFrame)
{
	bool bAllTracksComplete = true;
	AWLSettings *settings = AWLSettings::GetGlobalSettings();

	// Update the coaslesced tracks
   Track::Vector::iterator  trackIterator = currentFrame->tracks.begin();

	while (trackIterator != currentFrame->tracks.end()) 
	{
		Track::Ptr track = *trackIterator;
		if (track->IsComplete()) 
		{
			if (track->velocity > 0) 
			{
				track->timeToCollision = PredictTimeToCollisionConstant(track->distance, track->velocity);
			}

			// Deceleration to stop should eventually include track->acceleration, but right now,
			// that information is judged unreliable.  
			// Will need adjustement later
			track->decelerationToStop = -	CalculateAccelerationToStop(track->distance, track->velocity, 0);

			if (track->decelerationToStop > settings->threatLevelCriticalThreshold)  track->threatLevel = Detection::eThreatCritical;
			else if (track->decelerationToStop > settings->threatLevelWarnThreshold) track->threatLevel = Detection::eThreatWarn;
			else if (track->decelerationToStop > settings->threatLevelLowThreshold)  track->threatLevel = Detection::eThreatLow;
			else track->threatLevel = Detection::eThreatNone;
		}  // if (track...
		else 
		{
			bAllTracksComplete = false;
		}


		trackIterator++;
	} // while (trackIterator...

	return (bAllTracksComplete);
} 


bool AcquisitionSequence::BuildEnhancedDetectionsFromTracks(SensorFrame::Ptr currentFrame)
{
	bool bAllTracksComplete = true;

	for (int channelIndex = 0; channelIndex < currentFrame->channelQty; channelIndex++) 
	{
		uint8_t channelMask = 0x01 << channelIndex;

		int detectionIndex = 0;

		// Re-Create detections from the coalesced tracks
		int trackQty = currentFrame->tracks.size();

		Track::Vector::iterator  trackIterator = currentFrame->tracks.begin();
		while (trackIterator != currentFrame->tracks.end()) 
		{
			Track::Ptr track = *trackIterator;
			if (track->IsComplete() && (track->channels & channelMask)) 
			{
				Detection::Ptr detection = currentFrame->MakeUniqueDetection(currentFrame->enhancedDetections,channelIndex, detectionIndex++);
				detection->channelID = channelIndex;
				detection->distance = track->distance;

				detection->intensity = track->intensity; 
				detection->velocity = track->velocity;
				detection->acceleration = track->acceleration;
				detection->probability = track->probability;
				detection->timeStamp = track->timeStamp;
				detection->firstTimeStamp = track->timeStamp;  // TBD
				detection->trackID = track->trackID;
				detection->timeToCollision = track->timeToCollision;
				detection->decelerationToStop = track->decelerationToStop;
				detection->threatLevel = track->threatLevel;

				// Place the coordinates relative to all their respective reference systems
				TransformationNode::Ptr channelCoords = AWLCoordinates::GetChannel(detection->receiverID, channelIndex);
				SphericalCoord sphericalPointInChannel(detection->distance, M_PI_2, 0);
				detection->relativeToSensorCart = channelCoords->ToReferenceCoord(eSensorToReceiverCoord, sphericalPointInChannel);
				detection->relativeToVehicleCart = channelCoords->ToReferenceCoord(eSensorToVehicleCoord, sphericalPointInChannel);
				detection->relativeToWorldCart = channelCoords->ToReferenceCoord(eSensorToWorldCoord, sphericalPointInChannel);

				detection->relativeToSensorSpherical = detection->relativeToSensorCart;
				detection->relativeToVehicleSpherical = detection->relativeToVehicleCart;
				detection->relativeToWorldSpherical = detection->relativeToWorldCart;
			}  // if (track...
			else if (!track->IsComplete() || (track->channels == 0))
			{
				bAllTracksComplete = false;
			}

			trackIterator++;
		} // while (trackIterator...
	} // for (channelIndex)

	return (bAllTracksComplete);
}
#endif

bool AcquisitionSequence::FindTrack(SensorFrame::Ptr currentFrame, TrackID trackID, Track::Ptr &outTrack)
{

	Track::Vector::iterator  trackIterator = currentFrame->tracks.begin();
	while (trackIterator != currentFrame->tracks.end()) 
	{
		Track::Ptr track = *trackIterator;		
		if (track->GetTrackID() == trackID) 
		{
			outTrack = track;
			return(true);
		}

		trackIterator++;
	}

	return(false);
}

Track::Ptr AcquisitionSequence::MakeUniqueTrack(SensorFrame::Ptr currentFrame, TrackID trackID)

{
	Track::Ptr track;
	bool bExists = FindTrack(currentFrame, trackID, track);
	if (!bExists) 
	{
		track = Track::Ptr(new Track(trackID));
		currentFrame->tracks.push_back(track);
	}

	return(track);
}

bool AcquisitionSequence::FindSensorFrame(uint32_t frameID, SensorFrame::Ptr &outSensorFrame)
{
	for (int i = 0; i < sensorFrames.size(); i++) 
	{
		SensorFrame::Ptr sensorFrame = sensorFrames._Get_container().at(i);
		if (sensorFrame->GetFrameID() == frameID) 
		{
			outSensorFrame = sensorFrame;
			return(true);
		}
	}

	return(false);
}


SensorFrame::SensorFrame(int inReceiverID, uint32_t inFrameID, int inChannelQty) :
receiverID(inReceiverID),
frameID(inFrameID),
channelQty(inChannelQty),
rawDetections(),
enhancedDetections(),
tracks(),
timeStamp(0)

{
}

Detection::Ptr SensorFrame::MakeUniqueDetection(Detection::Vector &detectionVector, int channelID, int detectionID)

{
	Detection::Ptr detection;
	bool bExists = FindDetection(detectionVector, channelID, detectionID, detection);
	if (!bExists) 
	{
		detection = Detection::Ptr(new Detection(receiverID, channelID, detectionID));
		detectionVector.push_back(detection);
	}

	return(detection);
}

bool SensorFrame::FindDetection(Detection::Vector &detectionVector, int inChannelID, int inDetectionID, Detection::Ptr &outDetection)
{
	Detection::Vector::iterator  detectionIterator = detectionVector.begin();
	while (detectionIterator != detectionVector.end()) 
	{
		Detection::Ptr detection = *detectionIterator;
		if (detection->channelID == inChannelID && detection->detectionID == inDetectionID) 
		{
			outDetection = detection;
			return(true);
		}

		detectionIterator++;
	}

	return(false);
}


Detection::Detection():
receiverID(0),
channelID(0), 
detectionID(0),
distance(0.0),
intensity(0.0),
velocity(0.0),
acceleration(0.0),
timeToCollision(NAN),
decelerationToStop(NAN),
probability(0.0),
timeStamp(0),
firstTimeStamp(0),
relativeToSensorCart(),
relativeToSensorSpherical(),
relativeToVehicleCart(),
relativeToVehicleSpherical(),
relativeToWorldCart(),
relativeToWorldSpherical()
{
}



Detection::Detection(int inReceiverID, int inChannelID, int inDetectionID):
receiverID(inReceiverID),
channelID(inChannelID), 
detectionID(inDetectionID),
distance(0.0),
intensity(0.0),
velocity(0.0),
acceleration(0.0),
timeToCollision(NAN),
decelerationToStop(NAN),
probability(0.0),
timeStamp(0),
firstTimeStamp(0),
relativeToSensorCart(),
relativeToSensorSpherical(),
relativeToVehicleCart(),
relativeToVehicleSpherical(),
relativeToWorldCart(),
relativeToWorldSpherical()
{

}

Detection::Detection(int inReceiverID, int inChannelID, int inDetectionID, float inDistance, float inIntensity, float inVelocity, 
		float inTimeStamp, float inFirstTimeStamp, TrackID inTrackID, ThreatLevel inThreatLevel):
receiverID(inReceiverID),
channelID(inChannelID),
detectionID(inDetectionID),
distance(inDistance),
intensity(inIntensity),
velocity(inVelocity),
acceleration(0.0),
timeToCollision(NAN),
probability(0.0),
timeStamp(inTimeStamp),
firstTimeStamp(inFirstTimeStamp),
trackID(inTrackID),
threatLevel(inThreatLevel),
relativeToSensorCart(),
relativeToSensorSpherical(),
relativeToVehicleCart(),
relativeToVehicleSpherical(),
relativeToWorldCart(),
relativeToWorldSpherical()


{
}


Track::Track(int inTrackID):
distance(0.0f),
velocity(NAN),
acceleration(NAN),
timeToCollision(NAN),
decelerationToStop(NAN),
timeStamp(0.0f),
firstTimeStamp(0.0),
trackID(inTrackID),
threatLevel(Detection::eThreatNone),
part1Entered(false),
part2Entered(false),
part3Entered(false),
part4Entered(false)

{
	channels = 0;
}

 
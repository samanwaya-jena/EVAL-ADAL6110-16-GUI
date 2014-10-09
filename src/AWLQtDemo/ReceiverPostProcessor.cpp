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
#define _USE_MATH_DEFINES 1  // Makes sure we have access to all math constants, like M_PI
#include <math.h>

#include <boost/foreach.hpp>

#include "AWLSettings.h"
#include "DetectionStruct.h"
#include "ReceiverPostProcessor.h"
#include "awlcoord.h"
#include "DebugPrintf.h"
#include "ReceiverCapture.h"

using namespace std;
using namespace awl;




ReceiverPostProcessor::ReceiverPostProcessor()

{
}

bool ReceiverPostProcessor::CompleteTrackInfo(SensorFrame::Ptr currentFrame)
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



bool ReceiverPostProcessor::BuildEnhancedDetectionsFromTracks(SensorFrame::Ptr currentFrame, Detection::Vector &outDetections)
{
	AWLSettings *settings = AWLSettings::GetGlobalSettings();
	ReceiverSettings &receiverSettings = settings->receiverSettings[currentFrame->GetReceiverID()];
	bool bAllTracksComplete = true;

	// Make sure that we start from scratch. All output detectio vesctor is cleared.
	outDetections.clear();

	// In channel order, recreate individual detections from the tracks.
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
			if ( track->IsComplete() && (track->channels & channelMask) &&
				(track->distance >= receiverSettings.displayedRangeMin) && 
				(track->distance <=  receiverSettings.channelsConfig[channelIndex].maxRange)) 
 			{
				Detection::Ptr detection = Detection::Ptr(new Detection(currentFrame->receiverID, channelIndex, detectionIndex++));
				outDetections.push_back(detection);
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


bool ReceiverPostProcessor::GetEnhancedDetectionsFromFrame(ReceiverCapture::Ptr receiver, uint32_t inFrameID,  Publisher::SubscriberID inSubscriberID, Detection::Vector &detectionBuffer)
{

	SensorFrame::Ptr currentFrame = SensorFrame::Ptr(new SensorFrame(receiver->receiverID, inFrameID, receiver->receiverChannelQty));

	// Get a local copy of the currentFrame to proceess;
	if (!receiver->CopyReceiverFrame(inFrameID, currentFrame, inSubscriberID))
	{
		return(false);
	}


	// Complete the track information that is not yet processed at the receiver level.
	if (!CompleteTrackInfo(currentFrame))
	{
		DebugFilePrintf("Incomplete frame in UpdateTrackInfo- %lu", inFrameID);
		return(false);
	}

	// Build distances from the tracks that were accumulated during the frame
	if (!BuildEnhancedDetectionsFromTracks(currentFrame, detectionBuffer))
	{
		DebugFilePrintf("Incomplete frame- %lu", inFrameID);
		return(false);
	}

	return(true);
}


	/** \brief Predict the time to collision (distance = 0) between sensor and obstacle,  
	  *        given current distance and speed, assuming constant deceleration.
 	  * \param[in] currentDistance to obstacle, in meters
 	  * \param[in] relativeSpeed relative speed between sensor and obstacle, in meters/sec
	  * \return Predicted time to collision, in seconds.
	  * \remarks This corresponds to TTC1 measure defined in:
	  *          Yizhen Zhang, Erik K. Antonsson and Karl Grote: A New Threat Assessment Measure for Collision Avoidance Systems
      */

float ReceiverPostProcessor::PredictTimeToCollisionConstant (float currentDistance, float relativeSpeed)
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

float ReceiverPostProcessor::PredictDistance(float currentDistance, float relativeSpeed, float acceleration, float time)
	
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
float ReceiverPostProcessor::CalculateAccelerationToStop(float currentDistance, float relativeSpeed, float currentAcceleration)

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


float ReceiverPostProcessor::PredictTTC(float distance, float speed, float acceleration, float time)  // Value for acceleration should be negative when 
{
	if (isNAN(acceleration)) acceleration = 0;

	// TTC at "time".  
	float at2 = (acceleration * time * time);
	float ttc = time + ((1/(2*speed)) * at2);

	return(ttc);
}



 
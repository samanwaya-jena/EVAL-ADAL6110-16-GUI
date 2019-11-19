/* ReceiverPostProcessor.cpp */
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

#include <stdint.h>
#include <string>
#include <vector>


#include <boost/foreach.hpp>

#include "SensorCoreClassesGlobal.h"
#include "SensorSettings.h"
#include "DetectionStruct.h"
#include "ReceiverPostProcessor.h"
#include "SensorCoord.h"
#include "DebugPrintf.h"
#include "ReceiverCapture.h"

#define _USE_MATH_DEFINES 1  // Makes sure we have access to all math constants, like M_PI
#include <math.h>

SENSORCORE_USE_NAMESPACE

ReceiverPostProcessor::ReceiverPostProcessor()

{
}

bool ReceiverPostProcessor::CompleteTrackInfo(SensorFrame::Ptr currentFrame)
{
	bool bAllTracksComplete = true;

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

			track->threatLevel = AlertCondition::FindTrackThreat(currentFrame->GetReceiverID(), track);
		}  // if (track...
		else 
		{
			bAllTracksComplete = false;
		}


		trackIterator++;
	} // while (trackIterator...

	return (bAllTracksComplete);
} 


bool ReceiverPostProcessor::BuildEnhancedDetectionsFromTracks(ReceiverCapture::Ptr receiver, SensorFrame::Ptr currentFrame, Detection::Vector &outDetections)
{
	SensorSettings *settings = SensorSettings::GetGlobalSettings();
	ReceiverSettings &receiverSettings = settings->receiverSettings[currentFrame->GetReceiverID()];
	bool bAllTracksComplete = true;

	// Make sure that we start from scratch. All output detection vector is cleared.
	outDetections.clear();
	std::vector<int> detectionIndex(currentFrame->voxelQty);


	// In voxel order, recreate individual detections from the tracks.
	for (int voxelIndex = 0; voxelIndex < currentFrame->voxelQty; voxelIndex++)
	{
		VoxelMask voxelMask;
		voxelMask.wordData = 0x01 << (voxelIndex % 8);


		// Re-Create detections from the coalesced tracks

		Track::Vector::iterator  trackIterator = currentFrame->tracks.begin();
		while (trackIterator != currentFrame->tracks.end()) 
		{
			Track::Ptr track = *trackIterator;


			int trackVoxel = track->trackMainVoxel;
			CellID cellID(trackVoxel % receiver->receiverColumnQty, trackVoxel / receiver->receiverColumnQty);
			float trackDistance = track->distance;

#if 1 // Process voxel wraparound if set
			if (receiver->lineWrapAround > 0.0)
			{
				int rowIndex = 0;
				int columnIndex = 0;

				rowIndex = (int) (track->distance / receiver->lineWrapAround);
				if (rowIndex > (receiver->receiverRowQty - 1))
				{
					rowIndex = 0;
				}

				columnIndex = voxelIndex % receiver->receiverColumnQty;

				trackVoxel = (rowIndex * receiver->receiverColumnQty) + columnIndex;
				cellID = CellID(trackVoxel % receiver->receiverColumnQty, trackVoxel / receiver->receiverColumnQty);

				trackDistance = track->distance - (rowIndex * receiver->lineWrapAround);
			}
#endif

			if ( track->IsComplete() && (trackVoxel == voxelIndex) &&
				(trackDistance >= receiverSettings.displayedRangeMin) && 
				(trackDistance <=  receiverSettings.voxelsConfig[voxelIndex].maxRange)) 
 			{
				Detection::Ptr detection = Detection::Ptr(new Detection(currentFrame->receiverID, cellID, detectionIndex[trackVoxel]++));
				outDetections.push_back(detection);

				detection->cellID = cellID;
				detection->distance = trackDistance;

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
				TransformationNode::Ptr voxelCoords = SensorCoordinates::GetVoxel(detection->receiverID, trackVoxel);
				SphericalCoord sphericalPointInChannel(detection->distance, (float) M_SENSORCORE_2, 0);
				detection->relativeToSensorCart = voxelCoords->ToReferenceCoord(eSensorToReceiverCoord, sphericalPointInChannel);
				detection->relativeToVehicleCart = voxelCoords->ToReferenceCoord(eSensorToVehicleCoord, sphericalPointInChannel);
				detection->relativeToWorldCart = voxelCoords->ToReferenceCoord(eSensorToWorldCoord, sphericalPointInChannel);

				detection->relativeToSensorSpherical = detection->relativeToSensorCart;
				detection->relativeToVehicleSpherical = detection->relativeToVehicleCart;
				detection->relativeToWorldSpherical = detection->relativeToWorldCart;
			}  // if (track...
			else if (!track->IsComplete())
			{
				bAllTracksComplete = false;
			}

			trackIterator++;
		} // while (trackIterator...
	} // for (voxelIndex)

	return (bAllTracksComplete);
}


bool ReceiverPostProcessor::GetEnhancedDetectionsFromFrame(ReceiverCapture::Ptr receiver, FrameID inFrameID,  Publisher::SubscriberID inSubscriberID, Detection::Vector &detectionBuffer)
{

	SensorFrame::Ptr currentFrame = SensorFrame::Ptr(new SensorFrame(receiver->receiverID, inFrameID, receiver->receiverVoxelQty));

	// Get a local copy of the currentFrame to proceess;
	if (!receiver->CopyReceiverFrame(inFrameID, currentFrame, inSubscriberID))
	{
		return(false);
	}


	// Complete the track information that is not yet processed at the receiver level.
	if (!CompleteTrackInfo(currentFrame))
	{
		DebugFilePrintf("Incomplete frame in UpdateTrackInfo- %lu", inFrameID);
	}

	// Build distances from the tracks that were accumulated during the frame
	if (!BuildEnhancedDetectionsFromTracks(receiver, currentFrame, detectionBuffer))
	{
		DebugFilePrintf("Incomplete frame- %lu", inFrameID);
	}

	if (detectionBuffer.size() <= 0) 
		return(false);
	else 
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
	float predictedDistance = currentDistance + (relativeSpeed * time) + (0.5f * at2);

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


float ReceiverPostProcessor::PredictTTC(float /*distance*/, float speed, float acceleration, float time)  // Value for acceleration should be negative when 
{
	if (isNAN(acceleration)) acceleration = 0;

	// TTC at "time".  
	float at2 = (acceleration * time * time);
	float ttc = time + ((1/(2*speed)) * at2);

	return(ttc);
}



 
/* DetectionStruct.cpp */
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

#include "CoordinateSystem.h"
#include "DetectionStruct.h"

using namespace std;
using namespace awl;



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

 
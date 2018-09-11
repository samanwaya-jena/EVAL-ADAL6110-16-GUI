/* DetectionStruct.cpp */
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

#include "CoordinateSystem.h"
#include "DetectionStruct.h"

#define _USE_MATH_DEFINES 1  // Makes sure we have access to all math constants, like M_PI
#include <math.h>

#include <float.h>

using namespace std;
using namespace awl;


AlertCondition::Vector AlertCondition::globalAlertsVector;


AcquisitionSequence::AcquisitionSequence():
frameID(1)

{
}

FrameID AcquisitionSequence::AllocateFrameID()

{
	return(frameID++);
}

FrameID	AcquisitionSequence::GetLastFrameID()

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

bool AcquisitionSequence::FindSensorFrame(FrameID frameID, SensorFrame::Ptr &outSensorFrame)
{
	for (uint16_t i = 0; i < sensorFrames.size(); i++) 
	{
		SensorFrame::Ptr sensorFrame = sensorFrames.at(i);
		if (sensorFrame->GetFrameID() == frameID) 
		{
			outSensorFrame = sensorFrame;
			return(true);
		}
	}

	return(false);
}


SensorFrame::SensorFrame(int inReceiverID, FrameID inFrameID, int inChannelQty) :
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

AScan::Ptr SensorFrame::MakeUniqueAScan(AScan::Vector &aScanVector, int receiverID, int channelID)

{
	AScan::Ptr aScan;
	bool bExists = FindAScan(aScanVector, receiverID, channelID, aScan);
	if (!bExists) 
	{
		aScan = AScan::Ptr(new AScan(receiverID, channelID));
		aScanVector.push_back(aScan);
	}

	return(aScan);
}

void AScan::FindMinMaxMean(float *min, float *max, float *mean)
{
	uint16_t *u16;
	int16_t *i16;
	uint32_t *u32;
	int32_t *i32;
	int i;

	u16 = (uint16_t*)samples;
	i16 = (int16_t*)samples;
	u32 = (uint32_t*)samples;
	i32 = (int32_t*)samples;

	*mean = 0.0;
	if (sampleSigned) {
		*min = FLT_MAX;
		*max = -FLT_MAX;
	} else {
		*min = FLT_MAX;
		*max = 0;
	}

	for (i = sampleOffset; i < sampleCount ; i ++) {
		switch (sampleSize) {
		default:
				return;
		case 2:
			if (sampleSigned) {
				if (i16[i] < *min) *min = i16[i];
				if (i16[i] > *max) *max = i16[i];
				*mean += i16[i];
			} else {
				if (u16[i] < *min) *min = u16[i];
				if (u16[i] > *max) *max = u16[i];
				*mean += u16[i];
			}
			break;
		case 4:
			if (sampleSigned) {
				if (i32[i] < *min) *min = i32[i];
				if (i32[i] > *max) *max = i32[i];
				*mean += i32[i];
			} else {
				if (u32[i] < *min) *min = u32[i];
				if (u32[i] > *max) *max = u32[i];
				*mean += u32[i];
			}
		}
	}
	if (i) *mean = *mean / i;
}

float AScan::GetScaleFactorForRange(int range)
{
	float min, max, mean;
	FindMinMaxMean (&min, &max, &mean);
	if (max - min) return range / (max - min);
	else return 0.0;
}

void AScan::Plot(int top, int left, int width, int height, AScanPlotter *plotter, float maxRange)
{
	int16_t *b16;
	int32_t *b32;
	float xScaleFactor = 0.0;
	float yScaleFactor = 0.0;
	int32_t x1, y1, x2, y2 = 0;
	int i;

	plotter->LabelAScan(receiverID, channelID);

	if (samples) {
		x1 = left;
		y1 = top;
		//printf ("%d %d\n", sampleCount, width);
		if (sampleCount > width) {

			if (width) xScaleFactor = (float) sampleCount / (float) width;
			yScaleFactor = (maxRange != 0.0F) ? (height / maxRange) : GetScaleFactorForRange(height);
			b16 = (int16_t *)(samples);
			b32 = (int32_t *)(samples);
			for (int x = 0; x < width; x ++) {
				x2 = left + x;
				i = x * xScaleFactor;
				switch (sampleSize) {
				default:
					return;
				case 2:
					y2 = top - b16[i + sampleOffset] * yScaleFactor;
					break;
				case 4:
					y2 = top - b32[i + sampleOffset] * yScaleFactor;
					break;
				}
				if (x == 0) {
					x1 = x2;
					y1 = y2;
				}
				plotter->PlotAScan(x1, y1, x2, y2);
				x1 = x2;
				y1 = y2;
			}
		} else {

			if (sampleCount) xScaleFactor = (float) width / (float) sampleCount;
			yScaleFactor = (maxRange != 0.0F) ? (height / maxRange) : GetScaleFactorForRange(height);
			b16 = (int16_t *)(samples);
			b32 = (int32_t *)(samples);
			for (int x = 0; x < sampleCount; x ++) {
				i = x * xScaleFactor;
				x2 = left + i;
				switch (sampleSize) {
				default:
					return;
				case 2:
					y2 = top - (b16[x + sampleOffset] * yScaleFactor);
					break;
				case 4:
					y2 = top - (b32[x + sampleOffset] * yScaleFactor);
					break;
				}
				if (x == 0) {
					x1 = x2;
					y1 = y2;
				}
				plotter->PlotAScan(x1, y1, x2, y2);
				x1 = x2;
				y1 = y2;
			}
		}
	}
}

void AScanPlotter::ShowAScan(bool show)
{
	showAScan = show;
	if (showAScan) printf ("show ascan\n");
	else printf ("hide ascan\n");
}

void AScanPlotter::PlotAScan(int x1, int y1, int x2, int y2)
{
	printf("%d %d, %d %d\n", x1, y1, x2, y2);
}

void AScanPlotter::LabelAScan(int receiver, int channel)
{
	printf("Ch %d-%d\n", receiver, channel);
}

void AScanPlotter::AScanDataChanged(const AScan::Vector& data)
{
	aScanData = data;
	AScan::Vector::iterator  aScanIterator = aScanData.begin();
	while (aScanIterator != aScanData.end()) 
	{
		AScan::Ptr aScan = *aScanIterator;
		//printf ("changed %d-%d\n", aScan->receiverID, aScan->channelID);
		aScanIterator++;
	}
}

bool SensorFrame::FindAScan(AScan::Vector &aScanVector, int inReceiverID, int inChannelID, AScan::Ptr &outAScan)
{
	AScan::Vector::iterator  aScanIterator = aScanVector.begin();
	while (aScanIterator != aScanVector.end()) 
	{
		AScan::Ptr aScan = *aScanIterator;
		if (aScan->receiverID == inReceiverID && aScan->channelID == inChannelID)
		{
			outAScan = aScan;
			return(true);
		}

		aScanIterator++;
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
		float inTimeStamp, float inFirstTimeStamp, TrackID inTrackID, AlertCondition::ThreatLevel inThreatLevel):
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
threatLevel(AlertCondition::eThreatNone),
part1Entered(false),
part2Entered(false),
part3Entered(false),
part4Entered(false)

{
	trackChannels.byteData = 0;
	trackMainChannel = 0;
}


AlertCondition::AlertCondition(AlertCondition::AlertType inAlertType, int inReceiverID, AlertChannelMask inChannelMask, float inMinRange, float inMaxRange, ThreatLevel inThreatLevel):
alertType(inAlertType),
receiverID(inReceiverID),
alertChannelMask(inChannelMask),
minRange(inMinRange),
maxRange(inMaxRange),
threatLevel(inThreatLevel)
{
}

AlertCondition::AlertCondition(AlertCondition &sourceCondition)
{
	*this = sourceCondition;
}

AlertCondition::ThreatLevel AlertCondition::FindDetectionThreat(boost::shared_ptr<Detection> detection)
{
	AlertCondition::ThreatLevel maxThreatLevel = AlertCondition::eThreatNone;
	
	AlertCondition::Vector::iterator  alertIterator = globalAlertsVector.begin();
	while (alertIterator != globalAlertsVector.end())
	{
		AlertCondition::Ptr alert = *alertIterator;
		AlertChannelMask theChannelMask;
		theChannelMask.byteData = 0x01 << detection->channelID;

		if (alert->receiverID == detection->receiverID && (alert->alertChannelMask.byteData & theChannelMask.byteData))
		{
			AlertCondition::ThreatLevel currentThreatLevel = AlertCondition::eThreatNone;
			switch (alert->alertType) {
			case eAlertDistanceWithin:
				if (detection->relativeToSensorSpherical.rho >= alert->minRange && detection->relativeToSensorSpherical.rho <= alert->maxRange)
				{
					currentThreatLevel = alert->threatLevel;
				}
				break;
			case eAlertDistanceOutside:
				if (!(detection->relativeToSensorSpherical.rho >= alert->minRange && detection->relativeToSensorSpherical.rho <= alert->maxRange))
				{
					currentThreatLevel = alert->threatLevel;
				}
				break;
			case eAlertSpeed:
				if (detection->velocity >= alert->minRange && detection->velocity <= alert->maxRange)
				{
					currentThreatLevel = alert->threatLevel;
				}
				break;

			case eAlertAcceleration:
				if (detection->acceleration >= alert->minRange && detection->acceleration<= alert->maxRange)
				{
					currentThreatLevel = alert->threatLevel;
				}
				break;

			case eAlertDecelerationToStop:
				if (detection->decelerationToStop >= alert->minRange && detection->decelerationToStop <= alert->maxRange)
				{
					currentThreatLevel = alert->threatLevel;
				}
				break;

			case eAlertTTC:
				if (detection->timeToCollision >= alert->minRange && detection->timeToCollision <= alert->maxRange)
				{
					currentThreatLevel = alert->threatLevel;
				}
				break;

			default:
				break;
			}

			if (currentThreatLevel > maxThreatLevel) maxThreatLevel = currentThreatLevel;
		}

		alertIterator++;
	}

	return(maxThreatLevel);

}

AlertCondition::ThreatLevel AlertCondition::FindTrackThreat(int inReceiverID, boost::shared_ptr<Track> track)
{
	AlertCondition::ThreatLevel maxThreatLevel = AlertCondition::eThreatNone;

	AlertCondition::Vector::iterator  alertIterator = globalAlertsVector.begin();
	uint16_t trackMask = 1 << track->trackMainChannel;

	while (alertIterator != globalAlertsVector.end())
	{
		AlertCondition::Ptr alert = *alertIterator;
		if (alert->receiverID == inReceiverID && (alert->alertChannelMask.byteData & trackMask))
		{
			AlertCondition::ThreatLevel currentThreatLevel = AlertCondition::eThreatNone;
			switch (alert->alertType) {
			case eAlertDistanceWithin:
				if (track->distance >= alert->minRange && track->distance <= alert->maxRange)
				{
					currentThreatLevel = alert->threatLevel;
				}
				break;

			case eAlertDistanceOutside:
				if (!(track->distance >= alert->minRange && track->distance <= alert->maxRange))
				{
					currentThreatLevel = alert->threatLevel;
				}
				break;

			case eAlertSpeed:
				if (track->velocity >= alert->minRange && track->velocity <= alert->maxRange)
				{
					currentThreatLevel = alert->threatLevel;
				}
				break;

			case eAlertAcceleration:
				if (track->acceleration >= alert->minRange && track->acceleration <= alert->maxRange)
				{
					currentThreatLevel = alert->threatLevel;
				}
				break;

			case eAlertDecelerationToStop:
				if (track->decelerationToStop >= alert->minRange && track->decelerationToStop <= alert->maxRange)
				{
					currentThreatLevel = alert->threatLevel;
				}
				break;

			case eAlertTTC:
				if (track->timeToCollision >= alert->minRange && track->timeToCollision <= alert->maxRange)
				{
					currentThreatLevel = alert->threatLevel;
				}
				break;

			default:
				break;
			}

			if (currentThreatLevel > maxThreatLevel) maxThreatLevel = currentThreatLevel;
		}

		alertIterator++;
	}

	return(maxThreatLevel);

}

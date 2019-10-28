/* DetectionStruct.cpp */
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

#include "SensorCoreClassesGlobal.h"
#include "CoordinateSystem.h"
#include "DetectionStruct.h"

#define _USE_MATH_DEFINES 1  // Makes sure we have access to all math constants, like M_PI
#include <math.h>

#include <float.h>

SENSORCORE_USE_NAMESPACE

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

bool AcquisitionSequence::FindSensorFrame(FrameID inFrameID, SensorFrame::Ptr &outSensorFrame)
{
	for (uint16_t i = 0; i < sensorFrames.size(); i++) 
	{
		SensorFrame::Ptr sensorFrame = sensorFrames.at(i);
		if (sensorFrame->GetFrameID() == inFrameID) 
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

AScan::Ptr SensorFrame::MakeUniqueAScan(AScan::Vector &aScanVector, int inReceiverID, int channelID)

{
	AScan::Ptr aScan;
	bool bExists = FindAScan(aScanVector, inReceiverID, channelID, aScan);
	if (!bExists) 
	{
		aScan = AScan::Ptr(new AScan(inReceiverID, channelID));
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

	size_t i = 0;
	for (i = (int)sampleOffset; i < sampleCount ; i ++) {
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
				if ((float)i32[i] < *min) *min = (float)i32[i];
				if ((float)i32[i] > *max) *max = (float)i32[i];
				*mean += i32[i];
			} else {
				if ((float)u32[i] < *min) *min = (float)u32[i];
				if ((float)u32[i] > *max) *max = (float)u32[i];
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


Track::Track(TrackID inTrackID):
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
	trackChannels.wordData = 0;
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
		theChannelMask.wordData = 0x01 << (uint16_t)detection->channelID;

		if (alert->receiverID == detection->receiverID && (alert->alertChannelMask.wordData & theChannelMask.wordData))
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
	uint16_t trackMask = 1 << (uint16_t)track->trackMainChannel;

	while (alertIterator != globalAlertsVector.end())
	{
		AlertCondition::Ptr alert = *alertIterator;
		if (alert->receiverID == inReceiverID  && (alert->alertChannelMask.wordData & trackMask))
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
				if (!(track->distance <= alert->minRange || track->distance >= alert->maxRange))
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

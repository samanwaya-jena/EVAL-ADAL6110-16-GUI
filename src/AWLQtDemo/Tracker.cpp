

#include <stdint.h>
#include <iostream>
#include <fstream>
#include <string>

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"


#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

#include "Tracker.h"
#include "DebugPrintf.h"
#include "AWLSettings.h"

using namespace std;
using namespace awl;



const AcquisitionSequence::TrackingMode defaultTrackingMode = AcquisitionSequence::eTrackAllChannels;


AcquisitionSequence::AcquisitionSequence(int inSequenceID):
sequenceID(inSequenceID),
frameID(0)

{
}

AcquisitionSequence::AcquisitionSequence(int inSequenceID, int inChannelQty, int inDetectionQty):
sequenceID(inSequenceID),
channelQty(inChannelQty),
detectionQty(inDetectionQty),
frameID(0)

{
}

AcquisitionSequence::AcquisitionSequence(int inSequenceID, int inChannelQty, int inDetectionQty, ifstream &inTrackFile):
sequenceID(inSequenceID),
channelQty(inChannelQty),
detectionQty(inDetectionQty),
frameID(0)

{
	ReadFile(inTrackFile, inChannelQty, inDetectionQty);
}

AcquisitionSequence::AcquisitionSequence(int inSequenceID, int inChannelQty, int inDetectionQty, std::string trackFileName):
sequenceID(inSequenceID),
channelQty(inChannelQty),
detectionQty(inDetectionQty),
frameID(0)

{
	ReadFile(trackFileName,  inChannelQty, inDetectionQty);
}

void AcquisitionSequence::ReadFile(std::string inFileName, int inChannelQty, int inDetectionQty)

{
	ifstream trackFile;
	trackFile.open(inFileName);
	ReadFile(trackFile, inChannelQty, inDetectionQty);
}

void AcquisitionSequence::ReadFile(ifstream &inTrackFile, int inChannelQty, int inDetectionQty)
{
	if (!inTrackFile.is_open()) return;
	// read the first  line for display purposes
	getline(inTrackFile, infoLine);
	
	while (!inTrackFile.eof()) 
	{
		SensorFrame::Ptr sensorFrame(new SensorFrame(inChannelQty, inDetectionQty, inTrackFile));
		sensorFrames.push(sensorFrame);
	}
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

void AcquisitionSequence::Clear()

{
	while (sensorFrames.size()) sensorFrames.pop();
}


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


void AcquisitionSequence::UpdateTrackInfo(SensorFrame::Ptr currentFrame)
{
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
			track->decelerationToStop = -CalculateAccelerationToStop(track->distance, track->velocity, 0);

			if (track->decelerationToStop > settings->threatLevelCriticalThreshold)  track->threatLevel = Detection::eThreatCritical;
			else if (track->decelerationToStop > settings->threatLevelWarnThreshold) track->threatLevel = Detection::eThreatWarn;
			else if (track->decelerationToStop > settings->threatLevelLowThreshold)  track->threatLevel = Detection::eThreatLow;
			else track->threatLevel = Detection::eThreatNone;
		}  // if (track...

		trackIterator++;
	} // while (trackIterator...
} 


void AcquisitionSequence::BuildDetectionsFromTracks(SensorFrame::Ptr currentFrame)
{
	UpdateTrackInfo(currentFrame);

	for (int channelIndex = 0; channelIndex < channelQty; channelIndex++) 
	{
		uint8_t channelMask = 0x01 << channelIndex;

		int detectionIndex = 0;

		// Update the coaslesced tracks
		int trackQty = currentFrame->tracks.size();

		Track::Vector::iterator  trackIterator = currentFrame->tracks.begin();

		while (trackIterator != currentFrame->tracks.end()) 
		{
			Track::Ptr track = *trackIterator;
			if (track->IsComplete() && (track->channels & channelMask)) 
			{
				int detectionIndex = currentFrame->channelFrames[channelIndex]->detections.size();
				Detection::Ptr detection = currentFrame->MakeUniqueDetection(channelIndex, detectionIndex);
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
			}  // if (track...

			trackIterator++;
		} // while (trackIterator...
	} // for (channelIndex)
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

int AcquisitionSequence::FindFrameIndex(uint32_t frameID)
{
	for (int i = 0; i < sensorFrames.size(); i++) 
	{
		SensorFrame::Ptr sensorFrame = sensorFrames._Get_container().at(i);
		if (sensorFrame->GetFrameID() == frameID) 
		{
			return(i);
		}
	}

	return(-1);
}

int AcquisitionSequence::GetLastFrameIndex()
{
	return(sensorFrames.size()- 1); 
}

ChannelFrame::Ptr & AcquisitionSequence::GetChannelAtIndex(int frameIndex, int channelIndex)

{
	SensorFrame::Ptr sensorFrame;
	if (frameIndex < sensorFrames.size()) 
	{
		sensorFrame = sensorFrames._Get_container().at(frameIndex);
	}
	else 
	{
		// Frame does not exist.  use the last frame
		sensorFrame = sensorFrames.back();
	}
	
	if (channelIndex >= sensorFrame->channelFrames.size()) 
	{
		// Channel does not exist. Return channel 0.
		channelIndex = 0;
	}

	return(sensorFrame->channelFrames.at(channelIndex));
}


SensorFrame::SensorFrame(uint32_t inFrameID) :
frameID(inFrameID),
timeStamp(0)
{
}

SensorFrame::SensorFrame(uint32_t inFrameID, int inChannelQty,  int inDetectionQty) :
frameID(inFrameID),
timeStamp(0)

{
	for (int channel = 0; channel < inChannelQty; channel++) 
	{
		ChannelFrame::Ptr myChannelFrame(new ChannelFrame(channel));
		channelFrames.push_back(myChannelFrame);
	}
}


SensorFrame::SensorFrame(int inChannelQty,  int inDetectionQty, ifstream &inTrackFile):
timeStamp(0)
{
	if (!inTrackFile.is_open()) return;

	std:string line;

	getline(inTrackFile, line); // Blank line between every frame
	getline(inTrackFile, line);
	sscanf(line.c_str(), "T=%d",  &frameID);

	for (int channel = 0; channel < inChannelQty; channel++) 
	{
		ChannelFrame::Ptr myChannelFrame(new ChannelFrame(channel, inDetectionQty, inTrackFile));
		channelFrames.push_back(myChannelFrame);
	}
}

Detection::Ptr SensorFrame::MakeUniqueDetection(int channelID, int detectionID)

{
	Detection::Ptr detection;
	bool bExists = channelFrames[channelID]->FindDetection(detectionID, detection);
	if (!bExists) 
	{
		detection = Detection::Ptr(new Detection(channelID, detectionID));
		channelFrames[channelID]->detections.push_back(detection);
	}

	return(detection);
}


ChannelFrame::ChannelFrame(int inChannelID):
channelID(inChannelID)
{

}



ChannelFrame::ChannelFrame(int inChannelID, int inDetectionQty, ifstream &inTrackFile) :
channelID(inChannelID)

{
	if (!inTrackFile.is_open()) return;
	std:string line;

	getline(inTrackFile, line);
	sscanf(line.c_str(), "sensor %d", &channelID);

	for (int detectionID = 0; detectionID < inDetectionQty; detectionID++) 
	{
		Detection::Ptr detection(new Detection(inChannelID, detectionID, inTrackFile));
		detections.push_back(detection);
	}
}

bool ChannelFrame::FindDetection(int inDetectionID, Detection::Ptr &outDetection)

{
	Detection::Vector::iterator detectionIterator = detections.begin();
	while (detectionIterator != detections.end())  
	{
		Detection::Ptr detection = *detectionIterator;
		if (detection->detectionID == inDetectionID) 
		{
			outDetection = detection;
			return(true);
		}

		detectionIterator++;
	}

	return(false);
}




Detection::Detection(int inChannelID, int inDetectionID):
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
trackID(0),
threatLevel(eThreatNone) 

{

}

Detection::Detection(int inChannelID, int inDetectionID, float inDistance, float inIntensity, float inVelocity, 
		float inTimeStamp, float inFirstTimeStamp, TrackID inTrackID, ThreatLevel inThreatLevel):
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
threatLevel(inThreatLevel)

{

}

Detection::Detection(int inChannelID, int inDetectionID, ifstream &inTrackFile) :
channelID(inChannelID),
detectionID(inDetectionID)

{
	std:string line;

	if (inTrackFile.is_open())
	{
		uint32_t tmpTimeStamp;
		uint32_t tmpFirstTimeStamp;
		detectionID = inDetectionID;

		getline(inTrackFile, line);
		sscanf(line.c_str(), "%f, %f, %ld, %ld, %ld", 
			&distance, &velocity,
			&tmpTimeStamp,
			&trackID,
			&tmpFirstTimeStamp);

		timeStamp = tmpTimeStamp;
		firstTimeStamp = tmpFirstTimeStamp;
		threatLevel = eThreatNone;
		acceleration = 0.0;
		timeToCollision = NAN;
		probability = 0.0;
	}
}

bool Detection::IsValid()

{
	if (distance != 0.0) 
		return(true);
	
	return(false);
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


bool Track::IsValid()

{
	if (distance != 0.0) 
		return(true);
	
	return(false);
}

 bool Track::Contains(Detection::Ptr & inDetectionPtr)
 {
	 int detectionQty = detections.size();
	 for (int detectionIndex = 0; detectionIndex < detectionQty; detectionIndex++) 
	 {
		 if (detections.at(detectionIndex) == inDetectionPtr) 
		 {
			 return(true);
		 }
	 }

	 return(false);
 }

 bool Track::Contains(int channelID)
 {
	 int detectionQty = detections.size();
	 for (int detectionIndex = 0; detectionIndex < detectionQty; detectionIndex++) 
	 {
		 if (detections.at(detectionIndex)->channelID == channelID) 
		 {
			 return(true);
		 }
	 }

	 return(false);
 }

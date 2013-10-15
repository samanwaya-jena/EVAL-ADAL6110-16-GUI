

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

using namespace std;
using namespace awl;
#if 0
const double defaultDistanceDamping = 0.5; //0.6 // damping to compute the last position of the track. Range 0-1.
const double defaultSpeedDamping =0.6;	// 0.6 //damping to compute the last velocity of the track	
const double defaultTrackTimeOut = (0.5*1000); //0.5 * 10000 // Track inactivity time out (converted to milliseconds) 
const double defaultDistanceThreshold = 0.5; //1-3 or 0.5// Maximum error to correlate, in meters
const double defaultMinimumDistance = 1.0; // Minimum distance at which we consider an obstacle valid.
const double defaultRoundError = 0.5;  // Time difference between two time stamps at which we consider these as same.

const double defaultProbabilityThreshold = 0.75; // Probability threshold for estimation of a valid track.
const double  defaultTrackInitialProbability = 0.3; // Initial probability for a track with single detection.
const double  defaultTrackProbabilityLoss = 0.05;  // Loss of probability for a track that has no detection in a frame
const double  defaultTrackProbabilityGain = 0.3;  // Gain of probability for a track after sucessful detection
#else
const double defaultDistanceDamping = 0.6; //0.6 // damping to compute the last position of the track. Range 0-1.
const double defaultSpeedDamping =0.6;	// 0.6 //damping to compute the last velocity of the track	
const double defaultTrackTimeOut = (1.0*1000); //0.5 * 10000 // Track inactivity time out (converted to milliseconds) 
const double defaultDistanceThreshold = 1.5; //1-3 or 0.5// Maximum error to correlate, in meters
const double defaultMinimumDistance = 1.0; // Minimum distance at which we consider an obstacle valid.
const double defaultRoundError = 0.2;  // Time difference between two time stamps at which we consider these as same.

const double defaultProbabilityThreshold = 0.6; // Probability threshold for estimation of a valid track.
const double  defaultTrackInitialProbability = 0.1; // Initial probability for a track with single detection.
const double  defaultTrackProbabilityLoss = 0.2;  // Loss of probability for a track that has no detection in a frame
const double  defaultTrackProbabilityGain = 0.3;  // Gain of probability for a track after sucessful detection
#endif


const AcquisitionSequence::TrackingMode defaultTrackingMode = AcquisitionSequence::eTrackAllChannels;

#if 0
static const unsigned long __nan[2] = {0xffffffff, 0x7fffffff};
#define NAN (*(const float *) __nan)
#else
const float NAN = std::numeric_limits<float>::quiet_NaN ();
#endif


AcquisitionSequence::AcquisitionSequence(int inSequenceID):
sequenceID(inSequenceID),
frameID(0),
distanceDamping(defaultDistanceDamping),  // damping to compute the last position of the track. Range 0-1.
speedDamping(defaultSpeedDamping),	// damping to compute the last velocity of the track	
trackTimeOut(defaultTrackTimeOut),		// Track inactivity time out (seconds) 
distanceThreshold(defaultDistanceThreshold), // Maximum error to correlate, in meters
minimumDistance (defaultMinimumDistance), // Minimum distance at which we consider an obstacle valid.
trackingMode(defaultTrackingMode)

{
}

AcquisitionSequence::AcquisitionSequence(int inSequenceID, int inChannelQty, int inDetectionQty):
sequenceID(inSequenceID),
channelQty(inChannelQty),
detectionQty(inDetectionQty),
frameID(0),
distanceDamping(defaultDistanceDamping),  // damping to compute the last position of the track. Range 0-1.
speedDamping(defaultSpeedDamping),	// damping to compute the last velocity of the track	
trackTimeOut(defaultTrackTimeOut),		// Track inactivity time out (seconds) 
distanceThreshold(defaultDistanceThreshold), // Maximum error to correlate, in meters
minimumDistance (defaultMinimumDistance), // Minimum distance at which we consider an obstacle valid.
trackingMode(defaultTrackingMode)

{
}

AcquisitionSequence::AcquisitionSequence(int inSequenceID, int inChannelQty, int inDetectionQty, ifstream &inTrackFile):
sequenceID(inSequenceID),
channelQty(inChannelQty),
detectionQty(inDetectionQty),
frameID(0),
distanceDamping(defaultDistanceDamping),  // damping to compute the last position of the track. Range 0-1.
speedDamping(defaultSpeedDamping),	// damping to compute the last velocity of the track	
trackTimeOut(defaultTrackTimeOut),		// Track inactivity time out (seconds) 
distanceThreshold(defaultDistanceThreshold), // Maximum error to correlate, in meters
minimumDistance (defaultMinimumDistance), // Minimum distance at which we consider an obstacle valid.
trackingMode(defaultTrackingMode)

{
	ReadFile(inTrackFile, inChannelQty, inDetectionQty);
}

AcquisitionSequence::AcquisitionSequence(int inSequenceID, int inChannelQty, int inDetectionQty, std::string trackFileName):
sequenceID(inSequenceID),
channelQty(inChannelQty),
detectionQty(inDetectionQty),
frameID(0),
distanceDamping(defaultDistanceDamping),  // damping to compute the last position of the track. Range 0-1.
speedDamping(defaultSpeedDamping),	// damping to compute the last velocity of the track	
trackTimeOut(defaultTrackTimeOut),		// Track inactivity time out (seconds) 
distanceThreshold(defaultDistanceThreshold), // Maximum error to correlate, in meters
minimumDistance (defaultMinimumDistance), // Minimum distance at which we consider an obstacle valid.
trackingMode(defaultTrackingMode)

{
	ReadFile(trackFileName,  inChannelQty, inDetectionQty);
}


AcquisitionSequence::TrackingMode AcquisitionSequence::GetTrackingMode()

{
	return(trackingMode);
}

AcquisitionSequence::TrackingMode AcquisitionSequence::SetTrackingMode(AcquisitionSequence::TrackingMode inTrackingMode)

{
	trackingMode = inTrackingMode;
	return(trackingMode);
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


void AcquisitionSequence::BuildTracks(double inTimeStamp) 
{
	int tracksAdded = 0;

	int frameIndex = sensorFrames.size()-1;

	tracksAdded = 0;
	//For each of the channels (to index the detections)
	for (int channelIndex = 0; channelIndex < channelQty; channelIndex++)
	{
		// For each of the detections in the current frame
		for (int detectionIndex = 0; detectionIndex < detectionQty; detectionIndex++)
		{
			Detection::Ptr detection = this->GetDetectionAtIndex(frameIndex, channelIndex, detectionIndex);

			// Negative or near zero detections are ignored.
			bool bDetectionValid = (detection->distance >= minimumDistance);
			if (bDetectionValid) 
			{
				tracksAdded += FitDetectionToTrack(inTimeStamp, detection);	
			} // if (bDetectionValid)
		} // for detection index
	}// for channelIndex


	// Merge duplicate tracks and kill old tracks
	UpdateTracks(inTimeStamp);
	CleanTracks(inTimeStamp);
}

void AcquisitionSequence::UpdateTracks(double inTimeStamp)

{
	// Update the coaslesced tracks
	int trackQty = tracks.size();
	Track::Vector::iterator  trackIterator = tracks.begin();

	while (trackIterator != tracks.end()) 
	{
		Track::Ptr track = *trackIterator;
		
		// Calculate the average distance for all detections associated to the track;

		double avgDistance = 0;
		int detectionsFound = 0;
		double maxTimeStamp = track->timeStamp;
		std::vector<Detection::Ptr>::iterator detectionIterator = track->detections.begin();

		int detectionQty = track->detections.size();
		while (detectionIterator != track->detections.end())
		{
			Detection::Ptr detection = *detectionIterator;
			if (abs (detection->timeStamp - inTimeStamp) < defaultRoundError) 
			{
				avgDistance += detection->distance;
				detectionsFound++;
				if (detection->timeStamp > maxTimeStamp) maxTimeStamp = detection->timeStamp;
			}

			detectionIterator++;
		}

		// Estimate distance and velocity for the track
		if (!detectionsFound) 
		{
			track->probability -= defaultTrackProbabilityLoss;
			if (track->probability <= 0.0) track->probability = 0.0;
		}
		else // if (detectionsFound) 
		{
			avgDistance /= detectionsFound;
			
			float timeDelta = (inTimeStamp - track->timeStamp);  // Time stamps are in msec
			bool bSameTime = timeDelta <= defaultRoundError;
			timeDelta /= 1000;
	
			// We only revise estimate for tracks that were not created this frame;
			// Tracks created in this frame already have estimate information embedded.

			if (!bSameTime) 
			{
				if (_isnan(track->velocity) && track->detections.size() >= 2)
				{
					track->distance = track->detections[1]->distance;

					track->velocity = (track->detections[1]->distance - track->detections[0]->distance) /
						               (timeDelta);
					track->probability = defaultTrackInitialProbability;
				}
				else 
				{
					float distanceDelta = avgDistance - track->distance;
					track->velocity =  ((distanceDelta / timeDelta) * speedDamping) +
									   ((1-speedDamping) * track->velocity);
					track->distance = (avgDistance * distanceDamping) +
									  ((1 - distanceDamping) * (track->distance + (track->velocity * timeDelta)));
#ifndef DEBUG_JYD
					int debugZ;
					if (track->distance <= 8.0) 
					{
						debugZ++;
					}
#endif

					track->probability += defaultTrackProbabilityGain;
					if (track->probability > 1.0) track->probability = 1.0;
				}
			}

			// tag the track with the most recent update time
			track->timeStamp = inTimeStamp;		
		}	

		// Store estimated results in all the detections

		detectionIterator = track->detections.begin();
		while (detectionIterator != track->detections.end())
		{
			Detection::Ptr detection = *detectionIterator;
			if (detection->timeStamp == inTimeStamp) 
			{
				detection->velocity = track->velocity;
				detection->threatLevel = track->threatLevel;
				detection->trackID = track->trackID;

			}

			detectionIterator++;
		} // while (detectionIterator...

		trackIterator++;
	}// while (trackIterator
}

void AcquisitionSequence::CleanTracks(double inTimeStamp)

{
	int trackQty = tracks.size();
	Track::Vector::iterator  trackIterator = tracks.begin();

	int isExpiredCount = 0;
	int isOldCount = 0;
	int isFastCount = 0;	
	int keepCount = 0;
	int remainingCount = 0;

	while (trackIterator != tracks.end()) 
	{
		Track::Ptr track = *trackIterator;
		bool bSameTime = (inTimeStamp - track->timeStamp) < defaultRoundError;
		bool bIsExpiredTrack =  abs(inTimeStamp - track->timeStamp) > trackTimeOut;
#if 0
		bool bIsOldEmptyTrack = (track->detections.size() <= 1) && !bSameTime;
#else
		bool bIsOldEmptyTrack = (track->detections.size() <= 1) && (abs(inTimeStamp - track->timeStamp) > (0.1*1000));
#endif
		bool bIsTooFastTrack = (!_isnan(track->velocity)) && (track->velocity > 50); // 100 m/s is arbitrary number

		if (bIsExpiredTrack) 
		{
			trackIterator = tracks.erase(trackIterator);
			isExpiredCount++;
		}
		else if (bIsOldEmptyTrack)
		{
			trackIterator = tracks.erase(trackIterator);
			isOldCount++;
		}
#if 1
		else if (bIsTooFastTrack)
		{
DebugFilePrintf("TooFast %u", track->trackID);
			trackIterator = tracks.erase(trackIterator);
			isFastCount++;
		}
#endif
		else
		{
			// Clear all detections from previous frames.
			std::vector<Detection::Ptr>::iterator detectionIterator = track->detections.begin();
			while (detectionIterator != track->detections.end())
			{
				Detection::Ptr detection = *detectionIterator;
				if (abs(detection->timeStamp - inTimeStamp) > trackTimeOut) 
				{
					detectionIterator = track->detections.erase(detectionIterator);
				}
				else
				{
					detectionIterator++;
				}
			
			}
			trackIterator++;
			keepCount++;
		}
	}// while.
	
	remainingCount = tracks.size();
#ifndef DEBUG_JYD
if (remainingCount < 7) {
	int debugK = 32;
}
#endif

}


static int trackIDGenerator = 1;

int AcquisitionSequence::FitDetectionToTrack(double inTimeStamp, Detection::Ptr & detection)
{
	float estimatedTrackDistance = 0;	
	float distanceDelta = 0;
	
	// Scan to see if the detection can be attached to an existing track
	int tracksAdded = 0;
	int trackQty = tracks.size();
	bool bTrackFound = false;

	Track::Vector  tracksToCreate;

	for (int trackIndex = 0; trackIndex < trackQty; trackIndex++) 
	{
		Track::Ptr track = tracks[trackIndex];
		float timeDelta = (detection->timeStamp - track->timeStamp);  
		bool bSameTime = timeDelta <= defaultRoundError;
		timeDelta /= 1000;// Time stamps are in msec. Bring them back in seconds

		// Determine if the track is worthy of consideration for a match
		bool bTrackValid = false;

		if (trackingMode == eTrackSingleChannel) 
		{
			if ((track->Contains(detection->channelID)) && (!track->Contains(detection)))
			{
				if (!bSameTime)	bTrackValid =true;
			}
		}
		else if (trackingMode == eTrackAllChannels)
		{
			if ((track->Contains(detection->channelID)) && (!track->Contains(detection)))
			{
				if (!bSameTime)	bTrackValid =true;
			}
		}

		// If the track is a candidate to receive our detection....
		if (bTrackValid) 
		{
			// The track is a new single-point track. Always spin out(create) a track from it.
			if (_isnan(track->velocity)) 
			{

				Track::Ptr newTrack = CreateTrack(inTimeStamp, tracksToCreate, track->detections[0]);
				newTrack->firstTimeStamp = track->firstTimeStamp;
				newTrack->timeStamp = track->timeStamp;

				newTrack->distance = detection->distance;
#ifndef DEBUG_JYD
				int debugS;
				if (newTrack->distance <= 8.0)
				{
					debugS++;
				}

#endif
#if 0
				newTrack->velocity = detection->velocity;
#else
				newTrack->velocity = (detection->distance - track->detections[0]->distance) /
			  	                   (timeDelta);
#endif
				newTrack->detections.push_back(detection);

				bTrackFound = true;
				tracksAdded++;
			}
			// The track is a complete track.  See if we fit.
			else
			{
				estimatedTrackDistance = track->distance + (track->velocity * timeDelta);
				distanceDelta = detection->distance - estimatedTrackDistance;

#if 0
				if (abs(distanceDelta) <= distanceThreshold)
#else
				double localDistanceThreshold = abs(track->velocity) * timeDelta;
				if (localDistanceThreshold < distanceThreshold) localDistanceThreshold = distanceThreshold;

				if (abs(distanceDelta) <= localDistanceThreshold)
#endif
				{
					track->detections.push_back(detection);
					bTrackFound = true;
				}
			}
		}  // if bTrackValid
	}// for (tracklndex)


	// Create the new tracks that need to be created
	for (int trackIndex = 0; trackIndex < tracksToCreate.size(); trackIndex++)
	{
		tracks.push_back(tracksToCreate[trackIndex]);
	}
#if 1
	// If no track found, add a new track at this point....

	if ((!bTrackFound)) 
#else
	if (true)
#endif
	{
		Track::Ptr track = CreateTrack(inTimeStamp, tracks, detection);
		detection->velocity = track->velocity;
		detection->threatLevel = track->threatLevel;
		detection->trackID = track->trackID;

		tracksAdded++;
	}

	return(tracksAdded);
}


Track::Ptr & AcquisitionSequence::CreateTrack(double inTimeStamp, Track::Vector &trackVector, Detection::Ptr & detection)
{
	
		uint32_t frameID =  this->GetLastFrameID();
		Track::Ptr &track = Track::Ptr(new Track(trackIDGenerator++, detection->distance, (float) 0.0, detection->firstTimeStamp, detection->firstTimeStamp, Detection::eThreatNone));
		track->velocity = NAN;  // NAN - Not a number yet
		track->distance = detection->distance;

		track->timeStamp = detection->firstTimeStamp;
		track->firstTimeStamp = detection->firstTimeStamp;
		trackVector.push_back(track);
		track->detections.push_back(detection);

		// Probability of a track with single detection
		track->probability = defaultTrackInitialProbability;

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

Detection::Ptr & AcquisitionSequence::GetDetectionAtIndex(int frameIndex, int channelIndex, int detectionIndex)

{
	SensorFrame::Ptr sensorFrame;
	if (frameIndex < sensorFrames.size()) 
	{
		sensorFrame = sensorFrames._Get_container().at(frameIndex);
	}
	else 
	{
		// Frame does not exist.  use the lasrt frame
		sensorFrame = sensorFrames.back();
	}
	
	if (channelIndex >= sensorFrame->channelFrames.size()) 
	{
		// Channel does not exist. Return channel 0.
		channelIndex = 0;
	}
	ChannelFrame::Ptr channelFrame = sensorFrame->channelFrames.at(channelIndex);
	
	if (detectionIndex >= channelFrame->detections.size())
	{
		// Detection dows not exist, return detection 0.
		detectionIndex = 0;
	}

	return(channelFrame->detections.at(detectionIndex));
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
		ChannelFrame::Ptr myChannelFrame(new ChannelFrame(channel, inDetectionQty));
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

ChannelFrame::ChannelFrame(int inChannelID):
channelID(inChannelID)
{

}


ChannelFrame::ChannelFrame(int inChannelID, int inDetectionQty) :
channelID(inChannelID)

{
	for (int detectionID = 0; detectionID < inDetectionQty; detectionID++) 
	{
		Detection::Ptr detection = Detection::Ptr(new Detection(inChannelID, detectionID));
		detections.push_back(detection);
	}
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



Detection::Detection(int inChannelID, int inDetectionID):
channelID(inChannelID), 
detectionID(inDetectionID),
distance(0.0),
intensity(0.0),
velocity(0.0),
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
timeStamp(0.0f),
firstTimeStamp(0.0),
trackID(inTrackID),
threatLevel(Detection::eThreatNone) 

{

}

Track::Track(TrackID inTrackID, float inDistance, float inVelocity, 
		float inTimeStamp, float inFirstTimeStamp,  Detection::ThreatLevel inThreatLevel):
trackID(inTrackID),
distance(inDistance),
velocity(inVelocity),
timeStamp(inTimeStamp),
firstTimeStamp(inFirstTimeStamp),
threatLevel(inThreatLevel)

{

}


bool Track::IsValid()

{
	if (distance != 0.0) 
		return(true);
	
	return(false);
}

bool Track::IsProbable()

{
	return (probability >= defaultProbabilityThreshold);
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



#if 0
% ReadMultiple
% reads multiple tekxxxx.csv to form the processing datacube
% returns:
%  Y: (n,m,a) cube containing n samples for the m sensors for a ASCAN
%  X: (n,1) relative distance of samples
%  T: (a,1) relative time of ASCAN

/** \brief Light speed in seconds/meter, round trip */
const double lightSpeed =  6.66928e-9f;

/** \brief time between fpga trigger and begining of laser output (s) */
double triggerT0Delay = 85e-9f;  // time between fpga trigger and begining of laser output (s)

/** \brief max distance to record */
double maxDistance = 35


function [X Y T C Tracks] = ReadMultiple(basedir, baseindex, numtoread)

lightspeed = 6.66928e-9; %s/m round trip
T0 = 85e-9; % time between fpga trigger and begining of laser output (s)
distmax = 35; time between fpga trigger and begining of laser output (s)
numch = 4;
leftcolumn = 'B';
rightcolumn = 'E';

filename = sprintf('%s/tek%04d.csv',basedir,baseindex);
Dataread=csvread(filename,0,0,'B7..B8'); % read the header
sampling = Dataread(2,2);
maxpt = Dataread(3,2);

start = floor(T0/sampling); 
numpt = min(ceil(distmax*lightspeed/sampling),maxpt);
stop = numpt+start;

FIR=[8 4 2 1 1 ];



X = 0:sampling/lightspeed:(numpt-1)*sampling/lightspeed;
T = 0:.01:0.01*(numtoread-1); % lock the time at 10ms per ASCAN
Y = zeros(numch,numpt,numtoread); 
C = zeros(numch,32,numtoread,2); % max 32 candidates
%D = zeros(numch,8,numtoread); % max 8 detections

datarange = sprintf('%c%d..%c%d',leftcolumn,start+16,rightcolumn, stop+16);
for i=0:numtoread-1
    filename = sprintf('%s/tek%04d.csv',basedir,baseindex+i); 
    Dataread=csvread(filename,0,0,datarange); % read the data
    Y(:,:,i+1)=filter(FIR,1,-Dataread(2:numpt+1,2:1+numch).',zeros(1,size(FIR,2)-1),1);
end;


%%% fin de l'acquisition
%%% début du traitement
measureOffset = 2.5;
derivatedamping = 0.5;
secwidth = ceil(13e-9/sampling);

% treshold management made by host?
% fuzzy logic implementation to compute next frame treshold,
% for now implemented on all four channel at the same time but can be
% separated.
treshold = [1 1 1 1]; % adaptative treshold(per channel)...
fuzzypeak = [1 10 22];%[4 16 32 60 100];
fuzzyregion = [4 6 12];%[8 8 10 25 40];
fuzzyratio =[0.75 1.5 2];

for i=0:numtoread-1   
    for j=1:numch
        [~, Ctemp] = DoubleWavelet(X,Y(j,:,i+1),...
                        'display','off',...
                        'firstdamping',derivatedamping,...
                        'secondwidth',secwidth,...
                        'treshold',treshold(j),...
                        'offset',measureOffset);
        n = min(32,size(Ctemp,1));
        C(j,1:n,i+1,:) = permute(Ctemp(1:n,:),[3 1 4 2]);
        
        % Fuzzy computation       
        mult = 0;
        weight = 0;
        for r=1:size(fuzzypeak,2)
            w=(fuzzyregion(r)-abs(n-fuzzypeak(r)))/fuzzyregion(r);
            if w>0
                weight=weight+w;
                mult = mult+(w*fuzzyratio(r));
            end;
        end;
        treshold(j) = treshold(j)*mult/weight; 
    end  
end;


Tracks = Track(C,...
               'maxVelocity',20,'frameRate',10,...
               'posErrMax',1,'velErrMax',2,...
               'distanceDamping',.65,'velDamping',.5,...
               'timeOut',0.6, 'minPos',2.5);
           
 ChannelDisplay(X,Y,T,C,Tracks);
 
 A = Alarm(Tracks);
#endif

#if 0
% ReadMultiple
% reads multiple tekxxxx.csv to form the processing datacube
% returns:
%  Y: (n,m,a) cube containing n samples for the m sensors for a ASCAN
%  X: (n,1) relative distance of samples
%  T: (a,1) relative time of ASCAN


function [X Y T C Tracks] = ReadMultiple(basedir, baseindex, numtoread)

lightspeed = 6.66928e-9; %s/m round trip
T0 = 85e-9; % time between fpga trigger and begining of laser output (s)
distmax = 35; %maximum distance to record (m)
numch = 4;
leftcolumn = 'B';
rightcolumn = 'E';

filename = sprintf('%s/tek%04d.csv',basedir,baseindex);
Dataread=csvread(filename,0,0,'B7..B8'); % read the header
sampling = Dataread(2,2);
maxpt = Dataread(3,2);

start = floor(T0/sampling); 
numpt = min(ceil(distmax*lightspeed/sampling),maxpt);
stop = numpt+start;

FIR=[8 4 2 1 1 ];

X = 0:sampling/lightspeed:(numpt-1)*sampling/lightspeed;
T = 0:.01:0.01*(numtoread-1); % lock the time at 10ms per ASCAN
Y = zeros(numch,numpt,numtoread); 
C = zeros(numch,32,numtoread,2); % max 32 candidates
%D = zeros(numch,8,numtoread); % max 8 detections

datarange = sprintf('%c%d..%c%d',leftcolumn,start+16,rightcolumn, stop+16);
for i=0:numtoread-1
    filename = sprintf('%s/tek%04d.csv',basedir,baseindex+i); 
    Dataread=csvread(filename,0,0,datarange); % read the data
    Y(:,:,i+1)=filter(FIR,1,-Dataread(2:numpt+1,2:1+numch).',zeros(1,size(FIR,2)-1),1);
end;


%%% fin de l'acquisition
%%% début du traitement
measureOffset = 2.5;
derivatedamping = 0.5;
secwidth = ceil(13e-9/sampling);

% treshold management made by host?
% fuzzy logic implementation to compute next frame treshold,
% for now implemented on all four channel at the same time but can be
% separated.
treshold = [1 1 1 1]; % adaptative treshold(per channel)...
fuzzypeak = [1 10 22];%[4 16 32 60 100];
fuzzyregion = [4 6 12];%[8 8 10 25 40];
fuzzyratio =[0.75 1.5 2];

for i=0:numtoread-1   
    for j=1:numch
        [~, Ctemp] = DoubleWavelet(X,Y(j,:,i+1),...
                        'display','off',...
                        'firstdamping',derivatedamping,...
                        'secondwidth',secwidth,...
                        'treshold',treshold(j),...
                        'offset',measureOffset);
        n = min(32,size(Ctemp,1));
        C(j,1:n,i+1,:) = permute(Ctemp(1:n,:),[3 1 4 2]);
        
        % Fuzzy computation       
        mult = 0;
        weight = 0;
        for r=1:size(fuzzypeak,2)
            w=(fuzzyregion(r)-abs(n-fuzzypeak(r)))/fuzzyregion(r);
            if w>0
                weight=weight+w;
                mult = mult+(w*fuzzyratio(r));
            end;
        end;
        treshold(j) = treshold(j)*mult/weight; 
    end  
end;


Tracks = Track(C,...
               'maxVelocity',20,'frameRate',10,...
               'posErrMax',1,'velErrMax',2,...
               'distanceDamping',.65,'velDamping',.5,...
               'timeOut',0.6, 'minPos',2.5);
           
 ChannelDisplay(X,Y,T,C,Tracks);
 
 A = Alarm(Tracks);
    
    

#endif

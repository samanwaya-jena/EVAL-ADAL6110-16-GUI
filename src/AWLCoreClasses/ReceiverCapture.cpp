/* ReceiverCapture.cpp */
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

#ifndef Q_MOC_RUN
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#endif

#include <fstream>
#include <limits>


#include "Publisher.h"
#include "ThreadedWorker.h"
#include "ReceiverCapture.h"
#include "DetectionStruct.h"
#include "DebugPrintf.h"

using namespace std;
using namespace awl;

const int ReceiverCapture::maximumSensorFrames(100);
// Sensor transitions going from left to right...

const std::string sDefaultReceiverType = "Generic";
const std::string sDefaultReceiverRegisterSet = "registerDescription_RevC";
const std::string sDefaultReceiverChannelGeometry = "GeometryAWL7";
const uint8_t defaultChannelMaskValue = 127;
const float defaultSignalToNoiseFloor = -10.0;


ReceiverCapture::ReceiverCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround,
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, 
					   const AlgorithmSet &inParametersAlgos,
					   const AlgorithmSet &inParametersTrackers):
ThreadedWorker(),
Publisher(),
receiverID(receiverID),
receiverChannelQty(inReceiverChannelQty),
receiverColumnQty(inReceiverColumns),
receiverRowQty(inReceiverRows), 
lineWrapAround(inLineWrapAround),
acquisitionSequence(new AcquisitionSequence()),
frameID(0),
currentFrame(new SensorFrame(receiverID, 0, inReceiverChannelQty)),
measurementOffset(inRangeOffset),
bFrameInvalidated(false),
registersFPGA(inRegistersFPGA),
registersADC(inRegistersADC),
registersGPIO(inRegistersGPIO),
parametersAlgos(inParametersAlgos),
parametersTrackers(inParametersTrackers),
sReceiverType(sDefaultReceiverType),
sReceiverRegisterSet(sDefaultReceiverRegisterSet),
sReceiverChannelGeometry(sDefaultReceiverChannelGeometry),
targetHintDistance(0.0),
targetHintAngle(0.0)

{
	// Initialize default status values
	InitStatus();

	receiverStatus.frameRate = inFrameRate;
	receiverStatus.currentAlgo = 0;
	receiverStatus.currentAlgoPendingUpdates = 0;

	receiverStatus.currentTracker = 0;
	receiverStatus.currentTrackerPendingUpdates = 0;


	// Update settings from application
	receiverStatus.frameRate = inFrameRate;
	receiverStatus.channelMask = inChannelMask;
	receiverStatus.messageMask = inMessageMask;

	// Reflect the settings in hardware
	SetMessageFilters(receiverStatus.frameRate, receiverStatus.channelMask, receiverStatus.messageMask);
}

ReceiverCapture::ReceiverCapture(int receiverID, boost::property_tree::ptree &propTree):
ThreadedWorker(),
Publisher(),
receiverID(receiverID),
acquisitionSequence(new AcquisitionSequence()),
frameID(0),
bFrameInvalidated(false),
sReceiverType(sDefaultReceiverType),
sReceiverRegisterSet(sDefaultReceiverRegisterSet),
sReceiverChannelGeometry(sDefaultReceiverChannelGeometry),
targetHintDistance(0.0),
targetHintAngle(0.0)


{
	// Read the configuration from the configuration file

	receiverStatus.signalToNoiseFloor = defaultSignalToNoiseFloor;

	ReadConfigFromPropTree(propTree);
	ReadGeometryFromPropTree(propTree);
	ReadRegistersFromPropTree(propTree);

	// Initialize default status values
	InitStatus();

	// make sure that the communication is reset.
	receiverStatus.currentAlgoPendingUpdates = 0;
	receiverStatus.currentTrackerPendingUpdates = 0;

	// Create a temporary SensorFrame object for storage of the current data
	currentFrame = SensorFrame::Ptr(new SensorFrame(receiverID, 0, receiverChannelQty));

	// Reflect the settings in hardware
	SetMessageFilters(receiverStatus.frameRate, receiverStatus.channelMask, receiverStatus.messageMask);
}

ReceiverCapture::~ReceiverCapture()
{
	EndDistanceLog();
	if (!WasStopped()) Stop();
}

void  ReceiverCapture::Go() 
{
 	assert(!mThread);
	mWorkerRunning = true;
	startTime = boost::posix_time::microsec_clock::local_time();
	mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverCapture::DoThreadLoop, this)));
}

void ReceiverCapture::DoThreadLoop()

{
	while (!WasStopped())
    {
		DoOneThreadIteration();
	} // while (!WasStoppped)
}

void ReceiverCapture::InitStatus()
{
	receiverStatus.bUpdated = false;
	receiverStatus.temperature = 0.0;
	receiverStatus.voltage = 0;
	receiverStatus.hardwareError.byteData = 0;
	receiverStatus.receiverError.byteData = 0;
	receiverStatus.status.byteData = 0;
	receiverStatus.version.major = 0;
	receiverStatus.version.minor = 0;
	receiverStatus.bootChecksumError.byteData = 0;
	receiverStatus.bootSelfTest.byteData = 0;

	receiverStatus.bInPlayback = false;
	receiverStatus.bInRecord = false;
	receiverStatus.sPlaybackFileName = "";
	receiverStatus.sRecordFileName = "";

	receiverStatus.lastCommandError = 0; 
	receiverStatus.fpgaRegisterAddressRead = 0;
	receiverStatus.fpgaRegisterValueRead = 0;
	receiverStatus.adcRegisterAddressRead = 0;
	receiverStatus.adcRegisterValueRead = 0;
	receiverStatus.gpioRegisterAddressRead = 0;
	receiverStatus.gpioRegisterValueRead = 0;

	receiverStatus.signalToNoiseFloor = defaultSignalToNoiseFloor;
}


int ReceiverCapture::GetFrameQty()

{
	return acquisitionSequence->sensorFrames.size();
}


bool ReceiverCapture::CopyReceiverFrame(FrameID inFrameID,  SensorFrame::Ptr &outSensorFrame, Publisher::SubscriberID inSubscriberID)
{
	if (!LockNews(inSubscriberID)) return(false);

	SensorFrame::Ptr sensorFrame;
	bool bFound = acquisitionSequence->FindSensorFrame(inFrameID, sensorFrame);
	if (bFound) 
	{ 
		*outSensorFrame = *sensorFrame;
	}

	UnlockNews(inSubscriberID);
	return(bFound);
}

bool ReceiverCapture::CopyReceiverRawDetections(FrameID inFrameID,  Detection::Vector &outDetections, Publisher::SubscriberID inSubscriberID)
{
	if (!LockNews(inSubscriberID)) return(false);

	SensorFrame::Ptr sensorFrame;
	bool bFound = acquisitionSequence->FindSensorFrame(inFrameID, sensorFrame);
	if (bFound) 
	{ 
		outDetections.clear();
		outDetections = sensorFrame->rawDetections;
	}

	UnlockNews(inSubscriberID);
	return(bFound);
}




bool ReceiverCapture::CopyReceiverStatusData(ReceiverStatus &outStatus, Publisher::SubscriberID inSubscriberID)
{
	boost::mutex::scoped_lock updateLock(GetMutex()); 

	outStatus = receiverStatus;
	receiverStatus.bUpdated = false;
	updateLock.unlock();
	return(true);
}

FrameID ReceiverCapture::GetFrameID(uint16_t  inFrameIndex)
{
	boost::mutex::scoped_lock updateLock(GetMutex());
	FrameID frameID;

	if (inFrameIndex > (int) acquisitionSequence->sensorFrames.size()-1) frameID = 0xFFFFFFFF;
	else 
	{ 
		SensorFrame::Ptr sensorFrame = acquisitionSequence->sensorFrames.at(inFrameIndex);
		frameID = sensorFrame->frameID;
	}

	updateLock.unlock();
	return(frameID);
}

void ReceiverCapture::SetMeasurementOffset(double inMeasurementOffset)
{
	measurementOffset = inMeasurementOffset;
}

void ReceiverCapture::GetMeasurementOffset(double &outMeasurementOffset)
{
	outMeasurementOffset = measurementOffset;
}

bool ReceiverCapture::SetPlaybackFileName(std::string inPlaybackFileName)
{
	receiverStatus.sPlaybackFileName = inPlaybackFileName;
	return(true);
}

bool ReceiverCapture::SetRecordFileName(std::string inRecordFileName)
{
	receiverStatus.sRecordFileName = inRecordFileName;
	return(true);
}

bool ReceiverCapture::StartPlayback(uint8_t frameRate, ChannelMask channelMask)
{
	receiverStatus.bInPlayback = true;
	return(true);
}

bool ReceiverCapture::StartRecord(uint8_t frameRate, ChannelMask channelMask)
{
	receiverStatus.bInRecord = true;
	return(true);
}

bool ReceiverCapture::StopPlayback()
{
	receiverStatus.bInPlayback = false;
	return(true);
}
	
bool ReceiverCapture::StopRecord()
{
	receiverStatus.bInRecord = false;
	return(true);
}


bool ReceiverCapture::SetMessageFilters(uint8_t frameRate, ChannelMask channelMask, MessageMask messageMask)

{
   return(true);
}

void ReceiverCapture::DoOneThreadIteration()

{
	// As the ReceiverCapture class is a virtual class, The code below means nothing.
	if (!WasStopped())
    {
		boost::mutex::scoped_lock rawLock(GetMutex());
		frameID++;
		rawLock.unlock();

			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	} // while (!WasStoppped)
}

double ReceiverCapture::GetElapsed()

{
	boost::posix_time::ptime nowTime(boost::posix_time::microsec_clock::local_time());
	boost::posix_time::time_duration msdiff = nowTime - startTime;
    return(msdiff.total_microseconds() / 1000.0);
}

void ReceiverCapture::InvalidateFrame()
{
	boost::mutex::scoped_lock rawLock(GetMutex());
	bFrameInvalidated = true;
	rawLock.unlock();
}

void ReceiverCapture::ProcessCompletedFrame()

{
	boost::mutex::scoped_lock rawLock(GetMutex());

	// timestamp the currentFrame
	double elapsed = GetElapsed();

	currentFrame->timeStamp = elapsed;

	// TimeStamp all detections and tracks
	TimestampTracks(currentFrame);

	TimestampDetections(currentFrame);

	// Log Tracks?
	if (receiverStatus.messageMask.bitFieldData.obstacle)
	{
		LogTracks(logFile, currentFrame);
	}

	// Log Distances?
	if (receiverStatus.messageMask.bitFieldData.distance_1_4 ||
	    receiverStatus.messageMask.bitFieldData.distance_5_8 || 
		receiverStatus.messageMask.bitFieldData.intensity_1_4 ||
		receiverStatus.messageMask.bitFieldData.intensity_5_8 ||
		receiverStatus.messageMask.bitFieldData.distance_intensity)
	{
		LogDistances(logFile, currentFrame);
	}

	FrameID completedFrameID = currentFrame->GetFrameID();
	bool bFrameToBePublished = false;
	if (!bFrameInvalidated)
	{
		// Push the current frame in the frame buffer
		acquisitionSequence->sensorFrames.push_front(currentFrame);
	
		// Make sure we do not keep too many of those frames around.
		// Remove the older frame if we exceed the buffer capacity
		if (acquisitionSequence->sensorFrames.size() > maximumSensorFrames) 
		{
			acquisitionSequence->sensorFrames.pop_back();
		}

		bFrameToBePublished = true;
	}
	
	// Create a new current frame.
	FrameID frameID = acquisitionSequence->AllocateFrameID();
	currentFrame = SensorFrame::Ptr(new SensorFrame(receiverID, frameID, receiverChannelQty));
	bFrameInvalidated = false;

	rawLock.unlock();

	if (bFrameToBePublished) 
	{
		PutNews(completedFrameID);
		DebugFilePrintf(debugFile, "FrameID- %lu", completedFrameID);
	}
	else 
	{
		DebugFilePrintf(debugFile, "FrameIDUnpublished- %lu", completedFrameID);
	}

}


void ReceiverCapture::TimestampTracks(SensorFrame::Ptr sourceFrame)
{
	BOOST_FOREACH(Track::Ptr &track, sourceFrame->tracks)
	{
		track->firstTimeStamp = currentFrame->timeStamp;
		track->timeStamp = currentFrame->timeStamp;
	} // BOOST_FOREACH(Track::Ptr track
} 

void ReceiverCapture::TimestampDetections(SensorFrame::Ptr sourceFrame)
{
	BOOST_FOREACH(Detection::Ptr &rawDetectionPtr, sourceFrame->rawDetections)
	{
		rawDetectionPtr->firstTimeStamp = currentFrame->timeStamp;
		rawDetectionPtr->timeStamp = currentFrame->timeStamp;
	} // BOOST_FOREACH(Detection::Ptr rawDetectionPtr
} 

int ReceiverCapture::FindRegisterByAddress(const RegisterSet &inRegisterSet, uint16_t inAddress)

{
	for (uint16_t i = 0; i < inRegisterSet.size(); i++) 
	{
		if (inRegisterSet.at(i).address == inAddress)
		{
			return(i);
		}
	}

	return(-1);
}

AlgorithmDescription * ReceiverCapture::FindAlgoDescriptionByID(AlgorithmSet &inAlgoSet, int inAlgoID)
{
	for (uint16_t i = 0; i < inAlgoSet.algorithms.size(); i++)
	{
		if (inAlgoSet.algorithms.at(i).algoID == inAlgoID)
		{
			return(&inAlgoSet.algorithms[i]);
		}
	}

	return(NULL);
}

AlgorithmParameter * ReceiverCapture::FindAlgoParamByAddress(int inAlgoID, uint16_t inAddress)
{

	AlgorithmDescription *algoDescription = FindAlgoDescriptionByID(parametersAlgos, inAlgoID);
	if (algoDescription == NULL) return NULL;

	for (uint16_t i = 0; i < algoDescription->parameters.size(); i++) 	
	{
		if (algoDescription->parameters[i].address == inAddress)
		{
			return(&algoDescription->parameters[i]);
		}
	}

	return(NULL);
}

AlgorithmParameter * ReceiverCapture::FindTrackerParamByAddress(int inTrackerID, uint16_t inAddress)
{
	AlgorithmDescription *algoDescription = FindAlgoDescriptionByID(parametersTrackers, inTrackerID);
	if (algoDescription == NULL) return NULL;

	for (uint16_t i = 0; i < algoDescription->parameters.size(); i++)
	{
		if (algoDescription->parameters[i].address == inAddress)
		{
			return(&algoDescription->parameters[i]);
		}
	}

	return(NULL);
}

bool ReceiverCapture::BeginDistanceLog()

{
	if (!logFile.is_open())
	{
		OpenLogFile(logFile, "DistanceLog.dat", true);
	}

	LogFilePrintf(logFile, "Start distance log");
	// Title Line
	LogFilePrintf(logFile, "Track Description:;Track;trackID;_;__;___;Expected;expectDistance;expectAngle;Val;distance;intensity;velocity;acceleration;ttc;decelerationToStop;probability;ThreatLevel;Ch.0;Ch.1;Ch.2;Ch.3;Ch.4;Ch.5;Ch.6;");
	LogFilePrintf(logFile, "Distance Description:;Dist;_;__;Channel;DetectionID;Expected;expectDistance;expectAngle;Val;distance;intensity;velocity;acceleration;ttc;decelerationToStop;probability;ThreatLevel");
	
	return(true);
}


bool ReceiverCapture::EndDistanceLog()

{
	if (logFile.is_open())
		CloseLogFile(logFile);
	return(false);
}


void ReceiverCapture::LogTracks(ofstream &logFile, SensorFrame::Ptr sourceFrame)
{
	// Update the coaslesced tracks
   Track::Vector::iterator  trackIterator = sourceFrame->tracks.begin();

	while (trackIterator != sourceFrame->tracks.end()) 
	{
		Track::Ptr track = *trackIterator;
		if (track->IsComplete()) 
		{
			//Date;Comment (empty);"TrackID", "Track"/"Dist";TrackID;"Channel";....Val;distance;intensity,speed;acceleration;probability;timeToCollision);
			LogFilePrintf(logFile, " ;Track;%d; ; ; ;Expected;%.2f;%.1f;Val;%.2f;%.1f;%.1f;%.1f;%.3f;%.1f;%.0f;%d;%d;%d;%d;%d;%d;%d;%d;%d;",
				track->trackID,
				targetHintDistance,
				targetHintAngle,
				track->distance,
				track->intensity,
				track->velocity, 
				track->acceleration, 
				track->timeToCollision,
				track->decelerationToStop,
				track->probability,
				track->threatLevel,
				track->trackMainChannel, 
				(track->trackChannels.bitFieldData.channel0)? 0 : 0, 
				(track->trackChannels.bitFieldData.channel1) ? 1 : 0,
				(track->trackChannels.bitFieldData.channel2) ? 2 : 0,
				(track->trackChannels.bitFieldData.channel3) ? 3 : 0,
				(track->trackChannels.bitFieldData.channel4) ? 4 : 0,
				(track->trackChannels.bitFieldData.channel5) ? 5 : 0,
				(track->trackChannels.bitFieldData.channel6) ? 6 : 0);

		}  // if (track...

		trackIterator++;
	} // while (trackIterator...
} 

void ReceiverCapture::LogDistances(ofstream &logFile, SensorFrame::Ptr sourceFrame)
{
	Detection::Vector::iterator  detectionIterator = sourceFrame->rawDetections.begin();
	while (detectionIterator != sourceFrame->rawDetections.end()) 
	{
		Detection::Ptr detection = *detectionIterator;
		LogFilePrintf(logFile, " ;Dist;;Channel;%d;%d;Expected;%.2f;%.1f;Val;%.2f;%.1f;%.2f;%.1f;%.3f;%.1f;%.0f;%d",
			detection->channelID, 
			detection->detectionID,
			targetHintDistance,
			targetHintAngle,
			detection->distance,
			detection->intensity,
			detection->velocity, 
			detection->acceleration, 
			detection->timeToCollision,
			detection->decelerationToStop,
			detection->probability,
			detection->threatLevel);

		detectionIterator++;
	}
}

// Configuration file related functions


bool ReceiverCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
		char receiverKeyString[32];
		sprintf(receiverKeyString, "config.receivers.receiver%d", receiverID);
		std::string receiverKey = receiverKeyString;

		boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);

		sReceiverType = receiverNode.get<std::string>("receiverType");
		sReceiverRegisterSet = receiverNode.get<std::string>("receiverRegisterSet");
		sReceiverChannelGeometry = receiverNode.get<std::string>("receiverChannelGeometry");
		measurementOffset = receiverNode.get<float>("rangeOffset");

		receiverStatus.frameRate =  receiverNode.get<uint8_t>("frameRate");	// Default frame rate is 100Hz

		receiverStatus.channelMask.byteData = receiverNode.get<uint8_t>("channelMask");

		receiverStatus.messageMask.byteData = 0;
		if (receiverNode.get<bool>("msgEnableObstacle")) receiverStatus.messageMask.bitFieldData.obstacle = 1;
		if (receiverNode.get<bool>("msgEnableDistance_1_4")) receiverStatus.messageMask.bitFieldData.distance_1_4 = 1;
		if (receiverNode.get<bool>("msgEnableDistance_5_8")) receiverStatus.messageMask.bitFieldData.distance_5_8 = 1;
		if (receiverNode.get<bool>("msgEnableIntensity_1_4")) receiverStatus.messageMask.bitFieldData.intensity_1_4 = 1;
		if (receiverNode.get<bool>("msgEnableIntensity_5_8")) receiverStatus.messageMask.bitFieldData.intensity_5_8 = 1;
		if (receiverNode.get<bool>("msgEnableDistanceIntensity")) receiverStatus.messageMask.bitFieldData.distance_intensity = 1;
		if (receiverNode.get<bool>("msgEnableObstacleCompact")) receiverStatus.messageMask.bitFieldData.obstacle_compact = 1;

		return(true);

}

bool ReceiverCapture::ReadGeometryFromPropTree(boost::property_tree::ptree &propTree)
{
	using boost::property_tree::ptree;

	// Read the geometry
	std::string geometryDescKey = "config." + sReceiverChannelGeometry;

	// The geometry configuration section may be absent from the configuration.
	// This is considered a normal situation.
	boost::property_tree::ptree *geometryNodePtr = NULL;

	try
	{
		// The register configuration section may be absent from the configuration.
		// This is considered a normal situation.
		geometryNodePtr = &propTree.get_child(geometryDescKey);
	}
	catch (boost::exception &e)
	{
		return (false);
	}

	// Geometry section is found.  Check if we have a channel based configuration or
	// array based configuration;

	lineWrapAround = geometryNodePtr->get<float>("lineWrapAround", -1.0);
	receiverChannelQty = geometryNodePtr->get<int>("channelQty", -1);
	receiverColumnQty = receiverChannelQty;
	receiverRowQty = 1;
	if (receiverChannelQty == -1)
	{
		float columns(0);
		float rows(0);
		receiverColumnQty = geometryNodePtr->get<int>("arraySize.x", -1);
		receiverRowQty = geometryNodePtr->get<int>("arraySize.y", -1);
		receiverChannelQty = ((int)receiverColumnQty) * ((int)receiverRowQty);
	}

	return(true);
}

bool ReceiverCapture::ReadRegistersFromPropTree(boost::property_tree::ptree &propTree)
{
	return(true);
}


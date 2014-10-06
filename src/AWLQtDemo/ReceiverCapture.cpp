
#ifndef Q_MOC_RUN
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#include <fstream>

#include "AWLSettings.h"
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
const int defaultFrameRate(50);
const uint8_t defaultChannelMaskValue = 127;


ReceiverCapture::ReceiverCapture(int receiverID, int inReceiverChannelQty, 
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, const AlgorithmSet &inParametersAlgos):
ThreadedWorker(),
Publisher(),
receiverID(receiverID),
receiverChannelQty(inReceiverChannelQty),
acquisitionSequence(new AcquisitionSequence()),
frameID(0),
currentFrame(new SensorFrame(receiverID, 0, inReceiverChannelQty)),
measurementOffset(inRangeOffset),
trackIDGenerator(0),
bFrameInvalidated(false),
registersFPGA(inRegistersFPGA),
registersADC(inRegistersADC),
registersGPIO(inRegistersGPIO),
parametersAlgos(inParametersAlgos),
sReceiverType(sDefaultReceiverType)

{
	// Initialize default status values
	InitStatus();

	receiverStatus.frameRate = inFrameRate;
	receiverStatus.currentAlgo = 0;
	receiverStatus.currentAlgoPendingUpdates = 0;

	// Update settings from application
	receiverStatus.frameRate = inFrameRate;
	receiverStatus.channelMask = inChannelMask;
	receiverStatus.messageMask = inMessageMask;

	// Reflect the settings in hardware
	SetMessageFilters();
}

ReceiverCapture::ReceiverCapture(int receiverID, boost::property_tree::ptree &propTree):
ThreadedWorker(),
Publisher(),
receiverID(receiverID),
acquisitionSequence(new AcquisitionSequence()),
frameID(0),
trackIDGenerator(0),
bFrameInvalidated(false),
sReceiverType(sDefaultReceiverType)

{
	// Read the configuration from the configuration file
	ReadConfigFromPropTree(propTree);
	ReadRegistersFromPropTree(propTree);

	// Initialize default status values
	InitStatus();

	// make sure that the communication is reset.
	receiverStatus.currentAlgoPendingUpdates = 0;

	// Create a temporary SensorFrame object for storage of the current data
	currentFrame = SensorFrame::Ptr(new SensorFrame(receiverID, 0, receiverChannelQty));

	// Reflect the settings in hardware
	SetMessageFilters();
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
}


int ReceiverCapture::GetFrameQty()

{
	return acquisitionSequence->sensorFrames.size();
}


bool ReceiverCapture::CopyReceiverFrame(uint32_t inFrameID,  SensorFrame::Ptr &outSensorFrame, Publisher::SubscriberID inSubscriberID)
{
	if (!LockNews(inSubscriberID)) return(false);

	SensorFrame::Ptr sensorFrame;
	bool bFound = acquisitionSequence->FindSensorFrame(inFrameID, sensorFrame);
	if (bFound) 
	{ 
#if 1
		*outSensorFrame = *sensorFrame;
#else
		int detectionQty = sensorFrame->enhancedDetections.size();
		for (int i = 0; i< detectionQty; i++) 
		{
 			outDetections.push_back(sensorFrame->enhancedDetections[i]);
		}	
#endif
	}

	UnlockNews(inSubscriberID);
	return(bFound);
}

bool ReceiverCapture::CopyReceiverRawDetections(uint32_t inFrameID,  Detection::Vector &outDetections, Publisher::SubscriberID inSubscriberID)
{
	if (!LockNews(inSubscriberID)) return(false);

	SensorFrame::Ptr sensorFrame;
	bool bFound = acquisitionSequence->FindSensorFrame(inFrameID, sensorFrame);
	if (bFound) 
	{ 
		outDetections.clear();
#if 1
		outDetections = sensorFrame->rawDetections;
#else
		int detectionQty = sensorFrame->rawDetections.size();
		for (int i = 0; i< detectionQty; i++) 
		{
 			outDetections.push_back(sensorFrame->rawDetections[i]);
		}	
#endif
	}

	UnlockNews(inSubscriberID);
	return(bFound);
}


bool ReceiverCapture::CopyReceiverEnhancedDetections(uint32_t inFrameID,  Detection::Vector &outDetections, Publisher::SubscriberID inSubscriberID)
{
	if (!LockNews(inSubscriberID)) return(false);

	SensorFrame::Ptr sensorFrame;
	bool bFound = acquisitionSequence->FindSensorFrame(inFrameID, sensorFrame);
	if (bFound) 
	{ 
		outDetections.clear();
#if 1
		outDetections = sensorFrame->enhancedDetections;
#else
		int detectionQty = sensorFrame->enhancedDetections.size();
		for (int i = 0; i< detectionQty; i++) 
		{
 			outDetections.push_back(sensorFrame->enhancedDetections[i]);
		}	
#endif
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

uint32_t ReceiverCapture::GetFrameID(int  inFrameIndex)
{
	boost::mutex::scoped_lock updateLock(GetMutex());
	uint32_t frameID;

	if (inFrameIndex > acquisitionSequence->sensorFrames.size()-1) frameID = 0xFFFFFFFF;
	else 
	{ 
		SensorFrame::Ptr sensorFrame = acquisitionSequence->sensorFrames._Get_container().at(inFrameIndex);
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


bool ReceiverCapture::SetMessageFilters()

{

	return(SetMessageFilters(receiverStatus.frameRate, receiverStatus.channelMask, receiverStatus.messageMask));
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

	currentFrame->timeStamp = GetElapsed();
#if 0
	// Complete the track information that is not yet processed at the module level.
	if (!acquisitionSequence->CompleteTrackInfo(currentFrame))
	{
		DebugFilePrintf(debugFile, "Incomplete frame in UpdateTrackInfo- %lu", frameID);
		bFrameInvalidated = true;  // Don't call InvalidateFrame() because of the lock contention.
	}

	// Build distances from the tracks that were accumulated during the frame
	if (!acquisitionSequence->BuildEnhancedDetectionsFromTracks(currentFrame))
	{
		DebugFilePrintf(debugFile, "Incomplete frame- %lu", frameID);
		bFrameInvalidated = true;  // Don't call InvalidateFrame() because of the lock contention.
	}
#endif

	// Log Tracks?
	if (receiverStatus.messageMask.bitFieldData.obstacle)
	{
		LogTracks(logFile, currentFrame);
	}

	// Log Distances?
	if (receiverStatus.messageMask.bitFieldData.distance_1_4 ||
	    receiverStatus.messageMask.bitFieldData.distance_5_8 || 
		receiverStatus.messageMask.bitFieldData.intensity_1_4 ||
		receiverStatus.messageMask.bitFieldData.intensity_5_8)
	{
		LogDistances(logFile, currentFrame);
	}

	uint32_t completedFrameID = currentFrame->GetFrameID();
	bool bFrameToBePublished = false;
	if (!bFrameInvalidated)
	{
		// Push the current frame in the frame buffer
		acquisitionSequence->sensorFrames.push(currentFrame);
	
		// Make sure we do not keep too many of those frames around.
		// Remove the older frame if we exceed the buffer capacity
		if (acquisitionSequence->sensorFrames.size() > maximumSensorFrames) 
		{
			acquisitionSequence->sensorFrames.pop();
		}

		bFrameToBePublished = true;
	}
	
	// Create a new current frame.
	uint32_t frameID = acquisitionSequence->AllocateFrameID();
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

int ReceiverCapture::FindRegisterByAddress(const RegisterSet &inRegisterSet, uint16_t inAddress)

{
	for (int i = 0; i < inRegisterSet.size(); i++) 
	{
		if (inRegisterSet.at(i).address == inAddress)
		{
			return(i);
		}
	}

	return(-1);
}

AlgorithmParameter * ReceiverCapture::FindAlgoParamByAddress(int inAlgoID, uint16_t inAddress)
{

	for (int i = 0; i < parametersAlgos.algorithms[inAlgoID].parameters.size(); i++) 	
	{
		if ( parametersAlgos.algorithms[inAlgoID].parameters[i].address == inAddress)
		{
			return(&parametersAlgos.algorithms[inAlgoID].parameters[i]);
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
	LogFilePrintf(logFile, "Track Description:;Track;trackID;_;__;___;Expected;expectDistance;expectAngle;Val;distance;____;velocity;acceleration;ttc;decelerationToStop;probability;ThreatLevel;Ch.0;Ch.1;Ch.2;Ch.3;Ch.4;Ch.5;Ch.6;");
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
	AWLSettings *settings = AWLSettings::GetGlobalSettings();

	// Update the coaslesced tracks
   Track::Vector::iterator  trackIterator = sourceFrame->tracks.begin();

	while (trackIterator != sourceFrame->tracks.end()) 
	{
		Track::Ptr track = *trackIterator;
		if (track->IsComplete()) 
		{
			//Date;Comment (empty);"TrackID", "Track"/"Dist";TrackID;"Channel";....Val;distance;speed;acceleration;probability;timeToCollision);
			LogFilePrintf(logFile, " ;Track;%d; ; ; ;Expected;%.2f;%.1f;Val;%.2f; ;%.1f;%.1f;%.3f;%.1f;%.0f;%d;%d;%d;%d;%d;%d;%d;%d;",
				track->trackID,
				AWLSettings::GetGlobalSettings()->targetHintDistance,
				AWLSettings::GetGlobalSettings()->targetHintAngle,
				track->distance,
				track->velocity, 
				track->acceleration, 
				track->timeToCollision,
				track->decelerationToStop,
				track->probability,
				track->threatLevel,
				(track->channels & 0x01)? 0 : 0, 
				(track->channels & 0x02)? 1 : 0, 
				(track->channels & 0x04)? 2 : 0, 
				(track->channels & 0x08)? 3 : 0, 
				(track->channels & 0x10)? 4 : 0, 
				(track->channels & 0x20)? 5 : 0, 
				(track->channels & 0x40)? 6 : 0);

		}  // if (track...

		trackIterator++;
	} // while (trackIterator...
} 

void ReceiverCapture::LogDistances(ofstream &logFile, SensorFrame::Ptr sourceFrame)
{
	AWLSettings *settings = AWLSettings::GetGlobalSettings();

	Detection::Vector::iterator  detectionIterator = sourceFrame->rawDetections.begin();
	while (detectionIterator != sourceFrame->rawDetections.end()) 
	{
		Detection::Ptr detection = *detectionIterator;
		LogFilePrintf(logFile, " ;Dist;;Channel;%d;%d;Expected;%.2f;%.1f;Val;%.2f;%.1f;%.2f;%.1f;%.3f;%.1f;%.0f;%d",
			detection->channelID, 
			detection->detectionID,
			AWLSettings::GetGlobalSettings()->targetHintDistance,
			AWLSettings::GetGlobalSettings()->targetHintAngle,
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
		measurementOffset = receiverNode.get<float>("rangeOffset");
		receiverChannelQty = receiverNode.get<int>("channelQty");
		receiverStatus.frameRate =  receiverNode.get<uint8_t>("frameRate");	// Default frame rate is 100Hz

		receiverStatus.channelMask.byteData = receiverNode.get<uint8_t>("channelMask");

		receiverStatus.messageMask.byteData = 0;
		if (receiverNode.get<bool>("msgEnableObstacle")) receiverStatus.messageMask.bitFieldData.obstacle = 1;
		if (receiverNode.get<bool>("msgEnableDistance_1_4")) receiverStatus.messageMask.bitFieldData.distance_1_4 = 1;
		if (receiverNode.get<bool>("msgEnableDistance_5_8")) receiverStatus.messageMask.bitFieldData.distance_5_8 = 1;
		if (receiverNode.get<bool>("msgEnableIntensity_1_4")) receiverStatus.messageMask.bitFieldData.intensity_1_4 = 1;
		if (receiverNode.get<bool>("msgEnableIntensity_5_8")) receiverStatus.messageMask.bitFieldData.intensity_5_8 = 1;

		return(true);

}

bool ReceiverCapture::ReadRegistersFromPropTree(boost::property_tree::ptree &propTree)
{
	return(true);
}


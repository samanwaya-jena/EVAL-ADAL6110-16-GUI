
#ifndef Q_MOC_RUN
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#include <fstream>

#include "AWLSettings.h"
#include "tracker.h"
#include "ReceiverCapture.h"
#include "DebugPrintf.h"

using namespace std;
using namespace awl;

const int ReceiverCapture::maximumSensorFrames(100);
// Sensor transitions going from left to right...

const int channelTransitionQty(13);
const int channelTransitions[channelTransitionQty][2] =
{
	{0, -1},
	{0, 1},
	{1, -1},
	{1, 4},
	{1, -1},
	{1, 5},
	{5, -1},
	{2, 5},
	{2, -1},
	{2, 6},
	{2, -1},
	{2, 3},
	{3, -1}
};

ReceiverCapture::ReceiverCapture(int inReceiverID, int sequenceID, int inReceiverChannelQty):
receiverID(inReceiverID),
receiverChannelQty(inReceiverChannelQty),
acquisitionSequence(new AcquisitionSequence(inReceiverID, sequenceID, inReceiverChannelQty)),
frameID(0),
snapshotFrameID(0),
currentFrame(new SensorFrame(inReceiverID, 0, inReceiverChannelQty)),
currentReceiverCaptureSubscriptions(new(Subscription)),
bIsThreaded(false),
bSimulatedDataEnabled(false),
bEnableDemo(false),
injectType(eInjectRamp),
lastElapsed(0),
lastDistance(0),
trackIDGenerator(0),
lastTransition(0),
transitionDirection(1),
directionPacing(5000/channelTransitionQty),  /* Every 3 seconds, we move from left to right */
nextElapsedDirection(0),
distanceIncrement(0.1),
distancePacing(120), /* 12 ms per move at 0.1m means we do 40m in 5 seconds */
nextElapsedDistance(0)

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	minDistance = globalSettings->receiverSettings[receiverID].displayedRangeMin;
	measurementOffset = globalSettings->receiverSettings[receiverID].rangeOffset;
	distanceScale = globalSettings->distanceScale;
	bEnableDemo = globalSettings->bEnableDemo;
	injectType = (InjectType) globalSettings->demoInjectType;

	receiverStatus.currentAlgo = globalSettings->defaultParametersAlgos.defaultAlgo;
	receiverStatus.currentAlgoPendingUpdates = 0;
	
	for (int channelID = 0; channelID < receiverChannelQty; channelID++)
	{
		maxDistances.push_back(globalSettings->receiverSettings[receiverID].channelsConfig[channelID].maxRange); 
	}

	startTime = boost::posix_time::microsec_clock::local_time();
	
	SetMessageFilters();
	InitStatus();
}

ReceiverCapture::~ReceiverCapture()
{
	EndDistanceLog();
	Stop();
}

void ReceiverCapture::InitStatus()
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	receiverStatus.bUpdated = false;
	receiverStatus.temperature = 0.0;
	receiverStatus.voltage = 0;
	receiverStatus.frameRate = globalSettings->receiverSettings[receiverID].receiverFrameRate;	// Default frame rate is 100Hz
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


float ReceiverCapture::GetMaxDistance(int channelIndex)
{
	return(maxDistances[channelIndex]);
}

float ReceiverCapture::SetMaxDistance(int channelIndex, float inMaxDistance)
{
	if (channelIndex >= maxDistances.size()) return(0.0);
	maxDistances[channelIndex] = inMaxDistance;
	return(inMaxDistance);
}


void  ReceiverCapture::Go(bool inIsThreaded) 
{
	bIsThreaded = inIsThreaded;
	assert(!mThread);
    mStopRequested = false;
	startTime = boost::posix_time::microsec_clock::local_time();

	if (bIsThreaded) 
		{
			mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverCapture::DoThreadLoop, this)));
	}
}
 

void  ReceiverCapture::Stop() 
{
	if (mStopRequested) return;
    mStopRequested = true;
	if (bIsThreaded) {
		bIsThreaded = false;
		assert(mThread);
		mThread->join();
	}
}

bool  ReceiverCapture::WasStopped()
{
	if (mStopRequested) return(true);
	return(false);
}

uint32_t ReceiverCapture::SnapSnapshotFrameID() 
{
	boost::mutex::scoped_lock updateLock(currentReceiverCaptureSubscriptions->GetMutex());

	snapshotFrameID = acquisitionSequence->GetLastFrameID();

	updateLock.unlock();

	return(snapshotFrameID);
};


int ReceiverCapture::GetFrameQty()

{
	return acquisitionSequence->sensorFrames.size();
}


bool ReceiverCapture::CopyReceiverChannelData(uint32_t inFrameID, int inChannelID, ChannelFrame::Ptr &outChannelFrame, Subscription::SubscriberID inSubscriberID)
{
	boost::mutex::scoped_lock updateLock(currentReceiverCaptureSubscriptions->GetMutex());


	SensorFrame::Ptr sensorFrame;
	bool bFound = acquisitionSequence->FindSensorFrame(inFrameID, sensorFrame);
	if (bFound) { 
		ChannelFrame::Ptr sourceChannelFrame =  sensorFrame->channelFrames[inChannelID];
		outChannelFrame->detections.clear();

		int detectionQty = sourceChannelFrame->detections.size();
		for (int i = 0; i< detectionQty; i++) 
		{
 			outChannelFrame->detections.push_back(sourceChannelFrame->detections[i]);
		}	
	
		outChannelFrame->channelID = sourceChannelFrame->channelID;
	}

	currentReceiverCaptureSubscriptions->GetNews(inSubscriberID);

	updateLock.unlock();
	return(bFound);
};

uint32_t ReceiverCapture::GetFrameID(int  inFrameIndex)
{
	boost::mutex::scoped_lock updateLock(currentReceiverCaptureSubscriptions->GetMutex());
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
	uint8_t frameRate;
	ChannelMask channelMask;
	MessageMask messageMask;

	// Update settings from application
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	receiverStatus.frameRate = globalSettings->receiverSettings[receiverID].receiverFrameRate;
	receiverStatus.channelMask.byteData = globalSettings->receiverSettings[receiverID].receiverChannelMask;
	receiverStatus.messageMask.byteData = 0;
	if (globalSettings->receiverSettings[receiverID].msgEnableObstacle) receiverStatus.messageMask.bitFieldData.obstacle = 1;
	if (globalSettings->receiverSettings[receiverID].msgEnableDistance_1_4) receiverStatus.messageMask.bitFieldData.distance_1_4 = 1;
	if (globalSettings->receiverSettings[receiverID].msgEnableDistance_5_8) receiverStatus.messageMask.bitFieldData.distance_5_8 = 1;
	if (globalSettings->receiverSettings[receiverID].msgEnableIntensity_1_4) receiverStatus.messageMask.bitFieldData.intensity_1_4 = 1;
	if (globalSettings->receiverSettings[receiverID].msgEnableIntensity_5_8) receiverStatus.messageMask.bitFieldData.intensity_5_8 = 1;

	return(SetMessageFilters(receiverStatus.frameRate, receiverStatus.channelMask, receiverStatus.messageMask));
}

bool ReceiverCapture::SetMessageFilters(uint8_t frameRate, ChannelMask channelMask, MessageMask messageMask)

{
   return(true);
}

void ReceiverCapture::DoThreadLoop()

{
	while (!WasStopped())
    {
		DoOneThreadIteration();
	} // while (!WasStoppped)
}

void ReceiverCapture::DoThreadIteration()

{
	if (!bIsThreaded)
    {
		DoOneThreadIteration();
	} // while (!WasStoppped)
}


void ReceiverCapture::DoOneThreadIteration()

{
	// As the ReceiverCapture class is a virtual class, The code below means nothing.
	if (!WasStopped())
    {
		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
		frameID++;
		currentReceiverCaptureSubscriptions->PutNews();

		rawLock.unlock();

		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	} // while (!WasStoppped)
}

double ReceiverCapture::GetElapsed()

{
	boost::posix_time::ptime nowTime(boost::posix_time::microsec_clock::local_time());
	boost::posix_time::time_duration msdiff = nowTime - startTime;
    return(msdiff.total_microseconds() / 1000.0);
}


void ReceiverCapture::ProcessCompletedFrame()

{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());

	// timestamp the currentFrame
	double elapsed = GetElapsed();

	currentFrame->timeStamp = GetElapsed();

	// Build distances from the tracks that were accumulated during the frame
	acquisitionSequence->BuildDetectionsFromTracks(currentFrame);


	// Log the tracks or distance, depending on options selected
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	// Log Tracks?
	if (globalSettings->receiverSettings[receiverID].msgEnableObstacle)
	{
		LogTracks(logFile, currentFrame);
	}

	// Log Distances?
	if (globalSettings->receiverSettings[receiverID].msgEnableDistance_1_4 ||
	    globalSettings->receiverSettings[receiverID].msgEnableDistance_5_8 || 
		globalSettings->receiverSettings[receiverID].msgEnableIntensity_1_4 ||
		globalSettings->receiverSettings[receiverID].msgEnableIntensity_5_8)
	{
		LogDistances(logFile, currentFrame);
	}

	// Push the current frame in the frame buffer
	acquisitionSequence->sensorFrames.push(currentFrame);
	currentReceiverCaptureSubscriptions->PutNews();

	// Make sure we do not keep too many of those frames around.
	// Remove the older frame if we exceed the buffer capacity
	if (acquisitionSequence->sensorFrames.size() > maximumSensorFrames) 
	{
		acquisitionSequence->sensorFrames.pop();
	}
	// Create a new current frame.
	uint32_t frameID = acquisitionSequence->AllocateFrameID();
	int channelsRequired = acquisitionSequence->channelQty;
	int detectionsRequired = acquisitionSequence->detectionQty;
	
	currentFrame = SensorFrame::Ptr(new SensorFrame(receiverID, frameID, channelsRequired));

	rawLock.unlock();

	DebugFilePrintf(debugFile, "FrameID- %lu", frameID);
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
	LogFilePrintf(logFile, "Distance Description:;Dist;_;__;Channel;DetectionID;Expected;expectDistance;expectAngle;Val;distance;intensity;velocity;acceletation;ttc;decelerationToStop;probability;ThreatLevel");
	
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

	ChannelFrame::Vector::iterator channelIterator = sourceFrame->channelFrames.begin();
	while (channelIterator !=sourceFrame->channelFrames.end()) 
	{

		ChannelFrame::Ptr channelFrame = *channelIterator;
		Detection::Vector::iterator  detectionIterator = channelFrame->detections.begin();
		while (detectionIterator != channelFrame->detections.end()) 
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

		channelIterator++;
	}	
}


void ReceiverCapture::FakeChannelDistanceRamp(int channel)

{

	int detectOffset = 0;

	if (channel >= 30) 
	{
		channel = channel - 30;
		detectOffset = 4;
	}
	else 
	{
		channel = channel - 20;
	}

	if (channel >= 0) 
	{

		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
		int elapsed = (int) GetElapsed();
		float distance = elapsed % 4000 ;
		distance /= 100;

		currentFrame->channelFrames[channel]->timeStamp = GetElapsed();

		if (distance < minDistance  || distance > maxDistances[channel]) distance = 0.0;

		lastDistance = distance;

		int detectionIndex = 0+detectOffset;
		Detection::Ptr detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
		detection->distance = distance;
		detection->trackID = 0;
		detection->velocity = 0;

		// Only the first channel displays a distance
		distance += 5;
		if (distance < minDistance  || distance  > maxDistances[channel]) distance = 0.0;
		detectionIndex = 1+detectOffset;
		detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
		detection->distance = distance;
		detection->trackID = 0;
		detection->velocity = 0;

		
		distance += 5;

		if (distance < minDistance  || distance  > maxDistances[channel]) distance = 0.0;
		detectionIndex = 2+detectOffset;
		detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
		detection->distance = distance;
		detection->trackID = 0;
		detection->velocity = 0;

		distance += 5;
	
		if (distance < minDistance  || distance > maxDistances[channel]) distance = 0.0;
		detectionIndex = 3+detectOffset;
		detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
		detection->distance = distance;
		detection->trackID = 0;
		detection->velocity = 0;
		rawLock.unlock();
	}

	if (channel == 6 && detectOffset == 4)
	{
		ProcessCompletedFrame();
	}

}

const float simulatedDistance1 = 20.0;
const float simulatedDistanced2 = 12.0;

const float maxSimulatedJitter = 0.9;
const float simulatedPresenceRatio = 0.5;
const int   maxSimulatedFalsePositives = 3;

void ReceiverCapture::FakeChannelDistanceNoisy(int channel)

{

	int detectOffset = 0;

	if (channel >= 30) 
	{
		channel = channel - 30;
		detectOffset = 4;
	}
	else 
	{
		channel = channel - 20;
	}

	if (channel == 0) 
	{

		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
		int elapsed = (int) GetElapsed();

		double distance = simulatedDistance1; 
		// Add jitter to the detection
		distance += ((maxSimulatedJitter * rand() / RAND_MAX)) - (maxSimulatedJitter/2);
		
		// Check if we should create a "false negative to the detection
		bool bIsPresent = (rand()*1.0 / RAND_MAX) < simulatedPresenceRatio;

		if (distance < minDistance  || distance  > maxDistances[channel]) distance = 0.0;

		int detectionIndex = 0+detectOffset;
		if (bIsPresent) 
		{
			Detection::Ptr detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
			detection->distance = distance;
			detection->trackID = 0;
			detection->velocity = 0;
			detectionIndex++;
		}

		int falsePositiveQty = ((maxSimulatedFalsePositives * rand() / RAND_MAX));
		for (int i = 0; i < falsePositiveQty; i++) 
		{
			// Simulate the distance between minDistance maxDistance
			distance = minDistance + ((maxDistances[channel] - minDistance) * rand() / RAND_MAX);
			Detection::Ptr detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
			detection->distance = distance;
			detection->trackID = 0;
			detection->velocity = 0;

			detectionIndex++;
		}


		rawLock.unlock();
	}

	if (channel == 6 && detectOffset == 4)
	{
		ProcessCompletedFrame();
	}

}





void ReceiverCapture::FakeChannelDistanceSlowMove(int channel)

{

	int detectOffset = 0;

	if (channel >= 30) 
	{
		channel = channel - 30;
		detectOffset = 4;
	}
	else 
	{
		channel = channel - 20;
	}

	if (channel >= 0) 
	{
		int elapsed = (int) GetElapsed();
		
		// Every "directionPacing" milliseconds, we move from left to right;
		if (elapsed > nextElapsedDirection) 
		{
			// We update ournext time stamp, and make sure it exceeds current time.
			while (nextElapsedDirection < elapsed) nextElapsedDirection += directionPacing;

			lastTransition += transitionDirection;
			
			// Going too far right, we change direction
			if (lastTransition >= channelTransitionQty) 
			{
				lastTransition = channelTransitionQty - 1;
				transitionDirection = -1;
			}

			// Going too far left, we change direction
			if (lastTransition < 0) 
			{
				lastTransition = 0;
				transitionDirection = +1;
			}
		}

		float distanceMin = minDistance;
		float distanceMax = 0.0;
		for (int channelID = 0; channelID < maxDistances.size(); channelID++) 
		{
			if (maxDistances[channelID] > distanceMax) distanceMax = maxDistances[channelID]; 
		}

		// Every "distancePacing" milliseconds, we move backwards or forward;
		if (elapsed > nextElapsedDistance) 
		{
				// We update ournext time stamp, and make sure it exceeds current time.
			while (nextElapsedDistance < elapsed) nextElapsedDistance += distancePacing;

			lastDistance += distanceIncrement;
			
			// Going too far , we change direction
			if (lastDistance >= distanceMax) 
			{
				lastDistance = distanceMax;
				distanceIncrement = -distanceIncrement;
			}

			// Going too far left, we change direction
			if (lastDistance < distanceMin) 
			{
				lastDistance = distanceMin;
				distanceIncrement = -distanceIncrement;
			}
		}

		currentFrame->channelFrames[channel]->timeStamp = elapsed;
		int channelA = channelTransitions[lastTransition][0];
		int channelB = channelTransitions[lastTransition][1];

		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());

		// Short range channels don't display at more than their maxDistance
		if (lastDistance < maxDistances[channelA])
		{
			Detection::Ptr detection = currentFrame->MakeUniqueDetection(channelA, 0);
			detection->distance = lastDistance;
			detection->trackID = 0;
			detection->velocity = 0;
			currentFrame->channelFrames[channelA]->timeStamp = elapsed;
			detection->firstTimeStamp = elapsed;
			detection->timeStamp = elapsed;
		}
		// There may be detection in a single channel
		if (channelB >= 0) 
		{
			if (lastDistance < maxDistances[channelB]) 
			{
				Detection::Ptr detection = currentFrame->MakeUniqueDetection(channelB, 0);
				detection->distance = lastDistance;
				detection->trackID = 0;
				detection->velocity = 0;
				currentFrame->channelFrames[channel]->timeStamp = elapsed;
				detection->firstTimeStamp = currentFrame->timeStamp;
				detection->timeStamp = currentFrame->timeStamp;
			}
		}



		rawLock.unlock();
		lastElapsed = elapsed;
	
	}

	if (channel == 6 && detectOffset == 4)
	{
		ProcessCompletedFrame();
	}

}



void ReceiverCapture::FakeChannelDistanceConstant(int channel)

{

	int detectOffset = 0;

	if (channel >= 30) 
	{
		channel = channel - 30;
		detectOffset = 4;
	}
	else 
	{
		channel = channel - 20;
	}	

	float steadyDistance = 10.0; // Evantually, change this for a INI File variable
	if (channel >= 0) 
	{
		int elapsed = (int) GetElapsed();
		
		float  distance = steadyDistance;
		distance += measurementOffset;

		currentFrame->channelFrames[channel]->timeStamp = elapsed;

		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());

		// Short range channels don't display at more than shortRangeMax.
		for (int channel = 0; channel < receiverChannelQty; channel++) 
		{
			Detection::Ptr detection = currentFrame->MakeUniqueDetection(channel, 0);
			detection->distance = distance;
			detection->trackID = 0;
			detection->velocity = 0;
			currentFrame->channelFrames[channel]->timeStamp = elapsed;
			detection->timeStamp = elapsed;
			detection->firstTimeStamp = elapsed;
		}

		rawLock.unlock();
		lastElapsed = elapsed;
	
	}

	if (channel == 6 && detectOffset == 4)
	{
		ProcessCompletedFrame();
	}

}

void ReceiverCapture::FakeChannelTrackSlowMove(int channel)

{

	int detectOffset = 0;

	if (channel >= 30) 
	{
		channel = channel - 30;
		detectOffset = 4;
	}
	else 
	{
		channel = channel - 20;
	}


	if (channel >= 0) 
	{
		int elapsed = (int) GetElapsed();
		
		// Every "directionPacing" milliseconds, we move from left to right;
		if (elapsed > nextElapsedDirection) 
		{
			// We update ournext time stamp, and make sure it exceeds current time.
			while (nextElapsedDirection < elapsed) nextElapsedDirection += directionPacing;

			lastTransition += transitionDirection;
			
			// Going too far right, we change direction
			if (lastTransition >= channelTransitionQty) 
			{
				lastTransition = channelTransitionQty - 1;
				transitionDirection = -1;
			}

			// Going too far left, we change direction
			if (lastTransition < 0) 
			{
				lastTransition = 0;
				transitionDirection = +1;
			}
		}

		float distanceMin = minDistance;
		float distanceMax = 0.0;
		for (int channelID = 0; channelID < maxDistances.size(); channelID++) 
		{
			if (maxDistances[channelID] > distanceMax) distanceMax = maxDistances[channelID]; 
		}

		float shortRangeMax = AWLSettings::GetGlobalSettings()->shortRangeDistance;

		// Every "distancePacing" milliseconds, we move backwards or forward;
		if (elapsed > nextElapsedDistance) 
		{
				// We update ournext time stamp, and make sure it exceeds current time.
			while (nextElapsedDistance < elapsed) nextElapsedDistance += distancePacing;

			lastDistance += distanceIncrement;
			
			// Going too far , we change direction
			if (lastDistance >= distanceMax) 
			{
				lastDistance = distanceMax;
				distanceIncrement = -distanceIncrement;
			}

			// Going too far left, we change direction
			if (lastDistance < distanceMin) 
			{
				lastDistance = distanceMin;
				distanceIncrement = -distanceIncrement;
			}
		}

		currentFrame->channelFrames[channel]->timeStamp = elapsed;
		int channelA = channelTransitions[lastTransition][0];
		int channelB = channelTransitions[lastTransition][1];

		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());

		// Short range channels don't display at more than shortRangeMax.
		if (lastDistance < maxDistances[channelA]) 
		{
			Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, trackIDGenerator++);
			track->distance = lastDistance;
#if 1
				track->distance = 10;
#endif
			uint8_t test = 0x001 << channelA;
			track->channels = test;

			track->velocity = (maxDistances[channelA]- (track->distance)) / 2;
			if (distanceIncrement <= 0) track->velocity = -track->velocity;

			track->acceleration = 0;
			track->part1Entered = true;
			track->part2Entered = true;
			track->part3Entered = true;
			track->part4Entered = true;

			track->probability = 99;
			track->timeStamp = elapsed;
			track->firstTimeStamp = elapsed;
			currentFrame->channelFrames[channelA]->timeStamp = elapsed;
		}

		// There may be detection in a single channel
		if (channelB >= 0) 
		{
			if (lastDistance < maxDistances[channelB]) 
			{
				Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, trackIDGenerator++);
				track->distance = lastDistance;
#if 1
				track->distance = 10;
#endif
				track->channels = (0x01 << channelB);
				track->velocity = (maxDistances[channelB]- (track->distance)) / 2;
				if (distanceIncrement <= 0) track->velocity = -track->velocity;
				track->acceleration = 0;
				track->part1Entered = true;
				track->part2Entered = true;
				track->part3Entered = true;
				track->part4Entered = true;
				track->probability = 99;
				track->timeStamp = elapsed;
				track->firstTimeStamp = elapsed;
				currentFrame->channelFrames[channelB]->timeStamp = elapsed;
			}
		}

		rawLock.unlock();
		lastElapsed = elapsed;
	
	}

	if (channel == 6 && detectOffset == 4)
	{
		ProcessCompletedFrame();
	}

}


float ReceiverCapture::SetMinDistance(float inMinDistance)
{
	minDistance = inMinDistance;
	return (minDistance);
}


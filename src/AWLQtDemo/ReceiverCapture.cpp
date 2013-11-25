#define CV_NO_BACKWARD_COMPATIBILITY

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#ifndef Q_MOC_RUN
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#include <iostream>
#include <cstdio>

#include "AWLSettings.h"
#include "tracker.h"
#include "ReceiverCapture.h"
#include "DebugPrintf.h"

#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>

using namespace std;
using namespace pcl;
using namespace awl;

const int ReceiverCapture::maximumSensorFrames(100);


ReceiverCapture::ReceiverCapture(int sequenceID, int inReceiverChannelQty, int inDetectionsPerChannel):
receiverChannelQty(inReceiverChannelQty),
detectionsPerChannel(inDetectionsPerChannel),
acquisitionSequence(new AcquisitionSequence(sequenceID, inReceiverChannelQty, inDetectionsPerChannel)),
frameID(0),
snapshotFrameID(0),
currentFrame(new SensorFrame(0, inReceiverChannelQty, inDetectionsPerChannel)),
currentReceiverCaptureSubscriptions(new(Subscription)),
bIsThreaded(false),
bSimulatedDataEnabled(false),
bEnableDemo(false),
injectType(eInjectRamp),
lastElapsed(0)

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	minDistance = globalSettings->displayedRangeMin;
	maxDistance = globalSettings->displayedRangeMax;
	measurementOffset = globalSettings->rangeOffset;
	sensorDepth = globalSettings->sensorDepth;
	distanceScale = globalSettings->distanceScale;
	bEnableDemo = globalSettings->bEnableDemo;
	injectType = (InjectType) globalSettings->demoInjectType;

	receiverStatus.currentAlgo = globalSettings->defaultAlgo;
	receiverStatus.currentAlgoPendingUpdates = 0;
	
	startTime = boost::posix_time::microsec_clock::local_time();
	
	SetMessageFilters();
	InitStatus();
}

ReceiverCapture::ReceiverCapture(int sequenceID, int inReceiverChannelQty, int inDetectionsPerChannel, std::string inFileName):
receiverChannelQty(inReceiverChannelQty),
detectionsPerChannel(inDetectionsPerChannel),
acquisitionSequence(new AcquisitionSequence(sequenceID, inReceiverChannelQty, inDetectionsPerChannel,  inFileName)),
frameID(0),
snapshotFrameID(0),
currentReceiverCaptureSubscriptions(new(Subscription)),
bIsThreaded(false),
bSimulatedDataEnabled(false)

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	minDistance = globalSettings->displayedRangeMin;
	maxDistance = globalSettings->displayedRangeMax;
	measurementOffset = globalSettings->rangeOffset;
	sensorDepth = globalSettings->sensorDepth;
	distanceScale = globalSettings->distanceScale;
	bEnableDemo = globalSettings->bEnableDemo;
	injectType = (InjectType) globalSettings->demoInjectType;

	// Update settings from configuration file
	receiverStatus.currentAlgo = globalSettings->defaultAlgo;
	receiverStatus.currentAlgoPendingUpdates = 0;

	SetMessageFilters();
	InitStatus();
}

ReceiverCapture::~ReceiverCapture()
{
	Stop();
}


void ReceiverCapture::ProcessCommandLineArguments(int argc, char** argv)

{
	const std::string minRangeOpt = "--minRange=";
	const std::string maxRangeOpt = "--maxRange=";

	// process input arguments
    for( int i = 1; i < argc; i++ )
    {
        if( minRangeOpt.compare( 0, minRangeOpt.length(), argv[i], minRangeOpt.length() ) == 0 )
        {
		   float range;
           if(sscanf( argv[i] + minRangeOpt.length(), "%lf", &range )) minDistance = range;
        }

	    if( maxRangeOpt.compare( 0, maxRangeOpt.length(), argv[i], maxRangeOpt.length() ) == 0 )
        {
		   float range;
           if(sscanf( argv[i] + maxRangeOpt.length(), "%lf", &range )) maxDistance = range;
        }

	}
}

void ReceiverCapture::InitStatus()
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	receiverStatus.bUpdated = false;
	receiverStatus.temperature = 0.0;
	receiverStatus.voltage = 0;
	receiverStatus.frameRate = globalSettings->receiverFrameRate;	// Default frame rate is 100Hz
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

void  ReceiverCapture::Go(bool inIsThreaded) 
{
	bIsThreaded = inIsThreaded;
	assert(!mThread);
    mStopRequested = false;
	startTime = boost::posix_time::microsec_clock::local_time();

	if (bIsThreaded) 
		{
			mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverCapture::DoThreadLoop, this)));
#if 0
		// Set the priority under windows.  This is the most critical display thread 
		// for user interaction
	

		 HANDLE th = mThread->native_handle();
		//   SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
		//   SetThreadPriority(th, THREAD_PRIORITY_ABOVE_NORMAL);
#endif
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

uint32_t ReceiverCapture::CopyCurrentReceiverChannelData(int inChannelID, ChannelFrame::Ptr &outChannelFrame, Subscription::SubscriberID inSubscriberID)
{
	boost::mutex::scoped_lock updateLock(currentReceiverCaptureSubscriptions->GetMutex());

	SensorFrame::Ptr sensorFrame = acquisitionSequence->sensorFrames.back();
	ChannelFrame::Ptr sourceChannelFrame =  sensorFrame->channelFrames[inChannelID];
	outChannelFrame->detections.clear();

	int detectionQty = sourceChannelFrame->detections.size();
	for (int i = 0; i< detectionQty; i++) {
 		outChannelFrame->detections.push_back(sourceChannelFrame->detections[i]);
	}	
	
	outChannelFrame->channelID = sourceChannelFrame->channelID;
	currentReceiverCaptureSubscriptions->GetNews(inSubscriberID);

	updateLock.unlock();

	return(sensorFrame->GetFrameID());
};


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


double ReceiverCapture::GetFrameTime(uint32_t inFrameID)
{
	boost::mutex::scoped_lock updateLock(currentReceiverCaptureSubscriptions->GetMutex());

	SensorFrame::Ptr sensorFrame;
	double timeStamp;
	bool bFound = acquisitionSequence->FindSensorFrame(inFrameID, sensorFrame);
	if (!bFound) timeStamp = -1.0;
	else timeStamp = sensorFrame->timeStamp;

	updateLock.unlock();
	return(timeStamp);
}

double ReceiverCapture::GetFrameTimeAtIndex(int  inFrameIndex)
{
	double timeStamp;

	if (inFrameIndex > acquisitionSequence->sensorFrames.size()-1) timeStamp = -1.0;
	else 
	{ 
		SensorFrame::Ptr sensorFrame = acquisitionSequence->sensorFrames._Get_container().at(inFrameIndex);
		timeStamp = sensorFrame->timeStamp;
	}

	return(timeStamp);
}

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

void ReceiverCapture::SetSensorDepth(double inSensorDepth)
{
	sensorDepth = inSensorDepth;
}

void ReceiverCapture::GetSensorDepth(double &outSensorDepth)
{
	outSensorDepth = sensorDepth;
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

bool ReceiverCapture::StartCalibration(uint8_t frameQty, float beta, ChannelMask channelMask)
{
	return(true);
}

bool ReceiverCapture::SetAlgorithm(uint16_t algorithmID)
{
	return(true);
}

bool ReceiverCapture::SetFPGARegister(uint16_t registerAddress, uint32_t registerValue)
{
	return(true);
}

bool ReceiverCapture::SetADCRegister(uint16_t registerAddress, uint32_t registerValue)
{
	return(true);
}


bool ReceiverCapture::SetGPIORegister(uint16_t registerAddress, uint32_t registerValue)
{
	return(true);
}

bool ReceiverCapture::SetAlgoParameter(QList<AlgorithmParameters> &parametersList, uint16_t registerAddress, uint32_t registerValue)
{
	return(true);
}

bool ReceiverCapture::SetGlobalAlgoParameter(QList<AlgorithmParameters> &parametersList, uint16_t registerAddress, uint32_t registerValue)
{
	return(true);
}

bool ReceiverCapture::SetMessageFilters()

{
	uint8_t frameRate;
	ChannelMask channelMask;
	MessageMask messageMask;

	// Update settings from application
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	receiverStatus.frameRate = globalSettings->receiverFrameRate;
	receiverStatus.channelMask.byteData = globalSettings->receiverChannelMask;
	receiverStatus.messageMask.byteData = 0;
	if (globalSettings->msgEnableObstacle) receiverStatus.messageMask.bitFieldData.obstacle = 1;
	if (globalSettings->msgEnableDistance_1_4) receiverStatus.messageMask.bitFieldData.distance_1_4 = 1;
	if (globalSettings->msgEnableDistance_5_8) receiverStatus.messageMask.bitFieldData.distance_5_8 = 1;
	if (globalSettings->msgEnableIntensity_1_4) receiverStatus.messageMask.bitFieldData.intensity_1_4 = 1;
	if (globalSettings->msgEnableIntensity_5_8) receiverStatus.messageMask.bitFieldData.intensity_5_8 = 1;

	return(SetMessageFilters(receiverStatus.frameRate, receiverStatus.channelMask, receiverStatus.messageMask));

}


bool ReceiverCapture::SetMessageFilters(uint8_t frameRate, ChannelMask channelMask, MessageMask messageMask)

{
	return(true);
}



bool ReceiverCapture::QueryAlgorithm()
{
	return(true);
}

bool ReceiverCapture::QueryFPGARegister(uint16_t registerAddress)
{
	return(true);
}

bool ReceiverCapture::QueryADCRegister(uint16_t registerAddress)
{
	return(true);
}

bool ReceiverCapture::QueryGPIORegister(uint16_t registerAddress)
{
	return(true);
}

bool ReceiverCapture::QueryAlgoParameter(QList<AlgorithmParameters> &parametersList, uint16_t registerAddress)
{
	return(true);
}

bool ReceiverCapture::QueryGlobalAlgoParameter(QList<AlgorithmParameters> &parametersList, uint16_t registerAddress)
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

#if 1
	currentFrame->timeStamp = GetElapsed();
#else
	float frameDelay  =  1.0/AWLSettings::GetGlobalSettings()->receiverFrameRate;
	currentFrame->timeStamp = (currentFrame->frameID  *  frameDelay);  // How many frames since start of unit
#endif

	// Build distances from the tracks that were accumulated during the frame
	acquisitionSequence->BuildDetectionsFromTracks(currentFrame);


	// And timestamp all the distances
	int channelQty = currentFrame->channelFrames.size();
	for (int channelIndex = 0; channelIndex < channelQty; channelIndex++) 
	{
			ChannelFrame::Ptr channelPtr = currentFrame->channelFrames.at(channelIndex);
#if 1
			channelPtr->timeStamp = currentFrame->timeStamp;
#else
			channelPtr->timeStamp = currentFrame->timeStamp;
#endif

			int detectionQty = channelPtr->detections.size();
			for (int detectionIndex = 0; detectionIndex < detectionQty; detectionIndex++) 
			{
				Detection::Ptr detection = channelPtr->detections.at(detectionIndex);
				detection->timeStamp = currentFrame->timeStamp;
				detection->firstTimeStamp = currentFrame->timeStamp;
			}
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
	
	currentFrame = SensorFrame::Ptr(new SensorFrame(frameID, channelsRequired, detectionsRequired));

	rawLock.unlock();

	DebugFilePrintf(debugFile, "FrameID- %lu", frameID);
}


#if 1
static double lastDistance = 0;
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
#if 0
		// JYD:  Watch out ---- Channel order is patched here, because of CAN bug
	channel = channelReorder[channel];
#endif
	if (channel >= 0) 
	{

		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
		int elapsed = (int) GetElapsed();
		float distance = elapsed % 4000 ;
		distance /= 100;
#if 1
		currentFrame->channelFrames[channel]->timeStamp = GetElapsed();
#else
		float frameDelay  =  1.0/AWLSettings::GetGlobalSettings()->receiverFrameRate;
		currentFrame->channelFrames[channel]->timeStamp = (currentFrame->frameID  *  frameDelay);  // How many frames since start of unit
#endif
		if (distance < minDistance  || distance > maxDistance) distance = 0.0;

		lastDistance = distance;

		int detectionIndex = 0+detectOffset;
		Detection::Ptr detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
		detection->distance = distance;
		detection->trackID = 0;
		detection->velocity = 0;

		// Only the first channel displays a distance
		distance += 5;
		if (distance < minDistance  || distance  > maxDistance) distance = 0.0;
		detectionIndex = 1+detectOffset;
		detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
		detection->distance = distance;
		detection->trackID = 0;
		detection->velocity = 0;

		
		distance += 5;

		if (distance < minDistance  || distance  > maxDistance) distance = 0.0;
		detectionIndex = 2+detectOffset;
		detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
		detection->distance = distance;
		detection->trackID = 0;
		detection->velocity = 0;

		distance += 5;
	
		if (distance < minDistance  || distance > maxDistance) distance = 0.0;
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
		DebugFilePrintf(debugFile, "Fake");
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
#if 0
		// JYD:  Watch out ---- Channel order is patched here, because of CAN bug
	channel = channelReorder[channel];
#endif
	if (channel == 0) 
	{

		boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
		int elapsed = (int) GetElapsed();

		double distance = simulatedDistance1; 
		// Add jitter to the detection
		distance += ((maxSimulatedJitter * rand() / RAND_MAX)) - (maxSimulatedJitter/2);
		
		// Check if we should create a "false negative to the detection
		bool bIsPresent = (rand()*1.0 / RAND_MAX) < simulatedPresenceRatio;

		if (distance < minDistance  || distance  > maxDistance) distance = 0.0;

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
			distance = minDistance + ((maxDistance - minDistance) * rand() / RAND_MAX);
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
		DebugFilePrintf(debugFile, "Fake");
		ProcessCompletedFrame();
	}

}


// Sensor transitions going from left to right...

const int channelTransitionQty(13);
int channelTransitions[channelTransitionQty][2] =
{
	{0, -1},
	{0, 1},
	{1, -1},
	{1, 4},
	{1, -1},
	{1, 5},
	{5, -1},
	{5, 2},
	{2, -1},
	{2, 6},
	{2, -1},
	{2, 3},
	{3, -1}
};


int lastTransition = 0;
int transitionDirection = 1;
int directionPacing = 5000/channelTransitionQty; /* Every 3 seconds, we move from left to right */
int nextElapsedDirection = 0;

float distanceIncrement = 0.1;
int distancePacing = 120; /* 12 ms per move at 0.1m means we do 40m in 5 seconds */
int nextElapsedDistance = 0;


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
#if 0	
	// JYD:  Watch out ---- Channel order is patched here, because of CAN bug
	channel = channelReorder[channel];
#endif
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
		float distanceMax = maxDistance;
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
		if (true /*(lastDistance < shortRangeMax) || (channelA >=4)*/) 
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
			if (true /*(lastDistance < shortRangeMax) || (channelA >=4)*/) 
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
		DebugFilePrintf(debugFile, "Fake");
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
#if 0	
	// JYD:  Watch out ---- Channel order is patched here, because of CAN bug
	channel = channelReorder[channel];
#endif
	float steadyDistance = 10.0; // Evantually, change this for a INI File variable
	if (channel >= 0) 
	{
		int elapsed = (int) GetElapsed();
		
		float  distance = steadyDistance;
		distance += measurementOffset;
		distance += sensorDepth;

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
		DebugFilePrintf(debugFile, "Fake");
	}

}

static TrackID trackIDGenerator = 1;

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
		float distanceMax = maxDistance;

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
		if (true /*(lastDistance < shortRangeMax) || (channelA >=4)*/) 
		{
			Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, trackIDGenerator++);
			track->distance = lastDistance;
			uint8_t test = 0x001 << channelA;
			track->channels = test;

			track->velocity = (maxDistance- (track->distance)) / 2;
			if (distanceIncrement <= 0) track->velocity = -track->velocity;

			track->acceleration = 0;
			track->part1Entered = true;
			track->part2Entered = true;
			track->probability = 99;
			track->timeStamp = elapsed;
			track->firstTimeStamp = elapsed;
			currentFrame->channelFrames[channelA]->timeStamp = elapsed;
		}

		// There may be detection in a single channel
		if (channelB >= 0) 
		{
			Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, trackIDGenerator++);
			track->distance = lastDistance;
			track->channels = (0x01 << channelB);
			track->velocity = (maxDistance- (track->distance)) / 2;
			if (distanceIncrement <= 0) track->velocity = -track->velocity;
			track->acceleration = 0;
			track->part1Entered = true;
			track->part2Entered = true;
			track->probability = 99;
			track->timeStamp = elapsed;
			track->firstTimeStamp = elapsed;
			currentFrame->channelFrames[channelA]->timeStamp = elapsed;
		}



		rawLock.unlock();
		lastElapsed = elapsed;
	
	}

	if (channel == 6 && detectOffset == 4)
	{
		ProcessCompletedFrame();
		DebugFilePrintf(debugFile, "Fake");
	}

}


float ReceiverCapture::SetMinDistance(float inMinDistance)
{
	minDistance = inMinDistance;
	return (minDistance);
}

float ReceiverCapture::SetMaxDistance(float inMaxDistance)
{
	maxDistance = inMaxDistance;
	return (maxDistance);
}

#endif
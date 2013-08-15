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

#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>

using namespace std;
using namespace pcl;
using namespace awl;

ReceiverCapture::ReceiverCapture(int sequenceID, int inReceiverChannelQty, int inDetectionsPerChannel):
receiverChannelQty(inReceiverChannelQty),
detectionsPerChannel(inDetectionsPerChannel),
acquisitionSequence(new AcquisitionSequence(sequenceID, inReceiverChannelQty, inDetectionsPerChannel)),
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

	startTime = boost::posix_time::microsec_clock::local_time();
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
	receiverStatus.bUpdated = false;
	receiverStatus.temperature = 0.0;
	receiverStatus.voltage = 0;
	receiverStatus.frameRate = 100;	// Default frame rate is 100Hz
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

bool ReceiverCapture::GetDetection(uint32_t inFrameIndex, int inChannelID, int inDetectionIndex, Detection::Ptr &outDetection, Subscription::SubscriberID inSubscriberID)
{
	if (inSubscriberID >= 0) 
	{
		boost::mutex::scoped_lock updateLock(currentReceiverCaptureSubscriptions->GetMutex());
		outDetection = acquisitionSequence->GetDetectionAtIndex(inFrameIndex, inChannelID, inDetectionIndex);
		currentReceiverCaptureSubscriptions->GetNews(inSubscriberID);
		updateLock.unlock();
	}
	else 
	{
		outDetection = acquisitionSequence->GetDetectionAtIndex(inFrameIndex, inChannelID, inDetectionIndex);
	}

	return(true);
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

bool ReceiverCapture::StartPlayback(uint8_t frameRate, ReceiverCapture::ChannelMask channelMask)
{
	receiverStatus.bInPlayback = true;
	return(true);
}

bool ReceiverCapture::StartRecord(uint8_t frameRate, ReceiverCapture::ChannelMask channelMask)
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

bool ReceiverCapture::StartCalibration(uint8_t frameQty, float beta, ReceiverCapture::ChannelMask channelMask)
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

bool ReceiverCapture::QueryFPGARegister(uint16_t registerAddress)
{
	return(true);
}

bool ReceiverCapture::QueryADCRegister(uint16_t registerAddress)
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



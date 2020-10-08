/* ReceiverCapture.cpp */
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

#ifndef Q_MOC_RUN
#include <boost/foreach.hpp>
#endif

#include <fstream>
#include <limits>

#include "SensorCoreClassesGlobal.h"
#include "Publisher.h"
#include "ThreadedWorker.h"
#include "ReceiverCapture.h"
#include "DetectionStruct.h"
#include "DebugPrintf.h"

SENSORCORE_USE_NAMESPACE

const int ReceiverCapture::maximumSensorFrames(100);
// Sensor transitions going from left to right...

const std::string sDefaultReceiverType = "Generic";
const std::string sDefaultReceiverRegisterSet = "registerDescription_RevC";
const std::string sDefaultReceiverVoxelGeometry = "GeometryAWL7";
const uint16_t defaultVoxelMaskValue = 0xFFFF;
const float defaultSignalToNoiseFloor = -10.0;


<<<<<<< HEAD:src/AWLQtDemo/AWLCoreClasses/ReceiverCapture.cpp
ReceiverCapture::ReceiverCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround,
	uint8_t inFrameRate, ChannelMask& inChannelMask, MessageMask& inMessageMask, float inRangeOffset,
=======
ReceiverCapture::ReceiverCapture(int receiverID, int inReceiverVoxelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround,
	ReceiverFrameRate inDemandedFrameRate, VoxelMask& inVoxelMask, MessageMask& inMessageMask, float inRangeOffset,
>>>>>>> 8032a407dc2ccbc58f53138121c5585d46d33401:src/SensorCoreClasses/ReceiverCapture.cpp
	const RegisterSet& inRegistersFPGA, const RegisterSet& inRegistersADC, const RegisterSet& inRegistersGPIO,
	const AlgorithmSet& inParametersAlgos,
	const AlgorithmSet& inParametersTrackers) :
	ThreadedWorker(),
	Publisher(),
	receiverID(receiverID),
<<<<<<< HEAD:src/AWLQtDemo/AWLCoreClasses/ReceiverCapture.cpp
	receiverChannelQty(inReceiverChannelQty),
=======
	receiverVoxelQty(inReceiverVoxelQty),
>>>>>>> 8032a407dc2ccbc58f53138121c5585d46d33401:src/SensorCoreClasses/ReceiverCapture.cpp
	receiverColumnQty(inReceiverColumns),
	receiverRowQty(inReceiverRows),
	lineWrapAround(inLineWrapAround),
	acquisitionSequence(new AcquisitionSequence()),
	frameID(0),
<<<<<<< HEAD:src/AWLQtDemo/AWLCoreClasses/ReceiverCapture.cpp
	currentFrame(new SensorFrame(receiverID, 0, inReceiverChannelQty)),
=======
	currentFrame(new SensorFrame(receiverID, 0, inReceiverVoxelQty)),
>>>>>>> 8032a407dc2ccbc58f53138121c5585d46d33401:src/SensorCoreClasses/ReceiverCapture.cpp
	measurementOffset(inRangeOffset),
	bFrameInvalidated(false),
	registersFPGA(inRegistersFPGA),
	registersADC(inRegistersADC),
	registersGPIO(inRegistersGPIO),
	parametersAlgos(inParametersAlgos),
	parametersTrackers(inParametersTrackers),
	sReceiverType(sDefaultReceiverType),
	sReceiverRegisterSet(sDefaultReceiverRegisterSet),
<<<<<<< HEAD:src/AWLQtDemo/AWLCoreClasses/ReceiverCapture.cpp
	sReceiverChannelGeometry(sDefaultReceiverChannelGeometry),
	targetHintDistance(0.0),
	targetHintAngle(0.0),
	m_FrameRate(0),
	m_FrameRateMS(0.0),
	m_nbrCompletedFrame(0),
	m_nbrCompletedFrameCumul(0),
	m_nbrRawCumul(0),
=======
	sReceiverVoxelGeometry(sDefaultReceiverVoxelGeometry),
	targetHintDistance(0.0),
	targetHintAngle(0.0),
	m_calculatedFrameRate(),
	m_frameRateMS(0.0),
	m_nbrCompletedFrame(0),
>>>>>>> 8032a407dc2ccbc58f53138121c5585d46d33401:src/SensorCoreClasses/ReceiverCapture.cpp
	logFilePtr(NULL)

{
	// Initialize default status values
	InitStatus();

	receiverStatus.demandedFrameRate = inDemandedFrameRate;
	receiverStatus.obtainedFrameRate = 0;
	receiverStatus.currentAlgo = 0;
	receiverStatus.currentAlgoPendingUpdates = 0;

	receiverStatus.currentTracker = 0;
	receiverStatus.currentTrackerPendingUpdates = 0;

	// Update settings from application
	receiverStatus.voxelMask = inVoxelMask;
	receiverStatus.messageMask = inMessageMask;

	// Reflect the settings in hardware
<<<<<<< HEAD:src/AWLQtDemo/AWLCoreClasses/ReceiverCapture.cpp
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
targetHintAngle(0.0),
m_FrameRate(0),
m_FrameRateMS(0.0),
m_nbrCompletedFrame(0),
m_nbrCompletedFrameCumul(0),
m_nbrRawCumul(0),
logFilePtr(NULL)
=======
	SetMessageFilters(receiverStatus.demandedFrameRate, receiverStatus.voxelMask, receiverStatus.messageMask);
}

ReceiverCapture::ReceiverCapture(int receiverID, boost::property_tree::ptree& propTree) :
	ThreadedWorker(),
	Publisher(),
	receiverID(receiverID),
	acquisitionSequence(new AcquisitionSequence()),
	frameID(0),
	bFrameInvalidated(false),
	sReceiverType(sDefaultReceiverType), 
	sReceiverRegisterSet(sDefaultReceiverRegisterSet),
	sReceiverVoxelGeometry(sDefaultReceiverVoxelGeometry),
	targetHintDistance(0.0),
	targetHintAngle(0.0),
	m_calculatedFrameRate(0),
	m_frameRateMS(0.0),
	m_nbrCompletedFrame(0),
	logFilePtr(NULL)
>>>>>>> 8032a407dc2ccbc58f53138121c5585d46d33401:src/SensorCoreClasses/ReceiverCapture.cpp

{
	// Read the configuration from the configuration file

	receiverStatus.signalToNoiseFloor = defaultSignalToNoiseFloor;

	ReadConfigFromPropTree(propTree);
	ReadGeometryFromPropTree(propTree);
	ReadRegistersFromPropTree(propTree);
	receiverStatus.obtainedFrameRate = 0;

	// Initialize default status values
	InitStatus();

	// make sure that the communication is reset.
	receiverStatus.currentAlgoPendingUpdates = 0;
	receiverStatus.currentTrackerPendingUpdates = 0;

	// Create a temporary SensorFrame object for storage of the current data
	currentFrame = SensorFrame::Ptr(new SensorFrame(receiverID, 0, receiverVoxelQty));

	// Reflect the settings in hardware
	SetMessageFilters(receiverStatus.demandedFrameRate, receiverStatus.voxelMask, receiverStatus.messageMask);
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

ReceiverFrameRate ReceiverCapture::GetCalculatedFrameRate()
{
	// timestamp the currentFrame
	Timestamp elapsed = GetElapsed();

	if (elapsed - m_frameRateMS > 1000.0)
	{
		m_calculatedFrameRate = (ReceiverFrameRate)m_nbrCompletedFrame;
		m_nbrCompletedFrame = 0;
		m_frameRateMS = elapsed;
	}

	return m_calculatedFrameRate;
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

bool ReceiverCapture::CopyReceiverAScans(FrameID inFrameID,  AScan::Vector &outAScans, Publisher::SubscriberID inSubscriberID)
{
	if (!LockNews(inSubscriberID)) return(false);

	SensorFrame::Ptr sensorFrame;
	bool bFound = acquisitionSequence->FindSensorFrame(inFrameID, sensorFrame);
	if (bFound) 
	{ 
		BOOST_FOREACH(AScan::Ptr &aScan, sensorFrame->aScans) {
			outAScans.push_back(aScan);
			//printf ("copy ascan %d\n", receiverID);
		}
	}

	UnlockNews(inSubscriberID);
	return(bFound);
}



bool ReceiverCapture::CopyReceiverStatusData(ReceiverStatus &outStatus)
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
	FrameID localFrameID;

	if (inFrameIndex > (int) acquisitionSequence->sensorFrames.size()-1) localFrameID = 0xFFFFFFFF;
	else 
	{ 
		SensorFrame::Ptr sensorFrame = acquisitionSequence->sensorFrames.at(inFrameIndex);
		frameID = sensorFrame->frameID;
	}

	updateLock.unlock();
	return(frameID);
}

void ReceiverCapture::SetMeasurementOffset(float inMeasurementOffset)
{
	measurementOffset = inMeasurementOffset;
}

void ReceiverCapture::GetMeasurementOffset(float &outMeasurementOffset)
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

bool ReceiverCapture::StartPlayback(ReceiverFrameRate/*frameRate*/, VoxelMask /*voxelMask*/)
{
	receiverStatus.bInPlayback = true;
	return(true);
}

bool ReceiverCapture::StartRecord(ReceiverFrameRate /*frameRate*/, VoxelMask /*voxelMask*/)
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


bool ReceiverCapture::SetMessageFilters(ReceiverFrameRate /*frameRate*/, VoxelMask /*voxelMask*/, MessageMask /*messageMask*/)

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

Timestamp ReceiverCapture::GetElapsed()

{
	boost::posix_time::ptime nowTime(boost::posix_time::microsec_clock::local_time());
	boost::posix_time::time_duration msdiff = nowTime - startTime;
    return((float)(msdiff.total_microseconds() / 1000.0));
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
	Timestamp elapsed = GetElapsed();

  ++m_nbrCompletedFrame;

	currentFrame->timeStamp = elapsed;

	// TimeStamp all detections and tracks
	TimestampTracks(currentFrame);

	TimestampDetections(currentFrame);

	// Log Tracks?
	if (receiverStatus.messageMask.bitFieldData.obstacle)
	{
		LogTracks(currentFrame);
	}

	// Log Distances?
	if (receiverStatus.messageMask.bitFieldData.distance_1_4 ||
	    receiverStatus.messageMask.bitFieldData.distance_5_8 || 
		receiverStatus.messageMask.bitFieldData.intensity_1_4 ||
		receiverStatus.messageMask.bitFieldData.intensity_5_8 ||
		receiverStatus.messageMask.bitFieldData.distance_intensity)
	{
		LogDistances(currentFrame);
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
	FrameID localFrameID = acquisitionSequence->AllocateFrameID();
	currentFrame = SensorFrame::Ptr(new SensorFrame(receiverID, localFrameID, receiverVoxelQty));
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

void ReceiverCapture::ClearRegisterSet(RegisterSet& inRegisterSet)

{
	for (uint16_t i = 0; i < inRegisterSet.size(); i++)
	{
		inRegisterSet.at(i).value = 0;
		inRegisterSet.at(i).pendingUpdates = updateStatusUpToDate;
	}
}

void ReceiverCapture::ClearAllRegisters()
{
	ClearRegisterSet(registersFPGA);
	ClearRegisterSet(registersADC);
	ClearRegisterSet(registersGPIO);
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


CellID ReceiverCapture::GetCellIDFromChannel(int inChannelID)
{
	int row = inChannelID/receiverColumnQty;
	int column = inChannelID % receiverRowQty;
	return (CellID(column, row));
}

int ReceiverCapture::GetChannelIDFromCell(CellID inCellID)
{
	int channelID= inCellID.row * receiverColumnQty;
	channelID += inCellID.column;

	return (channelID);
}

bool ReceiverCapture::BeginDistanceLog()

{
	logFileMutex.lock();

	if (!logFilePtr)
	{
<<<<<<< HEAD:src/AWLQtDemo/AWLCoreClasses/ReceiverCapture.cpp
		logFilePtr = new ofstream();
=======
		logFilePtr = new std::ofstream();
>>>>>>> 8032a407dc2ccbc58f53138121c5585d46d33401:src/SensorCoreClasses/ReceiverCapture.cpp
	}
		
	if (!logFilePtr->is_open())
	{
<<<<<<< HEAD:src/AWLQtDemo/AWLCoreClasses/ReceiverCapture.cpp
		OpenLogFile(*logFilePtr, GetLogFileName().c_str(), true);
	}

	LogFilePrintf(*logFilePtr, ",Comment,Type, Track ID, Channel, ChannelID,Type Specific");
	LogFilePrintf(*logFilePtr, ",,Start distance log");
	// Title Line
	LogFilePrintf(*logFilePtr, ",Track Description:,Track,trackID,Channel,ChannelID,___,Expected,expectDistance,expectAngle,Val,distance,intensity,velocity,acceleration,ttc,decelerationToStop,probability,ThreatLevel,Ch.0,Ch.1,Ch.2,Ch.3,Ch.4,Ch.5,Ch.6,");
	LogFilePrintf(*logFilePtr, ",Distance Description:,Dist,_,__,Channel,DetectionID,Expected,expectDistance,expectAngle,Val,distance,intensity,velocity,acceleration,ttc,decelerationToStop,probability,ThreatLevel");
	LogFilePrintf(*logFilePtr, ",Footer Description:,Footer,_,_,_,_,FooterData");
	LogFilePrintf(*logFilePtr, ",Wave Description: ,Wave,_,_,Channel,ChannelID,point0,point1,point3,etc");
=======
		OpenLogFile(*logFilePtr, true);
	}

	LogFilePrintf(*logFilePtr, ",Comment,Type, ReceiverID, Track ID, Channel, ColumnID, RowID, ChannelID,Type Specific");
	LogFilePrintf(*logFilePtr, ",,Start distance log");
	// Title Line. The title lines dscribe the column content for each type of message.
	LogFilePrintf(*logFilePtr, ",Track Description:,Track,trackID,ReceiverID,Channel,ColumnID, RowID,ChannelID,___,Expected,expectDistance,expectAngle,Val,distance,intensity,velocity,acceleration,ttc,decelerationToStop,probability,ThreatLevel,Ch.0,Ch.1,Ch.2,Ch.3,Ch.4,Ch.5,Ch.6,");
	LogFilePrintf(*logFilePtr, ",Distance Description:,Dist,_,ReceiverID,Channel,ColumnID,RowID,ChannelID,DetectionID,Expected,expectDistance,expectAngle,Val,distance,intensity,velocity,acceleration,ttc,decelerationToStop,probability,ThreatLevel");
	LogFilePrintf(*logFilePtr, ",Footer Description:,Footer,_,ReceiverID,_,_,_,_,FooterData");
	LogFilePrintf(*logFilePtr, ",Wave Description:,Wave,_,ReceiverID,Channel,ColumID, RowID, ChannelID,,point0,point1,point2,etc");
>>>>>>> 8032a407dc2ccbc58f53138121c5585d46d33401:src/SensorCoreClasses/ReceiverCapture.cpp

	logFileMutex.unlock();

	return(true);
}


bool ReceiverCapture::EndDistanceLog()

{
	logFileMutex.lock();

	if (!logFilePtr)
	{
		logFileMutex.unlock();
		return(false);
	}

	if (logFilePtr->is_open())
	{
		CloseLogFile(*logFilePtr);
		delete(logFilePtr);
		logFilePtr = NULL;
	}

	logFileMutex.unlock();
	return(true);
}


void ReceiverCapture::LogTracks(SensorFrame::Ptr sourceFrame)
{
	logFileMutex.lock();

	if (!logFilePtr)
	{
		logFileMutex.unlock();
		return;
	}


	// Update the coaslesced tracks
   Track::Vector::iterator  trackIterator = sourceFrame->tracks.begin();

	while (trackIterator != sourceFrame->tracks.end()) 
	{
		Track::Ptr track = *trackIterator;
		if (track->IsComplete()) 
		{
			// Track messages are ordered per voxel. trackMainVoxel corresponds to column

			CellID cellID(track->trackMainVoxel % receiverColumnQty, track->trackMainVoxel / receiverColumnQty);
			int channelID = GetChannelIDFromCell(cellID);

			//Date;Comment (empty);"TrackID", "Track"/"Dist";TrackID;"Channel";....Val;distance;intensity,speed;acceleration;probability;timeToCollision);
<<<<<<< HEAD:src/AWLQtDemo/AWLCoreClasses/ReceiverCapture.cpp
			LogFilePrintf(*logFilePtr, ", ,Track,%d,Channel,%d, ,Expected,%.2f,%.1f,Val,%.2f,%.1f,%.1f,%.1f,%.3f,%.1f,%.0f,%d,%d,%d,%d,%d,%d,%d,%d,%d,",
=======
			LogFilePrintf(*logFilePtr, ", ,Track,%d,%d,Channel,%d,%d,%d, ,Expected,%.2f,%.1f,Val,%.2f,%.1f,%.1f,%.1f,%.3f,%.1f,%.0f,%d,%d,%d,%d,%d,%d,%d,%d,%d,",
>>>>>>> 8032a407dc2ccbc58f53138121c5585d46d33401:src/SensorCoreClasses/ReceiverCapture.cpp
				track->trackID,
				receiverID,
				cellID.column,
				cellID.row,
				channelID,
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
				track->trackMainVoxel, 
				(track->trackChannels.bitFieldData.voxel0)? 0 : 0, 
				(track->trackChannels.bitFieldData.voxel1) ? 1 : 0,
				(track->trackChannels.bitFieldData.voxel2) ? 2 : 0,
				(track->trackChannels.bitFieldData.voxel3) ? 3 : 0,
				(track->trackChannels.bitFieldData.voxel4) ? 4 : 0,
				(track->trackChannels.bitFieldData.voxel5) ? 5 : 0,
				(track->trackChannels.bitFieldData.voxel6) ? 6 : 0);

		}  // if (track...

		trackIterator++;
	} // while (trackIterator...

	logFileMutex.unlock();
} 

void ReceiverCapture::LogDistances(SensorFrame::Ptr sourceFrame)
{
	logFileMutex.lock();

	if (!logFilePtr)
	{
		logFileMutex.unlock();
		return;
	}

	Detection::Vector::iterator  detectionIterator = sourceFrame->rawDetections.begin();
	while (detectionIterator != sourceFrame->rawDetections.end()) 
	{
		Detection::Ptr detection = *detectionIterator;
<<<<<<< HEAD:src/AWLQtDemo/AWLCoreClasses/ReceiverCapture.cpp
		LogFilePrintf(*logFilePtr, " , ,Dist,,Channel,%d,%d,Expected,%.2f,%.1f,Val,%.2f,%.1f,%.2f,%.1f,%.3f,%.1f,%.0f,%d",
			detection->channelID, 
=======
		// Track messages are ordered per voxel. trackMainVoxel corresponds to column

		CellID cellID = detection->cellID;
		int channelID = GetChannelIDFromCell(cellID);

		LogFilePrintf(*logFilePtr, " , ,Dist,,%d,Channel,%d,%d,%d,%d,Expected,%.2f,%.1f,Val,%.2f,%.1f,%.2f,%.1f,%.3f,%.1f,%.0f,%d",
			receiverID,
			cellID.column,
			cellID.row,
			channelID, 
>>>>>>> 8032a407dc2ccbc58f53138121c5585d46d33401:src/SensorCoreClasses/ReceiverCapture.cpp
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

	logFileMutex.unlock();
}


// Configuration file related functions


bool ReceiverCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
		std::string receiverKey = std::string("config.receivers.receiver") + std::to_string(receiverID);

		boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);

		sReceiverType = receiverNode.get<std::string>("receiverType");
		sReceiverRegisterSet = receiverNode.get<std::string>("receiverRegisterSet");
		sReceiverVoxelGeometry = receiverNode.get<std::string>("receiverVoxelGeometry");
		measurementOffset = receiverNode.get<float>("rangeOffset");

		receiverStatus.demandedFrameRate =  (ReceiverFrameRate) receiverNode.get<uint16_t>("frameRate");	// Default frame rate is 100Hz

		receiverStatus.voxelMask.wordData = receiverNode.get<uint16_t>("voxelMask");

		receiverStatus.messageMask.byteData = 0;
		if (receiverNode.get<bool>("msgEnableObstacle")) receiverStatus.messageMask.bitFieldData.obstacle = 1;
		if (receiverNode.get<bool>("msgEnableDistance_1_4")) receiverStatus.messageMask.bitFieldData.distance_1_4 = 1;
		if (receiverNode.get<bool>("msgEnableDistance_5_8")) receiverStatus.messageMask.bitFieldData.distance_5_8 = 1;
		if (receiverNode.get<bool>("msgEnableIntensity_1_4")) receiverStatus.messageMask.bitFieldData.intensity_1_4 = 1;
		if (receiverNode.get<bool>("msgEnableIntensity_5_8")) receiverStatus.messageMask.bitFieldData.intensity_5_8 = 1;
		if (receiverNode.get<bool>("msgEnableDistanceIntensity")) receiverStatus.messageMask.bitFieldData.distance_intensity = 1;
		if (receiverNode.get<bool>("msgEnableObstacleCompact")) receiverStatus.messageMask.bitFieldData.obstacle_compact = 1;
		if (receiverNode.get<bool>("msgEnableRaw", false)) receiverStatus.messageMask.bitFieldData.raw = 1;

		return(true);

}

bool ReceiverCapture::ReadGeometryFromPropTree(boost::property_tree::ptree &propTree)
{
	using boost::property_tree::ptree;

	// Read the geometry
	std::string geometryDescKey = "config." + sReceiverVoxelGeometry;

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
		(void)e;
		return (false);
	}

	// Geometry section is found.  Check if we have a channel based configuration or
	// array based configuration;

	lineWrapAround = geometryNodePtr->get<float>("lineWrapAround", -1.0);
	receiverVoxelQty = geometryNodePtr->get<int>("voxelQty", -1);
	receiverColumnQty = receiverVoxelQty;
	receiverRowQty = 1;
	if (receiverVoxelQty == -1)
	{
		receiverColumnQty = geometryNodePtr->get<int>("arraySize.x", -1);
		receiverRowQty = geometryNodePtr->get<int>("arraySize.y", -1);
		receiverVoxelQty = ((int)receiverColumnQty) * ((int)receiverRowQty);
	}

	return(true);
}

bool ReceiverCapture::ReadRegistersFromPropTree(boost::property_tree::ptree & /*propTree*/)
{
	return(true);
}


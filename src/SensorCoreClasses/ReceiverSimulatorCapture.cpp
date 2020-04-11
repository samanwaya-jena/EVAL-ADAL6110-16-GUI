/* ReceiverSimulatorCapture.cpp */
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

#include <string>

#include "SensorCoreClassesGlobal.h"
#include "DebugPrintf.h"

#include "DetectionStruct.h"
#include "ReceiverCapture.h"
#include "ReceiverSimulatorCapture.h"

SENSORCORE_USE_NAMESPACE


ReceiverSimulatorCapture::ReceiverSimulatorCapture(int receiverID, int inReceiverVoxelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround,
	ReceiverFrameRate inDemandedFrameRate, VoxelMask &inVoxelMask, MessageMask &inMessageMask, float inRangeOffset,
	const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO,
	const AlgorithmSet &inParametersAlgos, const AlgorithmSet &inParametersTrackers) :
ReceiverCapture(receiverID, inReceiverVoxelQty, inReceiverColumns, inReceiverRows, inLineWrapAround, inDemandedFrameRate, inVoxelMask, inMessageMask, inRangeOffset,
					   inRegistersFPGA, inRegistersADC, inRegistersGPIO, inParametersAlgos, inParametersTrackers)

{
}


ReceiverSimulatorCapture::ReceiverSimulatorCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCapture(receiverID, propTree)

{
	// Read the configuration from the configuration file
	ReadConfigFromPropTree(propTree);

}

ReceiverSimulatorCapture::~ReceiverSimulatorCapture()
{
	CloseDebugFile(debugFile);
	EndDistanceLog();
	Stop(); // Stop the thread
}

int threadCount = 0;
static float trackDistance = 0.0;

void ReceiverSimulatorCapture::DoOneThreadIteration()

{
	if (!WasStopped())
	{
		// Simulate some tracks for debug purposes
		Timestamp elapsed = GetElapsed();

		trackDistance += 0.0001f;
		if (trackDistance > 10.0) trackDistance = 0.0;

		for (uint16_t voxelIndex= 0; voxelIndex < 16; voxelIndex++)
		{
			for (uint16_t detection = 0; detection < 8; detection++)
			{
				// Only display even on even lines and odd on odd lines
				size_t lineIndex = (voxelIndex / this->receiverColumnQty);
				size_t lineQty = (GetVoxelQty() / this->receiverColumnQty);  
				if (lineIndex == (voxelIndex%lineQty)) 
				{
					Track::Ptr track = acquisitionSequence->MakeUniqueTrack(currentFrame, (voxelIndex * 8) + detection);
					track->firstTimeStamp = currentFrame->timeStamp;

					track->timeStamp = currentFrame->timeStamp;
					track->distance = (detection * 2) + trackDistance;
					track->intensity = 1.00;
					track->trackMainVoxel = voxelIndex;

					track->trackChannels.wordData = 0x01 << (track->trackMainVoxel % 8);
					

					track->velocity = 3;
					track->acceleration = 0;
					track->threatLevel = AlertCondition::eThreatLow;
					track->part1Entered = true;
					track->part2Entered = true;
					track->part3Entered = true;
					track->part4Entered = true;

					track->probability = 99;
					track->timeStamp = elapsed;
					track->firstTimeStamp = elapsed;
				}
			}
		}

		threadCount = ++ threadCount % 7;
		ProcessCompletedFrame();
	} // if  (!WasStoppped)
}


bool ReceiverSimulatorCapture::QueryUniqueID()
{
	// Basic Simulator device does not have UniqueID
	return(true);
}

bool ReceiverSimulatorCapture::QueryProductID()
{
	// Basic Simulator device does not have ProductID emmbeded in firmware 
	return(true);
}

uint32_t ReceiverSimulatorCapture::GetUniqueID()
{
	// A value of zero indicates that serial No is not available
	return(0);
}

uint32_t ReceiverSimulatorCapture::GetProductID()
{
	// Unknown device ID
	return(0);
}


bool ReceiverSimulatorCapture::SetMessageFilters(ReceiverFrameRate /*demandedFrameRate*/, VoxelMask /*voxelMask*/, MessageMask /*messageMask*/)

{
	return(true);
}


bool ReceiverSimulatorCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
		ReceiverCapture::ReadConfigFromPropTree(propTree);
		receiverStatus.signalToNoiseFloor = 18.0;
		return(true);
}


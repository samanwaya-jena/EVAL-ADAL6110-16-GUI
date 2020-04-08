/* ReceiverPolledCapture.cpp */
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

#include <stdio.h>
#include <string.h>
#include "DebugPrintf.h"
#include "SensorSettings.h"

#include "SensorCoreClassesGlobal.h"
#include "ReceiverPolledCapture.h"

SENSORCORE_USE_NAMESPACE

#define MAX_POLL_CAN_MESSAGES 32					// Message buffer size in the local polled capture structures
#define WAVEFORM_POINT_QTY       100				// Number of points per A-SCan Waveform
#define CHANNEL_QTY           16					// Number of channels in sensor
#define WAVEFORM_FOOTER_SIZE             96			// Footer in AScan Message: Size  Footer is not interpreted


typedef struct
{
	short AcqFifo[CHANNEL_QTY * WAVEFORM_POINT_QTY];
	short footer[WAVEFORM_FOOTER_SIZE];
} tDataFifo;

tDataFifo dataFifo[8];



ReceiverPolledCapture::ReceiverPolledCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCANCapture(receiverID, propTree),
handle(NULL),
swap_handle(NULL)
{
  reconnectTime = boost::posix_time::microsec_clock::local_time();
}

ReceiverPolledCapture::~ReceiverPolledCapture()
{
}

void ReceiverPolledCapture::Go()
{
  assert(!mThread);

  mWorkerRunning = true;

  startTime = boost::posix_time::microsec_clock::local_time();

  mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverPolledCapture::DoThreadLoop, this)));

#ifdef _WINDOWS_
  // Set the priority under windows.  This is the most critical display thread 
  // for user interaction
  HANDLE th = mThread->native_handle();
  SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
  //   SetThreadPriority(th, THREAD_PRIORITY_ABOVE_NORMAL);
#endif
}

void ReceiverPolledCapture::Stop()
{
  ReceiverCapture::Stop();
}

bool ReceiverPolledCapture::IsConnected()
{
  return (handle != NULL);
}

bool ReceiverPolledCapture::LidarQuery(size_t& cycleCount, size_t& messageCount)
{
  int transferred = 0;
  int received = 0;
  uint32_t dwCount;
  uint32_t dwReadPending;

  boost::mutex::scoped_lock rawLock(m_Mutex);

  if (!handle)
    return false;

  ReceiverCANMessage msg;
  msg.id = RECEIVERCANMSG_ID_LIDARQUERY;

  ReceiverCANMessage resp;

  transferred = WriteBytes((uint8_t*)&msg, sizeof(msg));
  if (transferred != sizeof(ReceiverCANMessage))
    return false;

  received = ReadBytes((uint8_t*)&resp, sizeof(resp));
  if (received != sizeof(ReceiverCANMessage))
    return false;

  dwCount = *((uint32_t*)&resp.data[0]);
  dwReadPending = *((uint32_t*)&resp.data[4]);

  cycleCount = (size_t)dwCount;
  messageCount = (size_t)dwReadPending;

  return true;
}


bool ReceiverPolledCapture::ReadDataFromUSB(char * dataBuffer, int payloadSize, size_t cycleCount)
{
  int transferred = 0;
  int received = 0;

  boost::mutex::scoped_lock rawLock(m_Mutex);

  if (!handle)
    return false;

  ReceiverCANMessage msg;
  msg.id = RECEIVERCANMSG_ID_GETDATA;
  msg.data[0] = (unsigned char) cycleCount;

  transferred = WriteBytes((uint8_t*)&msg, sizeof(msg));
  if (transferred != sizeof(ReceiverCANMessage))
    return false;

  received = ReadBytes((uint8_t*)dataBuffer, payloadSize);
  if (received != payloadSize)
    return false;

  return true;
}

bool ReceiverPolledCapture::DoOneLoop()
{
	size_t cycleCount = 0;
	size_t messageCount = 0;

	bool bRet = LidarQuery(cycleCount, messageCount);

	if (!bRet)
		return false;

	if (cycleCount)
	{
		bool bReadSuccess = true;

		size_t payloadSize = cycleCount * sizeof(tDataFifo);
		if (!xmitsFooterData)
		{
			payloadSize -= WAVEFORM_FOOTER_SIZE * sizeof(short);
		}
		bReadSuccess = ReadDataFromUSB((char*)dataFifo, (int) payloadSize, cycleCount);



		if (SensorSettings::GetGlobalSettings()->bWriteLogFile)
			{
				boost::mutex::scoped_lock rawLock(m_Mutex);
			
				if (SensorSettings::GetGlobalSettings()->bWriteLogFile) // Check again for file handle now that we have the mutex
				{
					for (size_t cycle = 0; cycle < cycleCount; cycle++)
					{
						LogWaveform((int) cycle);
					}
				}

			}

		ProcessRaw((uint8_t*)dataFifo->AcqFifo);
	}

	if (messageCount)
	{
		bRet = PollMessages(messageCount);
		if (!bRet)
			return false;
	}

	if (!cycleCount && !messageCount)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(5));
	}


	return bRet;
}

void ReceiverPolledCapture::DoOneThreadIteration()
{
  if (swap_handle) {
    handle = swap_handle;
    swap_handle = NULL;
  }
  if (handle)
  {
    bool bRet = DoOneLoop();

    if (!bRet)
    {
      CloseCANPort();
    }
  }
  else
  {
    if (boost::posix_time::microsec_clock::local_time() > reconnectTime)
    {
      if (OpenCANPort())
      {
        WriteCurrentDateTime();
        SetMessageFilters(receiverStatus.frameRate, receiverStatus.voxelMask, receiverStatus.messageMask);
        // Update all the info (eventually) from the status of the machine
        QueryAlgorithm();
        QueryTracker();
      }
    }
  }
}

void ReceiverPolledCapture::DoThreadLoop()
{
  while (!WasStopped())
  {
    DoOneThreadIteration();
  } // while (!WasStoppped)
}

bool ReceiverPolledCapture::PollMessages(size_t messageCount)
{
  ReceiverCANMessage canResp[MAX_POLL_CAN_MESSAGES];
  int transferred = 0;
  int received = 0;

  boost::mutex::scoped_lock rawLock(m_Mutex);

  if (!handle)
    return false;

  if (messageCount > MAX_POLL_CAN_MESSAGES)
    messageCount = MAX_POLL_CAN_MESSAGES;

  ReceiverCANMessage msg;
  msg.id = RECEIVERCANMSG_ID_POLLMESSAGES;
  msg.data[0] = (unsigned char) messageCount;

  transferred = WriteBytes((uint8_t*)&msg, sizeof(msg));
  if (transferred != sizeof(msg))
    return false;

  received = ReadBytes((uint8_t*) &canResp[0], (int) (messageCount * sizeof(ReceiverCANMessage)));
  if (received != (int) (messageCount * sizeof(ReceiverCANMessage)))
    return false;

  for (uint32_t i=0; i<messageCount; i++)
  {
    if (canResp[i].id)
      ParseMessage(canResp[i]);
  }

  return true;
}

bool ReceiverPolledCapture::WriteMessage(const ReceiverCANMessage &inMsg)
{
  ReceiverCANMessage canResp;
  int transferred = 0;
  int received = 0;

  if (!handle)
    return false;

  boost::mutex::scoped_lock rawLock(m_Mutex);

  transferred = WriteBytes((uint8_t*)&inMsg, sizeof(inMsg));
  if (transferred != sizeof(ReceiverCANMessage))
    return false;

  received = ReadBytes((uint8_t*) &canResp, sizeof(canResp));
  if (received != sizeof(ReceiverCANMessage))
    return false;

  if (canResp.id)
    ParseMessage(canResp);

	return true;
}

bool ReceiverPolledCapture::SendSoftwareReset()
{
  if (!handle)
    return false;

  ReceiverCANMessage msg;
  msg.id = RECEIVERCANMSG_ID_COMMANDMESSAGE;
  msg.len = RECEIVERCANMSG_LEN;
  msg.data[0] = RECEIVERCANMSG_ID_CMD_SET_PARAMETER;
  msg.data[1] = RECEIVERCANMSG_ID_CMD_PARAM_ADC_REGISTER;
  msg.data[2] = 0x00;
  msg.data[3] = 0x00;
  msg.data[4] = 0x00;
  msg.data[5] = 0x00;
  msg.data[6] = 0x00;
  msg.data[7] = 0x00;

  return WriteMessage(msg);
}


void * ReceiverPolledCapture::GetHandle(void) {
	return handle;
}

void ReceiverPolledCapture::SetHandle(void *h)
{
	swap_handle = h;
}

void ReceiverPolledCapture::LogWaveform(size_t cycle)
{
	logFileMutex.lock();

	if (!logFilePtr)
	{
		logFileMutex.unlock();
		return;
	}

	for (int ch = 0; ch < CHANNEL_QTY; ch++)
	{
		CellID cellID = GetCellIDFromChannel(ch);
		std::string theWaveString(", ,Wave, ,");
		theWaveString += std::to_string(receiverID);
		theWaveString += std::string(",Channel,");
		theWaveString += std::to_string(cellID.column) + "," + std::to_string(cellID.row) + ",";
		theWaveString += std::to_string(ch) + ", ,";

		short* pData = &dataFifo[cycle].AcqFifo[ch * WAVEFORM_POINT_QTY];
		for (size_t i = 0; i < WAVEFORM_POINT_QTY; i++)
		{
			if (i == (WAVEFORM_POINT_QTY - 1))
				theWaveString += std::to_string(*pData++);
			else
				theWaveString += std::to_string(*pData++) + ",";

		}
		LogFilePrintf(*logFilePtr, theWaveString.c_str());
	}

	short* pFooter = &dataFifo[cycle].footer[0];
	std::string theFooterString(", ,Footer,,");
	theFooterString += std::to_string(receiverID) + ",,,,";
	for (size_t i = 0; i < WAVEFORM_FOOTER_SIZE; i++)
	{
		if (i == WAVEFORM_FOOTER_SIZE - 1) {
			theFooterString += std::to_string(*pFooter++);
		}
		else
		{
			theFooterString += std::to_string(*pFooter++) + ",";
		}
	}
	LogFilePrintf(*logFilePtr, theFooterString.c_str());

	logFileMutex.unlock();
}


bool ReceiverPolledCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
	ReceiverCANCapture::ReadConfigFromPropTree(propTree);


	std::string receiverKey = std::string("config.receivers.receiver") + std::to_string(receiverID);

	boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
	// Communication parameters

	xmitsFooterData =  receiverNode.get<bool>("xmitsFooterData", true);

	return(true);
}

static int channelToColumnArray[] = {
  14,
  12,
  10,
  8,
  6,
  4,
  2,
  0,
  1,
  3,
  5,
  7,
  9,
  11,
  13,
  15
};

static int columnToChannelArray[]={
7,
8,
6,
9,
5,
10,
4,
11,
3,
12,
2,
13,
1,
14,
0
};



CellID ReceiverPolledCapture::GetCellIDFromChannel(int inChannelID)
{
	int row = 0;
	int column = channelToColumnArray[inChannelID];
	return (CellID(column, row));
}

int ReceiverPolledCapture::GetChannelIDFromCell(CellID inCellID)
{
	for (int i = 0; i < receiverVoxelQty; i++)
	{
		if (channelToColumnArray[i] == inCellID.column) return (i);
	}

	return (0);
}


void ReceiverPolledCapture::ProcessRaw(uint8_t* rawData)
{
	size_t sampleOffset = 0;
	size_t sampleDrop = 0;
	size_t sampleSize = 1;
	bool sampleSigned = false;

	uint16_t* rawData16;

	rawData16 = (uint16_t*)rawData;

	++m_nbrRawCumul;

	sampleOffset = 0;
	sampleSize = 2;
	sampleSigned = true;
	sampleCount = 100;
	sampleDrop = 1;


	for (int channel = 0; channel < 16; channel++)
	{
		if (!rawBuffers[channel])
			rawBuffers[channel] = new uint8_t[maxRawBufferSize];

		rawBufferCount++;


		CellID cellID = GetCellIDFromChannel(channel);
		memcpy(rawBuffers[channel], rawData + (channel * (100 * 2)), 100 * 2);
	}

	boost::mutex::scoped_lock rawLock(GetMutex());

	for (int column = 0; column < 16; column++)
	{
		CellID cellID(column, 0);
		int channelID = GetChannelIDFromCell(cellID);
		AScan::Ptr aScan = currentFrame->MakeUniqueAScan(currentFrame->aScans, receiverID, cellID, channelID);
		aScan->samples = rawBuffers[channelID];
		aScan->sampleSize = sampleSize;
		aScan->sampleOffset = sampleOffset;
		aScan->sampleCount = sampleCount - sampleDrop;
		aScan->sampleSigned = sampleSigned;
	}
}

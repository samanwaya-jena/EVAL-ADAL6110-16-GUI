/* ReceiverPolledCapture.cpp */
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
		bReadSuccess = ReadDataFromUSB((char*)dataFifo, payloadSize, cycleCount);



		if (SensorSettings::GetGlobalSettings()->bWriteLogFile)
			{
				boost::mutex::scoped_lock rawLock(m_Mutex);
			
				if (SensorSettings::GetGlobalSettings()->bWriteLogFile) // Check again for file handle now that we have the mutex
				{
					for (size_t cycle = 0; cycle < cycleCount; cycle++)
					{
						LogWaveform(cycle);
					}
				}

			}

		ProcessRaw(rawFromLibUSB, (uint8_t*)dataFifo->AcqFifo, CHANNEL_QTY * WAVEFORM_POINT_QTY * sizeof(short));
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
        SetMessageFilters(receiverStatus.frameRate, receiverStatus.channelMask, receiverStatus.messageMask);
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

  received = ReadBytes((uint8_t*) &canResp[0], messageCount * sizeof(ReceiverCANMessage));
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

void ReceiverPolledCapture::LogWaveform(int cycle)
{
	logFileMutex.lock();

	if (!logFilePtr)
	{
		logFileMutex.unlock();
		return;
	}

	for (int ch = 0; ch < CHANNEL_QTY; ch++)
	{
		std::string theWaveString(", ,Wave,,Channel,");
		theWaveString += std::to_string(ch) + ", ,";

		short* pData = &dataFifo[cycle].AcqFifo[ch * WAVEFORM_POINT_QTY];
		for (int i = 0; i < WAVEFORM_POINT_QTY; i++)
		{
			if (i == (WAVEFORM_POINT_QTY - 1))
				theWaveString += std::to_string(*pData++);
			else
				theWaveString += std::to_string(*pData++) + ",";

		}
		LogFilePrintf(*logFilePtr, theWaveString.c_str());
	}

	short* pFooter = &dataFifo[cycle].footer[0];
	std::string theFooterString(", ,Footer,,,,,");
	for (int i = 0; i < WAVEFORM_FOOTER_SIZE; i++)
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

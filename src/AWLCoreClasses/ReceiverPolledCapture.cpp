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
#include "ReceiverPolledCapture.h"


using namespace std;
using namespace awl;


#define MAX_POLL_CAN_MESSAGES 32


ReceiverPolledCapture::ReceiverPolledCapture(int receiverID, boost::property_tree::ptree &propTree):
ReceiverCANCapture(receiverID, propTree),
handle(NULL)
{
  reconnectTime = boost::posix_time::microsec_clock::local_time();

  m_pFile = NULL;
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

bool ReceiverPolledCapture::LidarQuery(uint32_t * pdwCount, uint32_t * pdwReadPending)
{
  int ret = 0;
  int transferred = 0;
  int received = 0;

  boost::mutex::scoped_lock rawLock(m_Mutex);

  if (!handle)
    return false;

  AWLCANMessage msg;
  msg.id = AWLCANMSG_ID_LIDARQUERY;

  AWLCANMessage resp;

  transferred = WriteBytes((uint8_t*)&msg, sizeof(msg));
  if (transferred != sizeof(AWLCANMessage))
    return false;

  received = ReadBytes((uint8_t*)&resp, sizeof(resp));
  if (received != sizeof(AWLCANMessage))
    return false;

  *pdwCount = *((uint32_t*)&resp.data[0]);
  *pdwReadPending = *((uint32_t*)&resp.data[4]);

  return true;
}


bool ReceiverPolledCapture::ReadDataFromUSB(char * ptr, int uiCount, uint32_t dwCount)
{
  int transferred = 0;
  int received = 0;
  int ret = 0;

  boost::mutex::scoped_lock rawLock(m_Mutex);

  if (!handle)
    return false;

  AWLCANMessage msg;
  msg.id = AWLCANMSG_ID_GETDATA;
  msg.data[0] = (unsigned char) dwCount;

  transferred = WriteBytes((uint8_t*)&msg, sizeof(msg));
  if (transferred != sizeof(AWLCANMessage))
    return false;

  received = ReadBytes((uint8_t*)ptr, uiCount);
  if (received != uiCount)
    return false;

  return true;
}

#include "algo.h"

typedef struct
{
  short AcqFifo[GUARDIAN_NUM_CHANNEL * GUARDIAN_SAMPLING_LENGTH];
} tDataFifo;

tDataFifo dataFifo[8];

bool ReceiverPolledCapture::DoOneLoop()
{
  uint32_t dwCount = 0;
  uint32_t dwReadPending = 0;

  bool bRet = LidarQuery(&dwCount, &dwReadPending);

  if (!bRet)
    return false;

  if (dwCount)
  {
    bRet = ReadDataFromUSB((char*)dataFifo, dwCount * sizeof(tDataFifo), dwCount);

    if (m_pFile)
    {
			boost::mutex::scoped_lock rawLock(m_Mutex);

			if (m_pFile) // Check again for m_pFile now that we have the mutex
			{
				int cycle, ch, i, j;

				for (cycle = 0; cycle < dwCount; cycle++)
				{
					for (ch = 0; ch < GUARDIAN_NUM_CHANNEL; ch++)
					{
						short * pData = &dataFifo[cycle].AcqFifo[ch * GUARDIAN_SAMPLING_LENGTH];
						for (i = 0; i < GUARDIAN_SAMPLING_LENGTH; i++)
						{
							char str[128];
							if (ch == (GUARDIAN_NUM_CHANNEL - 1) && i == (GUARDIAN_SAMPLING_LENGTH - 1))
								sprintf(str, "%d\n", *pData++);
							else
								sprintf(str, "%d,", *pData++);
							fwrite(str, strlen(str), 1, m_pFile);
						}
					}
				}
				fflush(m_pFile);
			}
    }

    ProcessRaw(rawFromLibUSB, (uint8_t*)dataFifo->AcqFifo, GUARDIAN_NUM_CHANNEL * GUARDIAN_SAMPLING_LENGTH * sizeof(short));
  }

  if (dwReadPending)
  {
    bRet = PollMessages(dwReadPending);
  }

  if (!dwCount && !dwReadPending)
  {
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }

  if (!bRet)
    return false;

  return bRet;
}

void ReceiverPolledCapture::DoOneThreadIteration()
{
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

bool ReceiverPolledCapture::PollMessages(uint32_t dwNumMsg)
{
  AWLCANMessage canReq;
  AWLCANMessage canResp[MAX_POLL_CAN_MESSAGES];
  int transferred = 0;
  int received = 0;
  int ret;

  boost::mutex::scoped_lock rawLock(m_Mutex);

  if (!handle)
    return false;

  if (dwNumMsg > MAX_POLL_CAN_MESSAGES)
    dwNumMsg = MAX_POLL_CAN_MESSAGES;

  AWLCANMessage msg;
  msg.id = AWLCANMSG_ID_POLLMESSAGES;
  msg.data[0] = (unsigned char) dwNumMsg;

  transferred = WriteBytes((uint8_t*)&msg, sizeof(msg));
  if (transferred != sizeof(msg))
    return false;

  received = ReadBytes((uint8_t*) &canResp[0], dwNumMsg * sizeof(AWLCANMessage));
  if (received != dwNumMsg * sizeof(AWLCANMessage))
    return false;

  for (uint32_t i=0; i<dwNumMsg; i++)
  {
    if (canResp[i].id)
      ParseMessage(canResp[i]);
  }

  return true;
}

bool ReceiverPolledCapture::WriteMessage(const AWLCANMessage &inMsg)
{
  AWLCANMessage canResp;
  int transferred = 0;
  int received = 0;
  int ret;

  if (!handle)
    return false;

  boost::mutex::scoped_lock rawLock(m_Mutex);

  transferred = WriteBytes((uint8_t*)&inMsg, sizeof(inMsg));
  if (transferred != sizeof(AWLCANMessage))
    return false;

  received = ReadBytes((uint8_t*) &canResp, sizeof(canResp));
  if (received != sizeof(AWLCANMessage))
    return false;

  if (canResp.id)
    ParseMessage(canResp);

	return true;
}

bool ReceiverPolledCapture::SendSoftwareReset()
{
  if (!handle)
    return false;

  AWLCANMessage msg;
  msg.id = AWLCANMSG_ID_COMMANDMESSAGE;
  msg.len = AWLCANMSG_LEN;
  msg.data[0] = AWLCANMSG_ID_CMD_SET_PARAMETER;
  msg.data[1] = AWLCANMSG_ID_CMD_PARAM_ADC_REGISTER;
  msg.data[2] = 0x00;
  msg.data[3] = 0x00;
  msg.data[4] = 0x00;
  msg.data[5] = 0x00;
  msg.data[6] = 0x00;
  msg.data[7] = 0x00;

  return WriteMessage(msg);
}

bool ReceiverPolledCapture::SetRecordFileName(std::string inRecordFileName)
{
	receiverStatus.sRecordFileName = inRecordFileName;
	return true;
}

bool ReceiverPolledCapture::StartRecord(uint8_t frameRate, ChannelMask channelMask)
{
	boost::mutex::scoped_lock rawLock(m_Mutex);

	if (!m_pFile && !receiverStatus.sRecordFileName.empty())
	{
		m_pFile = fopen(receiverStatus.sRecordFileName.c_str(), "a");
		receiverStatus.bInRecord = true;
	}

	return true;
}

bool ReceiverPolledCapture::StopRecord()
{
	boost::mutex::scoped_lock rawLock(m_Mutex);

	if (m_pFile)
	{
		fclose(m_pFile);
		m_pFile = NULL;
	}

	receiverStatus.bInRecord = false;

	return true;
}

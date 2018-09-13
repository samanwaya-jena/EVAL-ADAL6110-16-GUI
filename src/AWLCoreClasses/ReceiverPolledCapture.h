#ifndef AWL_RECEIVER_POLLED_CAPTURE_H
#define AWL_RECEIVER_POLLED_CAPTURE_H

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

#include "ReceiverCANCapture.h"

using namespace std;

namespace awl
{

/** \brief ReceiverPolledCapture class is a virtual implementation of the ReceiverCapture
   *        working by polling the device. Needs to be derived by an actual implementation (USB, TCP)
  */
class ReceiverPolledCapture: public ReceiverCANCapture
{
// Public types
public:

// public Methods
public:

  ReceiverPolledCapture(int receiverID,  boost::property_tree::ptree  &propTree);

	virtual ~ReceiverPolledCapture();
  virtual void  Go();
  virtual void  Stop();
  virtual bool IsConnected();

protected:

  void DoThreadLoop();
  bool DoOneLoop();
	virtual void DoOneThreadIteration();

  bool LidarQuery(uint32_t * pdwCount, uint32_t * pdwReadPending);
  bool ReadDataFromUSB(char * ptr, int uiCount, uint32_t dwCount);
	virtual bool WriteMessage(const AWLCANMessage &inMsg);
  bool PollMessages(uint32_t dwNumMsg);

  bool SendSoftwareReset();

  virtual int ReadBytes(uint8_t * pData, int num) = 0;
  virtual int WriteBytes(uint8_t * pData, int num) = 0;

// Protected variables
protected:

		void*handle;

		/** \brief Time-out without an input message after which we try to recomnnect the serial port. */
		boost::posix_time::ptime reconnectTime;

    boost::mutex m_Mutex;

};

} // namespace AWL

#endif //AWL_RECEIVER_POLLED_CAPTURE_H

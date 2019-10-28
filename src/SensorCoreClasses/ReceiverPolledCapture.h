#ifndef SENSORCORE_RECEIVER_POLLED_CAPTURE_H
#define SENSORCORE_RECEIVER_POLLED_CAPTURE_H

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

#include "SensorCoreClassesGlobal.h"
#include "ReceiverCANCapture.h"

SENSORCORE_BEGIN_NAMESPACE


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

  void * GetHandle(void);
  void SetHandle(void *);

protected:

  void DoThreadLoop();
  bool DoOneLoop();
	virtual void DoOneThreadIteration();

  bool LidarQuery(size_t & cycleCount, size_t & messageCount);
  bool ReadDataFromUSB(char * dataBuffer, int payloadSize, size_t cycleCount);
	virtual bool WriteMessage(const ReceiverCANMessage &inMsg);
  bool PollMessages(size_t messageCount);

  bool SendSoftwareReset();

  virtual int ReadBytes(uint8_t * pData, int num) = 0;
  virtual int WriteBytes(uint8_t * pData, int num) = 0;

  /** \brief Reads the configuration proerties from the configuration file
	* \param[in] propTree the boost propertyTree created from reading the configuration file.
	* \returns Returns true otherwise.
	* \throws  Throws boost error on read of the property keys.
	*/
  virtual bool ReadConfigFromPropTree(boost::property_tree::ptree& propTree);
  void LogWaveform(size_t cycle);


// Protected variables
protected:

		void*handle;
		void *swap_handle;

		/** \brief Time-out without an input message after which we try to recomnnect the serial port. */
		boost::posix_time::ptime reconnectTime;

		bool xmitsFooterData;   // There is some extra data in the wave acquisition payload for some USB sensors (Gordon and later Guardians).  Earlier versions did not produce footer payload


    boost::mutex m_Mutex;
};

SENSORCORE_END_NAMESPACE

#endif //SENSORCORE_RECEIVER_POLLED_CAPTURE_H

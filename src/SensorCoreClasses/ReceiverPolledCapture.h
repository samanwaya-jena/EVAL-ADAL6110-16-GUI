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
****************************************************************************/#ifndef SENSORCORE_RECEIVER_POLLED_CAPTURE_H

#define SENSORCORE_RECEIVER_POLLED_CAPTURE_H


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


  /** \brief Returns the CellID, given a channelID
   * \param[in] channelID the input "channel"
   * \return ChannelID, indicating unique pixel position in the detector array
   * \remarks For the ReceiverPolledCapture, channels may be out of order on receive for some messages.
   */
  virtual CellID GetCellIDFromChannel(int inChannelID);

  /** \brief Returns the CellID, given a channelID
  * \param[in] channelID the input "channel"
  * \return ChannelID, indicating unique pixel position in the detector array
  * \remarks For the PolledCapture, channels may be out of order on receive for some meesage.
 */

  virtual int GetChannelIDFromCell(CellID inCellID);

  /** \brief Reads the configuration proerties from the configuration file
	* \param[in] propTree the boost propertyTree created from reading the configuration file.
	* \returns Returns true otherwise.
	* \throws  Throws boost error on read of the property keys.
	*/
  virtual bool ReadConfigFromPropTree(boost::property_tree::ptree& propTree);
  void LogWaveform(size_t cycle);

 
	void ProcessRaw(uint8_t* rawData);

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

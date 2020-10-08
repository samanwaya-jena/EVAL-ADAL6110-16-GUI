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

    /** \brief ReceiverPolledCapture constructor from a configuration file information.
        * \param[in] inReceiverID  unique receiverID
        * \param[in] propTree propertyTree that contains teh configuration file information.
      */
    ReceiverPolledCapture(int receiverID,  boost::property_tree::ptree  &propTree);

    /** \brief ReceiverPolledCapture Destructor.  Insures that all threads are stopped before destruction.
      */
    virtual ~ReceiverPolledCapture();

    /** \brief Start the lidar Data Projection  thread
      */
    virtual void  Go();

    /** \brief Stop the lidar data projection thread
      */
    virtual void  Stop(); 
    
    /** \brief Return true if connexion with the device has been etablished.
      * \return True if device connexion is established.
      */
    virtual bool IsConnected();

  void * GetHandle(void);
  void SetHandle(void *);

protected:

    /** \brief Do one iteration of the thread loop.
      *        Call DoOneLoop() to acquire data and insure that the port stays open..
      */

  void DoThreadLoop();

  /** \brief Within one iteration of the thread loop, read a block of polled Data, if available.
    *         First, poll the Receiver for additional messages, and then get all returned data in one block.
    */
  bool DoOneLoop();

  /** \brief Do one iteration of the thread loop.
    *        Acquire CAN Data until ParseMessage() can be called with a CAN Message.
    *        Try to manage automatic connection/reconnection of the CAN communications in the loop.
    */
  virtual void DoOneThreadIteration();

  /** \brief Send the polling message to the receive and get the returned cycleCount and messageCount.
    */
  bool LidarQuery(size_t & cycleCount, size_t & messageCount);

  /** \brief Read formatted data from USB into dataBuffer.
   *         First, send a query to the receiver for "cycleCount" cycles of data.
   *         Expect payloadSize bytes as returned payload.
  */  
  bool ReadDataFromUSB(char * dataBuffer, int payloadSize, size_t cycleCount);

  /** \brief Write the provided CAN message to the USB port
    */
virtual bool WriteMessage(const ReceiverCANMessage &inMsg);

/** \brief Write to the Receiver to request "messageCOunt" CAN messages.
  *        Acquire the messages from the USB port, and process their contents..
    */
  bool PollMessages(size_t messageCount);

  /** \brief Send a Software Reset command to the USB port.
      */
  bool SendSoftwareReset();

  /** \brief Read num bytes from the USB port, into the memory pointed to by pData, using configured TimeOut
    * \param[in] pData pointer to the read data
    * \param[in] num quantity  of bytes to read
    * \return quantity of bytes read.
   */
  virtual int ReadBytes(uint8_t * pData, int num) = 0;

  /** \brief Write num bytes to the USB port, from  the memory pointed to by pData, using configured time out.
    * \param[in] pData pointer to the data to be written
    * \param[in] num quantity  of bytes to write
    * \return quantity of bytes actually written.
   */
  virtual int WriteBytes(uint8_t * pData, int num) = 0;


  /** \brief Returns the CellID, given a channelID
   * \param[in] channelID the input "channel"
   * \return ChannelID, indicating unique pixel position in the detector array
   * \remarks For the ReceiverPolledCapture, channels are out of order on receive for raw data messages.
   */
  virtual CellID GetCellIDFromChannel(int inChannelID);

  /** \brief Returns the CellID, given a channelID
  * \param[in] channelID the input "channel"
  * \return ChannelID, indicating unique pixel position in the detector array
  * \remarks For the ReceiverPolledCapture, channels are out of order on receive for raw data messages.
  */

  virtual int GetChannelIDFromCell(CellID inCellID);

  /** \brief Reads the configuration proerties from the configuration file
	* \param[in] propTree the boost propertyTree created from reading the configuration file.
	* \returns Returns true otherwise.
	* \throws  Throws boost error on read of the property keys.
	*/
  virtual bool ReadConfigFromPropTree(boost::property_tree::ptree& propTree);
  void LogWaveform(size_t cycle);

  /** \brief Process Raw Data messages from no-CAN devices
        * \param[in] rawData pointer to the raw data memeory block
      */
	virtual void ProcessRaw(uint8_t* rawData);

// Protected variables
protected:

    /** \brief USB port handle */
    void* handle;

    /** \brief Temporary variable used to hold USB port handle.  Used when we force reassignment of USB port order */
    void* swap_handle;


		/** \brief Time-out without an input message after which we try to recomnnect the serial port. */
		boost::posix_time::ptime reconnectTime;

        /**There is some extra data in the wave acquisition payload for some USB sensors (Gordon and later Guardians).  Earlier versions did not produce footer payload*/
		bool xmitsFooterData;   


    boost::mutex m_Mutex;
};

SENSORCORE_END_NAMESPACE

#endif //SENSORCORE_RECEIVER_POLLED_CAPTURE_H

#ifndef AWL_RECEIVER_SIMULATOR_CAPTURE_H
#define AWL_RECEIVER_SIMULATOR_CAPTURE_H

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

#include <stdint.h>

//#ifndef Q_MOC_RUN
//#include <boost/asio.hpp> 
//#include <boost/asio/serial_port.hpp> 
//#endif

#include "DetectionStruct.h"
#include "ReceiverCapture.h"

using namespace std;

namespace awl
{

/** \brief TheReceiverSimulatorCapture class is a specialized implementation of the ReceiverCapture
  *        that provides simulated data without any extanal device attached.
  * \Remarks Often useful for demos or debugging purposes.
  *          Since it does not manage connections to the device, it is a poor example for development, though.
  * \author Jean-Yves Desch�nes
  */

class ReceiverSimulatorCapture: public ReceiverCapture
{
// Public types
public:
	typedef boost::shared_ptr<ReceiverSimulatorCapture> Ptr;
    typedef boost::shared_ptr<ReceiverSimulatorCapture> ConstPtr;

// public Methods
public:
	/** \brief ReceiverSimulatorCapture constructor.
 	    * \param[in] inReceiverID  unique receiverID
	    * \param[in] inReceiverChannelQty number of channels in the receiver
		* \param[in] inReceiverColumns number of columns in receiver array
		* \param[in] inReceiverRows number of rows  in the receiver array
		* \param[in] inLineWrapAround "distance" coded between rows in the original communications protocol for arrayed sensors
		* \param[in] inFrameRate frameRate of the receiver
	    * \param[in] inChannelMask  channelMask indicating which channels are activated in the receiver
	    * \param[in] inMessageMask mask of the messages that are enabled in the communications protocol
	    * \param[in] inRangeOffset rangeOffset that corresponds to a calibration error in the sensor.
		*                          Will automatically be added to any range received.
		* \param[in] inRegistersFPGA default description of the FPGA registers
		* \param[in] inRegistersADC default description of the ADC registers
		* \param[in] inRegistersGPIO default description of the GPIO registers
        * \param[in] inParametersAlgos default description if the algorithm parameters
      */

	ReceiverSimulatorCapture(int receiverID, int inReceiverChannelQty, int inReceiverColumns, int inReceiverRows, float inLineWrapAround,
					   int inFrameRate, ChannelMask &inChannelMask, MessageMask &inMessageMask, float inRangeOffset, 
		               const RegisterSet &inRegistersFPGA, const RegisterSet & inRegistersADC, const RegisterSet &inRegistersGPIO, 
					   const AlgorithmSet &inParametersAlgos, const AlgorithmSet &inParametersTrackers);

	/** \brief ReceiverSimulatorCapture constructor from a configuration file information.
 	    * \param[in] inReceiverID  unique receiverID
	    * \param[in] propTree propertyTree that contains teh confoguration file information.
      */

	ReceiverSimulatorCapture(int receiverID,  boost::property_tree::ptree  &propTree);


	/** \brief ReceiverSimulatorCapture Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~ReceiverSimulatorCapture();
public:
	/** \brief Starts the internal calibration of the system. 
      * \param[in] frameQty number of frames on which calibration is calculated
      * \param[in] beta beta parameter for the calibration
      * \param[in] channelMask mask for the recorded channels. that an empty channelMask is equivalent to StopRecord().
      * \return true if success.  false on error
	  * \remarks In Simulation, does nothing. Returns true.
     */
	virtual bool StartCalibration(uint8_t frameQty, float beta, ChannelMask channelMask) {return(true);};

		/** \brief Issues the command to set the current algorithm in the sensor.
	  *\param[in] algorigthmID  ID of the selected algorithm.
	* \return true if success.  false on error.
	  * \remarks In Simulation, does nothing. Returns true.
	*/
		
	virtual bool SetAlgorithm(uint16_t algorithmID) {return(true);};

	/** \brief Issues the command to set the current Tracker in the sensor.
	*\param[in] trackerID  ID of the selected algorithm.
	* \return true if success.  false on error.
	* \remarks In Simulation, does nothing. Returns true.
	*/

	virtual bool SetTracker(uint16_t trackerID) { return(true); };

	/** \brief Sets an internal FPGA register to the value sent as argument. 
	  *\param[in] registerAddress Adrress of the register to change.
	  *\param[in] registerValue Value to put into register.
	* \return true if success.  false on error.
	  * \remarks In Simulation, does nothing. Returns true.
	*/
	
        virtual bool SspSetFrameRate(int FrameRate ) { return(true); };

        /** \brief Issues the command to set the frame rate from 10 to 50 by 5 Hz step
        * \return true if success.  false on error.
        */

        virtual bool EnableSystem(bool on) { return(true); };

        /** \brief Issues the command to enable the sensor.
        * \return true if success.  false on error.
        */

        virtual bool EnableLaser(bool on) { return(true); };

        /** \brief Issues the command to enable the Laser.
        * \return true if success.  false on error.
        */

        virtual bool EnableAutoGain(bool on) { return(true); };

        /** \brief Issues the command to enable the Auto Gain.
        * \return true if success.  false on error.
        */

        virtual bool EnableDCBalance(bool on) { return(true); };

        /** \brief Issues the command to enable the DC Balance.
        * \return true if success.  false on error.
        */



	virtual bool SetFPGARegister(uint16_t registerAddress, uint32_t registerValue) {return(true);};

	/** \brief Sets an ADC register to the value sent as argument. 
	  *\param[in] registerAddress Adrress of the register to change.
	  *\param[in] registerValue Value to put into register.
	* \return true if success.  false on error.
	  * \remarks In Simulation, does nothing. Returns true.
	*/
	virtual bool SetADCRegister(uint16_t registerAddress, uint32_t registerValue) {return(true);};

	/** \brief Sets an internal GPIO register to the value sent as argument. 
	  *\param[in] registerAddress Adrress of the register to change.
	  *\param[in] registerValue Value to put into register (values accepted are 0-1).
	* \return true if success.  false on error.
	  * \remarks In Simulation, does nothing. Returns true.
	*/
		
	virtual bool SetGPIORegister(uint16_t registerAddress, uint32_t registerValue) {return(true);};

	/** \brief Sets algorithm parameters to the value sent as argument. 
	  *\param[in] algoID ID of the detection algo affected by the change.
	  *\param[in] registerAddress Adrress of the parameter to change.
	  *\param[in] registerValue Value to put into register (values accepted are 0-1).
	* \return true if success.  false on error.
	  * \remarks In Simulation, does nothing. Returns true.
	*/
		
	virtual bool SetAlgoParameter(int algoID, uint16_t registerAddress, uint32_t registerValue) {return(true);};


	/** \brief Sets global  algorithm parameters to the value sent as argument. 
	  *\param[in] registerAddress Adrress of the parameter to change.
	  *\param[in] registerValue Value to put into register (values accepted are 0-1).
	* \return true if success.  false on error.
	  * \remarks In Simulation, does nothing. Returns true.
	*/
		
	virtual bool SetGlobalAlgoParameter(uint16_t registerAddress, uint32_t registerValue) {return(true);};

	/** \brief Sets tracker parameters to the value sent as argument.
	*\param[in] trackerID ID of the tracker algo affected by the change.
	*\param[in] registerAddress Adrress of the parameter to change.
	*\param[in] registerValue Value to put into register (values accepted are 0-1).
	* \return true if success.  false on error.
	* \remarks In Simulation, does nothing. Returns true.
	*/

	virtual bool SetTrackerParameter(int trackerID, uint16_t registerAddress, uint32_t registerValue) { return(true); };

	/** \brief Changes the controls of which messages are sent from AWL to the client to reflect provided settings
	* \param[in] frameRate new frame rate for the system. A value of 0 means no change
	* \param[in] channelMask mask for the analyzed channels.
	* \param[in] messageMask mask identifies which groups of target/distance/intensity messages are transmitted over CAN.
	* \return true if success.  false on error.
	*/

	virtual bool SetMessageFilters(uint8_t frameRate, ChannelMask channelMask, MessageMask messageMask);

	/** \  an asynchronous query command to get the current algorithm.
	* \return true if success.  false on error.
	  * \remarks In Simulation, does nothing. Returns true.
	*/
	virtual bool QueryAlgorithm() {return(true);};


	/** \  an asynchronous query command to get the current Tracker.
	* \return true if success.  false on error.
	* \remarks In Simulation, does nothing. Returns true.
	*/
	virtual bool QueryTracker() { return(true); };


	/** \brief Send an asynchronous query command for an internal FPGA register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks In Simulation, does nothing. Returns true.
		*/
	virtual bool QueryFPGARegister(uint16_t registerAddress) {return(true);};

	/** \brief Send an asynchronous query command for an ADC register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks In Simulation, does nothing. Returns true.
		*/
	virtual bool QueryADCRegister(uint16_t registerAddress) {return(true);};

	/** \brief Send an asynchronous query command for a GPIO register. 
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks In Simulation, does nothing. Returns true.
		*/
	virtual bool QueryGPIORegister(uint16_t registerAddress) {return(true);};

	/** \brief Send an asynchronous query command for an algorithm parameter. 
		  *\param[in] algoID ID of the detection algo for which we want to query.
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks In Simulation, does nothing. Returns true.
		*/
	virtual bool QueryAlgoParameter(int algoID, uint16_t registerAddress) {return(true);};

		/** \brief Send an asynchronous query command for a global algorithm parameter. 
		  *\param[in] algoID ID of the detection algo for which we want to query.
		 *\param[in] registerAddress Adrress of the register to query.
	  * \return true if success.  false on error.
	  * \remarks In Simulation, does nothing. Returns true.
		*/

        virtual bool SspGetFrameRate() { return(true); };

        /** \brief Issues the command to get the frame rate.
        * \return true if success.  false on error.
        */

        virtual bool StatusSystem() { return(true); };

        /** \brief Issues the command to know if the sensor.
        * \return true if success.  false on error.
        */

        virtual bool StatusLaser() { return(true); };

        /** \brief Issues the command to know if the Laser is on.
        * \return true if success.  false on error.
        */

        virtual bool StatusAutoGain() { return(true); };

        /** \brief Issues the command to know if the Auto Gain is on.
        * \return true if success.  false on error.
        */

        virtual bool StatusDCBalance() { return(true); };

        /** \brief Issues the command to know if the DC Balance is on.
        * \return true if success.  false on error.
        */

	virtual bool QueryGlobalAlgoParameter(uint16_t registerAddress) {return(true);};

	/** \brief Send an asynchronous query command for a Tracker parameter.
	*\param[in] trackerID ID of the Tracker algo for which we want to query.
	*\param[in] registerAddress Adrress of the register to query.
	* \return true if success.  false on error.
	* \remarks In Simulation, does nothing. Returns true.
	*/
	virtual bool QueryTrackerParameter(int algoID, uint16_t registerAddress) { return(true); };

// Protected methods
protected:

	/** \brief Do one iteration of the thread loop.
      */
	virtual void DoOneThreadIteration();

	/** \brief Reads the configuration proerties from the configuration file
	  * \param[in] propTree the boost propertyTree created from reading the configuration file.
	  * \returns Returns true otherwise.
	  * \throws  Throws boost error on read of the property keys.
      */
	virtual bool ReadConfigFromPropTree( boost::property_tree::ptree &propTree);

// Protected variables
protected:
};

} // namespace AWL

#endif //AWL_RECEIVER_SIMULATOR_CAPTURE_H

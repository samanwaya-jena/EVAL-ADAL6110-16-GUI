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

#ifndef _SensorSettings__H
#define _SensorSettings__H

#include <string>
#include <stdint.h>
#include <boost/container/vector.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "SensorCoreClassesGlobal.h"
#include "CoordinateSystem.h"
#include "DetectionStruct.h"

#define VelocityToKmH(velocity) (velocity * 3.6)

SENSORCORE_BEGIN_NAMESPACE

	/** \brief VolxelConfig holds parameters of individual voxels in a Receiver.
           Mostly used to  manage display mechanisms at the UI level.
	* \author Jean-Yves Deschênes
	*/
	typedef struct VoxelConfig 
	{
		/** Index of the voxel within configuration */
		int voxelIndex;
		/** FOV Width in degrees*/
		float fovWidth;
		/** FOV Height in degrees*/
		float fovHeight;

		/** Max range attainable for voxel.  Will be used to set display ranges in UI*/
		float maxRange;
		/** Max range displayable in the A-SCan windowsfor voxel.  Will be used to set display ranges in UI*/
		float maxAscanRange;

		/** Red Color component used for displaying the detection data for that voxel.*/
		uint8_t displayColorRed;
		/** Green Color component used for displaying the detection data for that voxel.*/
		uint8_t displayColorGreen;
		/** Blue Color component used for displaying the detection data for that voxel.*/
		uint8_t displayColorBlue;
	}
	VoxelConfig;

	/** \brief Array of VoxelConfig
	*/
	typedef boost::container::vector<VoxelConfig> VoxelConfigVector;

	/** \brief Used by the UI to determine if display units used
	  *        to display speed.
	* \author Jean-Yves Deschênes
	*/
	typedef enum {
		/**Display in meters per second*/
		eVelocityUnitsMS = 0,
		/**Display in kilometers per hour*/
		eVelocityUnitsKMH = 1
	}
	VelocityUnits;

	/** \brief ReceiverSettings hold the configuration parameters of a Receiver.
	* \author Jean-Yves Deschênes
	*/
	typedef struct ReceiverSettings
	{

	/** Channel configuration: number of voxel rows */
	int receiverRows;
	/** Channel configuration: number of voxel columns */
	int receiverColumns;
	/** Channel configuration: For each voxel, position, orientation and user interface hints */
	VoxelConfigVector voxelsConfig;

	/** Receiver type string: will determine which Receiver sub-class will be instantiated */
	std::string sReceiverType;
	/** Identification of the Register Configuration subsection in the XML config file */
	std::string sReceiverRegisterSet;

	/** Identification of the Receiver Geometry Configuration subsection in the XML config file */
	std::string sReceiverVoxelGeometry;


	/** Minimum Displayed Range */
	float displayedRangeMin;
	/** Maximum Displayed Range */
	float displayedRangeMax;
	}
	ReceiverSettings;

	/** \brief Vector holding all ReceiverSettings
	 * \author Jean-Yves Deschênes
     */
	typedef boost::container::vector<ReceiverSettings> ReceiverSettingsVector;


	/** \brief CameraSettings holds information about cameras, such as
	 *         naming and calibration parameters.
	* \author Jean-Yves Deschênes
	*/
typedef struct CameraSettings
	{
	/** CameraName */
	std::string sCameraName;
	/** Camera API library identifier. Should be "OpenCV" for cameras supported by OpenCV library */
	std::string sCameraAPI;
	
	/** True to force vertical flip of displayed image. (Some cameras can only be placed upside down within some settings!) */
	bool cameraFlip;

	/** Calibration info for the camera */
	CameraCalibration calibration;
	/** Camera FOV: Width in degrees. */
	float cameraFovWidthDegrees;
	/** Camera FOV: Height in degrees. */
	float cameraFovHeightDegrees;
	/** Camera Barrel - K1 */
	float barrelK1;
	/** Camera Barrel - K2 */
	float barrelK2;
	}
	CameraSettings;

	/** \brief Vector holding all CameraSettings
	* \author Jean-Yves Deschênes
	*/
	typedef boost::container::vector<CameraSettings> CameraSettingsVector;

	/** \brief SensorSettings manages parameters from the Settings file for the application.
	  * It holds Receiver, Camera and Geometry information.  It also holds Debug and LogFile properties. 
	  * A single instance of SensorSettings exists for the application.
	* \author Jean-Yves Deschênes
	*/
	class SensorSettings
	{
	public:

		/** \brief Initialize the settings: Instantiate a SensorSettings object, and place it in the global pointer. */
		static SensorSettings* InitSettings(const std::string sInSettingsFileName = std::string(""));
		/** \brief Return a pointer to the global variable that points to the unique instance of sensor settings*/
		static SensorSettings* GetGlobalSettings();

		/** \brief Return a pointer to the global variable that points to the unique instance of sensor settings*/
		SensorSettings(const std::string sInSettingsFileName = std::string(""));
		/** \brief Read settings from the config file*/
		bool ReadSettings();

		/** \brief Read settings from the config file*/
		boost::property_tree::ptree& GetPropTree() { return (propTree); };


		// Get/Set methods

		/** \brief Read a Position from the specified property tree node*/
		static void GetPosition(boost::property_tree::ptree& node, float& forward, float& left, float& up);

		/** \brief Read an Orientation from the specified property tree node*/
		static void GetOrientation(boost::property_tree::ptree& node, float& pitch, float& yaw, float& roll);
		/** \brief Read a 2D point (X, Y)  from the specified property tree node*/
		static void Get2DPoint(boost::property_tree::ptree& node, float& x, float& y);

		/** \brief Get a complete position and oreintation from the specified property tree node*/
		static void GetGeometry(boost::property_tree::ptree& geometryNode, float& forward, float& left, float& up, float& pitch, float& yaw, float& roll);
		/** \brief Read a color (R,G, B) from the specified property tree node*/
		static void GetColor(boost::property_tree::ptree& colorNodeNode, uint8_t& red, uint8_t& green, uint8_t& blue);

		/** \brief Read a complete AlertCondition structurefrom the specified property tree node*/
		static void GetAlertConditions(boost::property_tree::ptree& alertNode, AlertCondition& alert);

		/** \brief Read the geometry of a voxel  from the specified property tree node*/
		static void GetVoxelGeometry(boost::property_tree::ptree& voxelGeometryNode, ReceiverSettings* receiverPtr);
		/** \brief Read the geometry of all receiver's voxels  from the specified property tree node*/
		static void GetVoxelGeometryArray(boost::property_tree::ptree& voxelGeometryNode, ReceiverSettings* receiverPtr);


		/** \brief Write a position to the specified property tree node*/
		static void PutPosition(boost::property_tree::ptree& node, float forward, float left, float up);
		/** \brief Write a orientationto the specified property tree node*/
		static void PutOrientation(boost::property_tree::ptree& node, float pitch, float yaw, float roll);
		/** \brief Write a 2D point (X,Y) to the specified property tree node*/
		static void Put2DPoint(boost::property_tree::ptree& node, float x, float y);
		/** \brief Write a position  and orientation to the specified property tree node*/
		static void PutGeometry(boost::property_tree::ptree& geometryNode, float forward, float left, float up, float pitch, float yaw, float roll);
		/** \brief Write a color code (RGB) to the specified property tree node*/
		static void PutColor(boost::property_tree::ptree& colorNode, uint8_t red, uint8_t green, uint8_t blue);

		/** \brief Write a cvoxel's geometry  to the specified property tree node*/
		static void PutChannelGeometry(boost::property_tree::ptree& voxelGeometryNode, ReceiverSettings* receiverPtr);

		/** \brief Modify directory path for the log and debug file*/
		bool SetLogAndDebugFilePath(std::string newFilePath);
		/** \brief Get the directory path for the log and debug file*/
		std::string GetLogAndDebugFilePath();

		/** \brief Set the log file name for future writes*/
		bool SetLogFileName(std::string newFileName);
		/** \brief Get the current log file name*/
		std::string GetLogFileName();

		/** \brief Stores the current receiver calibration settings
			* \return true if storage processe dwithout error. False in case of a storage error.
		  */
		  //bool SensorSettings::StoreReceiverCalibration();
		  //Linux
		bool StoreReceiverCalibration();



	public:
		/** Receiver configuration */
		ReceiverSettingsVector receiverSettings;

		/** Camera configuration */
		CameraSettingsVector cameraSettings;

		/** Dynamic testing: Max deceleration of the vehicle with full brakes on
	     * Ref: Kusano-Gaeble
	     * (http://www.sbes.vt.edu/gabler/publications/Kusano-Gabler-SAE-TTC_EDRs-2011-01-0576.pdf)
	     * Give estimates of 0.52G (0.52 * 9,8m/s2) or 5,096 m/s2
	     * Specify in m/s2 
		 */
		float brakingDeceleration;

		/** Dynamic testing: Travel speed of the vehicle in meters per second 
		*
		*  Travel speed is a variable that should be taken from 
	    * CAN measurements.
	    * In the meantime, we have a variable for that
	    *  Specify in m/s, use <NN>km/h(60.0 * 1000)/(60*60)
		*/
		float travelSpeed;

		/** True activates writing to the debug file.  False deactivates writing to the debug file */
		bool bWriteDebugFile;
		/** True activates writing to the log file.  False deactivates writing to the log file */
		bool bWriteLogFile;
		/** Filename for the debug file (without complete path)*/
		std::string sDebugFileName;
		/** Filename for the log file (without complete path)*/
		std::string sLogFileName;
		/** Directory path for the log and debug files*/
		std::string sDebugAndLogFilePath;


	protected:
		std::string sSettingsFileName;
		static SensorSettings* globalSettings;
		// Property tree contains all the information from the configuration file
		boost::property_tree::ptree propTree;
	};

SENSORCORE_END_NAMESPACE          

#endif 

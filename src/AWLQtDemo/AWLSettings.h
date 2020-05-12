/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the CuteApplication of the
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

#ifndef _AWLSettings__H
#define _AWLSettings__H

#include <string>
#include <stdint.h>
#include <boost/container/vector.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "CoordinateSystem.h"
#include "DetectionStruct.h"
#include "SensorSettings.h"

namespace awl
{

/** \brief Manages storage, reading and writing of all settings contained in the configuration file
 *  \Note In an application, there is typically only one instance of the AWLSettings Settings class.
 * So typical usage is Constructor, InitSettings(), ReadSettings().  Then access to the variable through GetGlobalSettings().
*/
class AWLSettings: public SensorCoreScope::SensorSettings
{
public:

	/** \brief Sets up global variables to access the settings instance
	*/
	static AWLSettings *InitSettings(const std::string sInSettingsFileName = std::string(""));

	/** \brief Returns the global pointer to the single instance of AWLSettings within the application.
	*/
	static AWLSettings *GetGlobalSettings();

	/** \brief Constructor*/
	AWLSettings(const std::string sInSettingsFileName = std::string(""));


	/** \brief Read Settings from the settings file*/
	bool ReadSettings();
	
public:

	// Layout
	/** \brief Layout: Set to true to Display settings window on startup*/
	bool bDisplaySettingsWindow;
	/** \brief Layout: Set to true to Display 2D Window  on startup*/
	bool bDisplay2DWindow;
	/** \brief Layout: Set to true to Display Table View Window  on startup*/
	bool bDisplayTableViewWindow;
	/** \brief Layout: Set to true to Display A-Scan (Waveform view window)  on startup*/
	bool bDisplayAScanViewWindow;
	/** \brief Layout: Set to true to Display About window  on startup*/
	bool bDisplayAboutWindow;
	/** \brief Layout: Set to true to Display Camera windows on startup*/
	bool bDisplayCameraWindow;

	/** \brief Settings window: True when Calibration Tab is displayed in settings window*/
	bool bTabSettingCalibration;
	/** \brief Settings window: True when Control Tab is displayed in settings window - For legacy apps only*/
	bool bTabSettingControl;
	/** \brief Settings window: True when Status Tab is displayed in settings window*/
	bool bTabSettingStatus;
	/** \brief Settings window: True when Registers Tab is displayed in settings window*/
	bool bTabSettingRegisters;
	/** \brief Settings window: True when GPIOs Tab is displayed in settings window - For legacy sensors only*/
	bool bTabSettingGPIOs;
	/** \brief Settings window: True when Algorithm control parameters are  displayed in settings window*/
	bool bTabSettingAlgoControl;
	/** \brief Settings window: True when Tracking Algorithm parameters are  displayed in settings window - Not for Wagner*/
	bool bTabSettingTrackerControl;
	/** \brief Settings window: True when A-Scan display parameters are  displayed in settings window*/
	bool bTabSettingAScan;

	/** \brief Settings window: Calibration Tab: True when  Showing sensor calibration buttons (were used for AWL) */
	bool bCalibrationTabShowSensorCalib;


	/** \brief Initial application display size: Settings window: Calibration Tab: True when  Showing sensor calibration buttons (were used for AWL)
	 * \Note Accepted values are:
     *    "FullScreen" - Show full screen
     *    "Maximized" - Show maximized
     *    "Minimized" - Show Minimized
     *    "Normal" = Show normal
     *     Defaults to "Normal"
	 */
	std::string sDisplayShowSize;

	/** \brief Display velocity in km/h or m/s
	  *     0:  Units are in m/s
	  *     1: Units are in km/h
	  */
	SensorCoreScope::VelocityUnits velocityUnits;

	
	/** \brief Default path name for the logo file, used to display logo on lower left corner of 2D view window<
	  */
	std::string sLogoFileName;

	/** \brief Default path name for the Icon file, used as an icon in task bar and on upper left corner ow window totle bar.
	  */
	std::string sIconFileName;

	/** \brief Table View: number of detections displayed per channel.
	  */
	int displayedDetectionsPerVoxelInTableView;

	// 2D display options

	/** \brief 2D View: Width, in meters, of the gray rectangle showing outine of car.
	  */

	float carWidth;
	/** \brief 2D View: Length, in meters, of the gray rectangle showing outine of car.
	  */
	float carLength;

	/** \brief 2D View: Height, in meters, of the gray rectangle showing outine of car. 
	  * Not used in 2D view but kept for consistency.
    */
	float carHeight;

	/** \brief 2D View: Width, in meters, between optional dotted lines used to represent Lane witdh.
	  * A negative value will no show the lane lines.
	  * Not used in 2D view but kept for consistency.
	*/
	float laneWidth;

	/** \brief 2D View: True to show the "palette" bar that Width, that is used to indicale correspondance between color codes and
	  * actual distances, when applicable.
	*/
	bool showPalette;

	/** \brief 
	*/

	/** \brief  2D View: Merge individual detections according to merge rules defined in XML file.
	  * Corresponds to values in FOV2DScan::MergeDisplayMode enum.
	  * Values are:
	  * 0: NONE
	  * 1: Rectangle under detection  (Still display distance individually)
	  * 2: Rectangle without detection, display only one distance
	  * 3: Rectangle with detections, but only one distance
	*/
	int mergeDisplayMode;


	/** \brief  2D View: Method to calculate distance from sensor origin.
	  * Corresponds to values in FOV2DScan::MeasureMode enum.
	  * Values are:
	  *  0: Radial
	  * 1: Longitudinal distance from bumper 
      * 2: Cartesian coordinates from bumper
	*/
	int measureMode;

	/** \brief  2D View: Display or hide distance measurement text "bubbles".
	  * Corresponds to values in FOV2DScan::DisplayDistanceMode enum.
	  * Values are:
	  * 0: Hide
	  * 1: Show
	*/
	int displayDistanceMode2D;

	/** \brief  2D View: Zoom mode.
	  * Corresponds to values in FOV2DScan::DisplayZoomMode enum.
	  * Values are:
      * 0: Display only area to the front of car
	  * 1: Display front and rear of car
	  * 2: Display from displayedRangeMin to displayedRangeMax (autoZoom)
	*/
	int displayZoomMode2D;


	/** \brief  2D View: distance, in X, before we merge targets in appropriate "MergeDisplayMode"
      * Merge acceptance is in 2D view coordinates, so:
      *  - X is lateral
      *  - Y is forward  
	*/
	float mergeAcceptanceX;

	/** \brief  2D View: distance, in Y, before we merge targets in appropriate "MergeDisplayMode"
	  * Merge acceptance is in 2D view coordinates, so:
	  *	- X is lateral
	  *	- Y is forward
	*/
	float mergeAcceptanceY;

	/** \brief  2D View: Color coding of detections in the 2D view
	 * Corresponds to values in FOV2DScan::DisplayColorCode enum.
	 * 0: Colors indicate distances
	 * 1: Colors indicate speed 
     * 2: Colors indicate obstacle type according to classifier
     * 3: Colors correspond to voxel displayed "color"
     * 4: Colors indicate Threat Level
    */
	int colorCode2D;

	/** \brief  2D View: Maximum velocity, in meters per second, used for color coding. 
	 *   Anything above maxVelocity2D is coded "red".
	 */
	float maxVelocity2D;


	/** \brief  2D View: Velocity at which an obstacle is considered "immobile", and has not direction arrow affixed.
	 *   This allows for some imprecision and minimizes flickering between forward arrow / backwards arrow for very slow objects .
	 */
	float zeroVelocity;

	// Video display options
	/** \brief  Camera view: Display "crosshairs" by default in the camera view window.
	 *   Useful for alignment in the field.
	 */

	bool bDisplayVideoCrosshair;

	/** \brief  Camera view: Display Time in the bottom of window.
	 *   When recording video screenshots, is usefult to match events with timestaps in the log files.
	 */
	bool bDisplayVideoTime;

};

} // namespace AWL          

#endif 

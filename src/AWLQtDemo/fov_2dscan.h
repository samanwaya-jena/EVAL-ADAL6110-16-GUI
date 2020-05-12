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
#ifndef FOV_2DSCAN_H
#define FOV_2DSCAN_H

#include <QFrame>
#include <QLabel>
#include <QPainter>
#include <QAction>
#include <QActionGroup>
#include "boost/chrono/system_clocks.hpp"
#include "ui_fov_2dscan.h"
#include "DetectionStruct.h"

//#define USE_FPS_FOV_2DSCAN

namespace awl
{


/** \brief Structure containing FoV2DScan's 2D View configuration. */
typedef struct
{
     float spareDepth;				// Sensor distance from bumper 
	 float maxSensorsRange;		    // Max range of all sensors displayed
	 float maxAngularSpan;			// Max angular span displayed for the sensor
}ConfigSensor;


/** \brief Window managing the display of the 2D View in the application
*/
class FOV_2DScan : public QFrame
{
    Q_OBJECT
public:
	/** \brief Constructor.
	*/
    explicit FOV_2DScan(QWidget *parent = 0);

	/** \brief enum listing the "merging" algorithm used to merge between contiguous detections.
	*/
	typedef enum MergeDetectionMode 
	{
		/** No merge */
		eNoMerge=0, 
		/** Merge according to radial distance */
		eRadial=1, 
		/** Merge according to longitudinal distance from bumper */
		eLongitudinal=2 
	} MergeDetectionMode;


	/** \brief enum listing the user interface modes used to show "merged targets".
	*/
	typedef enum MergeDisplayMode
	{
		/** Don't display merged detections*/
		eNoMergeDisplay=0,
		/**Rectangle under detection point (Still display distance individually)*/
		eIndividualDistanceDisplay=1, 
		/**Rectangle without detection points, display only one distance*/
		eMergeDistanceDisplay=2, 
		/**Rectangle with detections, but only one distance displayed (avg)*/
		eClusteredDistanceDisplay= 3 
	} MergeDisplayMode;


	/** \brief enum showing how distance evaluation are calculated from the sensor (radial or cartesian)
	*/
	typedef enum MeasureMode
	{
		/**Radial distance from sensor*/
		eMeasureRadial=0,
		/**Longitudinal distance from bumper*/
		eMeasureLongitudinal=1, 
		/** Cartesian coordinates from bumper*/
		eMeasureCartesian=2
	} MeasureMode;
	
	/** \brief Usage of the color codes used on each detection
	*/
	typedef enum DisplayColorCode
	{	
		/**ColorCode distances*/
		eColorCodeDistance= 0, 
		/**Color code speeds*/
		eColorCodeVelocity = 1,   
		/**Color codes intensity/detection type*/
		eColorCodeIntensity = 2, 
		/**Color codes from Channel/layer*/
		eColorCodeChannel = 3,  
		/**Color codes alert level*/
		eColorCodeAlert = 4
	} DisplayColorCode;


	/** \brief Display or hide distance text "bubbles" in UI
	*/
	typedef enum DisplayDistanceMode
	{	
		/**Hide distance text "bubbles"*/
		eDisplayDistanceModeHide = 0, 
		/**Show distance text bubbles*/
		eDisplayDistanceModeShow = 1  
	} DisplayDistanceMode;


	/** \brief 2D view display "perspective" around center point
	*/
	typedef enum DisplayZoomMode
	{
		/**Show only front of vehicle*/
		eDisplayZoomModeFront = 0,  
		/**Show front and back: Objects can look smaller*/
		eDisplayZoomMode360 = 1,   
		/**Scale automatically based on displayedRangeMin*/
		eDisplayZoomModeAuto = 2  
	} DisplayZoomMode;


	/** \brief  Used by Qt in resizing, provides "ideal" size for the A-Scan Window.
		*/
	QSize sizeHint() const;
	/** \brief  Used by Qt in resizing, provides "minimum" acceptable size for the A-Scan Window.
	 */
	QSize minimumSizeHint() const;
	/** \brief  Used by Qt in resizing, provides "maximum" size for the A-Scan Window.
	 */
	QSize maximumSizeHint() const;

signals:
    void closed();
public slots:

	/** \brief  Slot controls what is done when application or sensor configuration changes.
	 */
    void slotConfigChanged();

	/** \brief  Slot controls what is done when sensor detection data is updated.
	 */
	void slotDetectionDataChanged(const SensorCoreScope::Detection::Vector & data);

	/** \brief  Slot sets-up and shows the right-click menu.
	 */
	void ShowContextMenu(const QPoint& pos);

	/** \brief  Slot updates the "Palette" display variables on initialization or right-click of menu.
	 */
	void slotPaletteAction();

	/** \brief  Slot updates the Merge DisplayMode on initialization or right-click of menu.
	*/
	void slotMergeDisplayAction();

	/** \brief  Slot updates the MeasureMode on initialization or right-click of menu.
	 */
	void slotMeasureModeAction();

	/** \brief  Slot updates the DisplayColorCode usage on initialization or right-click of menu.
	 */
	void slotColorCodeAction();

	/** \brief  Slot updates the Distance display mode  on initialization or right-click of menu.
	 */
	void slotDisplayDistanceModeAction();

	/** \brief  Slot updates the Zomming display mode on initialization or right-click of menu.
	 */
	void slotDisplayZoomModeAction();



protected :
	/** \brief  Update view on repaint.
	*/
    void paintEvent(QPaintEvent * /*p*/);
	void closeEvent(QCloseEvent * /*event*/);
	/** \brief  Resize and update view.
	 */
	void resizeEvent(QResizeEvent * /*event*/);
private:
	//Action Item
	Ui::FOV_2DScanFrame ui;

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

	
	/** \brief Ratio internally used used for ideal rescaling of the 2D view window.
	*/
	float Ratio;

	/** \brief Boolean indicates if the "Palette" side window needs to be displayed.
	*/
    bool ShowPalette;

	/** \brief Local copy of the detection data, so that we can be detached from main thread.
	*/
	SensorCoreScope::Detection::Vector copyData;

	/** \brief Local copy of detections, where only merged detections are kept.
	*/
	boost::container::vector<Detection::Vector> mergedData;

	/** \brief Local 2d view spcific configuration variables..
	*/
    ConfigSensor config;

	/** \brief Internal variable to determine max position of distance text.*/
	int lastRightTextHeight;
	/** \brief Number of distance text bubbles to display.*/
	int rightQty;

	/** \brief Current Merge Detection mode.*/
	MergeDetectionMode mergeDetectionMode;
	/** \brief Current Merge Display mode.*/
	MergeDisplayMode mergeDisplayMode;
	/** \brief Current Distance measureent mode.*/
	MeasureMode measureMode;
	/** \brief Current color scoding scheme for detections.*/
	DisplayColorCode colorCode;
	/** \brief Current Distance mode.*/
	DisplayDistanceMode displayDistanceMode;
	/** \brief Current Display zooom mode.*/
	DisplayZoomMode displayZoomMode;

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
	/** \brief  2D View: Maximum velocity, in meters per second, used for color coding.
	 *   Anything above maxVelocity2D is coded "red".
	*/
	float maxAbsVelocity;

	/** \brief  2D View: Velocity at which an obstacle is considered "immobile", and has not direction arrow affixed.
	 *   This allows for some imprecision and minimizes flickering between forward arrow / backwards arrow for very slow objects .
	 */
	float zeroVelocity;

	/** \brief Label Widget used to hold the logo (diplayed in lower left corner)
	*/
	QLabel	*logoLabel;

	//Action Item

	QActionGroup* groupMergeDisplayMode;
	QAction* noMergeDisplayAction;
	QAction* individualDistanceDisplayAction;
	QAction* mergeDistanceDisplayAction;
	QAction* clusteredDistanceDisplayAction;

	QActionGroup* groupMeasureMode;
	QAction* measureRadialAction;
	QAction* measureLongitudinalAction; 
	QAction* measureCartesianAction;

	QAction* showPaletteAction;

	QActionGroup* groupColorCode;
	QAction* colorCodeDistanceAction;
	QAction* colorCodeVelocityAction;
	QAction* colorCodeIntensityAction;
	QAction* colorCodeChannelAction;
	QAction* colorCodeAlertAction;

	QActionGroup* groupDisplayDistanceMode;
	QAction* displayDistanceModeShowAction;
	QAction* displayDistanceModeHideAction;

	QActionGroup* groupDisplayZoomMode;
	QAction* displayZoomModeFrontAction;
	QAction* displayZoomMode360Action;
	QAction* displayZoomModeAutoAction;

#ifdef USE_FPS_FOV_2DSCAN
	// FPS
   /** \brief  Frame per second calculator.  Used for performance evaluation purposes.
   */
	boost::chrono::time_point<boost::chrono::high_resolution_clock> m_timeFPS;
	/** \brief  Qty of frames used in the Frame per second calculator.  Used for performance evaluation purposes.
	*/
	int nFrames;
	/** \brief  resulting frame per second in the Frame per second calculator.  Used for performance evaluation purposes.
	 */
	int FPS;
#endif //USE_FPS_FOV_2DSCAN

	
	/** \brief  Draw an arc within the "pie" view.
	*/
	void drawArc(QPainter* p, float startAngle, float angularSpan, float radius, float xOffset = 0, float yOffset = 0);
	/** \brief  Draw the outline of the  "pie" view.
	*/
	void drawPie(QPainter* p, float startAngle, float angularSpan, float radius, float xOffset, float yOffset);

	/** \brief  Draw a line within the "pie" view.
	*/
	void drawLine(QPainter* p, float angle, float startRadius, float length);
	/** \brief  Drawtext at the provided position within the  "pie" view.
	*/
    void drawText(QPainter* p,float angle, float pos, QString text, QColor foregroundColor = Qt::black, int xOffset = 0);

	/** \brief  Draw a detection within the pie chart, with acoompanying distance tesxt if required.
	*/
	void drawTextDetection(QPainter* p, const SensorCoreScope::Detection::Ptr &detection, QString text, QColor backColor, Qt::BrushStyle backPattern, QColor lineColor, QColor textColor, bool drawTarget = true, bool drawLegend = true);
   
	/** \brief  Draw the angle indicators on top of the "pie" view.
	*/
	void drawAngularRuler(QPainter* p);

	/** \brief  Merge the detection according to rules set in current config.
	 *          Copy the result in mergedData.
	*/
	void mergeDetection();

	/** \brief Return the max aleter level of all current detections.
	*/
	AlertCondition::ThreatLevel getMaxThreat();

	/** \brief  Determine if two detections are within merging range of each other.
	*/
	bool isInRange(const SensorCoreScope::Detection::Ptr &detection1, const SensorCoreScope::Detection::Ptr &detection2 );

	/** \brief  Assign color to a detection based on distance.
	*/
    void getColorFromDistance(float distance, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);

	/** \brief  Assign color to a detection based on velocity.
	*/
	void getColorFromVelocity(float velocity, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);

	/** \brief  Assign color to a detection based on intensity.
	*/
	void getColorFromIntensity(int receiverID, CellID inCellID, float distance, float intensity, AlertCondition::ThreatLevel threatLevel, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);

	/** \brief  Assign color to a detection based on threat level.
	*/
	void getColorFromThreatLevel(SensorCoreScope::AlertCondition::ThreatLevel threatLevel, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);

	/** \brief  Assign color to a detection based on voxel row and column color configured in config file.
	*/
	void getColorFromChannel(int receiverID, CellID inCellID, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);

	/** \brief  Draw the optional "palette" bar, depending on settigs.
	*/
    void drawPalette(QPainter* p);

	/** \brief  Draw the "detection" element.
	*/
    void drawDetection(QPainter* p, const SensorCoreScope::Detection::Ptr &detection,  bool drawTarget = true, bool drawLegend = true);

	/** \brief  Draw a merged detection.  Format will depend on current configuration.
	*/
	void drawMergedData(QPainter* p, const SensorCoreScope::Detection::Vector &data, bool drawBoundingBox, bool drawTarget = true, bool drawLegend = true);

	void createAction();
	void calculateResize();

};


} // namespace awl
#endif // FOV_2DSCAN_H

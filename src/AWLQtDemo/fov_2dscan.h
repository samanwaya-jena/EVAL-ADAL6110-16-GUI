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


/** \brief Structure containing 2D View configuration. */
typedef struct
{
     float spareDepth;				// Sensor distance from bumper 
	 float maxSensorsRange;		    // Max range of all sensors displayed
	 float maxAngularSpan;			// Max angular span displayed for the sensor
}ConfigSensor;


class FOV_2DScan : public QFrame
{
    Q_OBJECT
public:
    explicit FOV_2DScan(QWidget *parent = 0);

	typedef enum MergeDetectionMode 
	{
		eNoMerge=0, //NONE
		eRadial=1, //According to radial distance
		eLongitudinal=2 //  According to bumper distance
	} MergeDetectionMode;


	typedef enum MergeDisplayMode
	{
		eNoMergeDisplay=0, //NONE
		eIndividualDistanceDisplay=1, // Rectangle under detection (Still display distance individually)
		eMergeDistanceDisplay=2, // Rectangle without detection, display only one distance
		eClusteredDistanceDisplay= 3 // Rectangle with detections, but only one distance
	} MergeDisplayMode;

	typedef enum MeasureMode
	{
		eMeasureRadial=0, //Radial distance from sensor
		eMeasureLongitudinal=1, // Distance from bumper
		eMeasureCartesian=2 // Cartesian coordinates from bumper
	} MeasureMode;
	
	typedef enum DisplayColorCode
	{	
		eColorCodeDistance= 0, // ColorCode distances
		eColorCodeVelocity = 1,  // Color code speeds
		eColorCodeIntensity = 2,  // Color codes intensity/detection type
		eColorCodeChannel = 3, // Color codes from Channel/layer
		eColorCodeAlert = 4
	} DisplayColorCode;

	typedef enum DisplayDistanceMode
	{	
		eDisplayDistanceModeHide = 0, // Hide all distances
		eDisplayDistanceModeShow = 1  // Show all distances
	} DisplayDistanceMode;

	typedef enum DisplayZoomMode
	{
		eDisplayZoomModeFront = 0, // Show only front of vehicle
		eDisplayZoomMode360 = 1,  // Show front and back: Objects can look smaller
		eDisplayZoomModeAuto = 2  // Scale automatically on displayedRangeMin
	} DisplayZoomMode;


	QSize sizeHint() const;
	QSize minimumSizeHint() const;
	QSize maximumSizeHint() const;

signals:
    void closed();
public slots:
    void slotConfigChanged();
    void slotDetectionDataChanged(const SensorCoreScope::Detection::Vector & data);
	void ShowContextMenu(const QPoint& pos);
	void slotPaletteAction();
	void slotMergeDisplayAction();
	void slotMeasureModeAction();
	void slotColorCodeAction();
	void slotDisplayDistanceModeAction();
	void slotDisplayZoomModeAction();



protected :
    void paintEvent(QPaintEvent * /*p*/);
	void closeEvent(QCloseEvent * /*event*/);
	void resizeEvent(QResizeEvent * /*event*/);
private:
	//Action Item
	Ui::FOV_2DScanFrame ui;

	float carWidth;
	float carLength;
	float carHeight;
	float laneWidth;

    float Ratio;
    bool ShowPalette;
	SensorCoreScope::Detection::Vector copyData;
	boost::container::vector<Detection::Vector> mergedData;
    ConfigSensor config;
    QRgb rgblongRangeLimited;
    QRgb rgblongRange;
    QRgb rgbshortRangeLimited;
    QRgb rgbshortRange;
    int lastRightTextHeight;
    int rightQty;
	MergeDetectionMode mergeDetectionMode;
	MergeDisplayMode mergeDisplayMode;
	MeasureMode measureMode;
	DisplayColorCode colorCode;
	DisplayDistanceMode displayDistanceMode;
	DisplayZoomMode displayZoomMode;

	float mergeAcceptanceX;
	float mergeAcceptanceY;
	float maxAbsVelocity;
	float zeroVelocity;

	// Logo
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
  boost::chrono::time_point<boost::chrono::high_resolution_clock> m_timeFPS;
  int nFrames;
  int FPS;
#endif //USE_FPS_FOV_2DSCAN

	void drawArc(QPainter* p, float startAngle, float angularSpan, float radius, float xOffset = 0, float yOffset = 0);
    void drawPie(QPainter* p, float startAngle, float angularSpan, float radius, float xOffset, float yOffset);
    void drawLine(QPainter* p, float angle, float startRadius, float length);
    void drawText(QPainter* p,float angle, float pos, QString text, QColor foregroundColor = Qt::black, int xOffset = 0);
    void drawTextDetection(QPainter* p, const SensorCoreScope::Detection::Ptr &detection, QString text, QColor backColor, Qt::BrushStyle backPattern, QColor lineColor, QColor textColor, bool drawTarget = true, bool drawLegend = true);
    void drawAngularRuler(QPainter* p);
	void mergeDetection();
	AlertCondition::ThreatLevel getMaxThreat();
	bool isInRange(const SensorCoreScope::Detection::Ptr &detection1, const SensorCoreScope::Detection::Ptr &detection2 );
    void getColorFromDistance(float distance, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);
	void getColorFromVelocity(float velocity, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);
	void getColorFromIntensity(int receiverID, CellID inCellID, float distance, float intensity, AlertCondition::ThreatLevel threatLevel, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);
	void getColorFromThreatLevel(SensorCoreScope::AlertCondition::ThreatLevel threatLevel, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);
	void getColorFromChannel(int receiverID, CellID inCellID, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);

    void drawPalette(QPainter* p);

    void drawDetection(QPainter* p, const SensorCoreScope::Detection::Ptr &detection,  bool drawTarget = true, bool drawLegend = true);
	void drawMergedData(QPainter* p, const SensorCoreScope::Detection::Vector &data, bool drawBoundingBox, bool drawTarget = true, bool drawLegend = true);

	void createAction();
	void calculateResize();

};


} // namespace awl
#endif // FOV_2DSCAN_H

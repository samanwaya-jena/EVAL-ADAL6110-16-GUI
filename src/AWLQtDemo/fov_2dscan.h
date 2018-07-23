#ifndef FOV_2DSCAN_H
#define FOV_2DSCAN_H

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

#include <QFrame>
#include <QLabel>
#include <QPainter>
#include <QAction>
#include <QActionGroup>

#include "ui_fov_2dscan.h"
#include "DetectionStruct.h"

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
    void slotDetectionDataChanged(const Detection::Vector & data);
    void aScanDataChanged(const AScan::Vector & data);
	void ShowContextMenu(const QPoint& pos);
	void slotPaletteAction();
	void slotMergeDisplayAction();
	void slotMeasureModeAction();
	void slotColorCodeAction();
	void slotDisplayDistanceModeAction();
	void slotDisplayZoomModeAction();



protected :
    void paintEvent(QPaintEvent *p);
	void closeEvent(QCloseEvent * event);
	void resizeEvent(QResizeEvent * event);
private:
	//Action Item
	Ui::FOV_2DScanFrame ui;

	float carWidth;
	float carLength;
	float carHeight;
	float laneWidth;
    AScan::Vector aScanData;
    void plotAScans(void);
    void plotAScan(AScan::Ptr aScan, QPainter *painter, int top, int left, int width, int height);

    float Ratio;
    bool ShowPalette;
    Detection::Vector copyData;
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


	void drawArc(QPainter* p, float startAngle, float angularSpan, float radius, float xOffset = 0, float yOffset = 0);
    void drawPie(QPainter* p, float startAngle, float angularSpan, float radius, float xOffset, float yOffset);
    void drawLine(QPainter* p, float angle, float startRadius, float length);
    void drawText(QPainter* p,float angle, float pos, QString text, QColor foregroundColor = Qt::black, int xOffset = 0);
    void drawTextDetection(QPainter* p, const Detection::Ptr &detection, QString text, QColor backColor, Qt::BrushStyle backPattern, QColor lineColor, QColor textColor, bool drawTarget = true, bool drawLegend = true);
    void drawAngularRuler(QPainter* p);
	void mergeDetection();
	AlertCondition::ThreatLevel getMaxThreat();
	bool isInRange(const Detection::Ptr &detection1, const Detection::Ptr &detection2 );
    void getColorFromDistance(float distance, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);
	void getColorFromVelocity(float velocity, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);
	void getColorFromIntensity(int channel, float distance, float intensity, AlertCondition::ThreatLevel threatLevel, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);
	void getColorFromThreatLevel(AlertCondition::ThreatLevel threatLevel, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);
	void getColorFromChannel(int receiverID, int channelID, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor);

    void drawPalette(QPainter* p);

    void drawDetection(QPainter* p, const Detection::Ptr &detection,  bool drawTarget = true, bool drawLegend = true);
	void drawMergedData(QPainter* p, const Detection::Vector &data, bool drawBoundingBox, bool drawTarget = true, bool drawLegend = true);

	void createAction();
	void calculateResize();

};


} // namespace awl
#endif // FOV_2DSCAN_H

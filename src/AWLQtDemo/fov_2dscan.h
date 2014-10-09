#ifndef FOV_2DSCAN_H
#define FOV_2DSCAN_H

/*
	Copyright 2014 Aerostar R&D Canada Inc.

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

#include "DetectionStruct.h"

namespace awl
{


/** \brief Structure containing 2D View configuration. */
typedef struct
{
    float shortRangeDistance;				// Max range for short range sensor (Including limited range)
    float shortRangeDistanceStartLimited;	// Limited range for short range sensor
    float shortRangeAngle;					// Max angle width for short range sensor (Including limited angle)
    float shortRangeAngleStartLimited;		// Limited angle for short range sensor 

    float longRangeDistance;				// Max range for long range sensor (Including limited range)
    float longRangeDistanceStartLimited;	// Limited range for long range sensor
    float longRangeAngle;					// Max angle width for long range sensor (Including limited angle)
    float longRangeAngleStartLimited;		// Limited angle for long range sensor 

    float spareDepth;				// Sensor distance from bumper 
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
		eMeasureLongitudinal=1 // Distance from bumper
	} MeasureMode;
	
	typedef enum DisplayColorCode
	{	
		eColorCodeDistance= 0, // ColorCode distances
		eColorCodeVelocity = 1  // Color code speeds
	} DisplayColorCode;

	typedef enum DisplayDistanceMode
	{	
		eDisplayDistanceModeHide = 0, // Hide all distances
		eDisplayDistanceModeShow = 1  // Show all distances
	} DisplayDistanceMode;

signals:
    void closed();

public slots:
    void slotConfigChanged(const ConfigSensor &inConfig);
    void slotDetectionDataChanged(const Detection::Vector & data);
	void ShowContextMenu(const QPoint& pos);
	void slotPaletteAction();
	void slotMergeDisplayAction();
	void slotMeasureModeAction();
	void slotColorCodeAction();
	void slotDisplayDistanceModeAction();

protected :
    void paintEvent(QPaintEvent *p);
	void closeEvent(QCloseEvent * event);
	void resizeEvent(QResizeEvent * event);
private:

	float carWidth;
	float carLength;
	float carHeight;
	float laneWidth;

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

	QAction* showPaletteAction;

	QActionGroup* groupColorCode;
	QAction* colorCodeDistanceAction;
	QAction* colorCodeVelocityAction;

	QActionGroup* groupDisplayDistanceMode;
	QAction* displayDistanceModeShowAction;
	QAction* displayDistanceModeHideAction;

	void drawArc(QPainter* p, float startAngle, float angularSpan, float radius, float xOffset = 0, float yOffset = 0);
    void drawPie(QPainter* p, float startAngle, float angularSpan, float radius, float xOffset, float yOffset);
    void drawLine(QPainter* p, float angle, float startRadius, float length);
    void drawText(QPainter* p,float angle, float pos, QString text);
    void drawText(QPainter* p,float angle, float pos, QString text, QColor foregroundColor = Qt::black, bool drawEllipse = false, QColor backgroundcolor = Qt::white);
    void drawTextDetection(QPainter* p, const Detection::Ptr &detection, QString text, QColor foregroundColor = Qt::black, QColor backgroundcolor = Qt::white, bool drawTarget = true, bool drawLegend = true);
    void drawAngularRuler(QPainter* p);
	void mergeDetection();
	bool isInRange(const Detection::Ptr &detection1, const Detection::Ptr &detection2 );
    QColor getColorFromDistance(float distance);
	QColor FOV_2DScan::getColorFromVelocity(float velocity);

    void drawPalette(QPainter* p);

    void drawDetection(QPainter* p, const Detection::Ptr &detection,  bool drawTarget = true, bool drawLegend = true);
	void drawMergedData(QPainter* p, const Detection::Vector &data, bool drawBoundingBox, bool drawTarget = true, bool drawLegend = true);

	void createAction();
	
};


} // namespace awl
#endif // FOV_2DSCAN_H

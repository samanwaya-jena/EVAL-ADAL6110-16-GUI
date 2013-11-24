#ifndef FOV_2DSCAN_H
#define FOV_2DSCAN_H

#include <QFrame>
#include <QPainter>
#include <QAction>
#include <QActionGroup>

#include "DetectionStruct.h"
#include "AWLSettings.h"

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

signals:
    void closed();

public slots:
    void slotConfigChanged(ConfigSensor*);
    void slotDetectionDataChanged(DetectionDataVect* data);
	void ShowContextMenu(const QPoint& pos);
	void slotPaletteAction();
	void slotMergeDisplayAction();
	void slotMeasureModeAction();
	void slotColorCodeAction();

protected :
    void paintEvent(QPaintEvent *p);
	void closeEvent(QCloseEvent * event);
	void resizeEvent(QResizeEvent * event);
private:

    float Ratio;
    bool ShowPalette;
    DetectionDataVect copyData;
	QVector<DetectionDataVect> mergedData;
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
	float mergeAcceptance;
	float maxAbsVelocity;
	float zeroVelocity;

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

	void drawArc(QPainter* p, float angle, float angleWidth, float length);
    void drawPie(QPainter* p, float angle, float angleWidth, float length);
    void drawLine(QPainter* p,float angle, float startLength,float length);
    void drawText(QPainter* p,float angle, float pos, QString text);
    void drawText(QPainter* p,float angle, float pos, QString text, QColor foregroundColor = Qt::black, bool drawEllipse = false, QColor backgroundcolor = Qt::white);
    void drawTextDetection(QPainter* p, DetectionData *detection, float angle, float pos, QString text, QColor foregroundColor = Qt::black, QColor backgroundcolor = Qt::white, bool drawTarget = true, bool drawLegend = true);
    float degree_to_rad (float degrees);
    void drawAngularRuler(QPainter* p);
	void mergeDetection();
	bool isInRange(DetectionData* detection1, DetectionData* detection2 );
    QColor getColorFromDistance(float distance);
	QColor FOV_2DScan::getColorFromVelocity(float velocity);

    void drawPalette(QPainter* p);

    void drawDetection(QPainter* p, DetectionData *detection, float angle, float width, float distanceRadial, float distanceLongitudinal, int channel, int id,  bool drawTarget = true, bool drawLegend = true);
	void drawMergedData(QPainter* p, DetectionDataVect* data, bool drawBoundingBox, bool drawTarget = true, bool drawLegend = true);

	void createAction();
	
};

#endif // FOV_2DSCAN_H

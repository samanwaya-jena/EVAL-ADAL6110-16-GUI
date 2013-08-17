#ifndef FOV_2DSCAN_H
#define FOV_2DSCAN_H

#include <QFrame>
#include <QPainter>

#include "DetectionStruct.h"

class FOV_2DScan : public QFrame
{
    Q_OBJECT
public:
    explicit FOV_2DScan(QWidget *parent = 0);
    
signals:
    void closed();

public slots:
    void slotConfigChanged(ConfigSensor*);
    void slotDetectionDataChanged(DetectionDataVect* data);

protected :
    void paintEvent(QPaintEvent *p);
	void closeEvent(QCloseEvent * event);
	void resizeEvent(QResizeEvent * event);
private:

    float Ratio;
    bool ShowPalette;
    DetectionDataVect copyData;
    ConfigSensor config;
    QRgb rgblongRangeLimited;
    QRgb rgblongRange;
    QRgb rgbshortRangeLimited;
    QRgb rgbshortRange;
    int lastLeftTextHeight;
    int lastRightTextHeight;
    bool leftRight;
    int leftQty;
    int rightQty;

    void drawArc(QPainter* p, float angle, float angleWidth, float length);
    void drawPie(QPainter* p, float angle, float angleWidth, float length);
    void drawLine(QPainter* p,float angle, float startLength,float length);
    void drawText(QPainter* p,float angle, float pos, QString text);
    void drawText(QPainter* p,float angle, float pos, QString text, QColor foregroundColor = Qt::black, bool drawEllipse = false, QColor backgroundcolor = Qt::white);
    void drawTextDetection(QPainter* p,float angle, float pos, QString text, QColor foregroundColor = Qt::black, bool drawEllipse = false, QColor backgroundcolor = Qt::white);
    float degree_to_rad (float degrees);
    void drawAngularRuler(QPainter* p);
    void drawDetection(QPainter* p, float angle, float width, float distance, int channel, int id);

    QColor getColorFromDistance(float distance);
    void drawPalette(QPainter* p);
};

#endif // FOV_2DSCAN_H

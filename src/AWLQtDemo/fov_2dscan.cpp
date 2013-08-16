#include "fov_2dscan.h"
#include <QPainter>
#include <math.h>

#include <QApplication>
#include <QDesktopWidget>

#define PI 3.1416

FOV_2DScan::FOV_2DScan(QWidget *parent) :
    QFrame(parent)
{
    Ratio = 1;
    ShowPalette = true;

	// Position the widget on the top left corner
	QRect scr = QApplication::desktop()->screenGeometry();
	move(scr.left(), scr.top()+5); 

    rgblongRangeLimited = qRgba(188,205,203,127);
    rgblongRange = qRgba(58,126,209,127);
    rgbshortRangeLimited = qRgba(184,220,175,127);
    rgbshortRange = qRgba(54,166,38,127);

    setMinimumSize(480,480);
}

void FOV_2DScan::slotConfigChanged(ConfigSensor *pConfig)
{
    config = *pConfig;
    Ratio = 1;

    Ratio = (height()-(height()*0.1)) / config.longRangeDistance;

    float angleInRad = degree_to_rad((pConfig->shortRangeAngle/2)+180);

    QPoint start(0, (pConfig->longRangeDistance*Ratio));
    QPoint temp;

    temp = start;
    start.setX(temp.x()*cosf(angleInRad) - temp.y()*sinf(angleInRad));
    start.setY(temp.y()*cosf(angleInRad) + temp.x()*sinf(angleInRad));

    setMinimumSize((start.x()*2)+250,480);
    update();
}

void FOV_2DScan::paintEvent(QPaintEvent *)
{

    QPainter painter(this);
    painter.fillRect(0,0,width(),height(),QBrush(Qt::white));

    painter.setPen(Qt::NoPen);
    painter.setBrush(QBrush(QColor(rgblongRangeLimited)));
    drawPie(&painter, -config.longRangeAngle/2, config.longRangeAngle, config.longRangeDistance);

    painter.setBrush(QBrush(QColor(rgblongRange)));
    drawPie(&painter, -config.longRangeAngleStartLimited/2, config.longRangeAngleStartLimited, config.longRangeDistanceStartLimited);

    painter.setBrush(QBrush(QColor(rgbshortRangeLimited)));
    drawPie(&painter, -config.shortRangeAngle/2, config.shortRangeAngle, config.shortRangeDistance);

    painter.setBrush(QBrush(QColor(rgbshortRange)));
    drawPie(&painter, -config.shortRangeAngleStartLimited/2, config.shortRangeAngleStartLimited, config.shortRangeDistanceStartLimited);

    //Angular Ruler
    painter.setPen(QPen(Qt::black));

    for (int i = config.longRangeDistance; i > 0; i-=5)
        drawArc(&painter, -config.shortRangeAngle/2, config.shortRangeAngle, i);

    painter.setPen(QPen(Qt::black));
    for (int i = config.shortRangeAngle; i >= 0; i-=5)
        drawLine(&painter, i-(config.shortRangeAngle/2), 0, config.longRangeDistance);

    drawAngularRuler(&painter);

    if (ShowPalette)
        drawPalette(&painter);

    lastLeftTextHeight = height();
    lastRightTextHeight = height();

    leftQty = 0;
    rightQty = 0;
    leftRight = true;
    DetectionDataVect::iterator i;
    for (i = copyData.begin(); i != copyData.end(); ++i)
        drawDetection(&painter, i->angle, i->angleWidth, i->distance, i->fromChannel, i->id);

}

void FOV_2DScan::drawAngularRuler(QPainter* p)
{
    p->setPen(QPen(Qt::red));
    drawArc(p, -config.shortRangeAngle/2, config.shortRangeAngle, config.longRangeDistance+(5.0/Ratio));
    for (int i = config.shortRangeAngle; i >= 0; i-=5)
    {
        drawLine(p, i-(config.shortRangeAngle/2), config.longRangeDistance+(5.0/Ratio), (10.0/Ratio));
        drawText(p, i-(config.shortRangeAngle/2), config.longRangeDistance+(20.0/Ratio), QString::number((i-(config.shortRangeAngle/2)))+"°", Qt::red);
    }
}

void FOV_2DScan::drawDetection(QPainter* p, float angle, float width, float distance, int channel, int id)
{
	QColor backColor = getColorFromDistance(distance);
	QColor penColor = Qt::black;

	if (backColor.lightness() < 128) penColor = Qt::white;

    drawTextDetection(p, angle, distance, QString("Ch.") + QString::number(channel) + " : " + QString::number(distance)+" m", penColor, true, backColor);
}

void FOV_2DScan::drawTextDetection(QPainter* p,float angle, float pos, QString text, QColor foregroundColor, bool drawEllipse, QColor backgroundcolor)
{
    float angleInRad = degree_to_rad(angle+180);

    QPoint start(0, (pos*Ratio));
    QPoint temp;
    QRect tempRect;
    QPolygon poly;

    temp = start;
    start.setX(temp.x()*cosf(angleInRad) - temp.y()*sinf(angleInRad));
    start.setY(temp.y()*cosf(angleInRad) + temp.x()*sinf(angleInRad));

    QRect rect = p->boundingRect(QRect(0,0,0,0), Qt::AlignCenter,  text);
    rect.setSize(rect.size()+QSize(10,10));
    rect.moveTo(start + QPoint((width()/2)-rect.width()/2, height()-rect.height()));

	QColor pencolor = backgroundcolor.darker(150);
    p->setPen(pencolor);
    p->setBrush(backgroundcolor);

    tempRect = rect;
    if (angle < 0)
    {
        if (lastLeftTextHeight < tempRect.bottom()+3)
        {
            tempRect.moveCenter(QPoint(tempRect.center().x(), lastLeftTextHeight - tempRect.height()));
        }

		tempRect.moveCenter(QPoint(60 + tempRect.width()/2, tempRect.bottom()-6));

        poly.append(QPoint(rect.center().x()+2,rect.center().y()+5));
        poly.append(QPoint(tempRect.center().x()+51,tempRect.center().y()-1));
        poly.append(QPoint(tempRect.center().x(),tempRect.center().y()-1));
        poly.append(QPoint(tempRect.center().x(),tempRect.center().y()+1));
        poly.append(QPoint(tempRect.center().x()+49,tempRect.center().y()+1));

        poly.append(QPoint(rect.center().x()-2,rect.center().y()+7));
        p->drawPolygon(poly);


        p->drawEllipse(QPoint(rect.center().x(), rect.bottom()-6), 6, 6);
        p->drawEllipse(QPoint(60 + tempRect.width()/2, tempRect.bottom()-12), tempRect.size().width()/2, tempRect.size().height()/2);
        lastLeftTextHeight = tempRect.top();
        leftQty++;
    }
    else
    {
        if (lastRightTextHeight < tempRect.bottom()+3)
        {
            tempRect.moveCenter(QPoint(tempRect.center().x(), lastRightTextHeight - tempRect.height()));
        }

        tempRect.moveCenter(QPoint(width()-tempRect.width(), tempRect.bottom()-6));

        poly.append(QPoint(rect.center().x()-2,rect.center().y()+5));
        poly.append(QPoint(tempRect.center().x()-51,tempRect.center().y()-1));
        poly.append(QPoint(tempRect.center().x(),tempRect.center().y()-1));
        poly.append(QPoint(tempRect.center().x(),tempRect.center().y()+1));
        poly.append(QPoint(tempRect.center().x()-49,tempRect.center().y()+1));

        poly.append(QPoint(rect.center().x()+2,rect.center().y()+7));
        p->drawPolygon(poly);


        p->drawEllipse(QPoint(rect.center().x(), rect.bottom()-6), 6, 6);
        p->drawEllipse(QPoint(width()-tempRect.width(), tempRect.bottom()-12), tempRect.size().width()/2, tempRect.size().height()/2);
        lastRightTextHeight = tempRect.top();
        rightQty++;
    }

    leftRight = !leftRight;

	 p->setPen(foregroundColor);
     p->drawText(tempRect, Qt::AlignCenter, text);

}

void FOV_2DScan::drawText(QPainter* p,float angle, float pos, QString text, QColor foregroundColor, bool drawEllipse, QColor backgroundcolor)
{
    float angleInRad = degree_to_rad(angle+180);

    QPoint start(0, (pos*Ratio));
    QPoint temp;

    temp = start;
    start.setX(temp.x()*cosf(angleInRad) - temp.y()*sinf(angleInRad));
    start.setY(temp.y()*cosf(angleInRad) + temp.x()*sinf(angleInRad));

    QRect rect = p->boundingRect(QRect(0,0,0,0), Qt::AlignCenter,  text);
    rect.setSize(rect.size()+QSize(10,10));
    rect.moveTo(start + QPoint((width()/2)-rect.width()/2, height()-rect.height()));

    p->setPen(foregroundColor);
    p->setBrush(backgroundcolor);

    if (drawEllipse)
    {
        p->drawEllipse(rect.center(), rect.size().width()/2, rect.size().height()/2);
    }

    p->drawText(rect, Qt::AlignCenter, text);

}

void FOV_2DScan::drawArc(QPainter* p, float angle, float angleWidth, float length)
{
    QRectF rectangle(width()/2-length*Ratio, height()-length*Ratio, length*Ratio*2, length*Ratio*2);
    p->drawArc(rectangle, (angle+90)*16, angleWidth*16);
}

void FOV_2DScan::drawPie(QPainter* p, float angle, float angleWidth, float length)
{

    QRectF rectangle(width()/2-length*Ratio, height()-length*Ratio, length*Ratio*2, length*Ratio*2);
    p->drawPie(rectangle, (angle+90)*16, angleWidth*16);
}

void FOV_2DScan::drawLine(QPainter* p, float angle, float startLength, float length)
{

    float angleInRad = degree_to_rad(angle+180);

    QPoint start(0, (startLength*Ratio));
    QPoint end(0, ((startLength+length)*Ratio));
    QPoint temp;

    temp = start;
    start.setX(temp.x()*cosf(angleInRad) - temp.y()*sinf(angleInRad));
    start.setY(temp.y()*cosf(angleInRad) + temp.x()*sinf(angleInRad));

    temp = end;
    end.setX(temp.x()*cosf(angleInRad) - temp.y()*sinf(angleInRad));
    end.setY(temp.y()*cosf(angleInRad) + temp.x()*sinf(angleInRad));

    p->drawLine(start + QPoint(width()/2, height()), end + QPoint(width()/2, height()));

}

float FOV_2DScan::degree_to_rad (float degrees)
{
    return (degrees * PI/ 180.0);
}

QColor FOV_2DScan::getColorFromDistance(float distance)
{
    QLinearGradient myGradient;
    QGradientStops myStopPoints;
    QImage myImage(10, config.longRangeDistance+2, QImage::Format_RGB32);

    myStopPoints.append(QGradientStop(0.0,Qt::blue));
    myStopPoints.append(QGradientStop(0.33,Qt::green));
    myStopPoints.append(QGradientStop(0.66,Qt::yellow));
    myStopPoints.append(QGradientStop(1.0,Qt::red));
    myGradient.setStops(myStopPoints);
    myGradient.setStart(0, config.longRangeDistance);

    QPainter painter(&myImage);
    painter.setBrush(myGradient);
    painter.drawRect(0, -2, 10, config.longRangeDistance+3 );
    painter.end();
    return QColor(myImage.pixel(QPoint(1, distance)));
}

void FOV_2DScan::drawPalette(QPainter* p)
{
    QLinearGradient myGradient;
    QGradientStops myStopPoints;

    myStopPoints.append(QGradientStop(0.0,Qt::blue));
    myStopPoints.append(QGradientStop(0.33,Qt::green));
    myStopPoints.append(QGradientStop(0.66,Qt::yellow));
    myStopPoints.append(QGradientStop(1.0,Qt::red));
    myGradient.setStops(myStopPoints);
    myGradient.setStart(width()-30, height()*0.1);
    myGradient.setFinalStop(width()-30, height()*0.9);
    p->setPen(Qt::black);
    p->setBrush(myGradient);
    p->drawRect(0, height()*0.1, 30, height()*0.9);

}


void FOV_2DScan::slotDetectionDataChanged(DetectionDataVect* data)
{
    DetectionDataVect::const_iterator i;
    int index;

    copyData.clear();

    for (i = data->begin(); i != data->end(); ++i)
    {
		if (i->distance > config.longRangeDistance)
			continue;

        for (index = 0; index < copyData.count(); ++index)
        {
			if (qFuzzyCompare(copyData[index].distance, i->distance))
            {
                if (copyData[index].angle < i->angle)
                    break;
            }
            else if (copyData[index].distance > i->distance)
            {
                break;
            }

        }

        copyData.insert(index,*i);
    }


    update();
}

void FOV_2DScan::closeEvent(QCloseEvent * event)
{
	emit closed();
}

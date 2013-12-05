#include "fov_2dscan.h"
#include <QPainter>
#include <math.h>
#include <QMenu>
#include <QApplication>
#include <QDesktopWidget>

using namespace awl;

#define PI 3.1416

FOV_2DScan::FOV_2DScan(QWidget *parent) :
    QFrame(parent)
{
    Ratio = 1;
    ShowPalette = true;
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	mergeDisplayMode = (MergeDisplayMode)globalSettings->mergeDisplayMode;
	measureMode = (MeasureMode)globalSettings->measureMode;
	mergeAcceptance = globalSettings->mergeAcceptance;
	ShowPalette = globalSettings->showPalette;
	colorCode = (DisplayColorCode )globalSettings->colorCode2D;
	maxAbsVelocity = globalSettings->maxVelocity2D;
	zeroVelocity = globalSettings->zeroVelocity;

	// Position the widget on the top left corner
	QRect scr = QApplication::desktop()->screenGeometry();
	move(scr.left(), scr.top()+5); 

    rgblongRangeLimited = qRgba(188,205,203,127);
    rgblongRange = qRgba(58,126,209,127);
    rgbshortRangeLimited = qRgba(184,220,175,127);
    rgbshortRange = qRgba(54,166,38,127);

	setContextMenuPolicy(Qt::CustomContextMenu);
	connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),this, SLOT(ShowContextMenu(const QPoint&)));

	createAction();
	

    setMinimumSize(480,480);
}

void FOV_2DScan::createAction()
{

	groupMergeDisplayMode = new QActionGroup( this );
	noMergeDisplayAction = new QAction("Don't merge", this);
	individualDistanceDisplayAction = new QAction("Show individual distances", this);
	mergeDistanceDisplayAction = new QAction("Show merged distances only", this);
	clusteredDistanceDisplayAction = new QAction("Show cluster, singleDistance", this);

	noMergeDisplayAction->setCheckable(true);
	noMergeDisplayAction->setActionGroup(groupMergeDisplayMode);

	individualDistanceDisplayAction->setCheckable(true);
	individualDistanceDisplayAction->setActionGroup(groupMergeDisplayMode);

	mergeDistanceDisplayAction->setCheckable(true);
	mergeDistanceDisplayAction->setActionGroup(groupMergeDisplayMode);

	clusteredDistanceDisplayAction->setCheckable(true);
	clusteredDistanceDisplayAction->setActionGroup(groupMergeDisplayMode);

	if (mergeDisplayMode == eNoMergeDisplay)
	{
		noMergeDisplayAction->setChecked(true);
		mergeDetectionMode = eNoMerge;
	}
	else if (mergeDisplayMode == eIndividualDistanceDisplay)
	{
		individualDistanceDisplayAction->setChecked(true);
		if (measureMode == eMeasureRadial)
			mergeDetectionMode = eRadial;
		else
			mergeDetectionMode = eLongitudinal;
	}
	else if (mergeDisplayMode == eMergeDistanceDisplay)
	{
		mergeDistanceDisplayAction->setChecked(true);
		if (measureMode == eMeasureRadial)
			mergeDetectionMode = eRadial;
		else
			mergeDetectionMode = eLongitudinal;
	}
	else // eClusteredDistanceDisplay
	{
		clusteredDistanceDisplayAction->setChecked(true);
		if (measureMode == eMeasureRadial)
			mergeDetectionMode = eRadial;
		else
			mergeDetectionMode = eLongitudinal;
	}

	connect(groupMergeDisplayMode, SIGNAL(triggered(QAction*)), this, SLOT(slotMergeDisplayAction()));

	groupMeasureMode = new QActionGroup( this );
	measureRadialAction = new QAction("Radial", this);
	measureLongitudinalAction = new QAction("Longitudinal", this);
	
	
	measureRadialAction->setCheckable(true);
	measureRadialAction->setActionGroup(groupMeasureMode);

	measureLongitudinalAction->setCheckable(true);
	measureLongitudinalAction->setActionGroup(groupMeasureMode);

	if (measureMode == eMeasureRadial)
	{
		measureRadialAction->setChecked(true);
	}
	else
	{
		measureLongitudinalAction->setChecked(true);
	}

	connect(groupMeasureMode, SIGNAL(triggered(QAction*)), this, SLOT(slotMeasureModeAction()));

	showPaletteAction = new QAction("Palette", this);
	showPaletteAction->setCheckable(true);
	showPaletteAction->setChecked(ShowPalette);

	connect(showPaletteAction, SIGNAL(triggered()), this, SLOT(slotPaletteAction()));

	groupColorCode = new QActionGroup( this );
	colorCodeDistanceAction = new QAction("Distances", this);
	colorCodeVelocityAction = new QAction("Velocity", this);
	
	
	colorCodeDistanceAction->setCheckable(true);
	colorCodeDistanceAction->setActionGroup(groupColorCode);

	colorCodeVelocityAction->setCheckable(true);
	colorCodeVelocityAction->setActionGroup(groupColorCode);

	if (colorCode == eColorCodeDistance)
	{
		colorCodeDistanceAction->setChecked(true);
	}
	else
	{
		colorCodeVelocityAction->setChecked(true);
	}

	connect(groupColorCode, SIGNAL(triggered(QAction*)), this, SLOT(slotColorCodeAction()));

}

void FOV_2DScan::slotPaletteAction()
{
	ShowPalette = showPaletteAction->isChecked();
}

void FOV_2DScan::slotMergeDisplayAction()
{
	if (noMergeDisplayAction->isChecked())
	{
		mergeDisplayMode = eNoMergeDisplay;
		mergeDetectionMode = eNoMerge;
	}
	else if (individualDistanceDisplayAction->isChecked())
	{
		mergeDisplayMode = eIndividualDistanceDisplay;
		if (measureMode == eMeasureRadial)
			mergeDetectionMode = eRadial;
		else
			mergeDetectionMode = eLongitudinal;
	}
	else if (mergeDistanceDisplayAction->isChecked())
	{
		mergeDisplayMode = eMergeDistanceDisplay;
		if (measureMode == eMeasureRadial)
			mergeDetectionMode = eRadial;
		else
			mergeDetectionMode = eLongitudinal;
	}
	else 
	{
		mergeDisplayMode = eClusteredDistanceDisplay;
		if (measureMode == eMeasureRadial)
			mergeDetectionMode = eRadial;
		else
			mergeDetectionMode = eLongitudinal;
	}

}

void FOV_2DScan::slotMeasureModeAction()
{
	if (measureRadialAction->isChecked())
	{
		measureMode = eMeasureRadial;
	}
	else
	{
		measureMode = eMeasureLongitudinal;
	}
}

void FOV_2DScan::slotColorCodeAction()
{
	if (colorCodeDistanceAction->isChecked())
	{
		colorCode = eColorCodeDistance;
	}
	else
	{
		colorCode = eColorCodeVelocity;
	}


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

void FOV_2DScan::resizeEvent(QResizeEvent * event)
{
	Ratio = (height()-(height()*0.1)) / config.longRangeDistance;
}

void FOV_2DScan::paintEvent(QPaintEvent *)
{

    QPainter painter(this);
    painter.fillRect(0,0,width(),height(),QBrush(Qt::white));

	//Draw bumper line
	painter.setBrush(QBrush(Qt::darkGray));

	// Sensor depth is a negative offset from bumper!
	painter.drawRect(QRect(width()/3, height()+config.sensorDepth*Ratio, width()/3, -config.sensorDepth*Ratio));

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

    lastRightTextHeight = height();

    rightQty = 0;


	// Draw the merged indicators, only if there are more than 1 detections in the area
	// Otherwise, they are displayed as a squate
	for (int index = 0; index < mergedData.count(); index++)
		{
			if (mergeDisplayMode == eNoMergeDisplay) 
			{
				// No merge display, do nothing
			}
			else if (mergeDisplayMode == eIndividualDistanceDisplay)
			{
				// Only draw the bounding rectangle, only if there is more than one detection
				// We never draw the targe or the legend
				if (mergedData[index].count() > 1) drawMergedData(&painter, &mergedData[index], true, false, false);
			}
			else if (mergeDisplayMode == eMergeDistanceDisplay || mergeDisplayMode == eClusteredDistanceDisplay)
			{
				// Draw the bounding rectangle,, only if there is more than one detection
				// Otherwise draw the merged position using the "individual" detections look.
				bool bDrawBoundingBox = true;
				bool bDrawTarget = false;
				bool bDrawLegend = true;
				if (mergedData[index].count() <= 1) 
				{
					bDrawBoundingBox = false;
					bDrawTarget = true;
				}

				drawMergedData(&painter, &mergedData[index], bDrawBoundingBox, bDrawTarget, bDrawLegend); 
			}
	} // for

	// Draw the individual detections
	DetectionDataVect::iterator i;
	for (i = copyData.begin(); i != copyData.end(); ++i)
	{
		if (mergeDisplayMode == eNoMergeDisplay || mergeDisplayMode == eIndividualDistanceDisplay)
		{
			drawDetection(&painter, i, i->angle, i->angleWidth, i->distanceRadial, i->distanceLongitudinal ,i->fromChannel, i->id, true, true);
		}
		else if (mergeDisplayMode == eMergeDistanceDisplay)
		{
			// Don't display the individual distances
		}
		else // if (mergeDisplayMode != eClusteredDistanceDisplay)
		{
			// Display the individual distance targets in s-scan without accompanying legend
			drawDetection(&painter, i, i->angle, i->angleWidth, i->distanceRadial, i->distanceLongitudinal ,i->fromChannel, i->id, true, false);
		}
	} // for
}

void FOV_2DScan::drawMergedData(QPainter* p, DetectionDataVect* data, bool drawBoundingBox, bool drawTarget, bool drawLegend)
{
	int index;
	float distanceMin = config.longRangeDistance;
	float distanceMax = 0;
	float distanceAverage = 0;
	float distanceLongitudinalMin = config.longRangeDistance;
	float distanceLongitudinalMax = 0;
	float distanceLongitudinalAverage = 0;
	float velocityMin = 999;

	float angleMin = config.shortRangeAngle/2;
	float angleMax = -config.shortRangeAngle/2;
	QPolygon poly;

    DetectionDataVect::const_iterator i;

    for (i = data->begin(); i != data->end(); ++i)
    {
		if (i->angle > angleMax)
			angleMax = i->angle;
		if (i->angle < angleMin)
			angleMin = i->angle;

		if (i->distanceRadial > distanceMax)
			distanceMax = i->distanceRadial;
		if (i->distanceRadial < distanceMin)
			distanceMin = i->distanceRadial;

		distanceAverage += i->distanceRadial;

		if (i->distanceLongitudinal > distanceLongitudinalMax)
			distanceLongitudinalMax = i->distanceLongitudinal;
		if (i->distanceLongitudinal < distanceLongitudinalMin)
			distanceLongitudinalMin = i->distanceLongitudinal;
		distanceLongitudinalAverage += i->distanceLongitudinal;

		if (i->velocity < velocityMin) velocityMin = i->velocity;
	}

	if (data->size()) 
	{
		distanceAverage /= data->size(); 
		distanceLongitudinalAverage /= data->size();
	}



	QString velocityLabel = " m/s";
	float distanceDisplayed = 0;0;
	float velocityDisplayed = velocityMin;

	if (measureMode == eMeasureRadial)
		distanceDisplayed = distanceMin;
	else
		distanceDisplayed = distanceLongitudinalMin;

	if (AWLSettings::GetGlobalSettings()->velocityUnits == eVelocityUnitsMS) 
	{
		velocityLabel = " m/s";
		velocityDisplayed = velocityMin;
	}
	else
	{
		velocityLabel = " km/h";
		velocityDisplayed = VelocityToKmH(velocityMin);
	}


	QColor backColor;
	
	if (colorCode == eColorCodeVelocity)
		backColor = getColorFromVelocity(velocityMin);
	else
		backColor = getColorFromDistance(distanceDisplayed);

	QColor pencolor = backColor.darker(180);
    p->setPen(pencolor);

	// Draw the legend and target, according to the flags
	if (mergeDisplayMode == eNoMergeDisplay || mergeDisplayMode == eIndividualDistanceDisplay)
	{
	}
	else // if (mergeDisplayMode == eMergeDistanceDisplay && mergeDisplayMode == eClusteredDistanceDisplay)
	{
		QString textToDisplay;

		textToDisplay = "Dist: " + QString::number(distanceDisplayed, 'f', 1)+" m | Vel: "+ QString::number(velocityDisplayed, 'f', 1)+ velocityLabel;


		if (backColor.lightness() < 128) 
			pencolor = Qt::white;
		else
			pencolor = Qt::black;

		// Draw the detection, but without the legend
		drawTextDetection(p, data->begin(), (angleMin+angleMax)/2, distanceMin, textToDisplay, pencolor, backColor, drawTarget, drawLegend);
	}


	// Now draw the bounding rectangle
	if (drawBoundingBox) 
	{
		pencolor = backColor.darker(180);
		p->setPen(pencolor);

		float angleMinInRad = degree_to_rad(angleMin+180);
		float angleMaxInRad = degree_to_rad(angleMax+180);

		QPoint bottomLeft(0, (distanceMin*Ratio));
		QPoint topRight(0, (distanceMax*Ratio));
		QPoint temp;

		temp = bottomLeft;
		bottomLeft.setX(temp.x()*cosf(angleMinInRad) - temp.y()*sinf(angleMinInRad));
		// JYD Real position of object, from sensor on  the grid is postion + bumperOffset
		// So, subtract the sensorDepth that was initially added in captured data.
		bottomLeft.setY(-((distanceLongitudinalMin-config.sensorDepth)*Ratio));

		temp = topRight;
		topRight.setX(temp.x()*cosf(angleMaxInRad) - temp.y()*sinf(angleMaxInRad));

		// JYD Real position of object, from sensor on  the grid is postion + bumperOffset
		// So, subtract the sensorDepth that was initially added in captured data.
		topRight.setY(-((distanceLongitudinalMax-config.sensorDepth)*Ratio));

		if (bottomLeft.y() < topRight.y())
		{
			temp = topRight;
			topRight.setY(bottomLeft.y());
			bottomLeft.setY(temp.y());
		}

		QRect rect;

		rect.setBottomLeft(bottomLeft);
		rect.setTopRight(topRight);
		rect.setSize(rect.size()+QSize(15,15));
		rect.moveTo(bottomLeft + QPoint((width()/2)-9, height()-rect.height()+1));

		p->drawRect(rect);
	}
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

void FOV_2DScan::drawDetection(QPainter* p, DetectionData *detection, float angle, float width, float distanceRadial, float distanceFromBumper, int channel, int id,
	                                bool drawTarget, bool drawLegend)
{
	QColor backColor;
	QColor penColor = Qt::black;
	QString textToDisplay;

	float distanceToDisplay;
	float velocityToDisplay;
	QString velocityLabel;

	if (measureMode == eMeasureRadial)
		distanceToDisplay = distanceRadial;
	else
		distanceToDisplay = distanceFromBumper;

	if (AWLSettings::GetGlobalSettings()->velocityUnits == eVelocityUnitsMS)
	{
		velocityToDisplay = detection->velocity;
		velocityLabel = " m/s";
	}
	else
	{
		velocityToDisplay = VelocityToKmH(detection->velocity);
		velocityLabel = " km/h";
	}

	textToDisplay = "Dist: " + QString::number(distanceToDisplay, 'f', 1)+" m | Vel: "+ QString::number(velocityToDisplay, 'f', 1)+ velocityLabel;

	if (colorCode == eColorCodeVelocity)
		backColor = getColorFromVelocity(detection->velocity);
	else
		backColor = getColorFromDistance(distanceToDisplay);
	if (backColor.lightness() < 128) penColor = Qt::white;

	// JYD; Real distance  is the radial distance, plus bumper offset.
    drawTextDetection(p, detection, angle, distanceRadial, textToDisplay, penColor,backColor, drawTarget, drawLegend);
}

void FOV_2DScan::drawTextDetection(QPainter* p, DetectionData *detection, float angle, float pos, QString text, QColor foregroundColor, QColor backgroundcolor,
	                                bool drawTarget, bool drawLegend)
{

    float angleInRad = degree_to_rad(angle+180);

	// Real position of object, from sensor on  the grid is postion + bumperOffset.
	// the sensorDepth was added at the moment of capture, so we have to remove it here.
	pos -= config.sensorDepth;

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

	QColor pencolor = backgroundcolor.darker(180);
    p->setPen(pencolor);
    p->setBrush(backgroundcolor);

    tempRect = rect;

	if (lastRightTextHeight < tempRect.bottom()+3)
	{
		tempRect.moveCenter(QPoint(tempRect.center().x(), lastRightTextHeight - tempRect.height()));
	}

	tempRect.moveCenter(QPoint(width()-(tempRect.width()/2)-5, tempRect.bottom()-6));

	if (drawLegend) 
	{
		// Next polygon draws a line between the target and the distance indicator
		poly.append(QPoint(rect.center().x()-2,rect.center().y()+5));
		poly.append(QPoint(tempRect.center().x()-51,tempRect.center().y()-1));
		poly.append(QPoint(tempRect.center().x(),tempRect.center().y()-1));
		poly.append(QPoint(tempRect.center().x(),tempRect.center().y()+1));
		poly.append(QPoint(tempRect.center().x()-49,tempRect.center().y()+1));

		poly.append(QPoint(rect.center().x()+2,rect.center().y()+7));
		p->drawPolygon(poly);

		// Draw the ellipse around the distance indication text
//		p->drawEllipse(QPoint(width()-(tempRect.width()/2), tempRect.bottom()-12), tempRect.size().width()/2, tempRect.size().height()/2);
		p->drawEllipse(QPoint(width()-(tempRect.width()/2)-5, tempRect.bottom()-tempRect.size().height()/2), tempRect.size().width()/2, tempRect.size().height()/2);

		// Write the distance text
		lastRightTextHeight = tempRect.top();
		rightQty++;


		p->setPen(foregroundColor);
		p->drawText(tempRect, Qt::AlignCenter, text);
	}

	if (drawTarget)
	{
		// Draw the ellipse that represents the target on the scan
		if (backgroundcolor.lightness() < 128) pencolor = backgroundcolor.lighter(180);
		else pencolor = backgroundcolor.darker(180);
 		p->setPen(pencolor);
		p->setBrush(backgroundcolor);

		if (detection->velocity >= zeroVelocity)
		{
			// Moving away from sensor is an uparrow
			QRect pieRect(rect.center().x() - 10, rect.bottom()-24, 20, 24);
			int startAngle = -55 * 16;
			int spanAngle = -70 * 16;

			p->drawPie(pieRect,startAngle, spanAngle);
		}
		else if (detection->velocity < -zeroVelocity)
		{
			// Moving towards sensor is a downarrow
			QRect pieRect(rect.center().x() - 10, rect.bottom()-12, 20, 24);
			int startAngle = 55 * 16;
			int spanAngle = 70 * 16;

			p->drawPie(pieRect, startAngle, spanAngle);
		}
		else
		{
			// Static object is an ellipse
			p->drawEllipse(QPoint(rect.center().x(), rect.bottom()-6), 6, 6);
		}

	}
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

QColor FOV_2DScan::getColorFromVelocity(float velocity)
{
    QLinearGradient myGradient;
    QGradientStops myStopPoints;

	QImage myImage(10, maxAbsVelocity+2, QImage::Format_RGB32);

	velocity = -velocity;
	if (velocity < 0.0) velocity = 0.1;
	if (velocity > maxAbsVelocity-1) velocity = maxAbsVelocity-1;

    myStopPoints.append(QGradientStop(0.0,Qt::blue));
    myStopPoints.append(QGradientStop(0.33,Qt::green));
    myStopPoints.append(QGradientStop(0.66,Qt::yellow));
    myStopPoints.append(QGradientStop(1.0,Qt::red));
    myGradient.setStops(myStopPoints);
    myGradient.setStart(0, maxAbsVelocity);

    QPainter painter(&myImage);
    painter.setBrush(myGradient);
    painter.drawRect(0, -2, 10, maxAbsVelocity+3 );
    painter.end();
    return QColor(myImage.pixel(QPoint(1, maxAbsVelocity - velocity)));
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
    myGradient.setStart(width()-40, height()*0.1);
    myGradient.setFinalStop(width()-40, height()*0.9);
    p->setPen(Qt::black);
    p->setBrush(myGradient);
    p->drawRect(0, height()*0.1, 50, height()*0.9);

	// Put a legend
	p->setPen(Qt::white);

	QString text;
	if (colorCode == eColorCodeDistance)
	{
		text = QString::number(config.longRangeDistance, 'f',  1)+ " m";
		QRect rect = p->boundingRect(QRect(0,0,0,0), Qt::AlignCenter,  text);
 
		p->drawText(QPoint(5, height()*0.1 + rect.height()), text);
		p->drawText(QPoint(5, height()-5), "0 m");
	}
	else
	{
		text = QString::number(VelocityToKmH(maxAbsVelocity), 'f',  1)+ " km/h";
		QRect rect = p->boundingRect(QRect(0,0,0,0), Qt::AlignCenter,  text);
 
		p->drawText(QPoint(5, height()*0.1 + rect.height()), "0 km/h");
		p->drawText(QPoint(5, height()-5), text);
	}
}


void FOV_2DScan::slotDetectionDataChanged(DetectionDataVect* data)
{
    DetectionDataVect::const_iterator i;
    int index;

    copyData.clear();

	//Ordering detection from bottom left to top right of 2D view. It's to simplify algo to draw detection in view.
    for (i = data->begin(); i != data->end(); ++i)
    {
		if (i->distanceRadial > config.longRangeDistance)
			continue;

        for (index = 0; index < copyData.count(); ++index)
        {
			if (qFuzzyCompare(copyData[index].distanceLongitudinal, i->distanceLongitudinal))
			//if (qFuzzyCompare(copyData[index].distanceRadial, i->distanceRadial))
            {
				if (i->angle > 0)
				{
					if (copyData[index].angle < i->angle)
						break;
				}
				else
				{
					if (copyData[index].angle > i->angle)
						break;
				}
            }
            else if (copyData[index].distanceLongitudinal > i->distanceLongitudinal)
            {
                break;
            }

        }

        copyData.insert(index,*i);
    }

	mergeDetection();
    update();
}

void FOV_2DScan::mergeDetection()
{
	int indexMerged;
	int index;
	int indexPoint;
	bool found;

	mergedData.clear();
	for (indexPoint = 0; indexPoint < copyData.count(); ++indexPoint)
	{
		found = false;
		indexMerged = 0;
		if (mergeDetectionMode != eNoMerge)
		{

			for (indexMerged = 0; indexMerged < mergedData.count(); ++indexMerged)
			{
				if (mergedData[indexMerged].count())
				{
					for (index = 0; index < mergedData[indexMerged].count(); ++index)
					{
						if (isInRange(&copyData[indexPoint],&mergedData[indexMerged][index] ))
						{
							found = true;
							break;
						}
					}

					if (found)
						break;
				}
			}
		}

		if (!found)
		{
			mergedData.append(DetectionDataVect());
			mergedData.last().append(copyData[indexPoint]);
		}
		else
			mergedData[indexMerged].append(copyData[indexPoint]);
    }

}

bool FOV_2DScan::isInRange(DetectionData* detection1, DetectionData* detection2 )
{
	bool distInRange = false;
	bool angleInRange = false;

	if (mergeDetectionMode == eRadial)
	{
		if ((detection1->distanceRadial < (detection2->distanceRadial + mergeAcceptance)) && 
		(detection1->distanceRadial > (detection2->distanceRadial - mergeAcceptance)))
		{
			distInRange = true;
		}
	}
	else if (mergeDetectionMode == eLongitudinal)
	{
		if ((detection1->distanceLongitudinal < (detection2->distanceLongitudinal + mergeAcceptance)) && 
		(detection1->distanceLongitudinal > (detection2->distanceLongitudinal - mergeAcceptance)))
		{
			distInRange = true;
		}
	}

	if (((detection1->angle+(detection1->angleWidth/2)) > (detection2->angle-(detection2->angleWidth/2))) || 
		((detection2->angle+(detection2->angleWidth/2)) > (detection1->angle-(detection1->angleWidth/2))))
	{
		angleInRange = true;
	}

	return (angleInRange && distInRange);
}

void FOV_2DScan::closeEvent(QCloseEvent * event)
{
	emit closed();
}

void FOV_2DScan::ShowContextMenu(const QPoint& pos) // this is a slot
{
    // for most widgets
    QPoint globalPos = mapToGlobal(pos);
    // for QAbstractScrollArea and derived classes you would use:
    // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos); 


	QMenu mainMenu;
	//QMenu* menuMergeDetection = mainMenu.addMenu("Merge Detection Mode");
	QMenu* menuMergeDisplay = mainMenu.addMenu("Merge Channels");
	QMenu* menuMeasureMode = mainMenu.addMenu("Distance calculation");
	QMenu* menuColorCode = mainMenu.addMenu("Distance vs velocity");
   
	//menuMergeDetection->addAction(noMergeAction);
	//menuMergeDetection->addAction(radialAction);
	//menuMergeDetection->addAction(distanceAction);

	menuMergeDisplay->addAction(noMergeDisplayAction);
	menuMergeDisplay->addAction(individualDistanceDisplayAction);
	menuMergeDisplay->addAction(mergeDistanceDisplayAction);
	menuMergeDisplay->addAction(clusteredDistanceDisplayAction);

	menuMeasureMode->addAction(measureRadialAction);
	menuMeasureMode->addAction(measureLongitudinalAction);

	mainMenu.addAction(showPaletteAction);

	menuColorCode->addAction(colorCodeDistanceAction);
	menuColorCode->addAction(colorCodeVelocityAction);
	
    mainMenu.exec(globalPos);
}

#include "fov_2dscan.h"
#include "Tracker.h"

#include <QPainter>
#include <QLabel>
#include <QMenu>
#include <QApplication>
#include <QDesktopWidget>

#include <boost/foreach.hpp>

#define _USE_MATH_DEFINES 1  // Makes sure we have access to all math constants, like M_PI
#include <math.h>

#include "awlcoord.h"
#include "AWLSettings.h"

using namespace awl;


const int transitionLightness= 160;  // Lightness at which we start to write in ligther shade
const QColor rgbRuler(128, 128, 128, 127); // Transparent gray
const QColor rgbBumper(63, 63, 63, 196); // Transparent gray
const QColor rgbLaneMarkings(0, 0 , 0, 196);  // Black


// Tricky part of the code:
// In our coordinate systsem, X axis is depth and Y is lateral (with Y positive towards left)
// In the display system, X is latereal (positive right) and y is depth....
// For this reason, when using the CartesianCoordinates class, we recommend using the 
// "forward, left, up" nomenclature.


float logoAspectRatio = 1.0;
const int paletteWidth = 50;

FOV_2DScan::FOV_2DScan(QWidget *parent) :
    QFrame(parent)
{
    Ratio = 1;
    ShowPalette = true;
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	mergeDisplayMode = (MergeDisplayMode)globalSettings->mergeDisplayMode;
	measureMode = (MeasureMode)globalSettings->measureMode;
	displayDistanceMode = (DisplayDistanceMode) globalSettings->displayDistanceMode2D;
	mergeAcceptanceX = globalSettings->mergeAcceptanceX;
	mergeAcceptanceY = globalSettings->mergeAcceptanceX;
	ShowPalette = globalSettings->showPalette;
	colorCode = (DisplayColorCode )globalSettings->colorCode2D;
	maxAbsVelocity = globalSettings->maxVelocity2D;
	zeroVelocity = globalSettings->zeroVelocity;
	carWidth = globalSettings->carWidth;
	carLength = globalSettings->carLength;
	carHeight = globalSettings->carHeight;
	laneWidth = globalSettings->laneWidth;


	// Position the widget on the top right side
	setMinimumSize(480,480);

    rgblongRangeLimited = qRgba(188,205,203,127);
    rgblongRange = qRgba(58,126,209,127);
    rgbshortRangeLimited = qRgba(184,220,175,127);
    rgbshortRange = qRgba(54,166,38,127);

	setContextMenuPolicy(Qt::CustomContextMenu);
	connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),this, SLOT(ShowContextMenu(const QPoint&)));

	createAction();

	// Create a label to hold a logo, only if there is one specified in INI file.

	logoLabel = new QLabel(this);
	QPixmap *myPix = NULL;
	if (!globalSettings->sLogoFileName.empty()) 
	{
		myPix = new QPixmap(globalSettings->sLogoFileName.c_str());
	}
	
	if (myPix && !myPix->isNull())
	{
		float pixWidth = myPix->width();
		float pixHeight = myPix->height();
		logoAspectRatio = pixWidth / pixHeight;
		logoLabel->setPixmap(*myPix);
	}
	else
	{
		logoAspectRatio = 3.0/1.0;
	}

	if (myPix) delete myPix;

	logoLabel->setScaledContents(true);

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

	//****
	groupDisplayDistanceMode = new QActionGroup( this );
	displayDistanceModeShowAction = new QAction("Show Distances", this);
	displayDistanceModeHideAction = new QAction("Hide Distances", this);
	
	displayDistanceModeShowAction->setCheckable(true);
	displayDistanceModeShowAction->setActionGroup(groupDisplayDistanceMode);

	displayDistanceModeHideAction->setCheckable(true);
	displayDistanceModeHideAction->setActionGroup(groupDisplayDistanceMode);

	if (displayDistanceMode == eDisplayDistanceModeShow)
	{
		displayDistanceModeShowAction->setChecked(true);
	}
	else
	{
		displayDistanceModeHideAction->setChecked(true);
	}

	connect(groupDisplayDistanceMode, SIGNAL(triggered(QAction*)), this, SLOT(slotDisplayDistanceModeAction()));
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

void FOV_2DScan::slotDisplayDistanceModeAction()
{
	if (displayDistanceModeShowAction->isChecked())
	{
		 displayDistanceMode = eDisplayDistanceModeShow;
	}
	else
	{
		 displayDistanceMode = eDisplayDistanceModeHide;
	}
}

double zeroY = 0.0;
double zeroX = 0.0;

void FOV_2DScan::slotConfigChanged(const ConfigSensor &inConfig)
{
    config = inConfig;

	// All distances reported are relative to bumper
	double totalDistance = config.longRangeDistance+config.spareDepth;

	// Calculate minimum window size;
	float minHeight = 240;
	Ratio = (minHeight-(minHeight*0.1)) / totalDistance;
 
	float angleInRad = DEG2RAD((config.shortRangeAngle/2)+180);
	float xWidth = abs((totalDistance*Ratio)*sinf(angleInRad));

	int minWidth = (xWidth*2)+125;
    setMinimumSize(minWidth, minHeight);

	QRect scr = QApplication::desktop()->availableGeometry(/*QApplication::desktop()->primaryScreen()*/);
	QRect frame = frameGeometry();
	QRect client = geometry();
	int verticalDecorationsHeight = frame.height() - client.height();
	int horizontalDecorationsWidth = frame.width() - client.width();

	float recommendedHeight =  scr.height() - verticalDecorationsHeight;
	Ratio = (recommendedHeight -(recommendedHeight*0.1)) / totalDistance;
	xWidth = abs((totalDistance*Ratio)*sinf(angleInRad));
	float recommendedWidth = (xWidth*2)+125;
	resize(recommendedWidth, recommendedHeight);
	move(scr.right()-(recommendedWidth + horizontalDecorationsWidth), scr.top());

	zeroY = height() - (config.spareDepth * Ratio);
	zeroX = width()/2;
    update();
}

void FOV_2DScan::resizeEvent(QResizeEvent * event)
{
	double totalDistance = config.longRangeDistance+config.spareDepth;
	Ratio = (height()-(height()*0.1)) / totalDistance;

	int labelWidth = width() * 0.3;
	int labelHeight = labelWidth / logoAspectRatio;

	logoLabel->resize(labelWidth, labelHeight);
	logoLabel->move(55, height() - labelHeight);

	zeroY = height() - (config.spareDepth * Ratio);
	zeroX = width()/2;
}

void FOV_2DScan::paintEvent(QPaintEvent *)
{

    QPainter painter(this);
    painter.fillRect(0,0,width(),height(),QBrush(Qt::white));

	// Draw sensor FOVs
	painter.setPen(Qt::NoPen);

	int receiverQty = AWLSettings::GetGlobalSettings()->receiverSettings.size();
	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		int channelQty = AWLSettings::GetGlobalSettings()->receiverSettings[receiverID].channelsConfig.size();
		for (int channelID = 0; channelID < channelQty; channelID++)
		{
			ReceiverSettings receiverSettings = AWLSettings::GetGlobalSettings()->receiverSettings[receiverID];
			ChannelConfig channelConfig =receiverSettings.channelsConfig[channelID];

			QColor channelColor(channelConfig.displayColorRed, channelConfig.displayColorGreen, channelConfig.displayColorBlue, 192);
			painter.setBrush(QBrush(channelColor));
			float startAngle = AWLSettings::GetGlobalSettings()->receiverSettings[receiverID].sensorYaw + 
				               channelConfig.centerX + (channelConfig.fovWidth/2);
			// Angles in drawPie are counter clockwise, our config is also counter clockwise. 
			// All distances are relative to bumper, subtract the sensor depth  
			// Angles are drawn from sensor position add the sensor depth
			drawPie(&painter, startAngle, -channelConfig.fovWidth, channelConfig.maxRange,
					-receiverSettings.sensorLeft, -receiverSettings.sensorForward);
		}
	}

    //Angular Ruler
    painter.setPen(QPen(rgbRuler));

    for (int i = config.longRangeDistance; i > 0; i-=5)
	{
		// All distances are relative to bumper  
        drawArc(&painter, -config.shortRangeAngle/2, config.shortRangeAngle, i);
	}

    painter.setPen(QPen(rgbRuler));
    for (int i = config.shortRangeAngle; i >= 0; i-=5)
	{
        drawLine(&painter, i-(config.shortRangeAngle/2), 0, config.longRangeDistance);
	}

    drawAngularRuler(&painter);


	//Paint Draw bumper area
	painter.setBrush(QBrush(rgbBumper));
	painter.setPen(Qt::NoPen);

	// Draw "car" rect at center with width of car and depth equal to spareDepth
	// Sensor depth is a negative offset from bumper!
	int carWidthScreen = (int) (carWidth * Ratio); // Car width in displayUnits. 
												   // Always should be an odd number to be spread equally across center
	if (!carWidthScreen & 0x01) carWidthScreen++;

	int centerX = width() / 2;

	painter.drawRect(QRect(centerX - (carWidthScreen/2), height()-(config.spareDepth*Ratio), carWidthScreen, carHeight*Ratio));

	// Draw lane markings
	QPen lanePen(rgbLaneMarkings);
	lanePen.setStyle(Qt::DashLine);
	lanePen.setWidth(2);
	painter.setPen(lanePen);
	int laneWidthScreen = (int) (laneWidth * Ratio); // Car width in displayUnits. 

	painter.drawLine(centerX - (laneWidthScreen/2), height()*Ratio, centerX - (laneWidthScreen/2), height()-((config.longRangeDistance+config.spareDepth)*Ratio));
	painter.drawLine(centerX + (laneWidthScreen/2), height()*Ratio, centerX + (laneWidthScreen/2), height()-((config.longRangeDistance+config.spareDepth)*Ratio));

    if (ShowPalette)
        drawPalette(&painter);

    lastRightTextHeight = height();

    rightQty = 0;

	// Draw the merged indicators, only if there are more than 1 detections in the area
	// Otherwise, they are displayed as a square
	BOOST_FOREACH(const Detection::Vector & mergedDataItem, mergedData)
	{
			if (mergeDisplayMode == eNoMergeDisplay) 
			{
				// No merge display, do nothing
			}
			else if (mergeDisplayMode == eIndividualDistanceDisplay)
			{
				// Only draw the bounding rectangle, only if there is more than one detection
				// We never draw the targe or the legend
				if (mergedDataItem.size() > 1) drawMergedData(&painter, mergedDataItem, true, false, false);
			}
			else if (mergeDisplayMode == eMergeDistanceDisplay || mergeDisplayMode == eClusteredDistanceDisplay)
			{
				// Draw the bounding rectangle,, only if there is more than one detection
				// Otherwise draw the merged position using the "individual" detections look.
				bool bDrawBoundingBox = true;
				bool bDrawTarget = false;
				bool bDrawLegend = (displayDistanceMode == eDisplayDistanceModeShow);
				if (mergedDataItem.size() <= 1) 
				{
					bDrawBoundingBox = false;
					bDrawTarget = true;
				}

				drawMergedData(&painter, mergedDataItem, bDrawBoundingBox, bDrawTarget, bDrawLegend); 
			}
	} // for

	// Draw the individual detections
	BOOST_FOREACH(const Detection::Ptr &detection, copyData)
	{
		if (mergeDisplayMode == eNoMergeDisplay || mergeDisplayMode == eIndividualDistanceDisplay)
		{
			drawDetection(&painter, detection, true, (displayDistanceMode == eDisplayDistanceModeShow));
		}
		else if (mergeDisplayMode == eMergeDistanceDisplay)
		{
			// Don't display the individual distances
		}
		else // if (mergeDisplayMode != eClusteredDistanceDisplay)
		{
			// Display the individual distance targets in s-scan without accompanying legend
			drawDetection(&painter, detection, true, false);
		}
	} // BOOST_FOREACH(const Detection::Ptr
}

void FOV_2DScan::drawMergedData(QPainter* p, const Detection::Vector& data, bool drawBoundingBox, bool drawTarget, bool drawLegend)
{
	int index;
	float sphericalDistanceMin = config.longRangeDistance;
	float sphericalDistanceMax = 0;
	float sphericalDistanceAverage = 0;

	float velocityMin = 999;

	float forwardMin = config.longRangeDistance;
	float forwardMax = -config.longRangeDistance;
	float distanceForwardAverage = 0;
	float leftMin = config.longRangeDistance;
	float leftMax = -config.longRangeDistance;

	QPolygon poly;

	BOOST_FOREACH(const Detection::Ptr detection, data)
	{
		if (detection->relativeToVehicleSpherical.rho > sphericalDistanceMax)
			sphericalDistanceMax = detection->relativeToVehicleSpherical.rho;
		if (detection->relativeToVehicleSpherical.rho < sphericalDistanceMin)
			sphericalDistanceMin = detection->relativeToVehicleSpherical.rho;
		sphericalDistanceAverage += detection->relativeToVehicleSpherical.rho;

		if (detection->relativeToVehicleCart.left > leftMax)
			leftMax = detection->relativeToVehicleCart.left;
		if (detection->relativeToVehicleCart.left < leftMin)
			leftMin = detection->relativeToVehicleCart.left;


		if (detection->relativeToVehicleCart.forward > forwardMax)
			forwardMax = detection->relativeToVehicleCart.forward;
		if (detection->relativeToVehicleCart.forward < forwardMin)
			forwardMin = detection->relativeToVehicleCart.forward;
		distanceForwardAverage += detection->relativeToVehicleCart.forward;

		if (detection->velocity < velocityMin) velocityMin = detection->velocity;
	} // BOOST_FOREACH (detection)

	if (data.size()) 
	{
		sphericalDistanceAverage /= data.size(); 
		distanceForwardAverage /= data.size();
	}

	QString velocityLabel = " m/s";
	float distanceDisplayed = 0;0;
	float velocityDisplayed = velocityMin;

	if (measureMode == eMeasureRadial)
		distanceDisplayed = sphericalDistanceMin;
	else
		distanceDisplayed = forwardMin;

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
	p->setBrush(backColor);

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


		if (backColor.lightness() < transitionLightness) 
			pencolor = Qt::white;
		else
			pencolor = Qt::black;

		// Draw the detection, but without the legend
		const Detection::Ptr detection = data.at(0);
		drawTextDetection(p, detection, textToDisplay, pencolor, backColor, drawTarget, drawLegend);
	}


	// Now draw the bounding rectangle
	if (drawBoundingBox) 
	{
		pencolor = backColor.darker(180);
		p->setPen(pencolor);


		// Remember coordinate axes X and Y are not in same orientation as QT drawing axes!
		QPoint bottomLeft(leftMin * Ratio , -(forwardMin+config.spareDepth)*Ratio);
		QPoint topRight(leftMax* Ratio, -(forwardMax+config.spareDepth)*Ratio);
		QPoint temp;

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
#if 0
	double pos = config.longRangeDistance-config.spareDepth;
#else
	double pos = config.longRangeDistance;
#endif

	drawArc(p, -config.shortRangeAngle/2, config.shortRangeAngle, pos + (5.0/Ratio));
	for (int i = config.shortRangeAngle; i >= 0; i-=5)
	{
		drawLine(p, i-(config.shortRangeAngle/2), pos+(5.0/Ratio), (10.0/Ratio));

		// drawText removes the spareDepthAlready. 
		drawText(p, i-(config.shortRangeAngle/2), config.longRangeDistance+(20.0/Ratio), QString::number((i-(config.shortRangeAngle/2)))+"°", Qt::red);
	}
}

void FOV_2DScan::drawDetection(QPainter* p, const Detection::Ptr &detection, bool drawTarget, bool drawLegend)
{
	QColor backColor;
	QColor penColor = Qt::black;
	QString textToDisplay;

	float distanceToDisplay;
	float velocityToDisplay;
	QString velocityLabel;

	if (measureMode == eMeasureRadial)
		distanceToDisplay = detection->relativeToVehicleSpherical.rho;
	else
		distanceToDisplay = detection->relativeToVehicleCart.forward;

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
	if (backColor.lightness() < transitionLightness) penColor = Qt::white;

    drawTextDetection(p, detection, textToDisplay, penColor,backColor, drawTarget, drawLegend);
}

void FOV_2DScan::drawTextDetection(QPainter* p, const Detection::Ptr &detection, QString text, QColor foregroundColor, QColor backgroundcolor,
	                                bool drawTarget, bool drawLegend)
{
	// Our detection Y axis is positive left, so we negate the lateral coordinate.
 	// Drawing coordinate system is reversed vertically , (0,0) is top left, so we negate Y position as well;
    QPoint start(-detection->relativeToVehicleCart.left * Ratio, 
	             -(detection->relativeToVehicleCart.forward+config.spareDepth) * Ratio);
    QRect tempRect;
    QPolygon poly;

    QRect rect = p->boundingRect(QRect(0,0,0,0), Qt::AlignCenter,  text);
    rect.setSize(rect.size()+QSize(10,10));
	int windowWidth = width();
	int windowHeight = height();

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
		if (backgroundcolor.lightness() < transitionLightness) pencolor = backgroundcolor.lighter(180);
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
	float angleInRad = DEG2RAD(angle+180);
	// Real position of object, from sensor on  the grid is postion + bumperOffset.
	// the spareDepth was added at the moment of capture, so we have to remove it here.
	pos += config.spareDepth;

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

void FOV_2DScan::drawArc(QPainter* p, float startAngle, float angularSpan, float radius, float xOffset, float yOffset)
{
	// Left is the 
	float left = zeroX - (radius * Ratio);

	// Bottom
	float top = zeroY;
	top -= radius * Ratio;
	top += yOffset * Ratio;

	float width = radius * Ratio * 2;
	float height = radius * Ratio * 2;

    QRectF rectangle(left, top, width, height);

    p->drawArc(rectangle, (startAngle+90)*16, angularSpan*16);
}

void FOV_2DScan::drawPie(QPainter* p, float startAngle, float angularSpan, float radius, float xOffset, float yOffset)
{
	float left = zeroX - (radius * Ratio);

	// Bottom
	float top = zeroY;
	top -= radius * Ratio;
	top += yOffset * Ratio;

	float width = radius * Ratio * 2;
	float height = radius * Ratio * 2;

    QRectF rectangle(left, top, width, height);
    p->drawPie(rectangle, (startAngle+90)*16, angularSpan*16);
}

void FOV_2DScan::drawLine(QPainter* p, float angle, float startRadius, float length)
{

    float angleInRad = DEG2RAD(angle+180);

    QPoint start(0, (startRadius*Ratio));
    QPoint end(0, ((startRadius+length)*Ratio));
    QPoint zeroPoint(zeroX, zeroY);
	QPoint temp;

    temp = start;
    start.setX(temp.x()*cosf(angleInRad) - temp.y()*sinf(angleInRad));
    start.setY(temp.y()*cosf(angleInRad) + temp.x()*sinf(angleInRad));

    temp = end;
    end.setX(temp.x()*cosf(angleInRad) - temp.y()*sinf(angleInRad));
    end.setY(temp.y()*cosf(angleInRad) + temp.x()*sinf(angleInRad));

    p->drawLine(start + zeroPoint, end + zeroPoint);
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
    p->drawRect(0, height()*0.1, paletteWidth, height()*0.9);

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

bool sortDetectionsBottomLeftTopRight (Detection::Ptr &left, Detection::Ptr &right) 

{ 
	// Same X (forward)  position, sort from left to right
	if (qFuzzyCompare(left->relativeToVehicleCart.forward,  right->relativeToVehicleCart.forward))
	{
		// Remember vehicle Y is positive going left, So we reverse the < operator.
		if (left->relativeToVehicleCart.left > right->relativeToVehicleCart.left)
		{
				return(true);
		}
		else
		{
			return(false);
		}
	}
	// Not same depth, compare forward
	else if (left->relativeToVehicleCart.forward < right->relativeToVehicleCart.forward)
	{
		return(true);
	}
	else 
	{
		return(false);
	}

}

void FOV_2DScan::slotDetectionDataChanged(const Detection::Vector& data)
{
    Detection::Vector::const_iterator i;
    int index;

	// Make a copy of the provided Detection::Vector to work with
    copyData.clear();
	copyData = data;


	//Ordering detection from bottom left to top right of 2D view. It's to simplify algo to draw detection in view.
	std::sort(copyData.begin(), copyData.end(), sortDetectionsBottomLeftTopRight);

	//Merge detections according to distance criteria
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
	for (indexPoint = 0; indexPoint < copyData.size(); ++indexPoint)
	{
		found = false;
		indexMerged = 0;
		if (mergeDetectionMode != eNoMerge)
		{

			for (indexMerged = 0; indexMerged < mergedData.size(); ++indexMerged)
			{
				if (mergedData[indexMerged].size())
				{
					for (index = 0; index < mergedData[indexMerged].size(); ++index)
					{
						if (isInRange(copyData[indexPoint], mergedData[indexMerged][index] ))
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
			mergedData.push_back(Detection::Vector());
			mergedData.at(mergedData.size()-1).push_back(copyData[indexPoint]);
		}
		else
			mergedData[indexMerged].push_back(copyData[indexPoint]);
    }

}

bool FOV_2DScan::isInRange(const Detection::Ptr &detection1, const Detection::Ptr &detection2 )
{
	bool distInRange = false;
	bool lateralInRange = false;

	if ((detection1->relativeToVehicleCart.left < (detection2->relativeToVehicleCart.left + mergeAcceptanceX)) && 
		(detection1->relativeToVehicleCart.left > (detection2->relativeToVehicleCart.left - mergeAcceptanceX)))
	{
			lateralInRange = true;
	}


	if ((detection1->relativeToVehicleCart.forward < (detection2->relativeToVehicleCart.forward + mergeAcceptanceY)) && 
		(detection1->relativeToVehicleCart.forward > (detection2->relativeToVehicleCart.forward - mergeAcceptanceY)))
	{
		distInRange = true;
	}

	return (lateralInRange && distInRange);
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
	QMenu* menuDistanceDisplayMode = mainMenu.addMenu("Distance display");
   
	//menuMergeDetection->addAction(noMergeAction);
	//menuMergeDetection->addAction(radialAction);
	//menuMergeDetection->addAction(distanceAction);

	menuMergeDisplay->addAction(noMergeDisplayAction);
	menuMergeDisplay->addAction(individualDistanceDisplayAction);
	menuMergeDisplay->addAction(mergeDistanceDisplayAction);
	menuMergeDisplay->addAction(clusteredDistanceDisplayAction);

	menuMeasureMode->addAction(measureRadialAction);
	menuMeasureMode->addAction(measureLongitudinalAction);

	menuDistanceDisplayMode->addAction(displayDistanceModeHideAction);
	menuDistanceDisplayMode->addAction(displayDistanceModeShowAction);

	mainMenu.addAction(showPaletteAction);

	menuColorCode->addAction(colorCodeDistanceAction);
	menuColorCode->addAction(colorCodeVelocityAction);
	
    mainMenu.exec(globalPos);
}

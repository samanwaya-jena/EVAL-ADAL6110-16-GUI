
/* Fov_2DScan.cpp */
/*
	Copyright (C) 2014, 2015  Phantom Intelligence Inc.

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

#include "fov_2dscan.h"
#include "DetectionStruct.h"

#include <QPainter>
#include <QLabel>
#include <QMenu>
#include <QApplication>
#include <QDesktopWidget>
#include <QEvent>

#include <boost/foreach.hpp>


#include "awlcoord.h"
#include "AWLSettings.h"

#define _USE_MATH_DEFINES 1  // Makes sure we have access to all math constants, like M_PI
#include <math.h>

using namespace awl;

//----------------------Intensity Classifier
#if 1

typedef enum ClassificationType
{
	eClassifyUnknown = 0,
	eClassifyMiner = 1,
	eClassifyMineWall = 2,
	eClassifyPedestrian = 3,
	eClassifyCar = 4
}
ClassificationType;

const unsigned int channel0Mask = 0x01;
const unsigned int channel1Mask = 0x02;
const unsigned int channel2Mask = 0x04;
const unsigned int channel3Mask = 0x08;
const unsigned int channel4Mask = 0x10;
const unsigned int channel5Mask = 0x20;
const unsigned int channel6Mask = 0x40;


typedef struct 
{
	unsigned int channelMask;
	float minDistance;
	float maxDistance;
	float minIntensity;
	float maxIntensity;
	ClassificationType classificationType;
}
ClassificationEntry;

ClassificationEntry classificationEntries[] = 
{
#if 1 // All types
	{channel0Mask | channel1Mask | channel2Mask | channel3Mask | channel4Mask|channel5Mask|channel6Mask, 0, 300, 0, 100.00, eClassifyMiner},
#endif
	// Long range
	{channel4Mask|channel5Mask|channel6Mask, 0.5, 2.5, 73.50, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 2.5, 3.5, 69.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 3.5, 4.5, 67.50, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 4.5, 5.5, 63.50, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 5.5, 6.5, 60.50, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 6.5, 7.5, 59.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 7.5, 8.5, 54.50, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 8.5, 9.5, 51.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 9.5, 10.5, 47.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 10.5, 11.5, 44.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 11.5, 12.5, 44.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 12.5, 13.5, 44.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 13.5, 14.5, 48.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 14.5, 15.5, 47.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 15.5, 16.5, 44.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 16.5, 17.5, 43.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 17.5, 18.5, 36.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 18.5, 19.5, 36.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 19.5, 20.5, 32.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 20.5, 21.5, 31.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 21.5, 22.5, 32.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 22.5, 23.5, 29.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 23.5, 24.5, 26.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 24.5, 25.5, 21.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 25.5, 26.5, 22.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 26.5, 27.5, 20.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 27.5, 28.5, 19.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 28.5, 29.5, 15.00, 80.00, eClassifyMiner},
	{channel4Mask|channel5Mask|channel6Mask, 29.5, 30.5, 11.00, 80.00, eClassifyMiner},

	// Side short-range
	{channel0Mask|channel3Mask, 0.5, 2.5, 58.80, 80.00, eClassifyMiner},
	{channel0Mask|channel3Mask, 2.5, 3.5, 55.20, 80.00, eClassifyMiner},
	{channel0Mask|channel3Mask, 3.5, 4.5, 54.00, 80.00, eClassifyMiner},
	{channel0Mask|channel3Mask, 4.5, 5.5, 50.80, 80.00, eClassifyMiner},
	{channel0Mask|channel3Mask, 5.5, 6.5, 48.4, 80.00, eClassifyMiner},
	{channel0Mask|channel3Mask, 6.5, 7.5, 47.20, 80.00, eClassifyMiner},
	{channel0Mask|channel3Mask, 7.5, 8.5, 45.60, 80.00, eClassifyMiner},
	{channel0Mask|channel3Mask, 8.5, 9.5, 40.8, 80.00, eClassifyMiner},
	{channel0Mask|channel3Mask, 9.5, 10.5, 36.80, 80.00, eClassifyMiner},

	// Center short-range
	{channel1Mask|channel2Mask, 0.5, 2.5, 58.80, 80.00, eClassifyMiner},
	{channel1Mask|channel2Mask, 2.5, 3.5, 55.20, 80.00, eClassifyMiner},
	{channel1Mask|channel2Mask, 3.5, 4.5, 54.50, 80.00, eClassifyMiner},
	{channel1Mask|channel2Mask, 4.5, 5.5, 52.80, 80.00, eClassifyMiner},
	{channel1Mask|channel2Mask, 5.5, 6.5, 50.4, 80.00, eClassifyMiner},
	{channel1Mask|channel2Mask, 6.5, 7.5, 49.20, 80.00, eClassifyMiner},
	{channel1Mask|channel2Mask, 7.5, 8.5, 47.6, 80.00, eClassifyMiner},
	{channel1Mask|channel2Mask, 8.5, 9.5, 42.8, 80.00, eClassifyMiner},
	{channel1Mask|channel2Mask, 9.5, 10.5, 38.80, 80.00, eClassifyMiner},

	{0, 0, 0, 0, 0, eClassifyUnknown}
};

ClassificationType classifyFromIntensity(int channel, float distance, float intensity)

{
	unsigned int channelMask = 0x0001 << channel;

	for (ClassificationEntry *entry = &classificationEntries[0]; entry->channelMask != 0; entry++) 
	{
		if (distance >=8.0 && distance <= 9.0)
		{
			int test = 1;
		}

		if (entry->channelMask & channelMask) {
			if (distance >= entry->minDistance && distance < entry->maxDistance) {
				if (intensity >= entry->minIntensity && intensity < entry->maxIntensity)
				{
					return(entry->classificationType);
				}
			}
		}
	}

	return(eClassifyUnknown);
}

#endif
//----------------------End of Intensity Classifier


const int transitionLightness= 65;  // Lightness at which we start to write in ligther shade

const QColor rgbBackground(0, 0, 0, 255); // Black
const QColor rgbRuler(192, 192, 192, 128); // Transparent gray dark
const QColor rgbRulerLight(255, 255, 255, 128); // Transparent gray light
const QColor rgbRulerText(255, 170, 0);
const QColor rgbBumper(32, 32, 32, 196); // Transparent gray
const QColor rgbLaneMarkings(192, 192 , 255, 196);  // Light blue
const int    fovTransparency = 32;  // Transparency level of the FOVs
const int    lineTransparency = 128;
//const QColor rgbBackground(0, 0, 10, 255); // DK Blue
//const QColor rgbRuler(128, 128, 128, 255); // Transparent gray dark
//const QColor rgbRulerLight(192, 192, 192, 255); // Transparent gray light
//const QColor rgbRulerText(Qt::red);
//const QColor rgbBumper(63, 63, 63, 196); // Transparent gray
//const QColor rgbLaneMarkings(192, 192 , 255, 196);  // Light blue
//const int    fovTransparency = 128;  // Transparency level of the FOVs
//const int    lineTransparency = 64;

#define squareGrid 1

// Tricky part of the code:
// In our coordinate systsem, X axis is depth and Y is lateral (with Y positive towards left)
// In the display system, X is latereal (positive right) and y is depth....
// For this reason, when using the CartesianCoordinates class, we recommend using the 
// "forward, left, up" nomenclature.


const float gridOffset = 1.0;  // Grid spacing in Cartesian Display
const int paletteWidth = 50;

const int logoLeftMargin = 55; // Do not place logo to muh to the left of the screen
const int logoRightMargin = 10; // Do not place logo to muh to the left of the screen
const int logoTopMargin = 10; // Do not place logo to muh to the left of the screen

double zeroY = 0.0;
double zeroX = 0.0;
const int topInPixels = 50;
const int rightInPixels = 200;

FOV_2DScan::FOV_2DScan(QWidget *parent) :
    QFrame(parent)
{
	ui.setupUi(this);
	setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

	Ratio = 1;
    ShowPalette = true;
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	mergeDisplayMode = (MergeDisplayMode)globalSettings->mergeDisplayMode;
	measureMode = (MeasureMode)globalSettings->measureMode;
	displayDistanceMode = (DisplayDistanceMode) globalSettings->displayDistanceMode2D;
	displayZoomMode = (DisplayZoomMode) globalSettings->displayZoomMode2D;
	mergeAcceptanceX = globalSettings->mergeAcceptanceX;
	mergeAcceptanceY = globalSettings->mergeAcceptanceY;
	ShowPalette = globalSettings->showPalette;
	colorCode = (DisplayColorCode )globalSettings->colorCode2D;
	maxAbsVelocity = globalSettings->maxVelocity2D;
	zeroVelocity = globalSettings->zeroVelocity;
	carWidth = globalSettings->carWidth;
	carLength = globalSettings->carLength;
	carHeight = globalSettings->carHeight;
	laneWidth = globalSettings->laneWidth;

    rgblongRangeLimited = qRgba(188,205,203,127);
    rgblongRange = qRgba(58,126,209,127);
    rgbshortRangeLimited = qRgba(184,220,175,127);
    rgbshortRange = qRgba(54,166,38,127);

	// Create a label to hold a logo

	logoLabel = new QLabel(this);

	setContextMenuPolicy(Qt::CustomContextMenu);
	connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),this, SLOT(ShowContextMenu(const QPoint&)));

	createAction();
	calculateResize();
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
	measureCartesianAction = new QAction("Cartesian", this);	
	
	measureRadialAction->setCheckable(true);
	measureRadialAction->setActionGroup(groupMeasureMode);

	measureLongitudinalAction->setCheckable(true);
	measureLongitudinalAction->setActionGroup(groupMeasureMode);

	measureCartesianAction->setCheckable(true);
	measureCartesianAction->setActionGroup(groupMeasureMode);


	if (measureMode == eMeasureRadial)
	{
		measureRadialAction->setChecked(true);
	}
	else if (measureMode == eMeasureLongitudinal)
	{
		measureLongitudinalAction->setChecked(true);
	}
	else 
	{
		measureCartesianAction->setChecked(true);
	}

	connect(groupMeasureMode, SIGNAL(triggered(QAction*)), this, SLOT(slotMeasureModeAction()));

	showPaletteAction = new QAction("Palette", this);
	showPaletteAction->setCheckable(true);
	showPaletteAction->setChecked(ShowPalette);

	connect(showPaletteAction, SIGNAL(triggered()), this, SLOT(slotPaletteAction()));

	groupColorCode = new QActionGroup( this );
	colorCodeDistanceAction = new QAction("Distances", this);
	colorCodeVelocityAction = new QAction("Velocity", this);
	colorCodeIntensityAction = new QAction("Intensity/Threat", this);
	colorCodeChannelAction = new QAction("Channel color", this);
	colorCodeAlertAction = new QAction("Threat Level", this);
	
	colorCodeDistanceAction->setCheckable(true);
	colorCodeDistanceAction->setActionGroup(groupColorCode);

	colorCodeVelocityAction->setCheckable(true);
	colorCodeVelocityAction->setActionGroup(groupColorCode);

	colorCodeIntensityAction->setCheckable(true);
	colorCodeIntensityAction->setActionGroup(groupColorCode);

	colorCodeChannelAction->setCheckable(true);
	colorCodeChannelAction->setActionGroup(groupColorCode);

	colorCodeAlertAction->setCheckable(true);
	colorCodeAlertAction->setActionGroup(groupColorCode);

	if (colorCode == eColorCodeDistance)
	{
		colorCodeDistanceAction->setChecked(true);
	}
	else if (colorCode == eColorCodeVelocity)
	{
		colorCodeVelocityAction->setChecked(true);
	}
	else if (colorCode == eColorCodeIntensity)
	{
		colorCodeIntensityAction->setChecked(true);
	}
	else if (colorCode == eColorCodeChannel)
	{
		colorCodeChannelAction->setChecked(true);
	}
	else if (colorCode == eColorCodeAlert)
	{
		colorCodeAlertAction->setChecked(true);
	}
	else
	{
		colorCodeChannelAction->setChecked(true);
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


	//****
	groupDisplayZoomMode = new QActionGroup( this );
	displayZoomModeFrontAction = new QAction("Front only", this);
	displayZoomMode360Action = new QAction("Front and Rear", this);
	displayZoomModeAutoAction = new QAction("Maximum", this);

	displayZoomModeFrontAction->setCheckable(true);
	displayZoomModeFrontAction->setActionGroup(groupDisplayZoomMode);

	displayZoomMode360Action->setCheckable(true);
	displayZoomMode360Action->setActionGroup(groupDisplayZoomMode);

	displayZoomModeAutoAction->setCheckable(true);
	displayZoomModeAutoAction->setActionGroup(groupDisplayZoomMode);

	if (displayZoomMode == eDisplayZoomModeFront)
	{
		displayZoomModeFrontAction->setChecked(true);
	}
	else if (displayZoomMode == eDisplayZoomMode360)
	{
		displayZoomMode360Action->setChecked(true);
	}
	else
	{
		displayZoomModeAutoAction->setChecked(true);
	}


	connect(groupDisplayZoomMode, SIGNAL(triggered(QAction*)), this, SLOT(slotDisplayZoomModeAction()));
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
	else if (measureLongitudinalAction->isChecked())
	{
		measureMode = eMeasureLongitudinal;
	}
	else
	{
		measureMode = eMeasureCartesian;
	}

	slotConfigChanged();
}

void FOV_2DScan::slotColorCodeAction()
{
	if (colorCodeDistanceAction->isChecked())
	{
		colorCode = eColorCodeDistance;
	}
	else if (colorCodeVelocityAction->isChecked())
	{
		colorCode = eColorCodeVelocity;
	}
	else if (colorCodeIntensityAction->isChecked())
	{
		colorCode = eColorCodeIntensity;
	}
	else if (colorCodeChannelAction->isChecked())
	{
		colorCode = eColorCodeChannel;
	}
	else if (colorCodeAlertAction->isChecked())
	{
		colorCode = eColorCodeAlert;
	}
	else
	{
		colorCode = eColorCodeChannel;
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

void FOV_2DScan::slotDisplayZoomModeAction()
{
	if (displayZoomModeFrontAction->isChecked())
	{
		 displayZoomMode = eDisplayZoomModeFront;
	}
	else if(displayZoomMode360Action->isChecked())
	{
		 displayZoomMode = eDisplayZoomMode360;
	}
	else 
	{
		displayZoomMode = eDisplayZoomModeAuto;
	}

	slotConfigChanged();
}


void FOV_2DScan::slotConfigChanged()
{
	// Calculate the maximum displayed range and angular span

	int receiverQty = AWLSettings::GetGlobalSettings()->receiverSettings.size();
	config.maxSensorsRange = 0.0;
	config.maxAngularSpan = 0.0;
	config.spareDepth = 0;

	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		RelativePosition receiverPosition = AWLCoordinates::GetReceiverPosition(receiverID);
		config.maxSensorsRange = fmax(config.maxSensorsRange, AWLSettings::GetGlobalSettings()->receiverSettings[receiverID].displayedRangeMax);

		config.spareDepth = fmin (config.spareDepth, -receiverPosition.position.forward);
		
		int channelQty = AWLSettings::GetGlobalSettings()->receiverSettings[receiverID].channelsConfig.size();
		for (int channelID = 0; channelID < channelQty; channelID++)
		{
			ReceiverSettings receiverSettings = AWLSettings::GetGlobalSettings()->receiverSettings[receiverID];
			ChannelConfig channelConfig = receiverSettings.channelsConfig[channelID];
			RelativePosition channelPosition = AWLCoordinates::GetChannelPosition(receiverID, channelID);

			float startAngle = RAD2DEG(receiverPosition.orientation.yaw) +
				RAD2DEG(channelPosition.orientation.yaw) + (channelConfig.fovWidth / 2);
			config.maxAngularSpan = fmax(config.maxAngularSpan, fabs(startAngle));
			config.maxAngularSpan = fmax(config.maxAngularSpan, fabs(startAngle - channelConfig.fovWidth));
		}
	}

	if (displayZoomMode == eDisplayZoomModeAuto)
		config.spareDepth -= AWLSettings::GetGlobalSettings()->receiverSettings[0].displayedRangeMin;

	config.maxAngularSpan *= 2;
	// Span is always a multiple of 10 degrees - just for display aethetics
	config.maxAngularSpan = (1+ ((int)config.maxAngularSpan / 10)) * 10;

	setMinimumSize(minimumSizeHint());
	calculateResize();

    update();
}

QSize FOV_2DScan::sizeHint() const 
{ 
	return (maximumSizeHint());
}

QSize FOV_2DScan::minimumSizeHint() const 

{ 
	float maxHeight = 300;

	double totalDistance = config.maxSensorsRange+config.spareDepth;
	float hintRatio = (maxHeight-topInPixels) / totalDistance;

	float angleInRad = DEG2RAD((config.maxAngularSpan/2)+180);
	float xWidth = abs((totalDistance*hintRatio)*sinf(angleInRad));

	float maxWidth = (xWidth*2)+rightInPixels;
	return(QSize((int)maxWidth, (int)maxHeight));
}

QSize FOV_2DScan::maximumSizeHint() const 

{ 
	QRect scr = QApplication::desktop()->availableGeometry();
	float maxHeight = scr.height() * 1;
	float maxWidth = scr.width()*0.6;
	return(QSize((int)maxWidth, (int)maxHeight));	
}


void FOV_2DScan::resizeEvent(QResizeEvent * theEvent)
{
	calculateResize();
}

void FOV_2DScan::calculateResize()
{

	// For default mode where we are eDisplayZommModeFront
	double totalDistance = config.maxSensorsRange+config.spareDepth;
//printf("MaxRange: %f %d\n", config.maxSensorsRange, config.spareDepth);
	if (config.maxSensorsRange+config.spareDepth < 0.000000000000000001) return;

	if (displayZoomMode == eDisplayZoomMode360)
		totalDistance = (config.maxSensorsRange * 2) + carLength;

	float maxWidth = width() + 1;
	float maxHeight = height();


	while (maxWidth > width()) 
	{
		Ratio = (maxHeight-topInPixels) / totalDistance;

		float angleInRad = DEG2RAD((config.maxAngularSpan/2)+180);
		float xWidth = abs((totalDistance*Ratio)*sinf(angleInRad));
//printf("Ratio0: %f %f %f \n", totalDistance, Ratio, angleInRad);
//printf("Width0: %f %d %f \n", maxWidth, width(), xWidth);

		maxWidth = (xWidth*2)+rightInPixels;
		if (maxWidth > width())
		{
			maxHeight = (maxHeight * (width() / maxWidth)) -1;
		}
//printf("Width1: %f %d %f \n", maxWidth, width(), xWidth);
	}

	if (displayZoomMode == eDisplayZoomModeFront || displayZoomMode == eDisplayZoomModeAuto)
	{
		zeroY = maxHeight - (config.spareDepth * Ratio);
		zeroX = width()/2;
	}
	else
	{
		zeroY = maxHeight * 0.5; // .1 x height offcenter to the bottom
		zeroX = width() / 2;
	}

	int labelWidth = width() * 0.3;
	int labelHeight = labelWidth;
	logoLabel->resize(labelWidth, labelHeight);

	QPixmap *myPix = NULL;
	if (!AWLSettings::GetGlobalSettings()->sLogoFileName.empty())
	{
		myPix = new QPixmap(AWLSettings::GetGlobalSettings()->sLogoFileName.c_str());
	}

	if (myPix && !myPix->isNull())
	{

		logoLabel->setPixmap((*myPix).scaled(labelWidth, labelHeight, Qt::KeepAspectRatio, Qt::SmoothTransformation));
	}


	logoLabel->move(logoLeftMargin, height() - labelHeight);
}

void FOV_2DScan::paintEvent(QPaintEvent *paintEvent)
{

    QPainter painter(this);
	int maxWidth = width();
	int maxHeight = height();

	// Draw ruler

	if (measureMode != eMeasureCartesian)
	{
		//Angular Ruler
		painter.setPen(QPen(rgbRuler));
		float angleIncrement = 5.0;
		float distanceIncrement = 1.0;
		if (config.maxSensorsRange >= 100.0) distanceIncrement = 10;
		else if (config.maxSensorsRange > 10.0) distanceIncrement = 5; 
		else distanceIncrement = 1.0;

		for (float i = config.maxSensorsRange; i > 0; i-=distanceIncrement)
		{
			// All distances are relative to bumper  
			drawArc(&painter, -config.maxAngularSpan/2, config.maxAngularSpan, i);
		}

		painter.setPen(QPen(rgbRuler));
		for (float i = -config.maxAngularSpan / 2; i <= config.maxAngularSpan / 2; i += angleIncrement)
		{
			drawLine(&painter, i, 0, config.maxSensorsRange);
		}

		// Draw the distance indicators 
		for (float i = config.maxSensorsRange; i > 0; i-=distanceIncrement)
		{
			drawText(&painter, -(config.maxAngularSpan/2), i, QString::number(i)+"m", rgbRulerText, -20);
		}

		// Draw the angle indicators
		drawAngularRuler(&painter);
	}
	else 
	{
		// Grid Ruler

		float gridWidth = (width() / Ratio) / 2;
		float rangeWidth = config.maxSensorsRange + (carWidth/2);  // Grid width is maximum of the range of sensor
		if (gridWidth > rangeWidth) gridWidth = rangeWidth;

		gridWidth = (1+((int) (gridWidth / gridOffset))) * gridOffset;
		gridWidth *= Ratio;

		float gridYSpacing = gridOffset * Ratio;
		float gridXSpacing = gridYSpacing;

		
		painter.setPen(QPen(rgbRulerLight));
		for (float gridY = 0; gridY <= (config.maxSensorsRange*Ratio); gridY += gridYSpacing)
		{ 
			painter.drawLine(zeroX - gridWidth, zeroY - gridY,  zeroX + gridWidth, zeroY - gridY);
		}

		for (float gridY = - gridYSpacing; gridY >= -((config.maxSensorsRange+ carLength)*Ratio); gridY -= gridYSpacing)
		{ 
			painter.drawLine(zeroX - gridWidth, zeroY - gridY,  zeroX + gridWidth, zeroY - gridY);
		}

		for (float gridX = -(gridWidth); gridX <= (gridWidth); gridX += gridXSpacing)
		{ 
			painter.drawLine(zeroX + gridX, zeroY - (config.maxSensorsRange*Ratio), zeroX + gridX, zeroY + ((config.maxSensorsRange + carLength)*Ratio));
		}
	}

		// Draw sensor FOVs
	painter.setPen(Qt::NoPen);

	int receiverQty = AWLSettings::GetGlobalSettings()->receiverSettings.size();
	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		RelativePosition receiverPosition = AWLCoordinates::GetReceiverPosition(receiverID);
		int channelQty = AWLSettings::GetGlobalSettings()->receiverSettings[receiverID].channelsConfig.size();
		for (int channelID = 0; channelID < channelQty; channelID++)
		{
			ReceiverSettings receiverSettings = AWLSettings::GetGlobalSettings()->receiverSettings[receiverID];
			ChannelConfig channelConfig =receiverSettings.channelsConfig[channelID];
			RelativePosition channelPosition = AWLCoordinates::GetChannelPosition(receiverID,channelID);

			QColor channelColor(channelConfig.displayColorRed, channelConfig.displayColorGreen, channelConfig.displayColorBlue, fovTransparency);
			painter.setBrush(QBrush(channelColor));

			float startAngle = RAD2DEG(receiverPosition.orientation.yaw) + 
				               RAD2DEG(channelPosition.orientation.yaw) + (channelConfig.fovWidth/2);

			// Angles in drawPie are counter clockwise, our config is also counter clockwise. 
			// All distances are relative to bumper, subtract the sensor depth  
			// Angles are drawn from sensor position add the sensor depth
			drawPie(&painter, startAngle, -channelConfig.fovWidth, channelConfig.maxRange,
				-receiverPosition.position.left, -receiverPosition.position.forward);
		}
	}

	//Paint Draw bumper area
	painter.setBrush(QBrush(rgbBumper));
	painter.setPen(Qt::NoPen);

	// Draw "car" rect at center with width of car and depth equal to spareDepth
	// Sensor depth is a negative offset from bumper!
	int carWidthScreen = (int) (carWidth * Ratio); // Car width in displayUnits. 
												   // Always should be an odd number to be spread equally across center
	if (!(carWidthScreen & 0x01)) carWidthScreen++;

	int centerX = width() / 2;


	// Car front is always displayed at zeroY
	painter.drawRect(QRect(zeroX - (carWidthScreen/2), zeroY, 
		             carWidthScreen, carLength*Ratio));

	// Draw lane markings
	QPen lanePen(rgbLaneMarkings);
	lanePen.setStyle(Qt::DashLine);
	lanePen.setWidth(2);
	painter.setPen(lanePen);
	int laneWidthScreen = (int) (laneWidth * Ratio); // Car width in displayUnits. 

	//Right line drawn first
	int labelHeight = logoLabel->height();
	int labelWidth = logoLabel->width();
	int laneX = centerX + (laneWidthScreen / 2);
	int laneYBottom = height();
	int laneYTop = zeroY - (config.maxSensorsRange*Ratio);

	painter.drawLine(laneX, laneYBottom, laneX, laneYTop);

	//Make sure left lane marking does not overwrite logoLabel
	laneX = centerX - (laneWidthScreen / 2);
	if (laneX < (labelWidth+logoLeftMargin+logoRightMargin)) laneYBottom -= labelHeight+logoTopMargin;

	painter.drawLine(laneX, laneYBottom, laneX, laneYTop);
	
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

	//plotAScans();
	//printf("2dPaint\n");
}

void FOV_2DScan::drawMergedData(QPainter* p, const Detection::Vector& data, bool drawBoundingBox, bool drawTarget, bool drawLegend)
{
	int index;
	float sphericalDistanceMin = config.maxSensorsRange;
	float sphericalDistanceMax = 0;
	float sphericalDistanceAverage = 0;

	float velocityMin = 999;

	float forwardMin = config.maxSensorsRange;
	float forwardMax = -config.maxSensorsRange;
	float distanceForwardAverage = 0;
	float leftMin = config.maxSensorsRange;
	float leftMax = -config.maxSensorsRange;

	float intensityMax = 0;
	int channelForIntensityMax = 0;
	float distanceForIntensityMax = 0.0;
	AlertCondition::ThreatLevel threatLevelMax = AlertCondition::eThreatNone;

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

		if (detection->intensity > intensityMax) 
		{
			intensityMax = detection->intensity;
			channelForIntensityMax = detection->channelID;
			distanceForIntensityMax = detection->distance;
		}

		if (detection->threatLevel > threatLevelMax) threatLevelMax = detection->threatLevel;
	} // BOOST_FOREACH (detection)

	if (data.size()) 
	{
		sphericalDistanceAverage /= data.size(); 
		distanceForwardAverage /= data.size();
	}

	QString velocityLabel = " m/s";
	float distanceDisplayed = 0.0;
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
	Qt::BrushStyle backPattern;
	QColor lineColor;
	QColor textColor;

	if (colorCode == eColorCodeVelocity)
		getColorFromVelocity(velocityMin, backColor, backPattern, lineColor, textColor);
	else if (colorCode == eColorCodeDistance)
		getColorFromDistance(distanceDisplayed, backColor, backPattern, lineColor, textColor);
	else if (colorCode == eColorCodeIntensity)
		getColorFromIntensity(channelForIntensityMax, distanceForIntensityMax, intensityMax, threatLevelMax, backColor, backPattern, lineColor, textColor);
	else if (colorCode == eColorCodeChannel)
		getColorFromChannel(-1, -1, backColor, backPattern, lineColor, textColor);
	else if (colorCode == eColorCodeAlert)
		getColorFromThreatLevel(threatLevelMax, backColor, backPattern, lineColor, textColor);
	else
		getColorFromChannel(-1, -1, backColor, backPattern, lineColor, textColor);


	p->setBrush(QBrush(backColor, backPattern));
	p->setPen(lineColor);

	// Draw the legend and target, according to the flags
	if (mergeDisplayMode == eNoMergeDisplay || mergeDisplayMode == eIndividualDistanceDisplay)
	{
	}
	else // if (mergeDisplayMode == eMergeDistanceDisplay && mergeDisplayMode == eClusteredDistanceDisplay)
	{
		QString textToDisplay;

		if (measureMode != eMeasureCartesian)
			textToDisplay = "Dist: " + QString::number(distanceDisplayed, 'f', 1)+" m | Vel: "+ QString::number(velocityDisplayed, 'f', 1)+ velocityLabel;
		else
			textToDisplay = "X:" + QString::number(-(leftMin+(leftMax-leftMin/2)), 'f', 1)+" Y:"+ QString::number(distanceDisplayed, 'f', 1)+ " V:"+ QString::number(velocityDisplayed, 'f', 1)+ velocityLabel;

		// Draw the detection, but without the legend
		const Detection::Ptr detection = data.at(0);
		drawTextDetection(p, detection, textToDisplay, backColor, backPattern, lineColor, textColor, drawTarget, drawLegend);
	}


	// Now draw the bounding rectangle
	if (drawBoundingBox) 
	{
		p->setBrush(QBrush(backColor, backPattern));
		p->setPen(lineColor);

		// Remember coordinate axes X and Y are not in same orientation as QT drawing axes!
		QPoint bottomLeft(-leftMax * Ratio , -(height()-zeroY)-((forwardMin)*Ratio));
		QPoint topRight(-leftMin* Ratio, -(height()-zeroY)-((forwardMax)*Ratio));

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
	p->setPen(QPen(rgbRulerText));

	double pos = config.maxSensorsRange;

	drawArc(p, -config.maxAngularSpan/2, config.maxAngularSpan, pos + (5.0/Ratio));
	for (int i = config.maxAngularSpan; i >= 0; i-=5)
	{
		drawLine(p, i-(config.maxAngularSpan/2), pos+(5.0/Ratio), (10.0/Ratio));

		// drawText removes the spareDepthAlready. 
		drawText(p, i - (config.maxAngularSpan / 2), config.maxSensorsRange + (20.0 / Ratio), QString::number((i - (config.maxAngularSpan / 2))) + "°", rgbRulerText);
	}
}

void FOV_2DScan::drawDetection(QPainter* p, const Detection::Ptr &detection, bool drawTarget, bool drawLegend)
{
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

	if (measureMode != eMeasureCartesian)
		textToDisplay = "Dist: " + QString::number(distanceToDisplay, 'f', 1)+" m | Vel: "+ QString::number(velocityToDisplay, 'f', 1)+ velocityLabel;
	else
		textToDisplay = "X:" + QString::number(-detection->relativeToVehicleCart.left, 'f', 1)+" Y:"+ QString::number(detection->relativeToVehicleCart.forward, 'f', 1)+ " V:"+ QString::number(velocityToDisplay, 'f', 1)+ velocityLabel;

	QColor backColor;
	Qt::BrushStyle backPattern;
	QColor lineColor;
	QColor textColor;

	if (colorCode == eColorCodeVelocity)
		getColorFromVelocity(detection->velocity, backColor, backPattern, lineColor, textColor);
	else if (colorCode == eColorCodeDistance)
		getColorFromDistance(distanceToDisplay, backColor, backPattern, lineColor, textColor);
	else if (colorCode == eColorCodeIntensity)
		getColorFromIntensity(detection->channelID, detection->distance, detection->intensity, detection->threatLevel, backColor, backPattern, lineColor, textColor);
	else if (colorCode == eColorCodeChannel)
		getColorFromChannel(detection->receiverID, detection->channelID, backColor, backPattern, lineColor, textColor);
	else if (colorCode == eColorCodeAlert)
		getColorFromThreatLevel(detection->threatLevel, backColor, backPattern, lineColor, textColor);
	else
		getColorFromChannel(detection->receiverID, detection->channelID, backColor, backPattern, lineColor, textColor);

	p->setBrush(QBrush(backColor, backPattern));
	p->setPen(lineColor);

    drawTextDetection(p, detection, textToDisplay, backColor, backPattern, lineColor, textColor, drawTarget, drawLegend);
}

void FOV_2DScan::drawTextDetection(QPainter* p, const Detection::Ptr &detection, QString text, QColor backColor, Qt::BrushStyle backPattern, QColor lineColor, QColor textColor,
	                                bool drawTarget, bool drawLegend)
{
	// Our detection Y axis is positive left, so we negate the lateral coordinate.
 	// Drawing coordinate system is reversed vertically , (0,0) is top left, so we negate Y position as well;
    QPoint start(-detection->relativeToVehicleCart.left * Ratio, 
	             -(detection->relativeToVehicleCart.forward) * Ratio);
	QRect tempRect;
    QPolygon poly;

    QRect rect = p->boundingRect(QRect(0,0,0,0), Qt::AlignCenter,  text);
    rect.setSize(rect.size()+QSize(10,10));
	int windowWidth = width();
	int windowHeight = height();

    rect.moveTo(start + QPoint((width()/2)-rect.width()/2, zeroY-rect.height()));
	
	QColor lineLineColor = lineColor;
	lineLineColor.setAlpha(lineTransparency);
	QColor lineBackColor(backColor); 
	lineBackColor.setAlpha(lineTransparency);

    p->setPen(lineLineColor);
    p->setBrush(QBrush(lineBackColor, backPattern));

    tempRect = rect;

	if (lastRightTextHeight < tempRect.bottom()+3)
	{
		tempRect.moveCenter(QPoint(tempRect.center().x(), lastRightTextHeight - tempRect.height()));
	}

	tempRect.moveCenter(QPoint(width()-(tempRect.width()/2)-5, tempRect.bottom()-6));

	if (drawLegend) 
	{

		// Next polygon draws a line between the target and the distance indicator
		p->setPen(lineLineColor);
		p->setBrush(QBrush(lineBackColor, backPattern));

		poly.append(QPoint(rect.center().x()-2,rect.center().y()+12));
		poly.append(QPoint(tempRect.center().x()-51,tempRect.center().y()-1));
		poly.append(QPoint(tempRect.center().x(),tempRect.center().y()-1));
		poly.append(QPoint(tempRect.center().x(),tempRect.center().y()+1));
		poly.append(QPoint(tempRect.center().x()-49,tempRect.center().y()+1));

		poly.append(QPoint(rect.center().x()+2,rect.center().y()+14));
		p->drawPolygon(poly);

		// Draw the ellipse around the distance indication text
		p->setPen(lineColor);
		p->setBrush(QBrush(backColor));
		p->drawEllipse(QPoint(width()-(tempRect.width()/2)-5, tempRect.bottom()-tempRect.size().height()/2), tempRect.size().width()/2, tempRect.size().height()/2);


		// Write the distance text
		lastRightTextHeight = tempRect.top();
		rightQty++;


		p->setPen(textColor);
		p->drawText(tempRect, Qt::AlignCenter, text);
	}

	if (drawTarget)
	{
	    int lineWidth = 4;
		// Draw the ellipse that represents the target on the scan
 		p->setPen(lineColor);
		p->setBrush(QBrush(backColor, backPattern));

		if (detection->velocity >= zeroVelocity)
		{
			// Moving away from sensor is an uparrow
			QRect pieRect(rect.center().x() - 5, rect.bottom()-24-(lineWidth/2), 10, 24);
			int startAngle = -55 * 16;
			int spanAngle = -70 * 16;

			p->drawPie(pieRect,startAngle, spanAngle);
		}
		else if (detection->velocity < -zeroVelocity)
		{
			// Moving towards sensor is a downarrow
			QRect pieRect(rect.center().x() - 5, rect.bottom(), 10, 24);
			int startAngle = 55 * 16;
			int spanAngle = 70 * 16;

			p->drawPie(pieRect, startAngle, spanAngle);
		}

		QPen thePen(lineColor);
		thePen.setBrush(QBrush(backColor, backPattern));
		thePen.setWidth(lineWidth);
		p->setPen(thePen);
		int receiverID = detection->GetReceiverID();
		RelativePosition receiverPosition = AWLCoordinates::GetReceiverPosition(receiverID);
		ReceiverSettings receiverSettings = AWLSettings::GetGlobalSettings()->receiverSettings[receiverID];
		ChannelConfig channelConfig =receiverSettings.channelsConfig[detection->channelID];
		RelativePosition channelPosition = AWLCoordinates::GetChannelPosition(receiverID, detection->channelID);

		float startAngle = RAD2DEG(receiverPosition.orientation.yaw) + 
			               RAD2DEG(channelPosition.orientation.yaw) + (channelConfig.fovWidth/2);

		// Angles in drawPie are counter clockwise, our config is also counter clockwise. 
		// All distances are relative to bumper, subtract the sensor depth  
		// Angles are drawn from sensor position add the sensor depth
		drawArc(p, startAngle, -channelConfig.fovWidth, detection->distance,
			-receiverPosition.position.left, -receiverPosition.position.forward);
	}
}

void FOV_2DScan::drawText(QPainter* p,float angle, float pos, QString text, QColor foregroundColor, int xOffset)
{
	float angleInRad = DEG2RAD(angle+180);
	// Real position of object, from sensor on  the grid is postion + bumperOffset.
	// the spareDepth was added at the moment of capture, so we have to remove it here.

	QPoint start(0, (pos*Ratio));
    QPoint temp;
    temp = start;

    start.setX(temp.x()*cosf(angleInRad) - temp.y()*sinf(angleInRad));
    start.setY((temp.y()*cosf(angleInRad) + temp.x()*sinf(angleInRad)));
    QRect rect = p->boundingRect(QRect(0,0,0,0), Qt::AlignCenter,  text);
    rect.setSize(rect.size()+QSize(10,10));

	rect.moveTo(start + QPoint((width()/2)-rect.width()/2 + xOffset, zeroY-rect.height()));

    p->setPen(foregroundColor);
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

void FOV_2DScan::getColorFromDistance(float distance, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor)
{
    QLinearGradient myGradient;
    QGradientStops myStopPoints;
	QImage myImage(10, config.maxSensorsRange + 2, QImage::Format_RGB32);

    myStopPoints.append(QGradientStop(0.0,Qt::blue));
    myStopPoints.append(QGradientStop(0.33,Qt::green));
    myStopPoints.append(QGradientStop(0.66,Qt::yellow));
    myStopPoints.append(QGradientStop(1.0,Qt::red));
    myGradient.setStops(myStopPoints);
	myGradient.setStart(0, config.maxSensorsRange);

    QPainter painter(&myImage);
    painter.setBrush(myGradient);
	painter.drawRect(0, -2, 10, config.maxSensorsRange + 3);
    painter.end();
    backColor = QColor(myImage.pixel(QPoint(1, distance)));
	backStyle = Qt::SolidPattern;

	if (backColor.lightness() < transitionLightness) 
	{
		lineColor = backColor.darker(120);
		textColor = Qt::black;
	}
	else
	{
		lineColor = backColor.lighter(120);
		textColor = Qt::white;
	}
}

void FOV_2DScan::getColorFromVelocity(float velocity, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor)
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
    backColor =QColor(myImage.pixel(QPoint(1, maxAbsVelocity - velocity)));
	backStyle = Qt::SolidPattern;
	
	if (backColor.lightness() < transitionLightness) 
	{
		lineColor = backColor.darker(120);
		textColor = Qt::black;
	}
	else
	{
		lineColor = backColor.lighter(120);
		textColor = Qt::white;
	}
}

void FOV_2DScan::getColorFromIntensity(int channel, float distance, float intensity, AlertCondition::ThreatLevel threatLevel, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor)
{
	ClassificationType classificationType = classifyFromIntensity(channel, distance, intensity);

	if (classificationType != eClassifyMiner)  
	{
		getColorFromThreatLevel(threatLevel, backColor, backStyle, lineColor, textColor);

		backColor = rgbBackground;
		backStyle = Qt::Dense4Pattern;
		textColor = QColor(Qt::darkGray);
	}
	else 
	{
		getColorFromThreatLevel(threatLevel, backColor, backStyle, lineColor, textColor);
	}
}

void FOV_2DScan::getColorFromThreatLevel(AlertCondition::ThreatLevel threatLevel, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor)
{
#if 0
	switch (threatLevel) 
	{
	case AlertCondition::eThreatNone:
		backColor = Qt::darkBlue;
		break;
	case AlertCondition::eThreatLow:
		backColor = Qt::green;
		break;
	case AlertCondition::eThreatWarn:
		backColor = Qt::yellow;
		break;
	case AlertCondition::eThreatCritical:
		backColor = Qt::red;
		break;
	default:
		backColor = Qt::darkBlue;
		break;
	}
#else
	switch (threatLevel) 
	{
	case AlertCondition::eThreatNone:
		backColor = QColor(160, 160, 255, 255);
		break;
	case AlertCondition::eThreatLow:
		backColor = Qt::green;
		break;
	case AlertCondition::eThreatWarn:
		backColor = Qt::yellow;
		break;
	case AlertCondition::eThreatCritical:
		backColor = Qt::red;
		break;
	default:
		backColor = QColor(160, 160, 255, 255);
		break;
	}
#endif
	backStyle = Qt::SolidPattern;
	int lightness = backColor.lightness();

	if (backColor.lightness() > transitionLightness) 
	{
		lineColor = backColor.darker(120);
		textColor = Qt::black;
	}
	else
	{
		lineColor = backColor.lighter(120);
		textColor = Qt::white;
	}
}


void FOV_2DScan::getColorFromChannel(int receiverID, int channelID, QColor &backColor, Qt::BrushStyle &backStyle, QColor &lineColor, QColor &textColor)
{

	if (receiverID < 0 || channelID < 0) 
	{
		backColor = QColor(64, 64, 64);
	}
	else 
	{
		ReceiverSettings receiverSettings = AWLSettings::GetGlobalSettings()->receiverSettings[receiverID];
		ChannelConfig channelConfig =receiverSettings.channelsConfig[channelID];
		backColor = QColor(channelConfig.displayColorRed, channelConfig.displayColorGreen, channelConfig.displayColorBlue);
	}

	backStyle = Qt::SolidPattern;
	int lightness = backColor.lightness();

	if (backColor.lightness() > transitionLightness) 
	{
		lineColor = backColor.darker(120);
		textColor = Qt::black;
	}
	else
	{
		lineColor = backColor.lighter(120);
		textColor = Qt::white;
	}
}


void FOV_2DScan::drawPalette(QPainter* p)
{
  	QString text;
	if (colorCode == eColorCodeDistance)
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
		text = QString::number(config.maxSensorsRange, 'f', 1) + " m";
		QRect rect = p->boundingRect(QRect(0,0,0,0), Qt::AlignCenter,  text);
 
		p->drawText(QPoint(5, height()*0.1 + rect.height()), text);
		p->drawText(QPoint(5, height()-5), "0 m");
	}
	else if (colorCode == eColorCodeVelocity)
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
		text = QString::number(VelocityToKmH(maxAbsVelocity), 'f',  1)+ " km/h";
		QRect rect = p->boundingRect(QRect(0,0,0,0), Qt::AlignCenter,  text);
 
		p->drawText(QPoint(5, height()*0.1 + rect.height()), "0 km/h");
		p->drawText(QPoint(5, height()-5), text);
	}
	else // Intensity or Channel color Codes
	{
		// Draw palette as alert level
		AlertCondition::ThreatLevel maxThreatLevel = getMaxThreat();
		QColor backColor;
		Qt::BrushStyle backPattern;
		QColor lineColor;
		QColor textColor;
		getColorFromThreatLevel(maxThreatLevel, backColor, backPattern, lineColor, textColor);

		p->setPen(Qt::black);
		p->setBrush(QBrush(backColor, backPattern));
		p->drawRect(0, height()*0.1, paletteWidth, height()*0.9);
	}
}

bool sortDetectionsBottomRightTopLeft (Detection::Ptr &left, Detection::Ptr &right) 

{ 
	// Same X (forward)  position, sort from left to right
	if (left->relativeToVehicleCart.forward == right->relativeToVehicleCart.forward)
	{
		if (left->relativeToVehicleCart.left == right->relativeToVehicleCart.left)
		{
			if (left->channelID < right->channelID)
			{
				return(true);
			}
			else
			{
				return(false);
			}
		}
		// Remember vehicle Y is positive going left, So we reverse the < operator.
		if (left->relativeToVehicleCart.left > right->relativeToVehicleCart.left)
		{
				return(false);
		}
		else
		{
			return(true);
		}
	}
	else if (left->relativeToVehicleCart.forward < right->relativeToVehicleCart.forward)
	{
		return(true);
	}
	// Not same depth, compare forward
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
	std::sort(copyData.begin(), copyData.end(), sortDetectionsBottomRightTopLeft);

	//Merge detections according to distance criteria
	mergeDetection();
    update();
}

#if 0
void FOV_2DScan::PlotAScan(int x1, int y1, int x2, int y2)
{
	QPainter painter(this);
	if (!showAScan) return;
	painter.setPen(QPen(rgbRulerLight));
	painter.drawLine(x1, y1, x2, y2);
}

void FOV_2DScan::LabelAScan(int channel)
{
	QPainter painter(this);
	if (!showAScan) return;
	painter.setPen(QPen(rgbRulerText));
	painter.drawText(100, 50 + 50 * channel, "Ch " + QString::number(channel));
	//printf("pixel %d\n", channel);
}

void FOV_2DScan::plotAScans()
{
	//QPainter painter(this);
	//if (!showAScan) return;
	//BOOST_FOREACH(const AScan::Ptr & aScan, aScanData)
	//{
	//	aScan->Plot(50 + 50 * aScan->channelID, 100, width(), 50, this);
	//}
	//update();
}
#endif

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

AlertCondition::ThreatLevel FOV_2DScan::getMaxThreat()
{
	AlertCondition::ThreatLevel maxThreatLevel = AlertCondition::eThreatNone;

	// Draw the individual detections
	BOOST_FOREACH(const Detection::Ptr &detection, copyData)
	{
		if (detection->threatLevel > maxThreatLevel) maxThreatLevel = detection->threatLevel;
    }

	return(maxThreatLevel);
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
	QMenu* menuZoomDisplayMode = mainMenu.addMenu("Zoom");

	//menuMergeDetection->addAction(noMergeAction);
	//menuMergeDetection->addAction(radialAction);
	//menuMergeDetection->addAction(distanceAction);

	menuMergeDisplay->addAction(noMergeDisplayAction);
	menuMergeDisplay->addAction(individualDistanceDisplayAction);
	menuMergeDisplay->addAction(mergeDistanceDisplayAction);
	menuMergeDisplay->addAction(clusteredDistanceDisplayAction);

	menuMeasureMode->addAction(measureRadialAction);
	menuMeasureMode->addAction(measureLongitudinalAction);
	menuMeasureMode->addAction(measureCartesianAction);


	menuDistanceDisplayMode->addAction(displayDistanceModeHideAction);
	menuDistanceDisplayMode->addAction(displayDistanceModeShowAction);

	menuZoomDisplayMode->addAction(displayZoomModeFrontAction);
	menuZoomDisplayMode->addAction(displayZoomMode360Action);
	menuZoomDisplayMode->addAction(displayZoomModeAutoAction);

	mainMenu.addAction(showPaletteAction);

	menuColorCode->addAction(colorCodeDistanceAction);
	menuColorCode->addAction(colorCodeVelocityAction);
	menuColorCode->addAction(colorCodeIntensityAction);
	menuColorCode->addAction(colorCodeChannelAction);
	menuColorCode->addAction(colorCodeAlertAction);

	mainMenu.exec(globalPos);
}


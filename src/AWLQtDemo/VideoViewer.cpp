/* VideoViewer.cpp */
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

#include "AWLSettings.h"
#include "VideoCapture.h"
#include "awlcoord.h"
#include "VideoViewer.h"
#include "DebugPrintf.h"
#include "DetectionStruct.h"



#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"

#if 0
#include <boost/foreach.hpp>
#endif

#ifndef Q_MOC_RUN
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

using namespace std;
using namespace awl;

#ifdef _WIN32
  #include <windows.h>
#endif

#include <QPainter>
#include <QPaintDevice>
#include <QFrame>
#include <QMenu>
#include <QApplication>
#include <QDesktopWidget>
#include <QEvent>

const long flashPeriodMillisec = 300;		 // Period of the flashes used in the target display

const int maxWindowWidth = 320;
const int maxWindowHeight = 320;

// Colors enhancements on individual pixels.
const QColor rgbEnhanceBlue(0, 0, 255, 128); 
const QColor rgbEnhanceGreen(0, 255, 0, 128);
const QColor rgbEnhanceYellow(128 ,128, 0, 128);
const QColor rgbEnhanceRed(255, 0, 0, 128);
const QColor rgbOpaqueGreen(0, 255, 0, 255);
const QColor rgbEnhanceBlack(0, 0, 0, 128);

VideoViewer::VideoViewer(std::string inCameraName, VideoCapture::Ptr inVideoCapture, QWidget *parentWidget):
QFrame(parentWidget),
cameraName(inCameraName),
videoCapture(inVideoCapture),
displayScaleFactor(1.0)

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	bDisplayCrosshair = globalSettings->bDisplayVideoCrosshair;
	bDisplayTime = globalSettings->bDisplayVideoTime;

	lastValidFrame.create(videoCapture->calibration.frameWidthInPixels, videoCapture->calibration.frameHeightInPixels, CV_8UC3);
	lastValidFrame.setTo(cv::Scalar(0, 0, 0));

	qtCameraFrame = QImage(videoCapture->calibration.frameWidthInPixels, videoCapture->calibration.frameHeightInPixels, QImage::Format_RGB888);
	QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
	setSizePolicy(sizePolicy); 

	SetVideoCapture(videoCapture);
	startTime = boost::posix_time::microsec_clock::local_time();

	setContextMenuPolicy(Qt::CustomContextMenu);
	connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),this, SLOT(ShowContextMenu(const QPoint&)));
	createAction();
}

VideoViewer::~VideoViewer()
{
}


void VideoViewer::SetVideoCapture( VideoCapture::Ptr inVideoCapture)
{
	videoCapture = inVideoCapture;
	
	// Subscribe to the video capture's image feed to get information
	// on when new frames are available
 	currentVideoSubscriberID = videoCapture->Subscribe();
}

void VideoViewer::createAction()
{
	groupVideoOptions = new QActionGroup( this );
	groupVideoOptions->setExclusive(false);
	crosshairOptionAction = new QAction("Crosshairs", this);
	crosshairOptionAction->setCheckable(true);
	crosshairOptionAction->setShortcut(QKeySequence(tr("Alt+H")));
    crosshairOptionAction->setShortcutContext(Qt::ApplicationShortcut);
	addAction(crosshairOptionAction);

	timeOptionAction = new QAction("Display Time", this);
	timeOptionAction->setCheckable(true);
	timeOptionAction->setShortcut(QKeySequence(tr("Alt+T")));
	timeOptionAction->setShortcutContext(Qt::ApplicationShortcut);
	addAction(timeOptionAction);

	crosshairOptionAction->setActionGroup(groupVideoOptions);

	if (bDisplayCrosshair)
	{
		crosshairOptionAction->setChecked(true);
	}
	else
	{
		crosshairOptionAction->setChecked(false);
	}

	if (bDisplayTime)
	{
		timeOptionAction->setChecked(true);
	}
	else
	{
		timeOptionAction->setChecked(false);
	}

	timeOptionAction->setActionGroup(groupVideoOptions);

	connect(groupVideoOptions, SIGNAL(triggered(QAction*)), this, SLOT(slotVideoOptionsChangedAction()));
}

void VideoViewer::ShowContextMenu(const QPoint& pos) // this is a slot
{
    // for most widgets
    QPoint globalPos = mapToGlobal(pos);
    // for QAbstractScrollArea and derived classes you would use:
    // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos); 


	QMenu mainMenu;
	QMenu* menuVideoOptions = mainMenu.addMenu("VideoOptions");
 	menuVideoOptions->addAction(crosshairOptionAction);
	menuVideoOptions->addAction(timeOptionAction);
 
	mainMenu.exec(globalPos);
}

void VideoViewer::slotVideoOptionsChangedAction()
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	if (crosshairOptionAction->isChecked())
	{
		bDisplayCrosshair = true;
	}
	else
	{
		bDisplayCrosshair = false;
	}

	if (timeOptionAction->isChecked())
	{
		bDisplayTime = true;
	}
	else
	{
		bDisplayTime = false;
	}
}

void VideoViewer::slotImageChanged()

{
	// Update the video frame
	cv::Mat tmpFrame;

	if (videoCapture != NULL && videoCapture->HasNews(currentVideoSubscriberID))
	{

		VideoCapture::FramePtr cameraFrame(new (cv::Mat));

		// Copy the contents of the cv::Mat
		videoCapture->CopyCurrentFrame(cameraFrame, currentVideoSubscriberID);
		// Convert the image to the RGB888 format
		int frameType = cameraFrame->type();
		switch (frameType) {
		case CV_8UC1:
			cvtColor(*cameraFrame, tmpFrame, CV_GRAY2RGB);
			break;
		case CV_8UC3:
			cvtColor(*cameraFrame, tmpFrame, CV_BGR2RGB);
			break;
		default:
			cvtColor(*cameraFrame, tmpFrame, CV_BGR2RGB);
			break;

		}
		lastValidFrame = tmpFrame.clone();
	}
	else 
	{
		tmpFrame = lastValidFrame.clone();
	}

	// QImage needs the data to be stored continuously in memory
	assert(tmpFrame.isContinuous());

	// Assign OpenCV's image buffer to the QImage. Note that the bytesPerLine parameter
	// (http://qt-project.org/doc/qt-4.8/qimage.html#QImage-6) is 3*width because each pixel
	// has three bytes.
	qtCameraFrame = QImage(tmpFrame.data, tmpFrame.cols, tmpFrame.rows, tmpFrame.cols * 3, QImage::Format_RGB888);
	qtEnhancedFrame = qtCameraFrame;

	QPainter painter(&qtEnhancedFrame);
	DisplayReceiverValues(qtCameraFrame, painter, detectionData);
	if (bDisplayCrosshair)
	{
		DisplayCrossHairs(qtCameraFrame, painter);
	}

	if (bDisplayTime)
	{
		boost::posix_time::ptime myTime(boost::posix_time::microsec_clock::local_time());
		QString timeStr(boost::posix_time::to_simple_string(myTime).c_str());

		QRect textRect(0, 0, tmpFrame.cols - 1, tmpFrame.rows - 1);
		DrawVideoText(qtCameraFrame, painter, textRect, timeStr);
	}

	repaint();
}

QSize VideoViewer::sizeHint() const 
{
	return(maximumSizeHint());
}

QSize VideoViewer::minimumSizeHint() const 

{ 
	QSize size = qtCameraFrame.size();
	size.setHeight(size.height()/8);
	size.setWidth(size.width()/8);

	return size;
}

QSize VideoViewer::maximumSizeHint() const 

{ 
	QSize size = qtCameraFrame.size();

	return size;
}


void VideoViewer::paintEvent(QPaintEvent* /*event*/) 
{
	// Display the image
    QPainter painter(this);
	int newWidth = width();
	int newHeight = height();

    painter.drawImage(QPoint(0,0), qtEnhancedFrame.scaled(size(), Qt::KeepAspectRatio));
    painter.end();
}

void VideoViewer::resizeEvent(QResizeEvent * theEvent)
{
	displayScaleFactor = 1.0;
	setBaseSize(sizeHint());
	setMinimumSize(minimumSizeHint());
	setMaximumSize(maximumSizeHint());

	// Size the window to fit the maximum width specified
	long nScreenWidth  = size().width();
	long nScreenHeight = size().height();

	float scaleWidth = videoCapture->calibration.frameWidthInPixels / nScreenWidth;
	float scaleHeight = videoCapture->calibration.frameWidthInPixels / nScreenHeight;

	displayScaleFactor = min(scaleWidth, scaleHeight);
}

bool SortDetectionsInThreatLevel (Detection::Ptr &left, Detection::Ptr &right) 

{ 
	// Sort by:
	//  - Threat Level.
	//  - Distance from vehicle
	//  - Left/Right

	// Same threat level, sort from left to right
	if (left->threatLevel == right->threatLevel)
	{
		// Same distance from vehicle, sort from left to right
		if (abs(left->relativeToVehicleCart.forward - right->relativeToVehicleCart.forward) < 0.01)
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
	else if (left->threatLevel < right->threatLevel)
	{
		return(true);
	}
	else 
	{
		return(false);
	}
}

void VideoViewer::slotDetectionDataChanged(const Detection::Vector& data)
{
	// Make a copy of the provided Detection::Vector to work with
    detectionData.clear();
	detectionData = data;

	// Sort the detection DataVect in threatLevel order
	// This insures that most urgent threats are displayed last.
	std::sort(detectionData.begin(), detectionData.end(), SortDetectionsInThreatLevel);
}


void VideoViewer::DisplayReceiverValues(QImage &sourceFrame, QPainter &painter, const Detection::Vector & iDetectionData)

{
	int detectionQty = iDetectionData.size();
	for (int i = 0; i < detectionQty; i++) 
	{
		DisplayTarget(sourceFrame, painter, iDetectionData[i]);
	}
}

void VideoViewer::DisplayTarget(QImage &sourceFrame, QPainter &painter, const Detection::Ptr &detection)
{
	int top;
	int left;
	int bottom;
	int right;

	CvRect rect;
	QColor colorEnhance(Qt::black);
	bool bFlash = false;
	int thickness = -1;
	GetDetectionColors(detection, colorEnhance, thickness);

	// Paint a square that corresponds to the receiver FOV, with the given line thickness
	CvPoint topLeft;
	CvPoint topRight;
	CvPoint bottomLeft;
	CvPoint bottomRight;
	if (!GetChannelRect(detection, topLeft, topRight, bottomLeft, bottomRight))
	{
		return;
	}


	// Inset the vertical lines horizontally, to compensate for line thickness.
	//Draw the vertical lines
	CvPoint startPoint;
	CvPoint endPoint;

	startPoint.x = topLeft.x - thickness/2;
	startPoint.y = topLeft.y;
	endPoint.x = topRight.x  + thickness/2;
	endPoint.y = topRight.y;
	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);

	startPoint.x = bottomLeft.x - thickness/2;
	startPoint.y = bottomLeft.y;
	endPoint.x =bottomRight.x  + thickness/2;
	endPoint.y = bottomRight.y;
	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);

	startPoint.x = topLeft.x;
	startPoint.y = topLeft.y + thickness/2;
	endPoint.x = bottomLeft.x;
	endPoint.y = bottomLeft.y - thickness/2;
	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);

	startPoint.x = topRight.x;
	startPoint.y = topRight.y + thickness/2;
	endPoint.x = bottomRight.x;
	endPoint.y = bottomRight.y - thickness/2;
	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);
}

void VideoViewer::DisplayCrossHairs(QImage &sourceFrame, QPainter &painter)
{
	float tickIncrement = 2.5; // 1 tick every 2.5 degrees.
	
	// Draw horizontal ticks
	float longTick = 25;  // Tick length in pixels
	float mediumTick = 15;
	float shortTick = 5;

	int tickQty =  1+ ((RAD2DEG(videoCapture->calibration.fovHeight) /2) / tickIncrement); // Add 1 for the tick at position 0
	for (int tickIndex = -tickQty; tickIndex <= tickQty; tickIndex++)
	{
		// Short tick every 2.5 degree.
		// Medium tick every 5 degree
		//  Long tick every 10 degree
		float tickLength = shortTick;
		int tickWidth = 1;
		if ((tickIndex % 4) == 0) 
		{
			tickLength = longTick;
			tickWidth = 2;
		}
		else if ((tickIndex % 2) == 0) 
		{
		tickLength = mediumTick;
		}

		DrawHorizontalTicks(sourceFrame, painter, tickIndex * tickIncrement, tickLength, 1 + floor(tickWidth * displayScaleFactor));

		//CvPoint textPoint(0 /* videoCapture->calibration.frameWidthInPixels - 1*/, videoCapture->calibration.frameHeightInPixels - 1);
	}


	// Draw vertical ticks
	longTick = 25;  // Tick length in pixels (not the same as horizontal ticks)
	mediumTick = 15;
	shortTick = 8;
	tickQty = 1+ ((RAD2DEG(videoCapture->calibration.fovWidth) /2) / tickIncrement); // Add 1 for the tick at position 0
	for (int tickIndex = -tickQty; tickIndex <= tickQty; tickIndex++)
	{
		// Short tick every 2.5 degree.
		// Medium tick every 5 degree
		//  Long tick every 10 degree
		float tickLength = shortTick;
		int tickWidth = 1;
		if ((tickIndex % 4) == 0) 
		{
			tickLength = longTick;
			tickWidth = 2;
		}
		else if ((tickIndex % 2) == 0) 
		{
		tickLength = mediumTick;
		}

		DrawVerticalTicks(sourceFrame, painter, tickIndex * tickIncrement, tickLength, 1 + floor(tickWidth * displayScaleFactor));
	}
}

void VideoViewer::DrawHorizontalTicks(QImage &sourceFrame, QPainter &painter, float tickAngle, float tickLength, int thickness)

{
	QColor colorEnhance = rgbEnhanceGreen; // Yellow

	CvPoint startPoint;
	CvPoint endPoint;

	// Cartesian coord at the outer edge of screen
	CartesianCoord lineCart(SphericalCoord(10, M_PI_2 - DEG2RAD(tickAngle), (videoCapture->calibration.fovWidth/2)));
	videoCapture->calibration.ToFrameXY(lineCart, startPoint.x, startPoint.y);
	startPoint.x = videoCapture->calibration.frameWidthInPixels-1;
	endPoint.y = startPoint.y;
	endPoint.x = startPoint.x - tickLength;

	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);

	startPoint.x = 0;
	endPoint.x = startPoint.x + tickLength;
	DrawDetectionLine(sourceFrame, painter,  startPoint, endPoint, colorEnhance, thickness);
	

	// Retake Cartesian coord at the center edge of screen: Wide FOV screens may have barrel effect
	lineCart = SphericalCoord(10, M_PI_2 - DEG2RAD(tickAngle), 0);
	videoCapture->calibration.ToFrameXY(lineCart, startPoint.x, startPoint.y);	
	startPoint.x = (videoCapture->calibration.frameWidthInPixels- tickLength) /2;
	endPoint.y = startPoint.y;
	endPoint.x = startPoint.x + tickLength;
	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);
}

void VideoViewer::DrawVerticalTicks(QImage &sourceFrame, QPainter &painter,  float tickAngle, float tickLength, int thickness)

{
	QColor colorEnhance = rgbEnhanceGreen; // Yellow

	CvPoint startPoint;
	CvPoint endPoint;

	// Cartesian coord at the outer edge of the screen
	CartesianCoord lineCart(SphericalCoord(10, M_PI_2-(videoCapture->calibration.fovHeight/2), DEG2RAD(-tickAngle)));
	videoCapture->calibration.ToFrameXY(lineCart, startPoint.x, startPoint.y);
	startPoint.y = videoCapture->calibration.frameHeightInPixels-1;
	endPoint.y = startPoint.y - tickLength;
	endPoint.x = startPoint.x;

	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);

	startPoint.y = 0;
	endPoint.y = startPoint.y + tickLength;
	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);

	// Retake Cartesian coord at the center edge of screen: Wide FOV screens may have barrel effect
	lineCart = SphericalCoord(10, M_PI_2, DEG2RAD(-tickAngle));
	videoCapture->calibration.ToFrameXY(lineCart, startPoint.x, startPoint.y);	
	startPoint.y = (videoCapture->calibration.frameHeightInPixels - tickLength) / 2;
	endPoint.y = startPoint.y + tickLength;
	endPoint.x = startPoint.x;
	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);
}

void VideoViewer::GetDetectionColors(const Detection::Ptr &detection, QColor &colorEnhance, int &iThickness)

{
	bool bFlash = false;

	long millisecs = (boost::posix_time::microsec_clock::local_time() - startTime).total_milliseconds();
	if ((millisecs/(flashPeriodMillisec/2)) & 0x01)  
	{
		bFlash = true;
	}

	int channelID = detection->channelID;
	AlertCondition::ThreatLevel threatLevel = detection->threatLevel;

	switch (threatLevel) 
	{
	case AlertCondition::eThreatNone:
		{
			colorEnhance = rgbEnhanceBlue;  // Blue
			iThickness = 5;
		}
		break;
	case AlertCondition::eThreatLow:
		{
			colorEnhance = rgbEnhanceGreen; // Green
			iThickness = 5;
		}
		break;

	case AlertCondition::eThreatWarn:
		{
			colorEnhance = rgbEnhanceYellow; // Yellow
			iThickness = 15;
			if (bFlash) iThickness = 5;	
		}
		break;

	case AlertCondition::eThreatCritical:
		{
			colorEnhance = rgbEnhanceRed;  // Red
			iThickness = 15;
			if (bFlash) iThickness = 5;
		}
		break;

	default:
		{
			colorEnhance = Qt::black;
		}

	} // case
}

bool VideoViewer::GetChannelRect(const Detection::Ptr &detection, CvPoint &topLeft, CvPoint &topRight, CvPoint &bottomLeft, CvPoint &bottomRight)
{	
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	AWLCoordinates *globalCoordinates = AWLCoordinates::GetGlobalCoordinates();
	bool bSomePointsInFront = false;

	float x, y;
	int receiverID = detection->receiverID;
	int channelID = detection->channelID;
	int cameraID = videoCapture->GetCameraID();

	// Channel description pointer
	ChannelConfig *channel = &globalSettings->receiverSettings[receiverID].channelsConfig[channelID];

	// Position of the topLeft corner of the channel FOV 
	SphericalCoord topLeftInChannel(detection->distance, M_PI_2 - DEG2RAD(channel->fovHeight/2), +DEG2RAD(channel->fovWidth/2));  // Spherical coordinate, relative to sensor
	bSomePointsInFront |= AWLCoordinates::SensorToCameraXY(receiverID, channelID, cameraID, videoCapture->calibration, topLeftInChannel, topLeft.x, topLeft.y);

	// Position of the topRight corner of the channel FOV 
	SphericalCoord topRightInChannel(detection->distance, M_PI_2 - DEG2RAD(channel->fovHeight/2), -DEG2RAD(channel->fovWidth/2)); 
    bSomePointsInFront |= AWLCoordinates::SensorToCameraXY(receiverID, channelID, cameraID, videoCapture->calibration, topRightInChannel, topRight.x, topRight.y);

	// Position of the bottomLeft corner of the channel FOV 
	SphericalCoord bottomLeftInChannel(detection->distance, M_PI_2 + DEG2RAD(channel->fovHeight/2), + DEG2RAD(channel->fovWidth/2));
	bSomePointsInFront |= AWLCoordinates::SensorToCameraXY(receiverID, channelID, cameraID, videoCapture->calibration, bottomLeftInChannel, bottomLeft.x, bottomLeft.y);

	// Position of the topRight corner of the channel FOV 
	SphericalCoord bottomRightInChannel(detection->distance, M_PI_2 + DEG2RAD(channel->fovHeight/2), -DEG2RAD(channel->fovWidth/2));
	bSomePointsInFront |= AWLCoordinates::SensorToCameraXY(receiverID, channelID, cameraID, videoCapture->calibration, bottomRightInChannel, bottomRight.x, bottomRight.y);

	if (bSomePointsInFront)
	{
		return(true);
	}
	else
	{
		return(false);
	}
}

void VideoViewer::DrawDetectionLine(QImage &sourceFrame, QPainter &painter, const CvPoint &startPoint, const CvPoint &endPoint,  QColor &colorEnhance, int penWidth)
{
	// To avoid overlapping colors for alarms, only the last line is displayed.
	// To make dure there is no other line displayed below, redraw the source frame behind the line
	QBrush backBrush;
	backBrush.setTextureImage(sourceFrame);
	painter.setBrush(backBrush);

	QPen pen(colorEnhance);
	pen.setWidth(penWidth);
	pen.setBrush(backBrush);
	pen.setCapStyle(Qt::FlatCap);
	painter.setPen(pen);

	painter.drawLine(QPoint(startPoint.x, startPoint.y), QPoint(endPoint.x, endPoint.y));

	// When lines behind are erased, , we can draw the line.
	QBrush frontBrush(colorEnhance);
	painter.setBrush(Qt::NoBrush);

	pen.setWidth(penWidth);
	pen.setBrush(QBrush(colorEnhance));
	pen.setCapStyle(Qt::FlatCap);
	painter.setPen(pen);

	painter.drawLine(QPoint(startPoint.x, startPoint.y), QPoint(endPoint.x, endPoint.y));
}

void VideoViewer::DrawVideoText(QImage &sourceFrame, QPainter &painter, const QRect &textRect, const QString &text)

{
	QFont font("Arial", 18);
	font.setBold(false);
	painter.setFont(font);
	QFontMetrics fm(font);
	int pixelsWide = fm.width(text);
	pixelsWide -= (pixelsWide % 30);
	pixelsWide += 30;

	// Draw black bnorder in the background
	QPainterPath backgroundPath;
	QColor colorBackground = rgbEnhanceBlack;
	QBrush backgroundBrush(colorBackground);
	QPen backgroundPen(backgroundBrush, 10);
	backgroundPen.setWidth(8);
	painter.setPen(backgroundPen);
	backgroundPath.addText(textRect.right() - pixelsWide -8, textRect.bottom() - 4, font, text); //Adjust the position

	painter.drawPath(backgroundPath);

	// Draw white text over the border

	QColor colorForeground = Qt::white;
	QBrush foregroundBrush(colorForeground);
	QPen foregroundPen(colorForeground);
	foregroundPen.setWidth(1);

	painter.setPen(foregroundPen);

	painter.fillPath(backgroundPath, foregroundBrush);
}




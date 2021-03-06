/* VideoViewer.cpp : Overlay LiDAR detection info on top of camera image*/
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

#include "AWLSettings.h"
#include "VideoCapture.h"
#include "SensorCoord.h"
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

using namespace awl;
SENSORCORE_USE_NAMESPACE

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
const QColor rgbEnhanceCyan(0, 255, 255, 128); 
const QColor rgbEnhancePurple(255, 0, 255, 128); 
const QColor rgbEnhanceBlack(0, 0, 0, 128);
const QColor rgbOpaqueGreen(0, 255, 0, 255);


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
	if (bDisplayCrosshair)
	{
		DisplayCrossHairs(qtCameraFrame, painter);
	}


	DisplayReceiverValues(qtCameraFrame, painter, detectionData);

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

	// Video should never occupy more than half of the screen;

	float maxWidth = window()->width() / 3;  // Must be float to force float division later in code
	if (size.width() > maxWidth)
	{
		float sizeRatio = maxWidth  / size.width();
		size.setHeight(size.height() * sizeRatio);
		size.setWidth(size.width() * sizeRatio);
	}

	return size;
}


void VideoViewer::paintEvent(QPaintEvent* /*event*/) 
{
	// Display the image
    QPainter painter(this);

    painter.drawImage(QPoint(0,0), qtEnhancedFrame.scaled(size(), Qt::KeepAspectRatio));
    painter.end();
}

void VideoViewer::resizeEvent(QResizeEvent * /*theEvent*/)
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
		if (abs(left->relativeToVehicleCart.bodyRelative.forward - right->relativeToVehicleCart.bodyRelative.forward) < 0.01)
		{
			// Remember vehicle Y is positive going left, So we reverse the < operator.
			if (left->relativeToVehicleCart.bodyRelative.left > right->relativeToVehicleCart.bodyRelative.left)
			{
				return(true);
			}
			else
			{
				return(false);
			}
		}
		// Not same depth, compare forward
		else if (left->relativeToVehicleCart.bodyRelative.forward < right->relativeToVehicleCart.bodyRelative.forward)
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

void VideoViewer::slotDetectionDataChanged(const Detection::Vector& inData)
{
	// Make a copy of the provided Detection::Vector to work with
    detectionData.clear();
	detectionData = inData;

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
	CvRect rect;
	QColor colorEnhance(Qt::black);
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
	// Display Camera squares
	DrawChannelOutlines(sourceFrame, painter);

	// Display ticks

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
	CartesianCoord lineCart(SphericalCoord(10, M_SENSORCORE_2 - DEG2RAD(tickAngle), (videoCapture->calibration.fovWidth/2)));
	videoCapture->calibration.ToFrameXY(lineCart, startPoint.x, startPoint.y);
	startPoint.x = videoCapture->calibration.frameWidthInPixels-1;
	endPoint.y = startPoint.y;
	endPoint.x = startPoint.x - tickLength;

	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);

	startPoint.x = 0;
	endPoint.x = startPoint.x + tickLength;
	DrawDetectionLine(sourceFrame, painter,  startPoint, endPoint, colorEnhance, thickness);
	

	// Retake Cartesian coord at the center edge of screen: Wide FOV screens may have barrel effect
	lineCart = SphericalCoord(10, M_SENSORCORE_2 - DEG2RAD(tickAngle), 0);
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
	CartesianCoord lineCart(SphericalCoord(10, M_SENSORCORE_2-(videoCapture->calibration.fovHeight/2), DEG2RAD(-tickAngle)));
	videoCapture->calibration.ToFrameXY(lineCart, startPoint.x, startPoint.y);
	startPoint.y = videoCapture->calibration.frameHeightInPixels-1;
	endPoint.y = startPoint.y - tickLength;
	endPoint.x = startPoint.x;

	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);

	startPoint.y = 0;
	endPoint.y = startPoint.y + tickLength;
	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);

	// Retake Cartesian coord at the center edge of screen: Wide FOV screens may have barrel effect
	lineCart = SphericalCoord(10., (float) M_SENSORCORE_2, (float) DEG2RAD(-tickAngle));
	videoCapture->calibration.ToFrameXY(lineCart, startPoint.x, startPoint.y);	
	startPoint.y = (videoCapture->calibration.frameHeightInPixels - tickLength) / 2;
	endPoint.y = startPoint.y + tickLength;
	endPoint.x = startPoint.x;
	DrawDetectionLine(sourceFrame, painter, startPoint, endPoint, colorEnhance, thickness);
}


void VideoViewer::DrawChannelOutlines(QImage &sourceFrame, QPainter &painter)

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	int receiverQty = globalSettings->receiverSettings.size();
	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		int voxelQty = globalSettings->receiverSettings[receiverID].voxelsConfig.size();
		int columnQty = globalSettings->receiverSettings[receiverID].receiverColumns;

		for (int voxelIndex = 0; voxelIndex < voxelQty; voxelIndex++)
		{
			CellID cellID(voxelIndex % columnQty, voxelIndex / columnQty);

			Detection::Ptr detection = Detection::Ptr(new Detection(receiverID, cellID, 0));
			detection->distance = 10.0;
			detection->distance = 10.0;

			detection->intensity = 99.0;
			detection->velocity = 99.0;
			detection->acceleration = 0.0;
			detection->probability = 0.0;

			detection->timeStamp = 0;
			detection->firstTimeStamp = 0;  // TBD
			detection->trackID = 0;
			detection->timeToCollision = 0;;
			detection->decelerationToStop = 0;;
			detection->threatLevel = AlertCondition::eThreatOutlineOnly;
			DisplayTarget(sourceFrame, painter, detection);
		}
	}
}

void VideoViewer::GetDetectionColors(const Detection::Ptr &detection, QColor &colorEnhance, int &iThickness)

{
	bool bFlash = false;

	long millisecs = (boost::posix_time::microsec_clock::local_time() - startTime).total_milliseconds();
	if ((millisecs/(flashPeriodMillisec/2)) & 0x01)  
	{
		bFlash = true;
	}

	AlertCondition::ThreatLevel threatLevel = detection->threatLevel;

	switch (threatLevel) 
	{

	case AlertCondition::eThreatOutlineOnly:
	{
		colorEnhance = rgbEnhancePurple; // Try any contrasting color!
		iThickness = 2;
	}
	break;

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
	bool bSomePointsInFront = false;

	int receiverID = detection->receiverID;
	int cameraID = videoCapture->GetCameraID();

	// Channel description pointer
	int columns = globalSettings->receiverSettings[receiverID].receiverColumns;
	int voxelIndex = (detection->cellID.row * columns) + detection->cellID.column;
	VoxelConfig *voxel= &globalSettings->receiverSettings[receiverID].voxelsConfig[voxelIndex];


	// Position of the topLeft corner of the voxel FOV 
	SphericalCoord topLeftInvoxel(detection->distance, M_SENSORCORE_2 - DEG2RAD(voxel->fovHeight/2), +DEG2RAD(voxel->fovWidth/2));  // Spherical coordinate, relative to sensor
	bSomePointsInFront |= SensorCoordinates::SensorToCameraXY(receiverID, voxelIndex, cameraID, videoCapture->calibration, topLeftInvoxel, topLeft.x, topLeft.y);

	// Position of the topRight corner of the voxel FOV 
	SphericalCoord topRightInvoxel(detection->distance, M_SENSORCORE_2 - DEG2RAD(voxel->fovHeight/2), -DEG2RAD(voxel->fovWidth/2)); 
    bSomePointsInFront |= SensorCoordinates::SensorToCameraXY(receiverID, voxelIndex, cameraID, videoCapture->calibration, topRightInvoxel, topRight.x, topRight.y);

	// Position of the bottomLeft corner of the voxel FOV 
	SphericalCoord bottomLeftInvoxel(detection->distance, M_SENSORCORE_2 + DEG2RAD(voxel->fovHeight/2), + DEG2RAD(voxel->fovWidth/2));
	bSomePointsInFront |= SensorCoordinates::SensorToCameraXY(receiverID, voxelIndex, cameraID, videoCapture->calibration, bottomLeftInvoxel, bottomLeft.x, bottomLeft.y);

	// Position of the topRight corner of the voxel FOV 
	SphericalCoord bottomRightInvoxel(detection->distance, M_SENSORCORE_2 + DEG2RAD(voxel->fovHeight/2), -DEG2RAD(voxel->fovWidth/2));
	bSomePointsInFront |= SensorCoordinates::SensorToCameraXY(receiverID, voxelIndex, cameraID, videoCapture->calibration, bottomRightInvoxel, bottomRight.x, bottomRight.y);

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

void VideoViewer::DrawVideoText(QImage & /*sourceFrame*/, QPainter &painter, const QRect &textRect, const QString &text)

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




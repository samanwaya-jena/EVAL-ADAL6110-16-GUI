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
#include "AWLCoord.h"
#include "VideoViewer.h"
#include "DebugPrintf.h"
#include "DetectionStruct.h"

#include <boost/foreach.hpp>

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace awl;

#include <windows.h>

const long flashPeriodMillisec = 300;		 // Period of the flashes used in the target display

const int  workFrameQty = 3;  // Number of buffers. That may depend on the display lag of the systems.

const int maxWindowWidth = 900;
const int maxWindowHeight = 900;

// Colors enhancements on individual pixels.
const cv::Vec3b colorEnhanceBlue = cv::Vec3b(128, 0, 0);  // Blue
const cv::Vec3b colorDehanceBlue = cv::Vec3b(0, 128, 128);
const cv::Vec3b colorEnhanceGreen = cv::Vec3b(0, 190, 0); // Green
const cv::Vec3b colorDehanceGreen = cv::Vec3b(190, 0,190);
const cv::Vec3b colorEnhanceYellow = cv::Vec3b(0, 160, 160); // Yellow
const cv::Vec3b colorDehanceYellow = cv::Vec3b(192, 0, 0);
const cv::Vec3b colorEnhanceRed = cv::Vec3b(0, 0, 128);  // Red
const cv::Vec3b colorDehanceRed = cv::Vec3b(128, 128, 0);


VideoViewer::VideoViewer(std::string inCameraName, VideoCapture::Ptr inVideoCapture):
LoopedWorker(),
cameraFrame(new (cv::Mat)),
currentWorkFrameIndex(0),
cameraName(inCameraName),
videoCapture(inVideoCapture)

{
	SetVideoCapture(videoCapture);

	for (int frameID = 0; frameID < workFrameQty; frameID++)
	{
		workFrames.push_back(new (cv::Mat));
	}
}

VideoViewer::~VideoViewer()
{
	Stop();
}


void VideoViewer::SetVideoCapture( VideoCapture::Ptr inVideoCapture)
{
	videoCapture = inVideoCapture;
	
	// Subscribe to the video capture's image feed to get information
	// on when new frames are available
 	currentVideoSubscriberID = videoCapture->Subscribe();
}

void  VideoViewer::Go() 
{
	if (WasStopped())
	{
		void *windowPtr = cvGetWindowHandle(cameraName.c_str());
		// Create output window, only if it does not already exist
		if (windowPtr == NULL)
		{
//			cv::namedWindow(cameraName.c_str(), cv::WINDOW_NORMAL);
//			cvNamedWindow(cameraName.c_str(), CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);//
//			cv::namedWindow(cameraName.c_str(),  CV_NORMAL);
			cv::namedWindow(cameraName.c_str(), cv::WINDOW_AUTOSIZE | CV_GUI_NORMAL);
			cv::setWindowProperty(cameraName.c_str(), cv::WND_PROP_ASPECT_RATIO, 1);
			SizeWindow();
		}

		startTime = boost::posix_time::microsec_clock::local_time();
		LoopedWorker::Go();
	}
}
	
void  VideoViewer::Stop() 
{
	if (!mWorkerRunning) return;

	void *windowPtr = cvGetWindowHandle(cameraName.c_str());
	if (windowPtr != NULL) cv::destroyWindow(cameraName.c_str());

	LoopedWorker::Stop();
}

bool  VideoViewer::WasStopped()
{
	if (LoopedWorker::WasStopped()) return (true);
	void *windowPtr = cvGetWindowHandle(cameraName.c_str());
	if (windowPtr == NULL) return(true);

	return(false);
}


void VideoViewer::SpinOnce()

{
	if (WasStopped()) return;

	// Update the video frame
	if (videoCapture != NULL && videoCapture->HasNews(currentVideoSubscriberID)) 
	{
		// Copy the contents of the cv::Mat
		videoCapture->CopyCurrentFrame(cameraFrame, currentVideoSubscriberID);

		// Copy to the working area
		cameraFrame->copyTo(*workFrames[currentWorkFrameIndex]);

		DisplayReceiverValues(cameraFrame, workFrames[currentWorkFrameIndex], detectionData);
		if (AWLSettings::GetGlobalSettings()->bDisplayVideoCrosshair)
		{
			DisplayCrossHairs(cameraFrame, workFrames[currentWorkFrameIndex]);
		}

		//  Get the window handle. IOf it is null, may be that the window was closed or destroyed.
		//  In that case, do NOT reopen it.  The thread may be terminating.
		void *windowPtr = cvGetWindowHandle(cameraName.c_str());
		// The current work frame becomes the display frame
		if (windowPtr != NULL) cv::imshow(cameraName, *workFrames[currentWorkFrameIndex]);

		// And we select a new work frame for later.
		currentWorkFrameIndex = ++currentWorkFrameIndex % workFrameQty;
	}
}


void VideoViewer::SizeWindow()

{
		// Size the window to fit the maximum width specified
		const long nScreenWidth  = maxWindowWidth;
		const long nScreenHeight = maxWindowHeight;

		int height = videoCapture->calibration.frameHeightInPixels * 2;
		int width  = videoCapture->calibration.frameWidthInPixels * 2;
		while (width > nScreenWidth || height > nScreenHeight)
		{
			width /= 2;
			height /=2;
		}

		cv::resizeWindow(cameraName.c_str(), nScreenWidth, nScreenHeight);
}


void VideoViewer::move(int left, int top)
{
	cv::moveWindow(cameraName.c_str(), left, top);
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


void VideoViewer::DisplayReceiverValues(VideoCapture::FramePtr &sourceFrame, VideoCapture::FramePtr &targetFrame, const Detection::Vector & iDetectionData)

{
	// Draw the individual detections
	BOOST_FOREACH(const Detection::Ptr &detection, iDetectionData)
	{
			DisplayTarget(sourceFrame, targetFrame, detection);
	}
}

void VideoViewer::DisplayTarget(VideoCapture::FramePtr &sourceFrame, VideoCapture::FramePtr &targetFrame, const Detection::Ptr &detection)
{
	int top;
	int left;
	int bottom;
	int right;

	CvRect rect;

	cv::Vec3b color;
	cv::Vec3b colorEnhance;
	cv::Vec3b colorDehance;
	bool bFlash = false;

	int thickness = -1;
	colorEnhance = cv::Vec3b(0, 0, 0);
	colorDehance = cv::Vec3b(0, 0, 0);
	GetDetectionColors(detection, colorEnhance, colorDehance, thickness);

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
	DrawDetectionLine(sourceFrame, targetFrame, startPoint, endPoint, colorEnhance, colorDehance, 1, thickness);

	startPoint.x = bottomLeft.x - thickness/2;
	startPoint.y = bottomLeft.y;
	endPoint.x =bottomRight.x  + thickness/2;
	endPoint.y = bottomRight.y;
	DrawDetectionLine(sourceFrame, targetFrame, startPoint, endPoint, colorEnhance, colorDehance, 1, thickness);

	startPoint.x = topLeft.x;
	startPoint.y = topLeft.y + thickness/2;
	endPoint.x = bottomLeft.x;
	endPoint.y = bottomLeft.y - thickness/2;
	DrawDetectionLine(sourceFrame, targetFrame, startPoint, endPoint, colorEnhance, colorDehance, thickness, 1);

	startPoint.x = topRight.x;
	startPoint.y = topRight.y + thickness/2;
	endPoint.x = bottomRight.x;
	endPoint.y = bottomRight.y - thickness/2;
	DrawDetectionLine(sourceFrame, targetFrame, startPoint, endPoint, colorEnhance, colorDehance, thickness, 1);

}

void VideoViewer::DisplayCrossHairs(VideoCapture::FramePtr &sourceFrame, VideoCapture::FramePtr &targetFrame)
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

		DrawHorizontalTicks(sourceFrame, targetFrame, tickIndex * tickIncrement, tickLength, tickWidth);
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

		DrawVerticalTicks(sourceFrame, targetFrame, tickIndex * tickIncrement, tickLength, tickWidth);
	}
}

void VideoViewer::DrawHorizontalTicks(VideoCapture::FramePtr &sourceFrame, VideoCapture::FramePtr &targetFrame, float tickAngle, float tickLength, int thickness)

{
	cv::Vec3b colorEnhance = colorEnhanceGreen; // Yellow
	cv::Vec3b colorDehance = colorDehanceGreen;

	CvPoint startPoint;
	CvPoint endPoint;

	// Cartesian coord at the outer edge of screen
	CartesianCoord lineCart(SphericalCoord(10, M_PI_2 - DEG2RAD(tickAngle), (videoCapture->calibration.fovWidth/2)));
	videoCapture->calibration.ToFrameXY(lineCart, startPoint.x, startPoint.y);
	startPoint.x = videoCapture->calibration.frameWidthInPixels-1;
	endPoint.y = startPoint.y;
	endPoint.x = startPoint.x - tickLength;

	DrawDetectionLine(sourceFrame, targetFrame, startPoint, endPoint, colorEnhance, colorDehance, 1, thickness);

	startPoint.x = 0;
	endPoint.x = startPoint.x + tickLength;
	DrawDetectionLine(sourceFrame, targetFrame, startPoint, endPoint, colorEnhance, colorDehance, 1, thickness);
	

	// Retake Cartesian coord at the center edge of screen: Wide FOV screens may have barrel effect
	lineCart = SphericalCoord(10, M_PI_2 - DEG2RAD(tickAngle), 0);
	videoCapture->calibration.ToFrameXY(lineCart, startPoint.x, startPoint.y);	
	startPoint.x = (videoCapture->calibration.frameWidthInPixels- tickLength) /2;
	endPoint.y = startPoint.y;
	endPoint.x = startPoint.x + tickLength;
	DrawDetectionLine(sourceFrame, targetFrame, startPoint, endPoint, colorEnhance, colorDehance, 1, thickness);
}

void VideoViewer::DrawVerticalTicks(VideoCapture::FramePtr &sourceFrame, VideoCapture::FramePtr &targetFrame, float tickAngle, float tickLength, int thickness)

{
	cv::Vec3b colorEnhance = colorEnhanceGreen; // Yellow
	cv::Vec3b colorDehance = colorDehanceGreen;

	CvPoint startPoint;
	CvPoint endPoint;

	// Cartesian coord at the outer edge of the screen
	CartesianCoord lineCart(SphericalCoord(10, M_PI_2-(videoCapture->calibration.fovHeight/2), DEG2RAD(-tickAngle)));
	videoCapture->calibration.ToFrameXY(lineCart, startPoint.x, startPoint.y);
	startPoint.y = videoCapture->calibration.frameHeightInPixels-1;
	endPoint.y = startPoint.y - tickLength;
	endPoint.x = startPoint.x;

	DrawDetectionLine(sourceFrame, targetFrame, startPoint, endPoint, colorEnhance, colorDehance, thickness, 1);

	startPoint.y = 0;
	endPoint.y = startPoint.y + tickLength;
	DrawDetectionLine(sourceFrame, targetFrame, startPoint, endPoint, colorEnhance, colorDehance, thickness, 1);

	// Retake Cartesian coord at the center edge of screen: Wide FOV screens may have barrel effect
	lineCart = SphericalCoord(10, M_PI_2, DEG2RAD(-tickAngle));
	videoCapture->calibration.ToFrameXY(lineCart, startPoint.x, startPoint.y);	
	startPoint.y = (videoCapture->calibration.frameHeightInPixels - tickLength) / 2;
	endPoint.y = startPoint.y + tickLength;
	endPoint.x = startPoint.x;
	DrawDetectionLine(sourceFrame, targetFrame, startPoint, endPoint, colorEnhance, colorDehance, thickness, 1);
}

void VideoViewer::GetDetectionColors(const Detection::Ptr &detection, cv::Vec3b &colorEnhance, cv::Vec3b &colorDehance, int &iThickness)

{
	bool bFlash = false;

	long millisecs = (boost::posix_time::microsec_clock::local_time() - startTime).total_milliseconds();
	if ((millisecs/(flashPeriodMillisec/2)) & 0x01)  
	{
		bFlash = true;
	}

	int channelID = detection->channelID;
	Detection::ThreatLevel threatLevel = detection->threatLevel;

	switch (threatLevel) 
	{
	case Detection::eThreatNone: 
		{
			colorEnhance = colorEnhanceBlue;  // Blue
			colorDehance = colorDehanceBlue;
			iThickness = 5;
		}
		break;
	case Detection::eThreatLow:
		{
			colorEnhance = colorEnhanceGreen; // Green
			colorDehance = colorDehanceGreen;
			iThickness = 5;
		}
		break;

	case Detection::eThreatWarn:
		{
			colorEnhance = colorEnhanceYellow; // Yellow
			colorDehance = colorDehanceYellow;
			iThickness = 15;
			if (bFlash) iThickness = 5;	
		}
		break;

	case Detection::eThreatCritical:
		{
			colorEnhance = colorEnhanceRed;  // Red
			colorDehance = colorDehanceRed;
			iThickness = 15;
			if (bFlash) iThickness = 5;
		}
		break;

	default:
		{
			colorEnhance = cv::Vec3b(0, 0, 0);
			colorDehance = cv::Vec3b(255, 255, 255);
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

void VideoViewer::DrawDetectionLine(VideoCapture::FramePtr &sourceFrame, VideoCapture::FramePtr &targetFrame, const CvPoint &startPoint, const CvPoint &endPoint,  const cv::Vec3b &colorEnhance, const cv::Vec3b &colorDehance, int iWidth, int iHeight)
{
	cv::LineIterator lineIter(*targetFrame, startPoint, endPoint, 8);

	float frameWidth = videoCapture->calibration.frameWidthInPixels;
	float frameHeight = videoCapture->calibration.frameHeightInPixels;

	// alternative way of iterating through the line
	for(int i = 0; i < lineIter.count; i++, ++lineIter)
	{
		cv::Point centerPos = lineIter.pos();
		cv::Point pixelPos = centerPos;
		pixelPos.x = centerPos.x - (iWidth /2);
		for (int width = 0; width < iWidth; width++, pixelPos.x++)
		{
			pixelPos.y = centerPos.y + (iHeight / 2); 
			for (int height = 0; height < iHeight; height++, pixelPos.y--)
			{
				if (pixelPos.x >= 0 && pixelPos.x < frameWidth && pixelPos.y >= 0 && pixelPos.y < frameHeight)
				{
					cv::Vec3b	color = sourceFrame->at<cv::Vec3b>(pixelPos);

					int r = (int)color[0];
					int g = (int)color[1];
					int b = (int)color[2];
					float lightness = ((r+g+b) / (3*255.0)); // Value from 0 (Dark) to 1.0 (Light)
					r -= (colorDehance[0]) * lightness;
					g -= (colorDehance[1]) * lightness;
					b -= (colorDehance[2]) * lightness;
					r += (colorEnhance[0]) * (1.0-lightness);
					g += (colorEnhance[1]) * (1.0-lightness);
					b += (colorEnhance[2]) * (1.0-lightness);
	
					if (r > 255) color[0] = 255;
					else if (r < 0) color[0] = 0;
					else		 color[0] = r;

					if (g > 255) color[1] = 255;
					else if (g < 0) color[1] = 0;
					else		 color[1] = g;

					if (b > 255) color[2] = 255;
					else if (b < 0) color[2] = 0;
					else		 color[2] = b;

					targetFrame->at<cv::Vec3b>(pixelPos) = color;
				}
			}
		}
	}
}


void VideoViewer::DrawContrastingLine(VideoCapture::FramePtr &sourceFrame, VideoCapture::FramePtr &targetFrame, const CvPoint &startPoint, const CvPoint &endPoint,  int iWidth, int iHeight)
{
	cv::LineIterator lineIter(*targetFrame, startPoint, endPoint, 8);

	float frameWidth = videoCapture->calibration.frameWidthInPixels;
	float frameHeight = videoCapture->calibration.frameHeightInPixels;

	// alternative way of iterating through the line
	for(int i = 0; i < lineIter.count; i++, ++lineIter)
	{
		cv::Point centerPos = lineIter.pos();
		cv::Point pixelPos = centerPos;
		pixelPos.x = centerPos.x - (iWidth /2);
		for (int width = 0; width < iWidth; width++, pixelPos.x++)
		{
			pixelPos.y = centerPos.y + (iHeight / 2); 
			for (int height = 0; height < iHeight; height++, pixelPos.y--)
			{
				if (pixelPos.x >= 0 && pixelPos.x < frameWidth && pixelPos.y >= 0 && pixelPos.y < frameHeight)
				{
					cv::Vec3b	color = sourceFrame->at<cv::Vec3b>(pixelPos);

					int r = (int)color[0];
					int g = (int)color[1];
					int b = (int)color[2];

					if (r + g + b > (3*128)) 
					{
						r = g = b  = 0;
					}
					else 
					{
						r = g = b = 255;
					}

					color[0] = r;
					color[1] = g;
					color[2] = b;

					targetFrame->at<cv::Vec3b>(pixelPos) = color;
				}
			}
		}
	}
}

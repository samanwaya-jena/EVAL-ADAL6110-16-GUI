
#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

#include "AWLSettings.h"
#include "VideoCapture.h"
#include "AWLCoord.h"
#include "VideoViewer.h"
#include "DebugPrintf.h"
#include "Tracker.h"

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


const long flashPeriodMillisec = 300;		 // Period of the flashes used in the target display

const int  workFrameQty = 3;  // Number of buffers. That may depend on the display lag of the systems.

const int maxWindowWidth = 640;
const int maxWindowHeight = 480;

VideoViewer::VideoViewer(std::string inCameraName, VideoCapture::Ptr inVideoCapture):
cameraFrame(new (cv::Mat)),
currentWorkFrame(0),
cameraName(inCameraName),
videoCapture(inVideoCapture),
mThread()

{
	startTime = boost::posix_time::microsec_clock::local_time();
	SetVideoCapture(videoCapture);
	mStopRequested = false;
	mThreadExited = false;

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
	boost::mutex::scoped_lock updateLock(mMutex);
 
	videoCapture = inVideoCapture;
	frameWidth = videoCapture->GetFrameWidth();
	frameHeight = videoCapture->GetFrameHeight();
	cameraFovWidth = videoCapture->GetCameraFovWidth();
	cameraFovHeight = videoCapture->GetCameraFovHeight();

	currentVideoSubscriberID = videoCapture->currentFrameSubscriptions->Subscribe();

	updateLock.unlock();
}

void  VideoViewer::Go() 
{
	if (!(mThread && mThread->joinable()))
	{
		void *windowPtr = cvGetWindowHandle(cameraName.c_str());
		// Create output window, only if it does not already exist
		if (windowPtr == NULL)
		{
			cvNamedWindow(cameraName.c_str(), CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL );
			SizeWindow();
        
		mStopRequested = false;
		mThreadExited = false;

		mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&VideoViewer::DoThreadLoop, this)));
		}
	}
}
 
void  VideoViewer::Stop() 
{
	if (mStopRequested) return;
	mStopRequested = true;

	if (!mThreadExited) 
	{
		void *windowPtr = cvGetWindowHandle(cameraName.c_str());
		if (windowPtr != NULL) cvDestroyWindow(cameraName.c_str());
	}

	if (mThreadExited) 
	{
		mThreadExited=false;
		assert(mThread);
		mThread->join();
	}


}

bool  VideoViewer::WasStopped()
{
	if (mStopRequested || mThreadExited) return(true);

	return(false);
}


void VideoViewer::SizeWindow()

{
		// Size the window to fit the maximum width specified
		const long nScreenWidth  = maxWindowWidth;
		const long nScreenHeight = maxWindowHeight;

		int height = 0, width = 0;
		if(frameWidth > nScreenWidth || frameHeight >nScreenHeight)
		{
			if((frameWidth/2) > nScreenWidth || (frameHeight/2) > nScreenHeight) 
			{
				width = frameWidth/4;
				height = frameHeight /4;
			} 
			else 
			{   
				width = frameWidth / 2;
				height = frameHeight /2; 
			}
		} 
		else 
		{   
			width = frameWidth;
			height = frameHeight; 
		}

		cvResizeWindow(cameraName.c_str(), width, height);
}

void VideoViewer::move(int left, int top)
{
	cvMoveWindow(cameraName.c_str(), left, top);
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
	boost::mutex::scoped_lock updateLock(mMutex);
	// Make a copy of the provided Detection::Vector to work with
    detectionData.clear();
	detectionData = data;

	// Sort the detection DataVect in threatLevel order
	// This insures that most urgent threats are displayed last.
	std::sort(detectionData.begin(), detectionData.end(), SortDetectionsInThreatLevel);

	updateLock.unlock();
}

void VideoViewer::DoThreadLoop()

{
	while (!WasStopped())
	{
		// Update the video frame
		if (videoCapture != NULL && videoCapture->currentFrameSubscriptions->HasNews(currentVideoSubscriberID)) 
		{
			// Copy the contents of the cv::Mat
			videoCapture->CopyCurrentFrame(cameraFrame, currentVideoSubscriberID);

			// Copy to the working area
			cameraFrame->copyTo(*workFrames[currentWorkFrame]);

			// Add the lidar range decorations to the video frame
			boost::mutex::scoped_lock updateLock(mMutex);
			DisplayReceiverValues(cameraFrame, workFrames[currentWorkFrame], detectionData);
			updateLock.unlock();

			//  Get the window handle. IOf it is null, may be that the window was closed or destroyed.
			//  In that case, do NOT reopen it.  The thread may be terminating.
			void *windowPtr = cvGetWindowHandle(cameraName.c_str());
			// The current work frame becomes the display frame
			if (windowPtr != NULL) cv::imshow(cameraName, *workFrames[currentWorkFrame]);

			// And we select a new work frame for later.
			currentWorkFrame = ++currentWorkFrame % workFrameQty;
		}

		//Give a break to other threads and wait for next frame
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));

		void *windowPtr = cvGetWindowHandle(cameraName.c_str());
		if(windowPtr == NULL)
		{
#if 1
			Stop(); 

			break;
#endif
		}
	} // while (!WasStoppped)

	//Give a break to other threads and wait for next frame
	boost::this_thread::sleep(boost::posix_time::milliseconds(300));

	mThreadExited = true;
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
	GetChannelRect(detection, topLeft, topRight, bottomLeft, bottomRight);

	// Inset the vertical lines horizontally, to compensate for line thickness.
	//Draw the vertical lines
	topLeft.x += (thickness/2);
	topRight.x -=  (thickness - (thickness/2) - 1); // In case thickness is odd 

	bottomLeft.x += (thickness/2); 
	bottomRight.x -= (thickness - (thickness/2) -1); // in case thickness is odd

	DrawDetectionLine(sourceFrame, targetFrame, topRight, bottomRight, colorEnhance, colorDehance, thickness, 1);
	DrawDetectionLine(sourceFrame, targetFrame, topLeft, bottomLeft, colorEnhance, colorDehance, thickness, 1);

	// Inset the horizontal lines vertically, to compensate for line height
	topLeft.y -= thickness/2;
	bottomLeft.y += (thickness - (thickness/2) -1);
	topRight.y -= thickness/2;
	bottomRight.y += (thickness - (thickness/2) -1);

	// Shorten the horizontal lines still, to avoid overlap with the verticals
	topLeft.x += (thickness - (thickness/2));
	topRight.x -= (thickness/2)+1;
	bottomLeft.x += (thickness - (thickness/2));
	bottomRight.x -= (thickness/2)+1;

	DrawDetectionLine(sourceFrame, targetFrame, topLeft, topRight, colorEnhance, colorDehance, 1, thickness);
	DrawDetectionLine(sourceFrame, targetFrame, bottomLeft, bottomRight, colorEnhance, colorDehance, 1, thickness);
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
			colorEnhance = cv::Vec3b(128, 0, 0);  // Blue
			colorDehance = cv::Vec3b(0, 64, 64);
			iThickness = 4;
		}
		break;
	case Detection::eThreatLow:
		{
			colorEnhance = cv::Vec3b(0, 128, 0); // Green
			colorDehance = cv::Vec3b(64, 0,64);
			iThickness = 4;
		}
		break;

	case Detection::eThreatWarn:
		{
			colorEnhance = cv::Vec3b(0, 64, 64); // Yellow
			colorDehance = cv::Vec3b(32, 0, 0);
			iThickness = 15;
			if (bFlash) iThickness = 5;	
		}
		break;

	case Detection::eThreatCritical:
		{
	 		colorEnhance = cv::Vec3b(0, 0, 128);  // Red
			colorDehance = cv::Vec3b(32, 32, 0);
			iThickness = 15;
			if (bFlash) iThickness = 5;
		}
		break;

	default:
		{
			colorEnhance = cv::Vec3b(0, 0, 0);
		}

	} // case
}

void VideoViewer::GetChannelRect(const Detection::Ptr &detection, CvPoint &topLeft, CvPoint &topRight, CvPoint &bottomLeft, CvPoint &bottomRight)
{	
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	AWLCoordinates *globalCoordinates = AWLCoordinates::GetGlobalCoordinates();

	float x, y;
	int receiverID = detection->receiverID;
	int channelID = detection->channelID;
	int cameraID = videoCapture->GetCameraID();

	// Channel description pointer
	ChannelConfig *channel = &globalSettings->receiverSettings[receiverID].channelsConfig[channelID];

	// Position of the topLeft corner of the channel FOV 
	SphericalCoord topLeftInChannel(detection->distance, M_PI_2 - DEG2RAD(channel->fovHeight/2), +DEG2RAD(channel->fovWidth/2));  // Spherical coordinate, relative to sensor
	globalCoordinates->SensorToCamera(receiverID, channelID, cameraID, cameraFovWidth, cameraFovHeight, frameWidth, frameHeight, topLeftInChannel, topLeft.x, topLeft.y);

	// Position of the topRight corner of the channel FOV 
	SphericalCoord topRightInChannel(detection->distance, M_PI_2 - DEG2RAD(channel->fovHeight/2), -DEG2RAD(channel->fovWidth/2));
	globalCoordinates->SensorToCamera(receiverID, channelID, cameraID, cameraFovWidth, cameraFovHeight, frameWidth, frameHeight, topRightInChannel, topRight.x, topRight.y);

	// Position of the bottomLeft corner of the channel FOV 
	SphericalCoord bottomLeftInChannel(detection->distance, M_PI_2 + DEG2RAD(channel->fovHeight/2), + DEG2RAD(channel->fovWidth/2));
	globalCoordinates->SensorToCamera(receiverID, channelID, cameraID, cameraFovWidth, cameraFovHeight, frameWidth, frameHeight, bottomLeftInChannel, bottomLeft.x, bottomLeft.y);

	// Position of the topRight corner of the channel FOV 
	SphericalCoord bottomRightInChannel(detection->distance, M_PI_2 + DEG2RAD(channel->fovHeight/2), -DEG2RAD(channel->fovWidth/2));
	globalCoordinates->SensorToCamera(receiverID, channelID, cameraID, cameraFovWidth, cameraFovHeight, frameWidth, frameHeight, bottomRightInChannel, bottomRight.x, bottomRight.y);
}

void VideoViewer::DrawDetectionLine(VideoCapture::FramePtr &sourceFrame, VideoCapture::FramePtr &targetFrame, const CvPoint &startPoint, const CvPoint &endPoint,  const cv::Vec3b &colorEnhance, const cv::Vec3b &colorDehance, int iWidth, int iHeight)
{
	cv::LineIterator lineIter(*targetFrame, startPoint, endPoint, 8);

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

					int r = (int)color[0]+ (int)colorEnhance[0] - (int)colorDehance[0];
					int g = (int)color[1] + (int)colorEnhance[1] - (int)colorDehance[1];
					int b = (int)color[2] + (int)colorEnhance[2] - (int)colorDehance[2];

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



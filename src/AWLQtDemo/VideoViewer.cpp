
#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

#include "AWLSettings.h"
#include "VideoCapture.h"
#include "ReceiverCapture.h"
#include "Sensor.h"
#include "AWLCoord.h"
#include "VideoViewer.h"
#include "DebugPrintf.h"

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"

#include "windows.h"

using namespace std;
using namespace awl;

#if 0
const char *szCameraWindowClassName = "Main HighGUI class";  // Class name for the camera windows created by OpenCV
															// We set NULL, as the default name of the class under Qt and under straight OpenCv is not the same.
#else
const char *szCameraWindowClassName = NULL;  // Class name for the camera windows created by OpenCV
															// We set NULL, as the default name of the class under Qt and under straight OpenCv is not the same.
#endif


VideoViewer::VideoViewer(std::string inCameraName, VideoCapture::Ptr inVideoCapture, ReceiverCapture::Ptr inReceiverCapture, ReceiverProjector::Ptr inProjector):
workFrame(new (cv::Mat)),
displayFrame(new (cv::Mat)),
cameraName(inCameraName),
videoCapture(inVideoCapture),
receiverCapture(inReceiverCapture),
projector(inProjector),
bWindowCreated(false),
mThread()

{

	SetVideoCapture(videoCapture);
	SetReceiverCapture(receiverCapture);
	mStopRequested = false;
	mThreadExited = false;


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
	frameRate = videoCapture->GetFrameRate();
	scale = videoCapture->GetScale();
	cameraFovWidth = videoCapture->GetCameraFovWidth();
	cameraFovHeight = videoCapture->GetCameraFovHeight();

	currentVideoSubscriberID = videoCapture->currentFrameSubscriptions->Subscribe();

	updateLock.unlock();
}

void VideoViewer::SetReceiverCapture( ReceiverCapture::Ptr inReceiverCapture)
{
	boost::mutex::scoped_lock updateLock(mMutex);
 
	receiverCapture = inReceiverCapture;

	currentReceiverSubscriberID = receiverCapture->currentReceiverCaptureSubscriptions->Subscribe();

	updateLock.unlock();
}

void VideoViewer::SetReceiverProjector(ReceiverProjector::Ptr inProjector)
{
	boost::mutex::scoped_lock updateLock(mMutex);
	projector = inProjector;
	updateLock.unlock();
}

void  VideoViewer::Go() 
{	
	if (!(mThread && mThread->joinable()))
	{

		// Create output window
		if (!bWindowCreated) 
		{
			cvNamedWindow(cameraName.c_str(), CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL );
			bWindowCreated = true;
			SetWindowIcon();
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
	HWND window = ::FindWindowA(szCameraWindowClassName, cameraName.c_str());
	if (window == NULL) bWindowCreated = false;
	else bWindowCreated = true;
	
	if (bWindowCreated) cvDestroyWindow(cameraName.c_str());
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

void VideoViewer::SetWindowIcon()

{
	// Set the icon for the window
	HWND window = ::FindWindowA(szCameraWindowClassName, cameraName.c_str());
	if (window != NULL) 
	{
		AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
		if (!globalSettings->sIconFileName.empty())
		{
			HICON hIcon = (HICON)::LoadImageA(NULL, globalSettings->sIconFileName.c_str(), IMAGE_ICON,
				GetSystemMetrics(SM_CXSMICON), 
				GetSystemMetrics(SM_CYSMICON),
				LR_LOADFROMFILE);

			::SendMessage(window, WM_SETICON, ICON_SMALL, (LPARAM)hIcon);
		}
	}
}


void VideoViewer::SizeWindow()

{
		// Size the window to fit the screen
		const long nScreenWidth  = GetSystemMetrics(SM_CXSCREEN);
		const long nScreenHeight = GetSystemMetrics(SM_CYSCREEN);

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



void VideoViewer::CopyWorkFrame(VideoCapture::FramePtr targetFrame) 
{
//	boost::mutex::scoped_lock updateLock(mMutex);
 	workFrame->copyTo(*targetFrame);
//	updateLock.unlock();
}

static int frameCount = 0;

void VideoViewer::DoThreadLoop()

{
	while (!WasStopped())
	{
		// Update the video frame
		if (videoCapture != NULL && videoCapture->currentFrameSubscriptions->HasNews(currentVideoSubscriberID)) 
		{
			// Copy the contents of the cv::Mat
			videoCapture->CopyCurrentFrame(workFrame, currentVideoSubscriberID);

			// Add the lidar range decorations to the video frame
			DisplayReceiverValues(workFrame);

			// Copy to the display (we are double-buffering)
#if 1
			workFrame->copyTo(*displayFrame);
#else
			CvSize displaySize(cvSize(frameWidth / 2, frameHeight /2));
			cvResize(workframe, displayFrame, displaySize);
#endif
			// 
			HWND window = ::FindWindowA(szCameraWindowClassName, cameraName.c_str());
			if (window == NULL) bWindowCreated = false;
			if (bWindowCreated && ::IsWindowVisible(window)) cv::imshow(cameraName, *displayFrame);
		}

		//Give a break to other threads and wait for next frame
		boost::this_thread::sleep(boost::posix_time::milliseconds(2));

		if((!bWindowCreated))
		{
#if 1
			Stop(); 

			break;
#endif
		}
	} // while (!WasStoppped)


	mThreadExited = true;
}
	

static uint16_t detectedCount[7] = {0, 0, 0, 0, 0, 0, 0};
const int flashFrequency = 2;

void VideoViewer::DisplayReceiverValues(VideoCapture::FramePtr &targetFrame)

{
		// Use the frame snapped by the main display timer as the current frame
	// display will �
	uint32_t lastDisplayedFrame = receiverCapture->GetSnapshotFrameID();

	for (int channelID = 0; channelID < 7; channelID++) 
	{
		if (channelID < receiverCapture->GetChannelQty())
		{
			if (receiverCapture->GetFrameQty()) 
			{

				ChannelFrame::Ptr channelFrame(new ChannelFrame(receiverCapture->receiverID, channelID));

				// Thread safe
				if (receiverCapture->CopyReceiverChannelData(lastDisplayedFrame, channelID, channelFrame, currentReceiverSubscriberID)) 
				{
					float minDistance = receiverCapture->GetMinDistance();
					float maxDistance = receiverCapture->GetMaxDistance(channelID);

					Detection::ThreatLevel maxThreatLevel = Detection::eThreatNone;
					Detection::Ptr selectedDetection;
					bool bDetected = false;

					int detectionQty = channelFrame->detections.size();
					for (int i = 0; i < detectionQty; i++)
					{	
						Detection::Ptr detection = channelFrame->detections.at(i);
						float distance = detection->distance;
						if ((distance >= minDistance) && (distance <= maxDistance))
						{  
							if (detection->threatLevel >= maxThreatLevel) 
							{
								selectedDetection = detection;
								maxThreatLevel = detection->threatLevel;
								bDetected = true;
							}
						}
					} // for detections

					if (bDetected) 
					{
						detectedCount[channelID]++;
						DisplayTarget(targetFrame, channelID, selectedDetection);
					}
					else 
					{
						detectedCount[channelID] = 0;
					}

				}
			}
		}
	}
}



#if 0
void VideoViewer::DisplayTarget(VideoCapture::FramePtr &targetFrame, int channelID,  Detection::Ptr &detection)
{
	int top;
	int left;
	int bottom;
	int right;

	if (!projector) return;

	CvRect rect;

	cv::Vec3b color;
	cv::Vec3b colorEnhance;
	cv::Vec3b colorDehance;
	bool bFlash = false;

	int width = -1;
	colorEnhance = cv::Vec3b(0, 0, 0);
	colorDehance = cv::Vec3b(0, 0, 0);

	Detection::ThreatLevel threatLevel = detection->threatLevel;
	switch (threatLevel) 
	{
	case Detection::eThreatNone: 
		{
			colorEnhance = cv::Vec3b(128, 0, 0);  // Blue
			colorDehance = cv::Vec3b(0, 64, 64);
			width = 4;
		}
		break;
	case Detection::eThreatLow:
		{
			colorEnhance = cv::Vec3b(0, 128, 0); // Green
			colorDehance = cv::Vec3b(64, 0,64);
			width = 4;
		}
		break;

	case Detection::eThreatWarn:
		{
			colorEnhance = cv::Vec3b(0, 64, 64); // Yellow
			colorDehance = cv::Vec3b(32, 0, 0);
			width = 15;
			bFlash = true;
			if (bFlash && !(detectedCount[channelID] % flashFrequency)) width = 5;
		}
		break;

	case Detection::eThreatCritical:
		{
	 		colorEnhance = cv::Vec3b(0, 0, 128);  // Red
//			colorEnhance = cv::Vec3b(0, 0, 196);  // Red
			colorDehance = cv::Vec3b(32, 32, 0);
			width = 15;
			bFlash = true;
			if (bFlash && !(detectedCount[channelID] % flashFrequency)) width = 5;
		}
		break;

	default:
		{
			colorEnhance = cv::Vec3b(0, 0, 0);
		}

	} // case


	// Paint a square that corresponds to the receiver FOV
	// If width argument is positive, will draw an empty square with
	// using the width argument as a line width.
	// If width argument is negative or zero, the square is filled.
	left = -1;
	top = -1;
	bottom = -1;
	right = -1;

#if 1
	projector->GetChannelRect(channelID, top, left, bottom, right);
#else

#endif

	// The rectagle is not just drawn as solid, but as a colored highlight so we manually address the pixels.

	for (int row = top; row <= bottom; row++) 
	{
		for (int column = left; column <= right; column++) 
		{
			bool overlay = false;
			if ((row < (top + width)) || (row > (bottom - width))) overlay = true;
			else if ((column < (left + width)) || (column > (right - width))) overlay = true;
			else if (width <= 0) overlay = true;
			
			if (overlay) 
			{
				color = targetFrame->at<cv::Vec3b>(row, column);
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
				targetFrame->at<cv::Vec3b>(row, column) = color;
			}
		}
	}
}
#else
void VideoViewer::DisplayTarget(VideoCapture::FramePtr &targetFrame, int channelID,  Detection::Ptr &detection)
{
	int top;
	int left;
	int bottom;
	int right;

	if (!projector) return;

	CvRect rect;

	cv::Vec3b color;
	cv::Vec3b colorEnhance;
	cv::Vec3b colorDehance;
	bool bFlash = false;

	int width = -1;
	colorEnhance = cv::Vec3b(0, 0, 0);
	colorDehance = cv::Vec3b(0, 0, 0);
	GetDetectionColors(detection, colorEnhance, colorDehance, width);



	// Paint a square that corresponds to the receiver FOV
	// If width argument is positive, will draw an empty square with
	// using the width argument as a line width.
	// If width argument is negative or zero, the square is filled.
	CvPoint topLeft;
	CvPoint topRight;
	CvPoint bottomLeft;
	CvPoint bottomRight;
	GetChannelRect(detection, topLeft, topRight, bottomLeft, bottomRight);

	DrawDetectionLine(targetFrame, topLeft, topRight, colorEnhance, colorDehance, width);
	DrawDetectionLine(targetFrame, topRight, bottomRight, colorEnhance, colorDehance, width);
	DrawDetectionLine(targetFrame, bottomLeft, bottomRight, colorEnhance, colorDehance, width);
	DrawDetectionLine(targetFrame, topLeft, bottomLeft, colorEnhance, colorDehance, width);
}

void VideoViewer::GetDetectionColors(const Detection::Ptr &detection, cv::Vec3b &colorEnhance, cv::Vec3b &colorDehance, int &iWidth)

{
	bool bFlash = false;

	int channelID = detection->channelID;
	Detection::ThreatLevel threatLevel = detection->threatLevel;

	switch (threatLevel) 
	{
	case Detection::eThreatNone: 
		{
			colorEnhance = cv::Vec3b(128, 0, 0);  // Blue
			colorDehance = cv::Vec3b(0, 64, 64);
			iWidth = 4;
		}
		break;
	case Detection::eThreatLow:
		{
			colorEnhance = cv::Vec3b(0, 128, 0); // Green
			colorDehance = cv::Vec3b(64, 0,64);
			iWidth = 4;
		}
		break;

	case Detection::eThreatWarn:
		{
			colorEnhance = cv::Vec3b(0, 64, 64); // Yellow
			colorDehance = cv::Vec3b(32, 0, 0);
			iWidth = 15;
			bFlash = true;
			if (bFlash && !(detectedCount[channelID] % flashFrequency)) iWidth = 5;	
		}
		break;

	case Detection::eThreatCritical:
		{
	 		colorEnhance = cv::Vec3b(0, 0, 128);  // Red
			colorDehance = cv::Vec3b(32, 32, 0);
			iWidth = 15;
			bFlash = true;
			if (bFlash && !(detectedCount[channelID] % flashFrequency)) iWidth = 5;
		}
		break;

	default:
		{
			colorEnhance = cv::Vec3b(0, 0, 0);
		}

	} // case
}

void VideoViewer::GetChannelRect(Detection::Ptr &detection, CvPoint &topLeft, CvPoint &topRight, CvPoint &bottomLeft, CvPoint &bottomRight)
{	
	
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	float x, y;
	int receiverID = detection->receiverID;
	int channelID = detection->channelID;
	int cameraID = videoCapture->GetCameraID();

	// Position of the target relative to world. Target is straight in the center of the detector.
	CartesianCoord targetRelativeToWorldCart = detection->relativeToWorldCart;

	// Channel description pointer
	ChannelConfig *channel = &globalSettings->receiverSettings[receiverID].channelsConfig[channelID];
	TransformationNode::Ptr channelCoords = AWLCoordinates::GetReceivers()[receiverID]->children[channelID];
	
	// Camera FOV description
	double cameraFovHeightInDegrees = RAD2DEG(cameraFovHeight);
	double cameraFovWidthInDegrees = RAD2DEG(cameraFovWidth);

	TransformationNode::Ptr cameraCoords = AWLCoordinates::GetCameras()[cameraID];
	CartesianCoord cameraTopLeft(SphericalCoord(10, M_PI_2 - (cameraFovHeight/2), +(cameraFovWidth/2)));
	CartesianCoord cameraBottomRight(SphericalCoord(10, M_PI_2 + (cameraFovHeight/2), - (cameraFovWidth/2)));


	// Position of the topLeft corner of the channel FOV 
#if 0
	SphericalCoord sphericalPointInChannel(detection->distance, M_PI_2, 0);
	detection->relativeToSensorCart = receiverCoords[receiverID]->children[channelIndex]->ToReferenceCoord(eSensorToReceiverCoord, sphericalPointInChannel);
	detection->relativeToVehicleCart = receiverCoords[receiverID]->children[channelIndex]->ToReferenceCoord(eSensorToVehicleCoord, sphericalPointInChannel);
	detection->relativeToWorldCart = receiverCoords[receiverID]->children[channelIndex]->ToReferenceCoord(eSensorToWorldCoord, sphericalPointInChannel);
#endif


	SphericalCoord topLeftInChannel(detection->distance, M_PI_2 - DEG2RAD(channel->fovHeight/2), +DEG2RAD(channel->fovWidth/2));  // Spherical coordinate, relative to sensor
	SphericalCoord topLeftInWorld = channelCoords->ToReferenceCoord(eSensorToWorldCoord, topLeftInChannel);          // Convert to world
	SphericalCoord topLeftInCamera = cameraCoords->FromReferenceCoord(eWorldToCameraCoord, topLeftInWorld);			 // Convert to camera
	topLeftInCamera.rho = 10;																						 // Place in projection Plane.
	CartesianCoord topLeftInCameraCart(topLeftInCamera);

	// Remember: In relation to a body the standard convention is
	//  x forward, y left and z up.
	// Careful when converting to projected plane, where X is right and y is up

	topLeft.x = frameWidth * (topLeftInCameraCart.left-cameraTopLeft.left) / (cameraBottomRight.left - cameraTopLeft.left);
	topLeft.y = frameHeight * (topLeftInCameraCart.up-cameraBottomRight.up) / (cameraTopLeft.up- cameraBottomRight.up);

	// Position of the topRight corner of the channel FOV 
	SphericalCoord topRightInChannel(detection->distance, M_PI_2 - DEG2RAD(channel->fovHeight/2), -DEG2RAD(channel->fovWidth/2));
	SphericalCoord topRightInWorld = channelCoords->ToReferenceCoord(eSensorToWorldCoord, topRightInChannel);
	SphericalCoord topRightInCamera = cameraCoords->FromReferenceCoord(eWorldToCameraCoord, topRightInWorld);
	topRightInCamera.rho = 10;																						 // Place in projection Plane.
	CartesianCoord topRightInCameraCart(topRightInCamera);

	topRight.x = frameWidth * (topRightInCameraCart.left-cameraTopLeft.left) / (cameraBottomRight.left - cameraTopLeft.left);
	topRight.y = frameHeight * (topRightInCameraCart.up-cameraBottomRight.up) / (cameraTopLeft.up - cameraBottomRight.up);

	// Position of the bottomLeft corner of the channel FOV 
	SphericalCoord bottomLeftInChannel(detection->distance, M_PI_2 + DEG2RAD(channel->fovHeight/2), + DEG2RAD(channel->fovWidth/2));
	SphericalCoord bottomLeftInWorld = channelCoords->ToReferenceCoord(eSensorToWorldCoord, bottomLeftInChannel);
	SphericalCoord bottomLeftInCamera = cameraCoords->FromReferenceCoord(eWorldToCameraCoord, bottomLeftInWorld);
	bottomLeftInCamera.rho = 10;																						 // Place in projection Plane.
	CartesianCoord bottomLeftInCameraCart(bottomLeftInCamera);

	bottomLeft.x = frameWidth * (bottomLeftInCameraCart.left-cameraTopLeft.left) / (cameraBottomRight.left - cameraTopLeft.left);
	bottomLeft.y = frameHeight * (bottomLeftInCameraCart.up-cameraBottomRight.up) / (cameraTopLeft.up - cameraBottomRight.up);

	// Position of the topRight corner of the channel FOV 
	SphericalCoord bottomRightInChannel(detection->distance, M_PI_2 + DEG2RAD(channel->fovHeight/2), -DEG2RAD(channel->fovWidth/2));
	SphericalCoord bottomRightInWorld = channelCoords->ToReferenceCoord(eSensorToWorldCoord, bottomRightInChannel);
	SphericalCoord bottomRightInCamera = cameraCoords->FromReferenceCoord(eWorldToCameraCoord, bottomRightInWorld);
	bottomRightInCamera.rho = 10;																						 // Place in projection Plane.
	CartesianCoord bottomRightInCameraCart(bottomRightInCamera);

	bottomRight.x = frameWidth * (bottomRightInCameraCart.left-cameraTopLeft.left) / (cameraBottomRight.left - cameraTopLeft.left);
	bottomRight.y = frameHeight * (bottomRightInCameraCart.up-cameraBottomRight.up) / (cameraTopLeft.up - cameraBottomRight.up);

}

void VideoViewer::DrawDetectionLine(VideoCapture::FramePtr &targetFrame, const CvPoint &startPoint, const CvPoint &endPoint,  const cv::Vec3b &colorEnhance, const cv::Vec3b &colorDehance, int iWidth)
{
	cv::LineIterator lineIter(*targetFrame, startPoint, endPoint, 8);

	// alternative way of iterating through the line
	for(int i = 0; i < lineIter.count; i++, ++lineIter)
	{
		cv::Vec3b	color = targetFrame->at<cv::Vec3b>(lineIter.pos());



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
		targetFrame->at<cv::Vec3b>(lineIter.pos()) = color;
	}


}

#endif


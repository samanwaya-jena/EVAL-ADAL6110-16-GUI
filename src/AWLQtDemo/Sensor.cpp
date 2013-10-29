#define CV_NO_BACKWARD_COMPATIBILITY

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"


#include <iostream>
#include <cstdio>

#include "VideoCapture.h"
#include "ReceiverCapture.h"
#include "Sensor.h"
#include "AWLSettings.h"

#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>

#include "windows.h"
using namespace std;
using namespace pcl;
using namespace awl;

const int threadSleepDelay = 10;  // Thread sleep time between iterations.

const cv::Vec3b cvBlack(0, 0, 0);

ReceiverChannel::ReceiverChannel(	const int inChannelID, const float inFovX, const float inFovY, 
						const float inCenterX, const float inCenterY, const float inRangeMax, 
						const std::string inMaskName, const std::string inFrameName,
						bool inDisplayUnderZero, 
						double inDisplayColorR, double inDisplayColorG, double inDisplayColorB):
	channelID(inChannelID),
	fovWidthX(inFovX),
	fovWidthY(inFovY),
	fovCenterX(inCenterX),
	fovCenterY(inCenterY),
	rangeMax(inRangeMax), 
	maskName(inMaskName),
	frameName(inFrameName),
	displayColorR(inDisplayColorR),
	displayColorG(inDisplayColorG),
	displayColorB(inDisplayColorB),
	displayUnderZero(inDisplayUnderZero),
	decimationX(1),
	decimationY(1),
	backgroundPtr(),
	colorPtr(),
	currentCloud()

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	sensorHeight = globalSettings->sensorHeight;
	sensorDepth = globalSettings->sensorDepth;
//	rangeMax = globalSettings->displayedRangeMax;


	WCHAR directoryName[255];
	::GetCurrentDirectoryW(255, directoryName);

	maskImage = cv::imread( maskName, 1 );
	if( maskImage.empty() )
	{
	cerr << "Error: invalid maskImage " << maskName << endl;
	}

	frameImage = cv::imread( frameName, 1 );
	if( frameImage.empty() )
	{
	cerr << "Error: invalid maskImage" << maskName << endl;
	}
} 

void ReceiverChannel::AddDistancesToCloud()
{
	float minDistance = receiverCapture->GetMinDistance();
	float maxDistance = receiverCapture->GetMaxDistance();

	if (channelID < receiverCapture->GetChannelQty())
	{
		if (receiverCapture->GetFrameQty()) 
		{
	
			ChannelFrame::Ptr channelFrame(new ChannelFrame(this->channelID));

			// Thread safe

			uint32_t lastDisplayedFrame = receiverCapture->GetSnapshotFrameID();
			if (receiverCapture->CopyReceiverChannelData(lastDisplayedFrame, channelID, channelFrame, receiverCaptureSubscriberID))
			{

				int detectionQty = channelFrame->detections.size();
				for (int i = 0; i < detectionQty; i++)
				{
					Detection::Ptr detection = channelFrame->detections.at(i);

					if (detection->distance >= minDistance && detection->distance <= maxDistance) 
					{
							AddDistanceToCloud(detection->distance, 255);
					}
				}
			}
		}
	}
}

void ReceiverChannel::AddDistanceToCloud(float inDistance, uint8_t inIntensity)

{
	PointXYZRGB newCloudPoint;
	PointWithRange pointWithRange;
	cv::Vec3b mask;
	cv::Vec3b color;

	assert(backgroundPtr!= NULL);
	assert(colorPtr != NULL);

	int size = maskPoints.size();
	CvPoint *point = (CvPoint *) maskPoints.data();

	for (int i = 0; i < size; i++, point++) 
	{
		int x = point->x;
		int y = point->y;

		// Check to see if X and Y are non-decimated points
		bool bIsOnY = (y / decimationY)*(decimationY) == y;
		bool bIsOnX = (x/ decimationX) * decimationX == x;

		if (bIsOnY && bIsOnX)// Display only points that are an even multimple of decimation Y
		{
			if (y < maskImage.rows && y < colorPtr->rows && y < backgroundPtr->rows &&
				x < maskImage.cols && x < colorPtr->cols && x < backgroundPtr->cols) 
			{
				AddDistanceToCloud((const cv::Vec3b &) maskImage.at<cv::Vec3b>(y, x), 
										  (const cv::Vec3b &) colorPtr->at<cv::Vec3b>(y, x), 
										  (cv::Vec3b &) backgroundPtr->at<cv::Vec3b>(y, x), 
										   x, y, inDistance, inIntensity);
			}
		}
	}
}


void ReceiverChannel::AddDistanceToCloud(const cv::Vec3b &mask,
										const cv::Vec3b &color,
										cv::Vec3b &background,
										int xPos,
										int yPos, 
										float inDistance, 
										uint8_t inIntensity)
{
	PointXYZRGB newCloudPoint;

	if (mask.val[1]  | mask.val[1] | mask.val[2]) 
	{
		// Define point location
		float pointX = ((float) xPos);
		float pointY = imageHeight - (float)yPos;
		receiverCoordinatesPtr->GetXYZFromRange(pointX, pointY, inDistance, newCloudPoint);

		if ((newCloudPoint.y > (-sensorHeight)) || displayUnderZero) {
			newCloudPoint.b = color[0];
			newCloudPoint.g = color[1];
			newCloudPoint.r = color[2];

			newCloudPoint.a = inIntensity;

			//	Add the point to the array;
			currentCloud->points.push_back((pcl::PointXYZRGB) newCloudPoint);

			// mask the background
			background = cvBlack;
		}
	}
}

void ReceiverChannel::AddPointToCloud(float x, float y, float z,
									  uint8_t r,
									  uint8_t g,
									  uint8_t b,
								      uint8_t inIntensity)

{
	PointXYZRGB newCloudPoint;

	// Define point location
	newCloudPoint.x = x;
	newCloudPoint.y = y;
	newCloudPoint.z = z;

	newCloudPoint.b = b;
	newCloudPoint.g = g;
	newCloudPoint.r = r;

	newCloudPoint.a = inIntensity;

	//	Add the point to the array;
	currentCloud->points.push_back((pcl::PointXYZRGB) newCloudPoint);
}

void ReceiverChannel::UpdateReceiverCoordinates(ReceiverCoordinates::Ptr &inReceiverCoordinatesPtr)
{
	receiverCoordinatesPtr = inReceiverCoordinatesPtr;
	imageWidth = receiverCoordinatesPtr->GetWidth();
	imageHeight = receiverCoordinatesPtr->GetHeight();
	imageCenterX = imageWidth / 2;
	imageCenterY = imageHeight / 2;

	float x, y;
	receiverCoordinatesPtr->getImagePointFromAngles(fovCenterX - (fovWidthX/2), 
											 fovCenterY - (fovWidthY/2),
											 x, y);
	topLeftX = (int) x;
	bottomRightY = imageHeight - (int) y;  // Y is reversed between video and cloudView!

			
	receiverCoordinatesPtr->getImagePointFromAngles(fovCenterX + (fovWidthX/2), 
											 fovCenterY + (fovWidthY/2),
											 x, y);
	bottomRightX = (int) x;
	topLeftY = imageHeight - (int) y; // Y is reversed between video and cloudView!

	if (topLeftX < 0) topLeftX = 0;
	if (topLeftY < 0) topLeftY = 0;
	if (bottomRightX >= imageWidth) bottomRightX = imageWidth - 1;
	if (bottomRightY >= imageHeight) bottomRightY = imageHeight - 1;

	topLeftX = 0;
	topLeftY = 0;
	bottomRightX = imageWidth-1;
	bottomRightY = imageHeight- 1;

	BuildPixelMask();
}

void ReceiverChannel::BuildPixelMask()

{
	PointXYZRGB newCloudPoint;
	PointWithRange pointWithRange;
	cv::Vec3b mask;
	cv::Vec3b color;

	for (int y = 0; y < imageHeight; y++)
	{
		for (int x = 0; x < imageWidth; x++) 
		{
			cv::Vec3b mask = maskImage.at<cv::Vec3b>(y, x);
			if (mask[0] | mask.val[1] | mask.val[2])  
			{
				CvPoint maskPoint;
				maskPoint.x = x;
				maskPoint.y = y;
				maskPoints.push_back(maskPoint);
			}
		}
	}
}

void ReceiverChannel::GetChannelLimits(ReceiverCoordinates::Ptr &inReceiverCoordinates, double &minX, double &minY, double &minZ, 
		double &maxX, double &maxY, double &maxZ, double &originX, double &originY, double &originZ)
{
	imageWidth = inReceiverCoordinates->GetWidth();
	imageHeight = inReceiverCoordinates->GetHeight();
	imageCenterX = imageWidth / 2;
	imageCenterY = imageHeight / 2;
	float x, y;

	inReceiverCoordinates->getImagePointFromAngles(fovCenterX - (fovWidthX/2), 
											 fovCenterY - (fovWidthY/2),
											 x, y);
	int topLeftX = (int) x;
	int bottomRightY = imageHeight - (int) y;  // Y is reversed between video and cloudView!

			
	inReceiverCoordinates->getImagePointFromAngles(fovCenterX + (fovWidthX/2), 
											 fovCenterY + (fovWidthY/2),
											 x, y);
	int bottomRightX = (int) x;
	int topLeftY = imageHeight - (int) y; // Y is reversed between video and cloudView!

	if (topLeftX < 0) topLeftX = 0;
	if (topLeftY < 0) topLeftY = 0;
	if (bottomRightX >= imageWidth) bottomRightX = imageWidth - 1;
	if (bottomRightY >= imageHeight) bottomRightY = imageHeight - 1;

	PointXYZRGB topLeftPoint;
	inReceiverCoordinates->GetXYZFromRange((float)topLeftX, (float)topLeftY, rangeMax, topLeftPoint);

	PointXYZRGB bottomRightPoint;
	inReceiverCoordinates->GetXYZFromRange((float)bottomRightX, (float)bottomRightY, rangeMax, bottomRightPoint);
	
	minX = topLeftPoint.x;
	minY = topLeftPoint.y;
	minZ = topLeftPoint.z;
	maxX = bottomRightPoint.x;
	maxY = bottomRightPoint.y;
	maxZ = bottomRightPoint.z;

	originX = 0;
	originY = sensorHeight;
	originZ = sensorDepth;
}

void ReceiverChannel::GetChannelRect(ReceiverCoordinates::Ptr &inReceiverCoordinates, int &top, int &left, int &bottom, int &right)
{
	imageWidth = inReceiverCoordinates->GetWidth();
	imageHeight = inReceiverCoordinates->GetHeight();
	imageCenterX = imageWidth / 2;
	imageCenterY = imageHeight / 2;
	float x, y;

	inReceiverCoordinates->getImagePointFromAngles(fovCenterX - (fovWidthX/2), 
											 fovCenterY - (fovWidthY/2),
											 x, y);
	left = (int) x;
	bottom = imageHeight - (int) y;  // Y is reversed between video and cloudView!

			
	inReceiverCoordinates->getImagePointFromAngles(fovCenterX + (fovWidthX/2), 
											 fovCenterY + (fovWidthY/2),
											 x, y);
	right = (int) x;
	top = imageHeight - (int) y; // Y is reversed between video and cloudView!

	// Make sure we are in order
	
	if (left > right) 
	{
		int temp = right;
		right = left;
		left = temp;
	}
	if (top > bottom)
	{
		int temp = bottom;
		bottom = top;
		top = temp;
	}

	// Make sure we are within display bounds
	if (left < 0) left = 0;
	if (top < 0) top = 0;
	if (right >= imageWidth) right = imageWidth - 1;
	if (bottom >= imageHeight) bottom = imageHeight - 1;

}

void ReceiverChannel::GetDisplayColor(double &outR, double &outG, double&outB)
{
	outR = displayColorR;
	outG = displayColorG;
	outB = displayColorB;
}

bool ReceiverChannel::SetDisplayUnderZero(bool inDisplayUnderZero)
{
	displayUnderZero = inDisplayUnderZero;
	return (displayUnderZero);
}


bool ReceiverChannel::GetDisplayUnderZero()
{
	return (displayUnderZero);
}


void ReceiverChannel::SetDecimation(int inDecimationX, int inDecimationY)
{
	decimationX = inDecimationX;
	decimationY = inDecimationY;
}


void ReceiverChannel::GetDecimation(int &outDecimationX, int &outDecimationY)
{
	outDecimationX = decimationX;
	outDecimationY = decimationY;
}

void ReceiverChannel::SetSensorHeight(double inSensorHeight)
{
	sensorHeight = inSensorHeight;
}

void ReceiverChannel::GetSensorHeight(double &outSensorHeight)
{
	outSensorHeight = sensorHeight;
}

void ReceiverChannel::SetSensorDepth(double inSensorDepth)
{
	sensorDepth = inSensorDepth;
}

void ReceiverChannel::GetSensorDepth(double &outSensorDepth)
{
	outSensorDepth = sensorDepth;
}

void ReceiverChannel::SetRangeMax(double inRangeMax)
{
	rangeMax = inRangeMax;
}

void ReceiverChannel::GetRangeMax(double &outRangeMax)
{
	outRangeMax = rangeMax;
}

void ReceiverChannel::SetBackgroundPtr(ReceiverChannel::FramePtr inBackgroundPtr) 

{
	backgroundPtr = inBackgroundPtr;
}

void ReceiverChannel::SetColorPtr(ReceiverChannel::FramePtr inColorPtr) 

{
	colorPtr = inColorPtr;
}

void ReceiverChannel::SetCurrentCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inCurrentCloud) 

{
	currentCloud = inCurrentCloud;
}

void ReceiverChannel::SetReceiver(ReceiverCapture::Ptr inReceiver, 
	Subscription::SubscriberID inCurrentReceiverCaptureSubscriberID)
{
	receiverCapture = inReceiver;
	receiverCaptureSubscriberID = inCurrentReceiverCaptureSubscriberID;
}


//*********************************************************************

ReceiverProjector::ReceiverProjector(VideoCapture::Ptr videoCapture, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & inCloud, 
	ReceiverCapture::Ptr receiverCapture):
displayUnderZero(true),
currentFrame(new (cv::Mat)),
backgroundFrame(new (cv::Mat)),
currentCloudSubscriptions(new(Subscription))

{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	sensorHeight = globalSettings->sensorHeight;
	sensorDepth = globalSettings->sensorDepth;
	rangeMax = globalSettings->displayedRangeMax;
	decimationX = globalSettings->decimation;
	decimationY = globalSettings->decimation;


	SetVideoCapture(videoCapture);
	SetReceiverCapture(receiverCapture);
	SetCloud(inCloud);
}

ReceiverProjector::~ReceiverProjector()
{
	Stop();
}

void  ReceiverProjector::Go() 
{
	assert(!mThread);
    mStopRequested = false;
//	mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverProjector::DoThreadLoop, this)));
}
 

void  ReceiverProjector::Stop() 
{
	if (mStopRequested) return;
    mStopRequested = true;
#if 0
	assert(mThread);
	mThread->join();
#endif
}

bool  ReceiverProjector::WasStopped()
{
	if (mStopRequested) return(true);
	if (videoCapture->WasStopped()) {
		Stop();
		return(true);
	}
	if (receiverCapture->WasStopped()) {
		Stop();
		return(true);
	}

	return(false);
}


void ReceiverProjector::SetVideoCapture( VideoCapture::Ptr inVideoCapture)
{
	videoCapture = inVideoCapture;

	boost::mutex::scoped_lock updateLock(mMutex);
	boost::mutex::scoped_lock videoLock(videoCapture->currentFrameSubscriptions->GetMutex());

	frameWidth = videoCapture->GetFrameWidth();
	frameHeight = videoCapture->GetFrameHeight();
	frameRate = videoCapture->GetFrameRate();
	scale = videoCapture->GetScale();
	cameraFovX = videoCapture->GetCameraFovX();
	cameraFovY = videoCapture->GetCameraFovY();

	mReceiverCoordinatesPtr = ReceiverCoordinates::Ptr(new ReceiverCoordinates(frameWidth, frameHeight, 
                                                cameraFovX, cameraFovY, sensorHeight, sensorDepth, rangeMax));

	currentVideoSubscriberID = videoCapture->currentFrameSubscriptions->Subscribe();
	videoLock.unlock();
	updateLock.unlock();
}

void ReceiverProjector::SetReceiverCapture( ReceiverCapture::Ptr inReceiverCapture)
{
	receiverCapture = inReceiverCapture;

	boost::mutex::scoped_lock updateLock(mMutex);
	boost::mutex::scoped_lock receiverLock(receiverCapture->currentReceiverCaptureSubscriptions->GetMutex());

	currentReceiverCaptureSubscriberID = receiverCapture->currentReceiverCaptureSubscriptions->Subscribe();
	receiverLock.unlock();
	updateLock.unlock();
}

void ReceiverProjector::SetCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr & inCloud)
{
	boost::mutex::scoped_lock updateLock(mMutex);
 	boost::mutex::scoped_lock cloudLock(currentCloudSubscriptions->GetMutex());

	cloud = inCloud;
 
	cloudLock.unlock();
	updateLock.unlock();
}

void ReceiverProjector::CopyCurrentCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr & outCloud, Subscription::SubscriberID inSubscriberID)
{
	boost::mutex::scoped_lock updateLock(currentCloudSubscriptions->GetMutex());
 	pcl::copyPointCloud(*cloud, *outCloud);
	currentCloudSubscriptions->GetNews(inSubscriberID);
	updateLock.unlock();
}


ReceiverChannel::Ptr & ReceiverProjector::GetChannel(int inChannelID) 
{
	return receiverChannels.at(inChannelID);
}

ReceiverChannel::Ptr &ReceiverProjector::AddChannel(ReceiverChannel::Ptr &inChannelPtr) 
{
	inChannelPtr->UpdateReceiverCoordinates(mReceiverCoordinatesPtr);
	receiverChannels.push_back(inChannelPtr);

	return inChannelPtr;
}

void ReceiverProjector::DoThreadLoop()

{
	while (!WasStopped())
    {
		boost::mutex::scoped_lock updateLock(mMutex);
		bool bUpdate = false;

		// Update the video frame before we build the point-cloud
		// if there is a video frame available.

		if (videoCapture != NULL && (videoCapture->currentFrameSubscriptions->HasNews(currentVideoSubscriberID))) 
		{
			// Copy the contents of the working frame, if is is updated
			videoCapture->CopyCurrentFrame(currentFrame, currentVideoSubscriberID);
			bUpdate = true;
		}

		// Update the point-cloud data.
		// Right now, we pace ourselves with the video frames
		if (true) 
		{
			boost::mutex::scoped_lock cloudlock(currentCloudSubscriptions->GetMutex());

			AddDistancesToCloud();
			currentCloudSubscriptions->PutNews();

			cloudlock.unlock();

			// Make sure we leave time for other threads to follow
			boost::this_thread::sleep(boost::posix_time::milliseconds(threadSleepDelay));
		}
	
		updateLock.unlock();


	} // while (!WasStoppped)

}

void ReceiverProjector::DoThreadIteration()

{
	if (!WasStopped())
    {
		boost::mutex::scoped_lock updateLock(mMutex);
		bool bUpdate = false;

		// Update the video frame before we build the point-cloud
		// if there is a video frame available.

		if (videoCapture != NULL && (videoCapture->currentFrameSubscriptions->HasNews(currentVideoSubscriberID))) 
		{
			// Copy the contents of the working frame, if is is updated
			videoCapture->CopyCurrentFrame(currentFrame, currentVideoSubscriberID);
			bUpdate = true;
		}

		// Update the point-cloud data.
		// Right now, we pace ourselves with the video frames
		if (true) 
		{
			boost::mutex::scoped_lock cloudlock(currentCloudSubscriptions->GetMutex());

			AddDistancesToCloud();
			currentCloudSubscriptions->PutNews();

			cloudlock.unlock();
		}
	
		updateLock.unlock();


	} // while (!WasStoppped)

}

void ReceiverProjector::AddDistancesToCloud()
{

	// Prepare a new background that will be drawn upon from the current frame 
	SetBackgroundFrame(GetCurrentFrame());

	// Reset the cloud.  Clear all current points
	ResetCloud();

	int channelQty = receiverChannels.size();
	// For each of the channels, update local information
	for (int channelID = 0; channelID < channelQty; channelID++) 
	{
		GetChannel(channelID)->SetBackgroundPtr(backgroundFrame);
		GetChannel(channelID)->SetColorPtr(currentFrame);
		GetChannel(channelID)->SetCurrentCloud(cloud);
		GetChannel(channelID)->SetDisplayUnderZero(displayUnderZero);
		GetChannel(channelID)->SetDecimation(decimationX, decimationY);
		GetChannel(channelID)->SetSensorHeight(sensorHeight);
		GetChannel(channelID)->SetSensorDepth(sensorDepth);
//		GetChannel(channelID)->SetRangeMax(rangeMax);
		GetChannel(channelID)->SetReceiver(receiverCapture, currentReceiverCaptureSubscriberID );
	}

	for (int i = 0; i < channelQty; i++) 
	{
		GetChannel(i)->AddDistancesToCloud();
	}

	AddBackgroundToCloud();
}


void ReceiverProjector::GetChannelLimits(int channel, double &minX, double &minY, double &minZ, 
	double &maxX, double &maxY, double &maxZ, double &originX, double &originY, double &originZ)
{
	GetChannel(channel)->GetChannelLimits(mReceiverCoordinatesPtr, minX, minY, minZ, maxX, maxY, maxZ, originX, originY, originZ);
}

void ReceiverProjector::GetChannelRect(int channel, int &top, int &left, int &bottom, int &right)
{
	GetChannel(channel)->GetChannelRect(mReceiverCoordinatesPtr, top, left, bottom, right);
}


void ReceiverProjector::GetDisplayColor(int channel, double &outR, double &outG, double&outB)
{
	GetChannel(channel)->GetDisplayColor(outR, outG, outB);
}

void ReceiverProjector::SetBackgroundFrame(ReceiverProjector::FramePtr &inFrame) 
{
	 inFrame->copyTo(*backgroundFrame);
}

void ReceiverProjector::SetCurrentFrame(ReceiverProjector::FramePtr &inFrame) 
{
	inFrame->copyTo(*currentFrame);
}

void ReceiverProjector::AddBackgroundToCloud()

{
	PointXYZRGB newCloudPoint;
	PointWithRange pointWithRange;
	cv::Vec3b cvColor;

	int width = mReceiverCoordinatesPtr->GetWidth();
	int height = mReceiverCoordinatesPtr->GetHeight();

	if (height > backgroundFrame->rows || width > backgroundFrame->cols)
		return;

	for (int i = 0; i < width; i+=decimationX) 
	{
		for (int j = 0; j < height; j+=decimationY)
		{
			// Define point location
			float pointX = ((float) i);
			float pointY = height - (float)j;

			mReceiverCoordinatesPtr->GetXYZFromRange(pointX, pointY, 50.0, newCloudPoint);

			cvColor = backgroundFrame->at<cv::Vec3b>(j, i); // extract color from Mat

			newCloudPoint.b = cvColor[0];
			newCloudPoint.g = cvColor[1];
			newCloudPoint.r = cvColor[2];

			newCloudPoint.a = 255;

			//	Add the point to the array;

			cloud->points.push_back((pcl::PointXYZRGB) newCloudPoint);
		}
	}

	
	// Add a reference point at 0, 0, 0.
	// This is used as baseline for color scheme of the range display.
	mReceiverCoordinatesPtr->GetXYZFromRange(0.0, 0.0, 0.0,newCloudPoint);
	newCloudPoint.b = newCloudPoint.g = newCloudPoint.b = 0;
	newCloudPoint.a = 255;

	//	Add the point to the array;
	cloud->points.push_back((pcl::PointXYZRGB) newCloudPoint);
}

bool ReceiverProjector::SetDisplayUnderZero(bool inDisplayUnderZero)
{
	// Update the displayStyle.

	boost::mutex::scoped_lock cloudlock(currentCloudSubscriptions->GetMutex());	

	displayUnderZero = inDisplayUnderZero;


	cloudlock.unlock();

	return (displayUnderZero);
}


bool ReceiverProjector::GetDisplayUnderZero()
{
	return (displayUnderZero);
}


void ReceiverProjector::SetDecimation(int inDecimationX, int inDecimationY)
{
	// Update the decimation values.

	boost::mutex::scoped_lock cloudlock(currentCloudSubscriptions->GetMutex());	

	decimationX = inDecimationX;
	decimationY = inDecimationY;

	cloudlock.unlock();
}


void ReceiverProjector::GetDecimation(int &outDecimationX, int &outDecimationY)
{
	outDecimationX = decimationX;
	outDecimationY = decimationY;
}

static int entryCount = 0;
static int lockDepth = 0;
void ReceiverProjector::SetSensorHeight(double inSensorHeight)
{

	entryCount++;
	lockDepth++;
	boost::mutex::scoped_lock cloudLock(currentCloudSubscriptions->GetMutex());	

	sensorHeight = inSensorHeight;
	mReceiverCoordinatesPtr->SetSensorHeight(inSensorHeight);
	cloudLock.unlock();
	lockDepth--;
}

void ReceiverProjector::GetSensorHeight(double &outSensorHeight)
{
	outSensorHeight = sensorHeight;
}

void ReceiverProjector::SetSensorDepth(double inSensorDepth)
{

	boost::mutex::scoped_lock cloudLock(currentCloudSubscriptions->GetMutex());	

	sensorHeight = inSensorDepth;
	mReceiverCoordinatesPtr->SetSensorHeight(inSensorDepth);
	cloudLock.unlock();
}


void ReceiverProjector::GetSensorDepth(double &outSensorDepth)
{
	outSensorDepth = sensorDepth;
}

void ReceiverProjector::GetRangeMax(double &outRangeMax)
{
	outRangeMax = rangeMax;
}

void ReceiverProjector::SetRangeMax(double inRangeMax)
{

	boost::mutex::scoped_lock cloudLock(currentCloudSubscriptions->GetMutex());	

	rangeMax = inRangeMax;
#if 0
	// There is no management of the range max value in the coordinates ptr
	mReceiverCoordinatesPtr->SetSensorHeight(inSensorDepth);
#endif
	cloudLock.unlock();
}

void ReceiverProjector::SetCameraFovX(double inFovX)
{

	boost::mutex::scoped_lock cloudLock(currentCloudSubscriptions->GetMutex());	

	cameraFovX = inFovX;
	mReceiverCoordinatesPtr->SetCameraFovX(inFovX);
	cloudLock.unlock();
}

void ReceiverProjector::GetCameraFovY(double &outFovY)
{
	outFovY = cameraFovY;
}

void ReceiverProjector::SetCameraFovY(double inFovY)
{

	boost::mutex::scoped_lock cloudLock(currentCloudSubscriptions->GetMutex());	

	cameraFovY = inFovY;
	mReceiverCoordinatesPtr->SetCameraFovY(inFovY);
	cloudLock.unlock();
}

void ReceiverProjector::GetCameraFovX(double &outFovX)
{
	outFovX = cameraFovX;
}

void ReceiverProjector::ResetCloud()
{
	size_t cloudSize = frameWidth*frameHeight*2;

	// Reset point cloud
	cloud->points.clear();
	cloud->points.reserve(cloudSize);

	// Define point cloud parameters
	cloud->width = static_cast<uint32_t>(frameWidth);
	cloud->height = static_cast<uint32_t>(frameHeight);
	cloud->is_dense = true;
}


ReceiverCoordinates::ReceiverCoordinates(const int inWidth, const int inHeight, const double inFovX, const double inFovY, 
	const double inSensorHeight, double inSensorDepth, double inRangeMax):
pcl::RangeImage(),
width(inWidth),
height(inHeight),
fovX(inFovX),
fovY(inFovY),
sensorHeight(inSensorHeight),
sensorDepth(inSensorDepth),
rangeMax(inRangeMax)
{
   // We now want to create a range image from the above point cloud, with an angular resolution
   // that corresponds to the video resolution
  float angularResolutionX = float (inFovX / width);
  float angularResolutionY = float (inFovY / height);
  float maxAngleWidth     = (float) inFovX;  // 180.0 degree in radians
  float maxAngleHeight    = (float) inFovY;  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinateFrame = pcl::RangeImage::CAMERA_FRAME;

  pcl::RangeImage::createEmpty(angularResolutionX, angularResolutionY, sensorPose, coordinateFrame, maxAngleWidth,  maxAngleHeight);
}

void ReceiverCoordinates::GetXYZFromRange(float inPointX, float inPointY, float inPointZ, 
									PointXYZRGB &ioCloudPoint)
{
	float angle_x, angle_y;
	getAnglesFromImagePoint(inPointX, inPointY, angle_x, angle_y);
	float cosY = cosf (angle_y);

	// We reverse the X coordinate to make this a left-handed projection
	// the PCL viewer uses a right-handed coordinate system.

	ioCloudPoint.x = -(inPointZ * sinf (angle_x) * cosY);
	ioCloudPoint.y = inPointZ * sinf (angle_y);
	ioCloudPoint.z = inPointZ * cosf (angle_x)*cosY;

#if 0
	// We offset the values to the sensor offset position
	ioCloudPoint.y += sensorHeight;
	ioCloudPoint.z += sensorDepth;
#else
	// We offset the values to the sensor offset position
	// depth is compensated for at receiver level, so we undo the offset here. 
	ioCloudPoint.z -= sensorDepth;
#endif
}

	/** \brief Sets   horizontal camera FOV.
      * \param[in] cameraFovX horizontal FOV of camera in radians.
	  */
void ReceiverCoordinates::SetCameraFovX(double inCameraFovX)
{
	fovX = inCameraFovX;
}

	/** \brief Sets   verticsl camera FOV.
      * \param[in] cameraFovY vertical FOV of camera in radians.
	      */
void ReceiverCoordinates::SetCameraFovY(double inCameraFovY)
{
	fovY = inCameraFovY;
}

void ReceiverCoordinates::SetSensorHeight(double inSensorHeight)
{
	sensorHeight = inSensorHeight;
}

void ReceiverCoordinates::GetSensorHeight(double &outSensorHeight)
{
	outSensorHeight = sensorHeight;
}

void ReceiverCoordinates::SetSensorDepth(double inSensorDepth)
{
	sensorHeight = inSensorDepth;
}

void ReceiverCoordinates::GetSensorDepth(double &outSensorDepth)
{
	outSensorDepth = sensorDepth;
}



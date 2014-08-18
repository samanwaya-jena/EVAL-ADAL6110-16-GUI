#ifndef _VIDEOVIEWER_H
#define _VIDEOVIEWER_H

#define CV_NO_BACKWARD_COMPATIBILITY


#include <iostream>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

using namespace std;

#include "sensor.h"
#include "ReceiverCapture.h"

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui_c.h"
//#include "opencv2/highgui/highgui.hpp"

namespace awl
{
/** \brief Threaded Video Display class that also overlays decorations based on lidar acquisition.
  *        The video display thread is based on OpenCV.
  *		   The VideoViewer takes a VideoCaptureDevice as input.
  *		   The VideoViewer also takes a ReceiverProjector as input
  * \author Jean-Yves Deschênes
  */
class VideoViewer
{
public:
	typedef boost::shared_ptr<cv::Mat> FramePtr;
	
	typedef boost::shared_ptr<VideoViewer> Ptr;
	typedef boost::shared_ptr<VideoViewer const> ConstPtr;
	typedef boost::container::vector<VideoViewer::Ptr> List;
	typedef VideoViewer::List *ListPtr;

	/** \brief Video Viewer constructor.
      * \param[in] inCameraName string used to identify camera and used as window title
      * \param[in] inVideoCapture videoCaptureDevice we feed image from.
      * \param[in] inreceiverCapture receiver Capture device that retuns us witha actual dinstance info.
	  * \param[in] inProjector receiverProjector that supplies us with range info
      */
	VideoViewer::VideoViewer(std::string inCameraName, VideoCapture::Ptr inVideoCapture, ReceiverCapture::Ptr inReceiverCapture, ReceiverProjector::Ptr inProjector);

	/** \brief Video Viewer destructor. Insures that the viewer thread is Stopped()
      */
	virtual VideoViewer::~VideoViewer();

	/** \brief Start the video display thread
      */
	void  Go(); 

	/** \brief Stop the video display thread
      */
	void  Stop(); 

	/** \brief Return the video display thread status
      * \return true if the video display thread is stoppped.
      */
	bool  WasStopped();

	/** \brief Return the video frame rate.
      * \return video acquisition frame rate in FPS.
      */
	double GetFrameRate() {return(frameRate);};

	/** \brief Return the video frame width.
      * \return videoframe width in pixels.
      */
	int	   GetFrameWidth() {return(frameWidth);}

	/** \brief Return the video frame width.
      * \return videoframe height in pixels.
      */
	int	   GetFrameHeight() {return(frameHeight);}


	/** \brief Return the image scale.
      * \return image scaling factor.
      */
	double GetScale() { return(scale);}


	/** \brief Return the  horizontal camera FOV.
      * \return horizontal camera FOV in radians.
      */
	double GetCameraFovWidth() {return(cameraFovWidth);}

	/** \brief Return the  vertical camera FOV.
      * \return vertical camera FOV in radians.
      */
	double GetCameraFovHeight() {return(cameraFovHeight);}

	/** \brief Return the work frame.
      * \return a boost shared pointer to the video image.
      */
	VideoViewer::FramePtr GetWorkFrame() {return(workFrame);}

#if 1	
	/** \brief Copy the work frame to the targetFrame.  The current frame is thread-locked during transfer
      * \note Locking of the target frame is under the responsibility of the calling thread.
      */
	void CopyWorkFrame(VideoViewer::FramePtr targetFrame);
#endif
			
	/** \brief Return the mutex of the video  viewer objects.
      * \return a boost mutex for the object.
      */
	boost::mutex& GetMutex() {return (mMutex);} 

	/** \brief Modify the videoCapture source.  Thread safe
	           Update the internal video format description variables to reflect the new source.
      * \param[in] inVideoCapture videoCaptureDevice we feed image from
      */
	
	void SetVideoCapture(VideoCapture::Ptr inVideoCapture);

	/** \brief Modify the receiverCapture source.  Thread safe
      * \param[in] inReceiverCapture receiverCaptureDevice we feed distance from.
      */
	
	void SetReceiverCapture(ReceiverCapture::Ptr inReceiverCapture);


	/** \brief Modify the receiverProjector that feeds us with data.  Thread safe
      * \param[in] inProjector receiverProjector that supplies us with range info
	  */
	void VideoViewer::SetReceiverProjector(ReceiverProjector::Ptr inProjector);

	/** \brief Move the window at position left, top.
	  * \remarks implemented for compatibility  with Qt.
      */
	void move(int left, int top); 

protected:
	/** \brief Perform the video display thread loop
      */
	void  DoThreadLoop();

	/** \brief Resize the window to fit the screen.
	  * \remarks implemented for compatibility  with Qt.
      */
	void SizeWindow();

	/** \brief Set the window Icon to the icon specified in the configuration file.
      */
	void SetWindowIcon();

	void DisplayReceiverValues(VideoCapture::FramePtr &targetFrame);
	void DisplayTarget(VideoCapture::FramePtr &targetFrame, int channelID,  Detection::Ptr &detection);

protected:
	void GetDetectionColors(const Detection::Ptr &detection, cv::Vec3b &colorEnhance, cv::Vec3b &colorDehance, int &iWidth);
	void GetChannelRect(Detection::Ptr &detection, CvPoint &topLeft, CvPoint &topRight, CvPoint &bottomLeft, CvPoint &bottomRight);
    void DrawDetectionLine(VideoCapture::FramePtr &targetFrame, const CvPoint &startPoint, const CvPoint &endPoint,  const cv::Vec3b &colorEnhance, const cv::Vec3b &colorDehance, int iWidth);

protected:
	
    /** \brief Local flag indicating a request for termination of thread. */
	volatile bool mStopRequested;

	/** \brief Local flag indicating the termination of thread. */
	volatile bool mThreadExited;

    /** \brief Video acquisition thread . */
    boost::shared_ptr<boost::thread> mThread;

	/** \brief Data sharing mutex. */
    boost::mutex mMutex;

	/** \brief indicator of the display window.  True if window exists */
	bool bWindowCreated;

	/** \brief Sunscriber identification to the video feed. */
	Subscription::SubscriberID currentVideoSubscriberID;
	
	/** \brief Sunscriber identification to the range data feed. */
	Subscription::SubscriberID currentReceiverSubscriberID;

    /** \brief Current video frame width. */
	int frameWidth;

    /** \brief Current video frame height. */
	int frameHeight;

	/** \brief Current video stram frame rate in FPS.  For still video, it defaults to 33FPS. */
	double frameRate;

	/** \brief Video scaling factor, that can be set on the command line. */
	double scale;

	/** \brief Horizontal field of view of the camera. */
	float cameraFovWidth;
	/** \brief Vertical field of view of the camera. */
	float cameraFovHeight;

	/** \brief Copy of the captured image. Used for work and transformation. */
    VideoViewer::FramePtr workFrame;

	/** \brief Copy of the work image. Used for display (we are double-buffering). */
    VideoViewer::FramePtr displayFrame;

	/** \brief An image as loaded on file stream.Captured image. Should be for reference purposes only. */
	cv::Mat image;

	/** \brief Camera name, also used as the window title */
	std::string cameraName;

	/** \brief video capture device that supplies the video data */
	VideoCapture::Ptr videoCapture; 

	/** \brief receiver capture device that supplies the range data */
	ReceiverCapture::Ptr receiverCapture; 

	/** \brief receiver projector that supplies the range data */
	ReceiverProjector::Ptr projector;
}; // VideoViewer

} // namespace awl


#endif //_CAPTUREVIEWER_H
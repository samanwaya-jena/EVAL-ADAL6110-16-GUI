#ifndef _VIDEOVIEWER_H
#define _VIDEOVIEWER_H

#define CV_NO_BACKWARD_COMPATIBILITY


#include <iostream>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

using namespace std;

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"

#include "Tracker.h"

namespace awl
{
/** \brief Threaded Video Display class that also overlays decorations based on lidar acquisition.
  *        The video display thread is based on OpenCV.
  *		   The VideoViewer takes a VideoCaptureDevice as input for image information.
  * \author Jean-Yves Deschênes
  */
class VideoViewer
{
public:
	typedef boost::shared_ptr<cv::Mat> FramePtr;
	typedef boost::container::vector<FramePtr> FrameList;
	
	typedef boost::shared_ptr<VideoViewer> Ptr;
	typedef boost::shared_ptr<VideoViewer const> ConstPtr;
	typedef boost::container::vector<VideoViewer::Ptr> List;
	typedef VideoViewer::List *ListPtr;

	/** \brief Video Viewer constructor.
      * \param[in] inCameraName string used to identify camera and used as window title
      * \param[in] inVideoCapture videoCaptureDevice we feed image from.
	  * \param[in] inProjector receiverProjector that supplies us with range info
      */
	VideoViewer::VideoViewer(std::string inCameraName, VideoCapture::Ptr inVideoCapture);

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

	/** \brief Return the video frame width.
      * \return videoframe width in pixels.
      */
	int	   GetFrameWidth() {return(frameWidth);}

	/** \brief Return the video frame width.
      * \return videoframe height in pixels.
      */
	int	   GetFrameHeight() {return(frameHeight);}

	/** \brief Return the  horizontal camera FOV.
      * \return horizontal camera FOV in radians.
      */
	double GetCameraFovWidth() {return(cameraFovWidth);}

	/** \brief Return the  vertical camera FOV.
      * \return vertical camera FOV in radians.
      */
	double GetCameraFovHeight() {return(cameraFovHeight);}
			
	/** \brief Return the mutex of the video  viewer objects.
      * \return a boost mutex for the object.
      */
	boost::mutex& GetMutex() {return (mMutex);} 

	/** \brief Modify the videoCapture source.  Thread safe
	           Update the internal video format description variables to reflect the new source.
      * \param[in] inVideoCapture videoCaptureDevice we feed image from
      */
	
	void SetVideoCapture(VideoCapture::Ptr inVideoCapture);

	/** \brief Move the window at position left, top.
	  * \remarks implemented for compatibility  with Qt (hence name convention).
      */
	void move(int left, int top); 

	/** \brief Update the detection positions.
	  * \remarks Udate is thread safe.
      */
	void slotDetectionDataChanged(const Detection::Vector & data);

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

	void DisplayReceiverValues(VideoCapture::FramePtr &sourceFame, VideoCapture::FramePtr &targetFrame, const Detection::Vector & data);


	void DisplayTarget(VideoCapture::FramePtr &sourceFame, VideoCapture::FramePtr &targetFrame, const Detection::Ptr &detection);

protected:
	void GetDetectionColors(const Detection::Ptr &detection, cv::Vec3b &colorEnhance, cv::Vec3b &colorDehance, int &iThickness);
	void GetChannelRect(const Detection::Ptr &detection, CvPoint &topLeft, CvPoint &topRight, CvPoint &bottomLeft, CvPoint &bottomRight);

	/** \brief Draw an "enhanced" detection line over the target frame.
	           We take the original background from the source frame.  
			   This way, we avoid a "pile-up" of enhancements that can be confusing.
			   Since the detections are sorted in order of threatLevel, this insures that the most menacing threats 
			   are always displayed correctly.
      */
    void DrawDetectionLine(VideoCapture::FramePtr &sourceFame, VideoCapture::FramePtr &targetFrame, 
						 const CvPoint &startPoint, const CvPoint &endPoint,  
						 const cv::Vec3b &colorEnhance, const cv::Vec3b &colorDehance, 
						 int iWidth, int iHeight);
 
protected:
	
    /** \brief Local flag indicating a request for termination of thread. */
	volatile bool mStopRequested;

	/** \brief Local flag indicating the termination of thread. */
	volatile bool mThreadExited;

    /** \brief Video acquisition thread . */
    boost::shared_ptr<boost::thread> mThread;

	/** \brief Data sharing mutex. */
    boost::mutex mMutex;

	/** \brief Sunscriber identification to the video feed. */
	Subscription::SubscriberID currentVideoSubscriberID;
	
	/** \brief Sunscriber identification to the range data feed. */
	Subscription::SubscriberID currentReceiverSubscriberID;

    /** \brief Current video frame width. */
	int frameWidth;

    /** \brief Current video frame height. */
	int frameHeight;

	/** \brief Horizontal field of view of the camera. */
	float cameraFovWidth;

	/** \brief Vertical field of view of the camera. */
	float cameraFovHeight;

	/** \brief Copy of the captured image. */
    VideoViewer::FramePtr cameraFrame;

	/** \brief Frames used for painting and display. 
	           As we are double-buffering, the displayed frame is
			   never the one that is being painted on.
	*/
	FrameList workFrames;

	/** \brief Index of frame currently being used for work, previous to display
	*/
	int currentWorkFrame;

	/** \brief Camera name, also used as the window title */
	std::string cameraName;

	/** \brief video capture device that supplies the video data */
	VideoCapture::Ptr videoCapture; 

	/** \brief Time the object was created.  Used to calculate flashing rates */
	boost::posix_time::ptime startTime;

	/** \brief Vector containing the detections to be displayed */
   Detection::Vector detectionData;
}; // VideoViewer

} // namespace awl


#endif //_CAPTUREVIEWER_H
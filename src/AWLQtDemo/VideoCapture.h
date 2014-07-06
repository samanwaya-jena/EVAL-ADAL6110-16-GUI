#ifndef VIDEOCAPTURE_H
#define VIDEOCAPTURE_H

#define CV_NO_BACKWARD_COMPATIBILITY

#include <fstream>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

using namespace std;

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include "subscription.h"


namespace awl
{


/** \brief Threaded Video Capture class.
  *        The video capture thread is based on OpenCV.
  * \author Jean-Yves Deschênes
  */
class VideoCapture
{
public:
	typedef boost::shared_ptr<cv::Mat> FramePtr;
	
	typedef boost::shared_ptr<VideoCapture> Ptr;
	typedef boost::shared_ptr<VideoCapture const> ConstPtr;

	/** \brief Video Capture constructor.
      * \param[in] argc command-line argument count
      * \param[in] argv command line argument strings
      */
	VideoCapture(int argc, char** argv);

	/** \brief Video Capture destructor.  Insures that the thread is stopped.

      */
	virtual ~VideoCapture();

	/** \brief Start the video acquisition thread
      */
	void  Go(); 

	/** \brief Stop the video acquisition thread
      */
	void  Stop(); 

	/** \brief Return the video acquisition thread status
      * \return true if the video acquisition thread is stoppped.
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

	/** \brief Return the current frame. 
      * \return a boost shared pointer to the current frame.
      */
	VideoCapture::FramePtr GetCurrentFrame();

	/** \brief Copy the current frame to the targetFrame.  The current frame is thread-locked during transfer
      * \param[out] targetFrame pointer to the target frame that will get copied to.
      * \note Locking of the target frame is under the responsibility of the calling thread.
      */
	void CopyCurrentFrame(VideoCapture::FramePtr targetFrame, Subscription::SubscriberID subscriberID = -1);

	/** \brief A public subscription checkpoint infrastructure for the currentFrame.
      */
	Subscription::Ptr currentFrameSubscriptions;	

	/** \brief Do a single iteration of the video capture loop.
	           (for calls from within the main event loop)
      */
	void  DoThreadIteration();


protected:
	/** \brief Return the video acquisition thread status
      * \return true if the video acquisition thread is stoppped.
      */
	void  DoThreadLoop();

	/** \brief Gets the mutex used to protect the thread
      * \return mutex used to protect the thread activities
      */
	boost::mutex& GetMutex() {return (mMutex);};


protected:
	
    /** \brief Local flag indicating a request for termination */
	volatile bool mStopRequested;

	    /** \brief Local flag indicating the termination of thread loop function. */
	volatile bool mThreadExited;

    /** \brief Video acquisition thread . */
    boost::shared_ptr<boost::thread> mThread;

	/**\brief thread mutex*/
	boost::mutex mMutex;


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

	/** \brief Video capture device. */
	CvCapture* capture;

	/** \brief Captured image. Should be for reference purposes. */
    boost::shared_ptr<cv::Mat> currentFrame; 

	/** \brief Captured image-directly from CVCapture. Should be for reference purposes. */
    boost::shared_ptr<cv::Mat> bufferFrame; 

	/** \brief An image as loaded on file stream.Captured image. Should be for reference purposes. */
	cv::Mat image;
}; // VideoCapture

} // namespace awl

#endif //_VIDEOCAPTURE_H
#ifndef APVIDEOCAPTURE_H
#define APVIDEOCAPTURE_H

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

#define CV_NO_BACKWARD_COMPATIBILITY

#include <fstream>

#include <boost/property_tree/ptree.hpp>
using namespace std;

#include <apbase.h>

#include "Publisher.h"
#include "ThreadedWorker.h"
#include "CoordinateSystem.h"
#include "AP.h"


namespace awl
{


/** \brief Threaded Video Capture class.
  *        The video capture thread is based on OpenCV.
  * \author Jean-Yves Deschênes
  */
class APVideoCapture: public ThreadedWorker, public Publisher
{
public:
	typedef boost::shared_ptr<APFrame> FramePtr;
	
	typedef boost::shared_ptr<APVideoCapture> Ptr;
	typedef boost::shared_ptr<APVideoCapture const> ConstPtr;
	typedef boost::container::vector<APVideoCapture::Ptr> List;
	typedef APVideoCapture::List *ListPtr;

	/** \brief Video Capture constructor.
      * \param[in] inCameraID index of the camera in the configuration parameters.
      * \param[in] argc command-line argument count.  Used in OpenCV cvInit() call.
      * \param[in] argv command line argument strings. Used in OpenCV cvInit() call.
	  * \param[in] propTree configuration property tree. Corresponds to configuration file contents.
      */
	APVideoCapture(int inCameraID, int argc, char** argv, boost::property_tree::ptree &propTree);

	/** \brief Video Capture destructor.  Insures that the thread is stopped.

      */
	virtual ~APVideoCapture();

	/** \brief Start the video acquisition thread
      */
	void  Go(); 

	/** \brief Return the camera ID.
      * \return cameraID (also is the index of the camera in our structures).
      */
	int GetCameraID() {return(cameraID);};


	/** \brief Return the video frame rate.
      * \return video acquisition frame rate in FPS.
      */
	float GetFrameRate() {return(frameRate);};

	/** \brief Copy the current frame to the targetFrame.  The current frame is thread-locked during transfer
      * \param[out] targetFrame pointer to the target frame that will get copied to.
      * \note Locking of the target frame is under the responsibility of the calling thread.
      */
	void CopyCurrentFrame(APVideoCapture::FramePtr targetFrame, Publisher::SubscriberID subscriberID = -1);

	/** \brief calibration information. */
	CameraCalibration calibration;

protected:
	/** \brief Return the video acquisition thread status
      * \return true if the video acquisition thread is stoppped.
      */
	void  DoThreadLoop();

	/** \brief Do a single iteration of the video capture loop.
	           (for calls from within the main event loop)
      */
	void  DoThreadIteration();

	/** \brief List all of the available cameras in file "CameraList.txt".
	           For debug purposes onlyle, the image is assumed to be 640x480 @ 30FPS.
  */
	void ListCameras();


	/** \brief With the configuration variable initialized, try to open the camera channel.
	           Set the calibration frameWidthInPixels, frameHeightInPixels and frameRate according 
			   to the information returned by the camera.
			   If no camera is available, the image is assumed to be 640x480 @ 30FPS.
		\return True if the camera is found and opened.  False otherwise.
	  */
	bool OpenCamera();

	/** \brief Read the configuration from configuration file
      */
	bool ReadConfigFromPropTree(boost::property_tree::ptree &propTree);

protected:

	AP_HANDLE apbase;

	unsigned char *rgbBuffer;

	/** \brief camera name */
	std::string sCameraName;

	/** \brief Index of the camera in the configuration parameter list */
	int cameraID;
	
    /** \brief Local flag indicating a request for termination */
	volatile bool mStopRequested;

	/** \brief Current video stram frame rate in FPS.  For still video, it defaults to 33FPS. */
	float frameRate;

	/** \brief Set cameraFlip to true for 180 degree rotation of the camera image. */
	bool bCameraFlip;

	/** \brief Captured image. Should be for reference purposes. */
    	APFrame currentFrame; 

	/** \brief Captured image-directly from CVCapture. Should be for reference purposes. */
    	APFrame bufferFrame; 
}; // APVideoCapture

} // namespace awl

#endif //_APVIDEOCAPTURE_H

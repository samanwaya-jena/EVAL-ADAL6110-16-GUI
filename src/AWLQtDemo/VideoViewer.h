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

#ifndef _VIDEOVIEWER_H
#define _VIDEOVIEWER_H

#define CV_NO_BACKWARD_COMPATIBILITY


#include <iostream>


#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>

#include "VideoCapture.h"
#include "DetectionStruct.h"

#include <QWidget>
#include <QFrame>
#include <QImage>
#include <QAction>
#include <QActionGroup>

namespace awl
{
/** \brief The VideoViewer class displays an image taken from a VideoCapture
           and overlays "decorations" based on the Detections provided.
  *        The video display class is a non-threaded class based on OpenCV.
  * \author Jean-Yves Deschênes
  */
class VideoViewer: public QFrame

{
	Q_OBJECT
public:
	/** \brief Video Viewer constructor.
      * \param[in] inCameraName string used to identify camera and used as window title

	  * \param[in] inVideoCapture videoCaptureDevice we feed image from.
	  * \param[in] inProjector receiverProjector that supplies us with range info
      */
	explicit VideoViewer(std::string inCameraName, VideoCapture::Ptr inVideoCapture, QWidget *parentWidget=0);

	typedef boost::shared_ptr<cv::Mat> FramePtr;
	typedef boost::container::vector<FramePtr> FrameList;
	
	typedef boost::shared_ptr<VideoViewer> Ptr;
	typedef boost::shared_ptr<VideoViewer const> ConstPtr;
	typedef boost::container::vector<VideoViewer::Ptr> List;
	typedef VideoViewer::List *ListPtr;


	/** \brief Video Viewer destructor. Insures that the viewer is Stopped()
      */
	virtual ~VideoViewer();

	/** \brief Prepare the video display for the display loop.  Creates the display window.
      */
	void  Go(); 

	/** \brief Stop the video display. Destroys the display window.
      */
	void  Stop(); 

	/** \brief Return the video display status
      * \return true if the video display is stoppped (the display Window is not available).
      */
	bool  WasStopped();

	QSize sizeHint() const;
	QSize minimumSizeHint() const;
	QSize maximumSizeHint() const;

public slots:
	void ShowContextMenu(const QPoint& pos);
	/** \brief Update the detection positions.
      */
	void slotDetectionDataChanged(const SensorCoreScope::Detection::Vector & data);
	void slotImageChanged();
	void slotVideoOptionsChangedAction();


protected:
	/** \brief Modify the videoCapture source. 
			   Update the internal video format description variables to reflect the new source.
      * \param[in] inVideoCapture videoCaptureDevice we feed the image from
      */
	
	void SetVideoCapture(VideoCapture::Ptr inVideoCapture);


	/** \brief Resize the window to fit the screen.
      */
	void resizeEvent(QResizeEvent * /*event*/);

	void DisplayReceiverValues(QImage &sourceFame, QPainter& painter, const SensorCoreScope::Detection::Vector & data);

protected:
	void paintEvent(QPaintEvent* /*event*/); 

	void GetDetectionColors(const SensorCoreScope::Detection::Ptr &detection, QColor &colorEnhance,  int &iThickness);
	/** \brief get the four corners of the specified Detection's channel FOV, as projected at the Detection distance
	  *         in the camera plane.
	  *\return Returns false if all the points in the projection are behind the camera plane. Returns true
	  *        if at least one of the points is in front of the camera.
      */

	bool GetChannelRect(const SensorCoreScope::Detection::Ptr &detection, CvPoint &topLeft, CvPoint &topRight, CvPoint &bottomLeft, CvPoint &bottomRight);

	

	void DisplayTarget(QImage &sourceFame, QPainter& painter, const SensorCoreScope::Detection::Ptr &detection);

	void DisplayCrossHairs(QImage &sourceFame, QPainter& painter);

	/** \brief Draw an "enhanced" detection line over the target frame.
	           We take the original background from the source frame.  
			   This way, we avoid a "pile-up" of enhancements that can be confusing.
			   Since the detections are sorted in order of threatLevel, this insures that the most menacing threats 
			   are always displayed correctly.
      */
    void DrawDetectionLine(QImage &sourceFame, QPainter& painter,
						 const CvPoint &startPoint, const CvPoint &endPoint,  
						 QColor &colorEnhance,
						 int penWidth);

	void DrawHorizontalTicks(QImage &sourceFrame, QPainter& painter, float tickAngle, float tickLength, int thickness);
	void DrawVerticalTicks(QImage &sourceFrame,  QPainter& painter, float tickAngle, float tickLength, int thickness);
	void DrawChannelOutlines(QImage &sourceFrame, QPainter &painter);
	void DrawVideoText(QImage &sourceFrame, QPainter &painter, const QRect &textRect, const QString &text);
protected:
	/** \brief Copy of the last Camera image in OpenCV */
	cv::Mat lastValidFrame;

	/** \brief Copy of the captured image. */
    QImage qtCameraFrame;

	/** \brief Copy of the captured image, but with the image enhancements */
    QImage qtEnhancedFrame;


	/** \brief Camera name, also used as the window title */
	std::string cameraName;

	/** \brief video capture device that supplies the video data */
	VideoCapture::Ptr videoCapture; 

	/** \brief Sunscriber identification to the video feed. */
	SensorCoreScope::Publisher::SubscriberID currentVideoSubscriberID;

	/** \brief Time the object was created.  Used to calculate flashing rates */
	boost::posix_time::ptime startTime;

	/** \brief Vector containing the detections to be displayed */
	SensorCoreScope::Detection::Vector detectionData;

   /** \brief Screen scale factor in Qt Widget */
   float displayScaleFactor;

   /** displayCrosshair, as initially spaecified in configuration file */
   bool bDisplayCrosshair;

   /** displayTime, as initially spaecified in configuration file */
   bool bDisplayTime;

private:
	void createAction();

	QActionGroup* groupVideoOptions;
	QAction* crosshairOptionAction;
	QAction *timeOptionAction;
}; // VideoViewer

} // namespace awl


#endif //_CAPTUREVIEWER_H

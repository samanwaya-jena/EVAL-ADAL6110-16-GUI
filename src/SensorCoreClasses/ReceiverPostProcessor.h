/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the SensorCoreClasses library of the
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
#ifndef SENSORCORE_RECEIVER_POSTPROCESSOR_H
#define SENSORCORE_RECEIVER_POSTPROCESSOR_H

#include <boost/shared_ptr.hpp>
#include <boost/container/vector.hpp>
#include "SensorCoreClassesGlobal.h"
#include "DetectionStruct.h"
#include "ReceiverCapture.h"

SENSORCORE_BEGIN_NAMESPACE


	/** \brief   The ReceiverPostProcessor is used to read the detection and track information acquired from the 
	  *          Receivers and return "enhanced" detection data.
	  *	\remarks The enhanced detection data add information that the receivers cannot provide due to
	  *		   implementation limitations.
	  *        Threat level is calculated.
	  *        All supplemental geometries (relative positions) are calculated here once, to save on processing time.
     */
class ReceiverPostProcessor

{
public:
	/** \brief Constructor
	*/
	ReceiverPostProcessor();

	/** \brief Build processsed detections from the current track set.
	 *Only detections from tracks that are complete (no errors are kept)
	 * Detections from tracks where distance is oustide the range are not kept.
     * \return Return true if all tracks have been intepreted correctly.
	 *  Return false if we have found incomplete tracks or invalid voxel info.
	*/
	bool GetEnhancedDetectionsFromFrame(ReceiverCapture::Ptr receiver, FrameID inFrameID,  Publisher::SubscriberID inSubscriberID, Detection::Vector &detectionBuffer);

protected:

	/** \brief Complete the track info that was not processed by the receiver Module.
	  * \return Return true if all tracks have been intepreted correctly.
	  *Return false if we have found incomplete tracks or invalid voxel info.
	*/
	bool CompleteTrackInfo(SensorFrame::Ptr currentFrame);

	/** \brief Rebuild processsed detections from the current track set.
	 *Only detections from tracks that are complete (no errors are kept) 
	 * Detections from tracks where distance is oustide the range are not kept. 
	 * \return Return true if all tracks have been intepreted correctly.
	 Return false if we have found incomplete tracks or invalid voxel info.
	*/
	bool BuildEnhancedDetectionsFromTracks(ReceiverCapture::Ptr receiver, SensorFrame::Ptr currentFrame, Detection::Vector &outDetections);

	/** \brief Predict the time to collision (distance = 0) between sensor and obstacle,  
	  *        given current distance and speed, assuming constant deceleration.
 	  * \param[in] currentDistance to obstacle, in meters
 	  * \param[in] relativeSpeed relative speed between sensor and obstacle, in meters/sec
	  * \return Predicted time to collision, in seconds.
	  * \remarks This corresponds to TTC1 measure defined in:
	  *          Yizhen Zhang, Erik K. Antonsson and Karl Grote: A New Threat Assessment Measure for Collision Avoidance Systems
      */
	float PredictTimeToCollisionConstant (float currentDistance, float relativeSpeed);

	/** \brief Predict the relative distance of an obstacle after a certain time delay,  
	  *        given current distance speed and  deceleration.
 	  * \param[in] currentDistance to obstacle, in meters
 	  * \param[in] relativeSpeed relative speed between sensor and obstacle, in meters/sec
 	  * \param[in] currentAcceleration current acceleration between sensor and obstacle, in meters/sec2
	  * \param[in] timeDelay timeDelay (in sec) for which updated position is calculated
	  * \return Predicted relative distance of the obstacle, , in m/s2.
	  * \remarks  arguments use acceleration, not deceleration! Positive acceleration means object is moving away.
      */
		float PredictDistance(float currentDistance, float relativeSpeed, float acceleration, float time);

	/** \brief Calculate the required accceleration to get to zero speed at the specified distance, 
	  *        given currentSpeed and current deceleration.
 	  * \param[in] currentDistance to obstacle, in meters
 	  * \param[in] relativeSpeed relative speed between sensor and obstacle, in meters/sec
 	  * \param[in] currentDeceleration current deceleration between sensor and obstacle, in meters/sec2
	  * \return Current required acceleration, in m/s2. Should be a negative value for objects movig towards sensor.
	  * \remarks  arguments use acceleration, not deceleration! Positive acceleration means object is moving away.
      */
	float CalculateAccelerationToStop(float currentDistance, float relativeSpeed, float currentAcceleration);

	/** \brief Calculate the predicted time to collision to an obstacle to get to zero speed at the specified distance, 
	  *        given currentSpeed and current deceleration, projected in deltaTime seconds.
 	  * \param[in] distance to obstacle, in meters
 	  * \param[in] speed relative speed between sensor and obstacle, in meters/sec
 	  * \param[in] acceleration current acceleration between sensor and obstacle, in meters/sec2
 	  * \param[in] deltaTime time at which the prediction is claculated for (in seconds from current time)
	  * \return Time to collision predicted at (currentTime + deltaTime)
	  * \remarks  arguments use acceleration, not deceleration! Positive acceleration means object is moving away.
      */
	float PredictTTC(float /*distance*/, float speed, float acceleration, float deltaTime);


};

SENSORCORE_END_NAMESPACE
#endif // SENSORCORE_RECEIVER_POSTPROCESSOR_H

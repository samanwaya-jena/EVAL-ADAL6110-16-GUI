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
#ifndef SENSOR_COORD_H
#define SENSOR_COORD_H

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/container/vector.hpp>
#include <boost/container/deque.hpp>

#include <boost/property_tree/ptree.hpp>


#include <cmath>

#include "SensorCoreClassesGlobal.h"
#include "CoordinateSystem.h"

SENSORCORE_BEGIN_NAMESPACE

/** \brief The SensorCoordinates class provides utility methods to perform  application-specific coordinates conversions.
  * \author Jean-Yves Deschênes
  */

class SensorCoordinates
{
public:
	static SensorCoordinates *InitCoordinates();
	static SensorCoordinates *GetGlobalCoordinates();
	static TransformationNode::Ptr GetFirstNode();
	static TransformationNode::List GetReceivers();
	static TransformationNode::Ptr GetReceiver(int receiverID);
	static TransformationNode::Ptr GetChannel(int receiverID, int channelID);
	static TransformationNode::List GetCameras();
	static TransformationNode::Ptr GetCamera(int cameraID);
	

	// Constructor
	SensorCoordinates();
	SensorCoordinates(boost::property_tree::ptree &propTree);
	bool BuildCoordinatesFromSettings(boost::property_tree::ptree &propTree);

    
	/** \brief convert sensor coordinates to image (XY) coordinates of the specified camera
	 *  \returns Returns true if the point is in front of the camera.  Returns false for points behind the camera.
	 * 
	 */
	static bool SensorToCameraXY(int receiverID, int channelID, int cameraID, const CameraCalibration &camera, const SphericalCoord &sensorCoord, int &cameraX, int &cameraY);

	/** \brief convert world coordinates to image (XY) coordinates of the specified camera
	 *  \returns Returns true if the point is in front of the camera.  Returns false for points behind the camera.
	 * 
	 */
	static bool WorldToCameraXY(int cameraID, const CameraCalibration &camera, const CartesianCoord &worldCoord, int &cameraX, int &cameraY);

	static RelativePosition GetReceiverPosition(int receiverID);
	static RelativePosition GetChannelPosition(int receiverID, int channelID);
	static RelativePosition GetCameraPosition(int cameraID);

	static RelativePosition SetReceiverPosition(int receiverID, const RelativePosition &inPosition);
	static RelativePosition SetChannelPosition(int receiverID, int channelID, const RelativePosition &inPosition);
	static RelativePosition SetCameraPosition(int cameraID, const RelativePosition &inPosition);


protected:
	static  TransformationNode::Ptr GetGeometryFromPropertyNode(boost::property_tree::ptree &propTree);
	static TransformationNode::Ptr GetGeometryFromChannelPropertyNode(boost::property_tree::ptree &channelNode);

protected:
	static SensorCoordinates *globalCoordinates;
	TransformationNode::Ptr	firstNode;

	TransformationNode::List receivers;
	TransformationNode::List cameras;
};
SENSORCORE_END_NAMESPACE
#endif // SENSOR_COORD_H
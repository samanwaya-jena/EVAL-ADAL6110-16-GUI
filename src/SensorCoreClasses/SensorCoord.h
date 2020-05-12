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

	/** \brief Initialize the Global coordinates variables.  Instantiate a SensorCoordinates and assign it to the GlobalPointer*/
	static SensorCoordinates *InitCoordinates();

	/** \brief Get the global pointer to the unique instance of the SensorCoordinates*/
	static SensorCoordinates *GetGlobalCoordinates();

	/** \brief Get the first node of the available Transformation Nodes*/
	static TransformationNode::Ptr GetFirstNode();

	/** \brief Get the first node of the receiver transformation nodess*/
	static TransformationNode::List GetReceivers();

	/** \brief Get the coordinate transformation nodes to a specific Receiver*/
	static TransformationNode::Ptr GetReceiver(int receiverID);


	/** \brief Get the coordinate transformation nodes to a specific voxel within a Receiver*/
	static TransformationNode::Ptr GetVoxel(int receiverID, int voxelIndex);

	/** \brief Get the coordinate transformation nodes to the Cameras*/
	static TransformationNode::List GetCameras();
	/** \brief Get the coordinate transformation nodes to a specific camera*/
	static TransformationNode::Ptr GetCamera(int cameraID);
	

	/** \brief Constructor */
	SensorCoordinates();
	/** \brief Constructor from settings file */
	SensorCoordinates(boost::property_tree::ptree &propTree);

	/** \brief Build the coordinate transformation arborescence from the information contained in the config file */
	bool BuildCoordinatesFromSettings(boost::property_tree::ptree &propTree);

    /** \brief convert sensor coordinates to image (XY) coordinates of the specified camera
	 *  \returns Returns true if the point is in front of the camera.  Returns false for points behind the camera.
	 * 
	 */
	static bool SensorToCameraXY(int receiverID, int voxelIndex, int cameraID, const CameraCalibration &camera, const SphericalCoord &sensorCoord, int &cameraX, int &cameraY);

	/** \brief convert world coordinates to image (XY) coordinates of the specified camera
	 *  \returns Returns true if the point is in front of the camera.  Returns false for points behind the camera.
	 * 
	 */
	static bool WorldToCameraXY(int cameraID, const CameraCalibration &camera, const CartesianCoord &worldCoord, int &cameraX, int &cameraY);

	/** \brief Get position of the receiver relative to car */
	static RelativePosition GetReceiverPosition(int receiverID);
	/** \brief Get position of the voxel within the receiver */
	static RelativePosition GetVoxelPosition(int receiverID, int voxelID);
	/** \brief Get position of the camera relative to car */
	static RelativePosition GetCameraPosition(int cameraID);

	/** \brief Set position of the receiver relative to car */
	static RelativePosition SetReceiverPosition(int receiverID, const RelativePosition &inPosition);
	/** \brief Set position of the voxel within the receiver */
	static RelativePosition SetVoxelPosition(int receiverID, int voxelIndex, const RelativePosition &inPosition);
	/** \brief Set position of the camera relative to car */
	static RelativePosition SetCameraPosition(int cameraID, const RelativePosition &inPosition);


protected:
	/** \brief Extract the relative position of a sensor or camera, which is in standard XML format, from the given property node in the XML config file */
	static  TransformationNode::Ptr GetGeometryFromPropertyNode(boost::property_tree::ptree &propTree);
	/** \brief Extract the relative position of the voxel within Receiver,  from the given property node in the XML config file */
	static TransformationNode::Ptr GetGeometryFromVoxelPropertyNode(boost::property_tree::ptree &voxelNode);

protected:
	/** \brief Global pointer to single instance of the SensorCoordinates object*/
	static SensorCoordinates *globalCoordinates;

	/** \brief Root of all transformation trees*/
	TransformationNode::Ptr	firstNode;

	/** \brief Root of all coordinate transformation trees for receivers*/
	TransformationNode::List receivers;

	/** \brief Root of all coordinate transformation trees for cameras*/
	TransformationNode::List cameras;
};
SENSORCORE_END_NAMESPACE
#endif // SENSOR_COORD_H
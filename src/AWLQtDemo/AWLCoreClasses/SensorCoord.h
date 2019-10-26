#ifndef SENSOR_COORD_H
#define SENSOR_COORD_H
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

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/container/vector.hpp>
#include <boost/container/deque.hpp>

#include <boost/property_tree/ptree.hpp>


#include <cmath>

#include "CoordinateSystem.h"

using namespace std;

namespace awl
{
/** \brief The SensorCoordinates class provides utility methods to perform  application-specific coordinates conversions.
  * \author Jean-Yves Desch�nes
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
} // namespace awl
#endif // SENSOR_COORD_H
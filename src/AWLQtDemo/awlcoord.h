#ifndef AWL_COORD_H
#define AWL_COORD_H


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
// Class AWL Coordinates handles the specific AWL Coordinates arrangement.
class AWLCoordinates
{
public:
	static AWLCoordinates *InitCoordinates();
	static AWLCoordinates *GetGlobalCoordinates();
	static TransformationNode::Ptr GetFirstNode();
	static TransformationNode::List GetReceivers();
	static TransformationNode::Ptr GetReceiver(int receiverID);
	static TransformationNode::Ptr GetChannel(int receiverID, int channelID);
	static TransformationNode::List GetCameras();
	static TransformationNode::Ptr GetCamera(int cameraID);
	

	// Constructor
	AWLCoordinates();
	AWLCoordinates(boost::property_tree::ptree &propTree);
	bool BuildCoordinatesFromSettings(boost::property_tree::ptree &propTree);

	static bool SensorToCamera(int receiverID, int channelID, int cameraID, double cameraFovWidthInRad, double cameraFovHeightInRad, int frameWidthInPixels, int frameHeightInPixels, const SphericalCoord &sensorCoord, int &cameraX, int &cameraY);

	static RelativePosition GetReceiverPosition(int receiverID);
	static RelativePosition GetChannelPosition(int receiverID, int channelID);
	static RelativePosition GetCameraPosition(int cameraID);

	static RelativePosition SetReceiverPosition(int receiverID, const RelativePosition &inPosition);
	static RelativePosition SetChannelPosition(int receiverID, int channelID, const RelativePosition &inPosition);
	static RelativePosition SetCameraPosition(int cameraID, const RelativePosition &inPosition);


protected:
	TransformationNode::Ptr GetGeometryFromPropertyNode(boost::property_tree::ptree &propTree);
	TransformationNode::Ptr GetGeometryFromChannelPropertyNode(boost::property_tree::ptree &channelNode);

protected:
	static AWLCoordinates *globalCoordinates;
	TransformationNode::Ptr	firstNode;

	TransformationNode::List receivers;
	TransformationNode::List cameras;
};

} // namespace awl
#endif // AWL_COORD_H
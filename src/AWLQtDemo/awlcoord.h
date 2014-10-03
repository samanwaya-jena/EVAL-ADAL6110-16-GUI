#ifndef AWL_COORD_H
#define AWL_COORD_H


#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/container/vector.hpp>
#include <boost/container/deque.hpp>

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
	// Constructor
	AWLCoordinates();
	bool BuildCoordinatesFromSettings();

	static bool SensorToCamera(int receiverID, int channelID, int cameraID, double cameraFovWidthInRad, double cameraFovHeightInRad, int frameWidthInPixels, int frameHeightInPixels, const SphericalCoord &sensorCoord, int &cameraX, int &cameraY);
protected:
	static AWLCoordinates *globalCoordinates;
	TransformationNode::Ptr	firstNode;

	TransformationNode::List receivers;
	TransformationNode::List cameras;
};

} // namespace awl
#endif // AWL_COORD_H
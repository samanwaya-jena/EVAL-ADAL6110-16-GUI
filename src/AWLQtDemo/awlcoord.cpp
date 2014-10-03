

#include <stdint.h>
#include <iostream>
#include <math.h>

#include "awlcoord.h"
#include "DebugPrintf.h"
#include "AWLSettings.h"

using namespace std;
using namespace awl;


AWLCoordinates *AWLCoordinates::globalCoordinates=NULL;

AWLCoordinates::AWLCoordinates()
{
}

AWLCoordinates * AWLCoordinates::InitCoordinates()
{
	globalCoordinates  = new AWLCoordinates();
	return(globalCoordinates);
}

AWLCoordinates *AWLCoordinates::GetGlobalCoordinates()
{
	return(globalCoordinates);
}

TransformationNode::Ptr AWLCoordinates::GetFirstNode()
{
	return(globalCoordinates->firstNode);
}

TransformationNode::List AWLCoordinates::GetReceivers()
{
	return(globalCoordinates->receivers);
}

TransformationNode::Ptr AWLCoordinates::GetReceiver(int receiverID)
{
	return(globalCoordinates->receivers[receiverID]);
}

TransformationNode::Ptr AWLCoordinates::GetChannel(int receiverID, int channelID)
{
	return(globalCoordinates->receivers[receiverID]->children[channelID]);
}

TransformationNode::List AWLCoordinates::GetCameras()
{
	return(globalCoordinates->cameras);
}

bool AWLCoordinates::SensorToCamera(int receiverID, int channelID, int cameraID, double cameraFovWidthInRad, double cameraFovHeightInRad, int frameWidthInPixels, int frameHeightInPixels, const SphericalCoord &sensorCoord, int &cameraX, int &cameraY)
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	// Channel description pointer
	TransformationNode::Ptr channelCoords = AWLCoordinates::GetChannel(receiverID, channelID);
	
	// Camera FOV description
	TransformationNode::Ptr cameraCoords = AWLCoordinates::GetCameras()[cameraID];
	CartesianCoord cameraTopLeft(SphericalCoord(10, M_PI_2 - (cameraFovHeightInRad/2), +(cameraFovWidthInRad/2)));
	CartesianCoord cameraBottomRight(SphericalCoord(10, M_PI_2 + (cameraFovHeightInRad/2), - (cameraFovWidthInRad/2)));

	SphericalCoord coordInWorld = channelCoords->ToReferenceCoord(eSensorToWorldCoord, sensorCoord);         // Convert to world
	SphericalCoord coordInCamera = cameraCoords->FromReferenceCoord(eWorldToCameraCoord, coordInWorld);		 // Convert to camera
	coordInCamera.rho = 10.0;																				     // Place in projection Plane.
	CartesianCoord coordInCameraCart(coordInCamera);

	// Remember: In relation to a body the standard convention is
	//  x forward, y left and z up.
	// Careful when converting to projected plane, where X is right and y is up

	cameraX = frameWidthInPixels * (coordInCameraCart.left-cameraTopLeft.left) / (cameraBottomRight.left - cameraTopLeft.left);
	cameraY = frameHeightInPixels * (cameraTopLeft.up-coordInCameraCart.up) / (cameraTopLeft.up- cameraBottomRight.up);

	return(true);
}

bool AWLCoordinates::BuildCoordinatesFromSettings()
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	// If the firstNode was already created, cler it so that we can rebuild the whole coordinate tree.

	if (firstNode.get()) firstNode.reset();

	firstNode  = TransformationNode::Ptr(new TransformationNode(CartesianCoord(0, 0, 0), Orientation(0, 0, 0)));

	// Build a transformation matrix for each of the pixels in each of the receivers

	// Loop for the receivers
	int receiverQty = globalSettings->receiverSettings.size();
	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		ReceiverSettings &receiverSettings = globalSettings->receiverSettings[receiverID];
		CartesianCoord receiverPosition(receiverSettings.sensorForward, receiverSettings.sensorLeft, receiverSettings.sensorUp);
		Orientation receiverOrientation(DEG2RAD(receiverSettings.sensorRoll), 
			                            DEG2RAD(receiverSettings.sensorPitch), 
										DEG2RAD(receiverSettings.sensorYaw));
		TransformationNode::Ptr receiverNode = TransformationNode::Ptr(new TransformationNode(receiverPosition, receiverOrientation));
		firstNode->AddChild(receiverNode);
		receivers.push_back(receiverNode);

		// Loop for the sensors
		for (int channelID = 0; channelID < receiverSettings.channelsConfig.size(); channelID++)
		{
			CartesianCoord channelPosition(0, 0, 0);
			float roll = 0.0;
			float pitch = DEG2RAD(receiverSettings.channelsConfig[channelID].centerY) /*+ M_PI_2*/;
			float yaw = DEG2RAD(receiverSettings.channelsConfig[channelID].centerX) /*+ M_PI_2*/;
			Orientation channelOrientation(roll, pitch, yaw);

			TransformationNode::Ptr channelNode = TransformationNode::Ptr(new TransformationNode(channelPosition, channelOrientation));
			receiverNode->AddChild(channelNode);

		}
	}

	// Build a transformation matrix for the cameras
	// Loop for the cameras
	int cameraQty = globalSettings->cameraSettings.size();
	for (int cameraID = 0; cameraID < cameraQty; cameraID++)
	{
		CameraSettings &cameraSettings = globalSettings->cameraSettings[cameraID];
		CartesianCoord cameraPosition(cameraSettings.cameraForward, cameraSettings.cameraLeft, cameraSettings.cameraUp);
		Orientation cameraOrientation(DEG2RAD(cameraSettings.cameraRoll),
									  DEG2RAD(cameraSettings.cameraPitch), 
									  DEG2RAD(cameraSettings.cameraYaw));
		TransformationNode::Ptr cameraNode = TransformationNode::Ptr(new TransformationNode(cameraPosition, cameraOrientation));
		firstNode->AddChild(cameraNode);
		cameras.push_back(cameraNode);

	}


	return(true);
}


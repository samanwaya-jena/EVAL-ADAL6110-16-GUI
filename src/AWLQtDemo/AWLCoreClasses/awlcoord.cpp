/* AWLCoord.cpp */
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

TransformationNode::Ptr AWLCoordinates::GetCamera(int cameraID)
{
	return(globalCoordinates->cameras[cameraID]);
}

RelativePosition AWLCoordinates::GetReceiverPosition(int receiverID)
{
	return(GetReceiver(receiverID)->relativePosition);
}

RelativePosition AWLCoordinates::GetChannelPosition(int receiverID, int channelID)
{
	return(GetChannel(receiverID, channelID)->relativePosition);
}

RelativePosition AWLCoordinates::GetCameraPosition(int cameraID)
{
	return(GetCamera(cameraID)->relativePosition);
}



RelativePosition AWLCoordinates::SetReceiverPosition(int receiverID, const RelativePosition &inPosition)
{
	TransformationNode::Ptr node = GetReceiver(receiverID);

	node->relativePosition = inPosition;
	node->RefreshGlobal();

	return(node->relativePosition);
}

RelativePosition AWLCoordinates::SetChannelPosition(int receiverID, int channelID, const RelativePosition &inPosition)
{
	TransformationNode::Ptr node = GetChannel(receiverID, channelID);

	node->relativePosition = inPosition;
	node->RefreshGlobal();

	return(node->relativePosition);
}


RelativePosition AWLCoordinates::SetCameraPosition(int cameraID, const RelativePosition &inPosition)
{
	TransformationNode::Ptr node = GetCamera(cameraID);

	node->relativePosition = inPosition;
	node->RefreshGlobal();

	return(node->relativePosition);
}


bool AWLCoordinates::SensorToCameraXY(int receiverID, int channelID, int cameraID, const CameraCalibration &camera, const SphericalCoord &sensorCoord, int &cameraX, int &cameraY)
{
	bool bInFront = false;

	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	// Channel description pointer
	TransformationNode::Ptr channelCoords = AWLCoordinates::GetChannel(receiverID, channelID);
	
	// Camera FOV description
	TransformationNode::Ptr cameraCoords = AWLCoordinates::GetCameras()[cameraID];
	CartesianCoord coordInWorld = channelCoords->ToReferenceCoord(eSensorToWorldCoord, sensorCoord);         // Convert to world
	CartesianCoord coordInCameraCart = cameraCoords->FromReferenceCoord(eWorldToCameraCoord, coordInWorld);		 // Convert to camera

	bInFront = camera.ToFrameXY(coordInCameraCart, cameraX, cameraY);

	return (bInFront);
}


bool AWLCoordinates::WorldToCameraXY(int cameraID, const CameraCalibration &camera, const CartesianCoord &worldCoord, int &cameraX, int &cameraY)
{
	bool bInFront = false;

	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	
	// Camera FOV description
	TransformationNode::Ptr cameraCoords = AWLCoordinates::GetCameras()[cameraID];
	CartesianCoord coordInCameraCart = cameraCoords->FromReferenceCoord(eWorldToCameraCoord, worldCoord);		 // Convert to camera

	bInFront = camera.ToFrameXY(coordInCameraCart, cameraX, cameraY);

	return (bInFront);
}

bool AWLCoordinates::BuildCoordinatesFromSettings(boost::property_tree::ptree &propTree)
{
	// If the firstNode was already created, cler it so that we can rebuild the whole coordinate tree.

	if (firstNode.get()) firstNode.reset();

	firstNode  = TransformationNode::Ptr(new TransformationNode(CartesianCoord(0, 0, 0), Orientation(0, 0, 0)));

	// Build a transformation matrix for each of the pixels in each of the receivers
	// Loop for the receivers
	int receiverQty = propTree.get<int>("config.receivers.receiverQty");
	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		// Get to the receiver in the configuration tree. It will be our anchor in the loop
		char receiverKeyString[32];
		sprintf(receiverKeyString,  "config.receivers.receiver%d", receiverID);
		std::string receiverKey = receiverKeyString;
		boost::property_tree::ptree &receiverPropNode = propTree.get_child(receiverKey);


		// Get to the receiver geometry in the configuration tree
		boost::property_tree::ptree &receiverGeometryPropNode = receiverPropNode.get_child("sensorGeometry");

		// Make the transformation node and add it to the tree
		TransformationNode::Ptr receiverGeometryNode = GetGeometryFromPropertyNode(receiverGeometryPropNode);
		firstNode->AddChild(receiverGeometryNode);
		receivers.push_back(receiverGeometryNode);

		string sChannelGeometryKey = receiverPropNode.get<std::string>("receiverChannelGeometry");

		// Loop for each individual channel
		boost::property_tree::ptree &channelGeometryPropNode = propTree.get_child(std::string("config.")+sChannelGeometryKey);
		int channelQty = channelGeometryPropNode.get<int>("channelQty", -1);
		if (channelQty != -1)
		{
			for (int channelID = 0; channelID < channelQty; channelID++)
			{
				char channelKeyString[32];
				sprintf(channelKeyString, "channel%d", channelID);
				std::string channelKey = channelKeyString;
				boost::property_tree::ptree &channelPropNode = channelGeometryPropNode.get_child(channelKey);

				// Make the transformation node and add it to the tree
				TransformationNode::Ptr channelGeometryNode = GetGeometryFromChannelPropertyNode(channelPropNode);
				receiverGeometryNode->AddChild(channelGeometryNode);
			}
		}
		else
		{
			float columnsFloat(0.0);
			float rowsFloat(0.0);
			int columns(0);
			int rows(0);
			float fovX(0);
			float fovY(0);
			float spacingX(0.0);
			float spacingY(0.0);
			float offsetX(0.0);
			float offsetY(0.0);

			AWLSettings::Get2DPoint(channelGeometryPropNode.get_child("arraySize"), columnsFloat, rowsFloat);
			columns = (int)columnsFloat;
			rows = (int)rowsFloat;
			AWLSettings::Get2DPoint(channelGeometryPropNode.get_child("arrayFOV"), fovX, fovY);
			AWLSettings::Get2DPoint(channelGeometryPropNode.get_child("pixelSpacing"), spacingX, spacingY);
			AWLSettings::Get2DPoint(channelGeometryPropNode.get_child("arrayOffset"), offsetX, offsetY);

			float pixelWidth = (fovX - ((columns - 1)*spacingX)) / columns;
			float pixelHeight = (fovY - ((rows - 1)*spacingY)) / rows;

			int channelQty = columns * rows;
			for (int channelIndex = 0; channelIndex < channelQty; channelIndex++)
			{
				int column = channelIndex % columns;
				int row = channelIndex / columns;

				float pixelYaw = DEG2RAD(offsetX + ((fovX / 2) - (pixelWidth / 2)) - (column * (pixelWidth + spacingX)));
				float pixelPitch = DEG2RAD(offsetY - ((fovY / 2) - (pixelHeight / 2)) + (row * (pixelHeight + spacingY)));
				float pixelRoll = 0.0;

				Orientation orientation(pixelRoll, pixelPitch, pixelYaw);

				// Simplification: We assume all channel sensors are at position 0.0
				CartesianCoord position(0, 0, 0);

				// Make the transformation node
				TransformationNode::Ptr channelGeometryNode = TransformationNode::Ptr(new TransformationNode(position, orientation));
				receiverGeometryNode->AddChild(channelGeometryNode);
			} // for (int channelIndex = ;
		}
	}


	// Build a transformation matrix for the cameras
	// Loop for the cameras
	int cameraQty = propTree.get<int>("config.cameras.cameraQty");
	for (int cameraID = 0; cameraID < cameraQty; cameraID++)
	{
		char cameraKeyString[32];
		sprintf(cameraKeyString, "config.cameras.camera%d", cameraID);
		std::string cameraKey = cameraKeyString;

		boost::property_tree::ptree &cameraPropNode =  propTree.get_child(cameraKey);			
		TransformationNode::Ptr cameraGeometryNode = GetGeometryFromPropertyNode(cameraPropNode);

		firstNode->AddChild(cameraGeometryNode);
		cameras.push_back(cameraGeometryNode);
	}


	return(true);
}



TransformationNode::Ptr AWLCoordinates::GetGeometryFromPropertyNode(boost::property_tree::ptree &propNode)

{
		// Read the geometry information and transform it into proper transformation node description
		// Note that some of the configuration properties are in dregrees and will need to be converted in radians.

		CartesianCoord position(0, 0, 0);
		float rollDegree = 0.0; // Read from ini file in degrees
		float pitchDegree = 0.0; // Read from ini file in degrees
		float yawDegree = 0.0; // Read from ini file in degrees

		AWLSettings::GetGeometry(propNode, 
			        position.forward, position.left, position.up,
					pitchDegree, yawDegree, rollDegree);

		Orientation orientation(DEG2RAD(rollDegree), 
			                            DEG2RAD(pitchDegree), 
										DEG2RAD(yawDegree));

		// Make the transformation node
		TransformationNode::Ptr destNode = TransformationNode::Ptr(new TransformationNode(position, orientation));

		return (destNode);
}

TransformationNode::Ptr AWLCoordinates::GetGeometryFromChannelPropertyNode(boost::property_tree::ptree &channelNode)

{
		float fovWidth(0.0);
		float fovHeight(0.0);
		float centerY(0.0);
		float centerX(0.0);
		float roll(0.0);  
		
		// Read the orientation from the configuration file.
		AWLSettings::GetOrientation(channelNode.get_child("orientation"), centerY, centerX, roll);

		roll = 0.0;
		float pitch = DEG2RAD(centerY);
		float yaw = DEG2RAD(centerX);
		Orientation orientation(roll, pitch, yaw);

		// Simplification: We assume all channel sensors are at position 0.0
		CartesianCoord position(0, 0, 0);

		// Make the transformation node
		TransformationNode::Ptr destNode = TransformationNode::Ptr(new TransformationNode(position, orientation));

		return (destNode);
}

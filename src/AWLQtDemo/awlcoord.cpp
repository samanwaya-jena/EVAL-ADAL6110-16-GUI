

#include <stdint.h>
#include <iostream>
#include <math.h>

#include "awlcoord.h"
#include "DebugPrintf.h"

using namespace std;
using namespace awl;

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

CartesianCoord::CartesianCoord():
x(0.0),
y(0.0),
z(0.0)
{
}

CartesianCoord::CartesianCoord(float inX, float inY, float inZ):
x(inX),
y(inY),
z(inZ)
{
}

CartesianCoord::CartesianCoord(const CartesianCoord &inCartesian)
{
	x = inCartesian.x;
	y = inCartesian.y;
	z = inCartesian.z;
}

CartesianCoord::CartesianCoord(const SphericalCoord &inSpherical)
{
	*this = inSpherical.ToCartesian();
}

CartesianCoord::CartesianCoord(const TransformationVector &inVect)
{
	x = inVect.vect[0];
	y = inVect.vect[1];
	z = inVect.vect[2];
}

void CartesianCoord::Set(float inX, float inY, float inZ)
{
	x = inX;
	y = inY;
	z = inZ;
}

// Convert from cartesian to spherical 
// Using formaulas indicated in http://mathworld.wolfram.com/SphericalCoordinates.html
// But, since we are using physics convention instead of math convention (as Wolfram), swap theta and phi.

SphericalCoord CartesianCoord::ToSpherical(const CartesianCoord &inCartesian) 
{
	SphericalCoord spherical;

	spherical.rho = sqrt((inCartesian.x*inCartesian.x) + (inCartesian.y*inCartesian.y) + (inCartesian.z * inCartesian.z));

	if (spherical.rho != 0.0)
	{
		spherical.theta = acos(inCartesian.z/spherical.rho);
	}

	if (inCartesian.x != 0.0) 
	{
		spherical.phi = atan(inCartesian.y / inCartesian.x);
	}

	return(spherical);
}

CartesianCoord& CartesianCoord::operator=(const SphericalCoord &sourceSpherical)

{
	*this = sourceSpherical.ToCartesian();
	return(*this);
}

CartesianCoord& CartesianCoord::operator=(const TransformationVector &inVector)

{
	x = inVector.vect[0];
	y = inVector.vect[1];
	z = inVector.vect[2];
	return(*this);
}

SphericalCoord CartesianCoord::ToSpherical() const
{
	return(ToSpherical(*this));
}
	

SphericalCoord::SphericalCoord():
rho(0.0),
theta(0.0),
phi(0.0)
{
}

SphericalCoord::SphericalCoord(float inRho, float inTheta, float inPhi):
rho(inRho),
theta(inTheta),
phi(inPhi)
{
}

SphericalCoord::SphericalCoord(const SphericalCoord &inSpherical)
{
	rho = inSpherical.rho;
	theta = inSpherical.theta;
	phi = inSpherical.phi;
}

SphericalCoord::SphericalCoord(const CartesianCoord &inCartesian)
{
	*this = inCartesian.ToSpherical();
}

SphericalCoord::SphericalCoord(const TransformationVector &inVect)
{
	*this = inVect;
}


void SphericalCoord::Set(float inRho, float inTheta, float inPhi)
{
	rho = inRho;
	theta = inTheta;
	phi = inPhi;
}

// Convert from spherical to cartesian 
// Using formaulas indicated in http://mathworld.wolfram.com/SphericalCoordinates.html
// But, since we are using physics convention instead of math convention (as Wolfram), swap theta and phi.

CartesianCoord SphericalCoord::ToCartesian(const SphericalCoord &inSpherical)
{
	CartesianCoord cartesian;
	float sinTheta = sin(inSpherical.theta);
	cartesian.x = inSpherical.rho * cos(inSpherical.phi) * sinTheta;
	cartesian.y = inSpherical.rho * sin(inSpherical.phi)* sinTheta;
	cartesian.z = inSpherical.rho * cos(inSpherical.theta);
	return(cartesian);
}

CartesianCoord SphericalCoord::ToCartesian() const 
{
	return(ToCartesian(*this));
}

SphericalCoord& SphericalCoord::operator=(const TransformationVector &inVector)

{
	CartesianCoord cartesian= inVector;
	*this = cartesian.ToSpherical();
	return(*this);
}

SphericalCoord & SphericalCoord::operator=(const CartesianCoord &sourceCartesian)
{
	*this = sourceCartesian.ToSpherical();
	return(*this);
}

Orientation::Orientation():
roll(0.0),
pitch(0.0),
yaw(0.0)
{
}

Orientation::Orientation(float inRoll, float inPitch, float inYaw):
roll(inRoll),
pitch(inPitch),
yaw(inYaw)
{
	Orientation orient;
}
	
Orientation::Orientation(const Orientation  &inOrientation):
roll(inOrientation.roll),
pitch(inOrientation.pitch),
yaw(inOrientation.yaw)
{
}

Orientation::Orientation(const TransformationVector &inVect):
roll(inVect.vect[0]),
pitch(inVect.vect[1]),
yaw(inVect.vect[2])
{
}

Orientation& Orientation::operator=(const TransformationVector &inVector)

{
	roll = inVector.vect[0];
	pitch = inVector.vect[1];
	yaw = inVector.vect[2];
	return(*this);
}

RelativePosition::RelativePosition():
position(0.0, 0.0, 0.0),
orientation(0.0, 0.0, 0.0)
{
}

RelativePosition::RelativePosition(float inX, float inY, float inZ, float inRoll, float inPitch, float inYaw):
position(inX, inY, inZ),
orientation(inRoll, inPitch, inYaw)
{
}

RelativePosition::RelativePosition(const CartesianCoord &inPosition, const Orientation &inOrientation):
position(inPosition),
orientation(inOrientation)
{
}

RelativePosition::RelativePosition(const SphericalCoord &inPosition, const Orientation &inOrientation):
position(inPosition),
orientation(inOrientation)
{
}

RelativePosition::RelativePosition(const RelativePosition &inRelativePosition):
position(inRelativePosition.position),
orientation(inRelativePosition.orientation)
{
}

TransformationMatrix::TransformationMatrix()
{
	 matrix[0][0] = 1;
	 matrix[0][1] = 0;
	 matrix[0][2] = 0;
	 matrix[0][3] = 0;       
	 matrix[1][0] = 0;
	 matrix[1][1] = 1;
	 matrix[1][2] = 0;     
	 matrix[1][3] = 0;       
	 matrix[2][0] = 0;     
	 matrix[2][1] = 0;     
	 matrix[2][2] = 1;
	 matrix[2][3] = 0;       
	 matrix[3][0] = 0.0;     
	 matrix[3][1] = 0.0;     
	 matrix[3][2] = 0.0;     
	 matrix[3][3] = 1.0; 

}

TransformationMatrix::TransformationMatrix(const CartesianCoord &inCartesian)
{
	*this = inCartesian;

}

TransformationMatrix::TransformationMatrix(const SphericalCoord &inSpherical)
{
	 *this = inSpherical;
}

TransformationMatrix::TransformationMatrix(const Orientation &inOrientation)
{
	*this = inOrientation;
}

TransformationMatrix::TransformationMatrix(const SphericalCoord &inSpherical, const Orientation &inOrientation)
{
	*this = TransformationMatrix(CartesianCoord(inSpherical), inOrientation);
}

TransformationMatrix::TransformationMatrix(const CartesianCoord &inCartesian, const Orientation &inOrientation)
{
	float cosAlpha = cos(inOrientation.yaw);
	float sinAlpha= sin(inOrientation.yaw);
	float cosBeta = cos(inOrientation.pitch);
	float sinBeta = sin(inOrientation.pitch);
	float cosGamma = cos(inOrientation.roll);
	float sinGamma = sin(inOrientation.roll);

	 matrix[0][0] = cosAlpha * cosBeta;
	 matrix[0][1] = (cosAlpha * sinBeta * sinGamma) - (sinAlpha * cosGamma);
	 matrix[0][2] = (cosAlpha * sinBeta *cosGamma) + (sinAlpha * sinGamma);
	 matrix[0][3] = inCartesian.x;       
	 matrix[1][0] = sinAlpha * cosBeta;
	 matrix[1][1] = (sinAlpha * sinBeta * sinGamma) + (cosAlpha * cosGamma);
	 matrix[1][2] = (sinAlpha * sinBeta * cosGamma) - (cosAlpha * sinGamma);     
	 matrix[1][3] = inCartesian.y;       
	 matrix[2][0] = -sinBeta;     
	 matrix[2][1] = cosBeta * sinGamma;     
	 matrix[2][2] = cosBeta * cosGamma;
	 matrix[2][3] = inCartesian.z;       
	 matrix[3][0] = 0.0;     
	 matrix[3][1] = 0.0;     
	 matrix[3][2] = 0.0;     
	 matrix[3][3] = 1.0; 
}

TransformationMatrix::TransformationMatrix(const RelativePosition &inRelativePosition)
{
	*this = inRelativePosition;
}

TransformationMatrix& TransformationMatrix::operator=(const CartesianCoord &inCartesian)
{
	matrix[0][0] = 1;
	matrix[0][1] = 0;
	matrix[0][2] = 0;
	matrix[0][3] = inCartesian.x;       
	matrix[1][0] = 0;
	matrix[1][1] = 1;
	matrix[1][2] = 0;     
	matrix[1][3] = inCartesian.y;       
	matrix[2][0] = 0;     
	matrix[2][1] = 0;     
	matrix[2][2] = 1;
	matrix[2][3] = inCartesian.z;       
	matrix[3][0] = 0.0;     
	matrix[3][1] = 0.0;     
	matrix[3][2] = 0.0;     
	matrix[3][3] = 1.0; 

	return(*this);
}


TransformationMatrix& TransformationMatrix::operator=(const SphericalCoord &inSpherical)
{
	CartesianCoord cartesian(inSpherical);
	matrix[0][0] = 1;
	matrix[0][1] = 0;
	matrix[0][2] = 0;
	matrix[0][3] = cartesian.x;       
	matrix[1][0] = 0;
	matrix[1][1] = 1;
	matrix[1][2] = 0;     
	matrix[1][3] = cartesian.y;       
	matrix[2][0] = 0;     
	matrix[2][1] = 0;     
	matrix[2][2] = 1;
	matrix[2][3] = cartesian.z;       
	matrix[3][0] = 0.0;     
	matrix[3][1] = 0.0;     
	matrix[3][2] = 0.0;     
	matrix[3][3] = 1.0; 

	return(*this);
}

TransformationMatrix& TransformationMatrix::operator=(const Orientation &inOrientation)
{
	float cosAlpha = cos(inOrientation.yaw);
	float sinAlpha= sin(inOrientation.yaw);
	float cosBeta = cos(inOrientation.pitch);
	float sinBeta = sin(inOrientation.pitch);
	float cosGamma = cos(inOrientation.roll);
	float sinGamma = sin(inOrientation.roll);

	 matrix[0][0] = cosAlpha * cosBeta;
	 matrix[0][1] = (cosAlpha * sinBeta * sinGamma) - (sinAlpha * cosGamma);
	 matrix[0][2] = (cosAlpha * sinBeta *cosGamma) + (sinAlpha * sinGamma);
	 matrix[0][3] = 0.0;       
	 matrix[1][0] = sinAlpha * cosBeta;
	 matrix[1][1] = (sinAlpha * sinBeta * sinGamma) + (cosAlpha * cosGamma);
	 matrix[1][2] = (sinAlpha * sinBeta * cosGamma) - (cosAlpha * sinGamma);     
	 matrix[1][3] = 0.0;       
	 matrix[2][0] = -sinBeta;     
	 matrix[2][1] = cosBeta * sinGamma;     
	 matrix[2][2] = cosBeta * cosGamma;
	 matrix[2][3] = 0.0;       
	 matrix[3][0] = 0.0;     
	 matrix[3][1] = 0.0;     
	 matrix[3][2] = 0.0;     
	 matrix[3][3] = 1.0; 

 	return(*this);
}

TransformationMatrix& TransformationMatrix::operator=(const RelativePosition &inRelativePosition)
{
	float cosAlpha = cos(inRelativePosition.orientation.yaw);
	float sinAlpha= sin(inRelativePosition.orientation.yaw);
	float cosBeta = cos(inRelativePosition.orientation.pitch);
	float sinBeta = sin(inRelativePosition.orientation.pitch);
	float cosGamma = cos(inRelativePosition.orientation.roll);
	float sinGamma = sin(inRelativePosition.orientation.roll);

	 matrix[0][0] = cosAlpha * cosBeta;
	 matrix[0][1] = (cosAlpha * sinBeta * sinGamma) - (sinAlpha * cosGamma);
	 matrix[0][2] = (cosAlpha * sinBeta *cosGamma) + (sinAlpha * sinGamma);
	 matrix[0][3] = inRelativePosition.position.x;       
	 matrix[1][0] = sinAlpha * cosBeta;
	 matrix[1][1] = (sinAlpha * sinBeta * sinGamma) + (cosAlpha * cosGamma);
	 matrix[1][2] = (sinAlpha * sinBeta * cosGamma) - (cosAlpha * sinGamma);     
	 matrix[1][3] = inRelativePosition.position.y;       
	 matrix[2][0] = -sinBeta;     
	 matrix[2][1] = cosBeta * sinGamma;     
	 matrix[2][2] = cosBeta * cosGamma;
	 matrix[2][3] = inRelativePosition.position.z;       
	 matrix[3][0] = 0.0;     
	 matrix[3][1] = 0.0;     
	 matrix[3][2] = 0.0;     
	 matrix[3][3] = 1.0; 

 	return(*this);
}

TransformationMatrix TransformationMatrix::Reverse()
{
	TransformationMatrix destinationMatrix;
	
	 // Ref: http://www.cse.psu.edu/~rcollins/CSE486/lecture12.pdf

	 // Transpose the rotation sub-matrix
	 destinationMatrix.matrix[0][0] = matrix[0][0];
	 destinationMatrix.matrix[0][1] = matrix[1][0];
	 destinationMatrix.matrix[0][2] = matrix[2][0];
	 destinationMatrix.matrix[1][0] = matrix[0][1];
	 destinationMatrix.matrix[1][1] = matrix[1][1];
	 destinationMatrix.matrix[1][2] = matrix[2][1];
	 destinationMatrix.matrix[2][0] = matrix[0][2];
	 destinationMatrix.matrix[2][1] = matrix[1][2];
	 destinationMatrix.matrix[2][2] = matrix[2][2];
	 
	 // Negate the translations
	 destinationMatrix.matrix[0][3] = -matrix[0][3];
	 destinationMatrix.matrix[1][3] = -matrix[1][3];
	 destinationMatrix.matrix[2][3] = -matrix[2][3];

	 // Carry over the other matrix items
 	 destinationMatrix.matrix[3][0] = matrix[3][0];     
	 destinationMatrix.matrix[3][1] = matrix[3][1];     
	 destinationMatrix.matrix[3][2] = matrix[3][2];     
	 destinationMatrix.matrix[3][3] = matrix[3][3]; 

 	return(destinationMatrix);
}

TransformationVector::TransformationVector()
{
	vect[0] = 0.0;
	vect[1] = 0.0;
	vect[2] = 0.0;
	vect[3] = 1.0;
}

TransformationVector::TransformationVector(const CartesianCoord &inCartesian)
{
	vect[0] = inCartesian.x;
	vect[1] = inCartesian.y;
	vect[2] = inCartesian.z;
	vect[3] = 1.0;
}

TransformationVector::TransformationVector(const SphericalCoord &inSpherical)
{
	*this = inSpherical;
}

TransformationVector::TransformationVector(const Orientation &inOrientation)
{
	*this = inOrientation;
}

TransformationVector& TransformationVector::operator=(const CartesianCoord &inCartesian)
{
	vect[0] = inCartesian.x;
	vect[1] = inCartesian.y;
	vect[2] = inCartesian.z;
	vect[3] = 1.0;

	return(*this);
}


TransformationVector& TransformationVector::operator=(const SphericalCoord &inSpherical)
{
	CartesianCoord cartesian(inSpherical);

	vect[0] = cartesian.x;
	vect[1] = cartesian.y;
	vect[2] = cartesian.z;
	vect[3] = 1.0;

	return(*this);
}

TransformationVector& TransformationVector::operator=(const Orientation &inOrientation)
{
	vect[0] = inOrientation.roll;
	vect[1] = inOrientation.pitch;
	vect[2] = inOrientation.yaw;
	vect[3] = 1.0;

	return(*this);
}

awl::TransformationMatrix awl::operator * (const awl::TransformationMatrix &left, const awl::TransformationMatrix &right) 
{
	TransformationMatrix destinationMatrix;

	for(int i = 0; i < 4; i++)
	 {         
		 for(int j = 0; j < 4; j++)
		 {             
			 destinationMatrix.matrix[i][j] = 0;
			 for(int k = 0; k < 4; k++)
			 {                 
				 destinationMatrix.matrix[i][j] += left.matrix[i][k] * right.matrix[k][j];
			 }         
		 }     
	 } 

	return(destinationMatrix);
}

awl::TransformationMatrix awl::operator + (const awl::TransformationMatrix &left, const awl::TransformationMatrix &right)
{
	TransformationMatrix destinationMatrix;

	for(int i = 0; i < 4; i++ )
	 {         
		 for(int j = 0; j < 4; j++)
		 {             
			 destinationMatrix.matrix[i][j] = left.matrix[i][j]  + right.matrix[i][j];
		 }     
	 } 

	return(destinationMatrix);
}

awl::TransformationMatrix awl::operator - (const awl::TransformationMatrix &left, const awl::TransformationMatrix &right)
{
	TransformationMatrix destinationMatrix;

	for(int i = 0; i < 4; i++ )
	 {         
		 for(int j = 0; j < 4; j++)
		 {             
			 destinationMatrix.matrix[i][j] = left.matrix[i][j]  - right.matrix[i][j];
		 }     
	 } 

	return(destinationMatrix);
}

awl::TransformationMatrix awl::operator * (float scalarLeft, const awl::TransformationMatrix &right)
{
	TransformationMatrix destinationMatrix;

	for(int i = 0; i < 4; i++ )
	 {         
		 for(int j = 0; j < 4; j++)
		 {             
			 destinationMatrix.matrix[i][j] = scalarLeft  * right.matrix[i][j];
		 }     
	 } 

	return(destinationMatrix);
}

awl::TransformationMatrix awl::operator * (const awl::TransformationMatrix &left, float scalarRight)
{
	TransformationMatrix destinationMatrix;

	for(int i = 0; i < 4; i++ )
	 {         
		 for(int j = 0; j < 4; j++)
		 {             
			 destinationMatrix.matrix[i][j] = scalarRight  * left.matrix[i][j];
		 }     
	 } 

	return(destinationMatrix);
}

awl::TransformationVector awl::operator * (const awl::TransformationMatrix &left, const awl::TransformationVector &right) 
{
	TransformationVector destinationVector;

	for(int i = 0; i < 4; i++)
	 {    
		 destinationVector.vect[i] = 0;
		 for(int j = 0; j < 4; j++)
		 {        
			 destinationVector.vect[i] += left.matrix[i][j] * right.vect[j];
		 }     
	 } 

	return(destinationVector);
}

TransformationNode::TransformationNode(const CartesianCoord &inCartesian, const Orientation &inOrientation):
parent(),
children(),
relativePosition(inCartesian, inOrientation),
transformations()
{
	
	RefreshGlobal();
}


TransformationNode::TransformationNode(const RelativePosition &inRelativePosition):
parent(),
children(),
relativePosition(inRelativePosition),
transformations()
{
	RefreshGlobal();
}

TransformationNode::TransformationNode():
parent(),
children(),
relativePosition(),
transformations()
{
	RefreshGlobal();
}

void TransformationNode::AddChild(TransformationNode::Ptr inChild)
{
	inChild->parent = shared_from_this();
	children.push_back(inChild);
	inChild->RefreshGlobal();
}

	
void TransformationNode::RefreshGlobal()
{
	// Clear the current iteration of transformations
	transformations.clear();

	TransformationMatrix currentTransformation(relativePosition);
	transformations.push_front(currentTransformation);
	
	//iterate through the parents to create transformations at each of the steps.
	TransformationNode::Ptr ancesterNode = parent;
	while (ancesterNode)
	{
		currentTransformation = currentTransformation * ancesterNode->transformations.at(ancesterNode->transformations.size()-1);
		transformations.push_front(currentTransformation);
		ancesterNode = ancesterNode->parent;
	}
}

CartesianCoord TransformationNode::ToReferenceCoord(eCoordLevel inLevel, const CartesianCoord & inCoord)

{
	TransformationVector coordVect(inCoord);
	CartesianCoord retCoord = transformations[inLevel] * coordVect;

	return (retCoord);
}

CartesianCoord TransformationNode::FromReferenceCoord(eCoordLevel inLevel, const CartesianCoord & inCoord)

{
	TransformationVector coordVect(inCoord);
	TransformationMatrix reverseMatrix = transformations[inLevel].Reverse();
	CartesianCoord retCoord = reverseMatrix * coordVect;

	return (retCoord);
}

//// This should all be transferred to separate file

#include "AWLSettings.h"

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

TransformationNode::List AWLCoordinates::GetCameras()
{
	return(globalCoordinates->cameras);
}

bool AWLCoordinates::SensorToCamera(int receiverID, int channelID, int cameraID, double cameraFovWidthInRad, double cameraFovHeightInRad, int frameWidthInPixels, int frameHeightInPixels, const SphericalCoord &sensorCoord, int &cameraX, int &cameraY)
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();

	// Channel description pointer
	TransformationNode::Ptr channelCoords = AWLCoordinates::GetReceivers()[receiverID]->children[channelID];
	
	// Camera FOV description
	TransformationNode::Ptr cameraCoords = AWLCoordinates::GetCameras()[cameraID];
	CartesianCoord cameraTopLeft(SphericalCoord(10, M_PI_2 - (cameraFovHeightInRad/2), +(cameraFovWidthInRad/2)));
	CartesianCoord cameraBottomRight(SphericalCoord(10, M_PI_2 + (cameraFovHeightInRad/2), - (cameraFovWidthInRad/2)));

	SphericalCoord coordInWorld = channelCoords->ToReferenceCoord(eSensorToWorldCoord, sensorCoord);         // Convert to world
	SphericalCoord coordInCamera = cameraCoords->FromReferenceCoord(eWorldToCameraCoord, coordInWorld);		 // Convert to camera
	coordInCamera.rho = 10;																				     // Place in projection Plane.
	CartesianCoord coordInCameraCart(coordInCamera);

	// Remember: In relation to a body the standard convention is
	//  x forward, y left and z up.
	// Careful when converting to projected plane, where X is right and y is up

	cameraX = frameWidthInPixels * (coordInCameraCart.left-cameraTopLeft.left) / (cameraBottomRight.left - cameraTopLeft.left);
	cameraY = frameHeightInPixels * (coordInCameraCart.up-cameraBottomRight.up) / (cameraTopLeft.up- cameraBottomRight.up);

	return(true);
}

bool AWLCoordinates::BuildCoordinatesFromSettings()
{
	AWLSettings *globalSettings = AWLSettings::GetGlobalSettings();
	
	firstNode  = TransformationNode::Ptr(new TransformationNode(CartesianCoord(0, 0, 0), Orientation(0, 0, 0)));

	// Build a transformation matrix for each of the pixels in each of the receivers

	// Loop for the receivers
	int receiverQty = globalSettings->receiverSettings.size();
	for (int receiverID = 0; receiverID < receiverQty; receiverID++)
	{
		ReceiverSettings &receiverSettings = globalSettings->receiverSettings[receiverID];
		CartesianCoord receiverPosition(receiverSettings.sensorForward, receiverSettings.sensorLeft, receiverSettings.sensorUp);
		Orientation receiverOrientation(receiverSettings.sensorRoll, receiverSettings.sensorPitch, receiverSettings.sensorYaw);
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
	// Loop for the receivers
	int cameraQty = globalSettings->cameraSettings.size();
	for (int cameraID = 0; cameraID < cameraQty; cameraID++)
	{
		CameraSettings &cameraSettings = globalSettings->cameraSettings[cameraID];
		CartesianCoord cameraPosition(cameraSettings.cameraForward, cameraSettings.cameraLeft, cameraSettings.cameraUp);
		Orientation cameraOrientation(cameraSettings.cameraRoll, cameraSettings.cameraPitch, cameraSettings.cameraYaw);
		TransformationNode::Ptr cameraNode = TransformationNode::Ptr(new TransformationNode(cameraPosition, cameraOrientation));
		firstNode->AddChild(cameraNode);
		cameras.push_back(cameraNode);

	}


	return(true);
}


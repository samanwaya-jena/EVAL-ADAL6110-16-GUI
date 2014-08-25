#ifndef AWL_COORD_H
#define AWL_COORD_H


#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/container/vector.hpp>
#include <boost/container/deque.hpp>

#include <cmath>

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

using namespace std;

namespace awl
{
#ifndef DEG2MRAD
#define DEG2MRAD(d)           (float)(17.453293*(d))     // 1000*PI/180
#endif

#ifndef MRAD2DEG
#define MRAD2DEG(r)           (float)(0.05729579513*(r)) // 180/(1000*PI)
#endif

#ifndef DEG2RAD
#define DEG2RAD(d)            (float)(0.017453293*(d))     // PI/180
#endif

#ifndef RAD2DEG
#define RAD2DEG(r)           (float)(57.295779513*(r)) // 180/(PI)
#endif

class SphericalCoord;
class CartesianCoord;
class Orientation;
class RelativePosition;
class RotationMatrix;
class TransformationVector;
class TransformationMatrix;
class TransformationNode;


typedef enum eCoordLevel
{
	eSensorToReceiverCoord = 2,
	eSensorToVehicleCoord = 1,
	eSensorToWorldCoord = 0,

	eReceiverToVehicleCoord = 1,
	eReceiverToWorldCoord = 0,

	eVehicleToWorldCoord = 0,

	eCameraToVehicleCoord = 1,
	eCameraToWorldCoord = 0,

	eWorldToVehicleCoord = 0,
	eWorldToReceiverCoord = 1,
	eWorldToCameraCoord = 1
}
eCoordLevel;

/** \brief Structure holding the values of a 4x4 homogeneous 3D transformation matrix. 
*/
typedef float (TransformationArray)[4][4];

/** \brief Structure holding the a vector (row or column) of a 1x4 homogeneous coordinate. 
*/
typedef float (TransformationRow)[4];

/** \brief The CartesianCoord class defines a position, in cartesian coordinates. 
  * \Notes uses coordinate conventions as defined in PCL / ROS.
  *  See: http://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions
  *  That is:
  * 
  * Chirality
  * 
  * All systems are right handed. This means they comply with the right hand rule [4].
  *  
  * Axis Orientation
  * 
  * In relation to a body the standard is :
  * �x forward
  * �y left
  * �z up
  * 
  *  Note that, in the case of cameras, there is often a different frame which uses a slightly 
  *  different convention, not used here, where:
  * �z forward
  * �x right
  * �y down
  */


class CartesianCoord 
{
public:
	CartesianCoord();
	CartesianCoord(float inX, float inY, float inZ);
	CartesianCoord(const CartesianCoord &inCartesian);
	CartesianCoord(const SphericalCoord &inSpherical);
	CartesianCoord(const TransformationVector &inVect);

	void Set(float x, float y, float z);

	static SphericalCoord ToSpherical(const CartesianCoord &inCartesian);
	SphericalCoord ToSpherical() const;


	CartesianCoord& operator=(const SphericalCoord &sourceSpherical);
	CartesianCoord& operator=(const TransformationVector &inVector);

//	operator SphericalCoord() const;

public:
	union 
	{
		// The standard "x,y,z" naming convention.
		struct {
			float x;
			float y;
			float z;
		};

		// The alternate "relative to body" naming convention can be useful for 
		// code that wants non-equivocal naming and avoid confusion,
		// for example when when converting to 
		// graphics X, Y coordinates.
		struct {
			float forward;
			float left;
			float up;
		};

	};

};


/** \brief The SpericalCoord class defines a position, in spherical coordinates. 
 *  \notes spherical coordinates are using right-handed notation				
 *         rho is distance.												
 *         theta is angle from z axis, clockwise.						
 *         phi is angle from x axis, counterclockwise					
 *  We use the notation that is common practice in physics, 
 *  As specified by ISO standard 31-11.
 *  This means use positive sign for azimuth angles that are measured in 
 *  the counter-clockwise sense from the reference direction on the 
 *  reference plane, as seen from the zenith side of the plane
 *
 * Conversions from spherical to cartesian are performed using formulas indicated 
 * in http://mathworld.wolfram.com/SphericalCoordinates.html
 * But, since we are using physics convention instead of math convention (as Wolfram), 
 * swap theta and phi from document.
 * 
 8 With x looking forward:
 * Theta: -: Up  +: Down
 * Phi:   + Left  -: Right
*/

class SphericalCoord 
{
public:
	SphericalCoord();
	SphericalCoord(float inRho, float inTheta, float inPhi);
	SphericalCoord(const SphericalCoord &inSpherical);	
	SphericalCoord(const CartesianCoord &inCartesian);
	SphericalCoord(const TransformationVector &inVect);

	void Set(float inRho, float inTheta, float inPhi);

	static CartesianCoord ToCartesian(const SphericalCoord &inSpherical);
	CartesianCoord ToCartesian() const;

	SphericalCoord & operator=(const CartesianCoord &sourceCartesian);
	SphericalCoord& operator=(const TransformationVector &inVector);

public:
	float rho;
	float theta;
	float phi;
};

/** \brief The Orientation class defines the relative orientation of an object in a frame of reference. 
	*  \notes
	*  yaw is a counterclockwise rotation of $ \alpha$ about the $ z$-axis (looking right is yaw negative).
	*  pitch is a counterclockwise rotation of $ \beta$ about the $ y$-axis (looking down is pitch negative). 
	 * roll is a counterclockwise rotation of $ \gamma$ about the $ x$-axis (rolling .
	*  This meansd that, for a standard reference frame, where X axis is looking forward from the object:
	*  - looking right is yaw negative.
	*  - looking down is pitch negative. 
	 * - rolling to the right side is negative.

	 * \reference: http://planning.cs.uiuc.edu/node102.html
*/

class Orientation 
{
public:
	Orientation();
	
	// We use the roll, pitch, yaw order, to keep consistent with eigen 
	Orientation(float inRoll, float inPitch, float inYaw);
	Orientation(const Orientation  &inOrientation);
	Orientation(const TransformationVector &inVect);

	Orientation& operator=(const TransformationVector &inVector);

public:
	float roll;
	float pitch;
	float yaw;
};

/** \brief The RelativePosition Class defines the relative position AND orientation of an object or frame of reference from another. 
 *  \notes The class contains the relative position and orientation of a frame of reference, from the origin frame of reference. 
*/

class RelativePosition
{
public:
	RelativePosition();
	RelativePosition(float inX, float inY, float inZ, float inRoll, float inPitch, float inYaw);
	RelativePosition(const CartesianCoord &inPosition, const Orientation &inOrientation);
	RelativePosition(const SphericalCoord &inPosition, const Orientation &inOrientation);
	RelativePosition(const RelativePosition &inRelativePosition);
	
public:
	CartesianCoord	position;
	Orientation		orientation;
};

/** \brief The TransformationMatrix class supports the basic operators on a 
 *          4x4 affine transformation matrix.
 *  \notes	Constructors allow the easy filling of a transformation matrix
 *          for Position (or translation) and Orientation or (Rotation)
*/
class TransformationMatrix
{
public:
	TransformationMatrix();	
	TransformationMatrix(const CartesianCoord &inCartesian);
	TransformationMatrix(const SphericalCoord &inSpherical);
	TransformationMatrix(const Orientation &inOrientation);

	TransformationMatrix(const CartesianCoord &inCartesian, const Orientation &inOrientation);	
	TransformationMatrix(const SphericalCoord &inSpherical, const Orientation &inOrientation);	
	TransformationMatrix(const RelativePosition &inRelativePosition);

	TransformationMatrix& operator=(const CartesianCoord &inCartesian);
	TransformationMatrix& operator=(const SphericalCoord &inSpherical);
	TransformationMatrix& operator=(const Orientation &inOrientation);
	TransformationMatrix& operator=(const RelativePosition &inRelativePosition);

	TransformationMatrix Reverse();

public:
	TransformationArray matrix; 
};

/** \brief The TransformationMatrix class supports the basic operators on a 
 *          1x4 vector.
 *  \notes	The vectors can be used as operands in affine transformation operations
 *          or as convenient placeholders for sub-sections of TransformationMatrix.
*/
class TransformationVector
{
public:
	TransformationVector();	
	TransformationVector(const CartesianCoord &inCartesian);
	TransformationVector(const SphericalCoord &inSpherical);
	TransformationVector(const Orientation &inOrientation);

	
	TransformationVector& operator=(const CartesianCoord &inCartesian);
	TransformationVector& operator=(const SphericalCoord &inSpherical);
	TransformationVector& operator=(const Orientation &inOrientation);

public:
	TransformationRow vect; 
};

TransformationMatrix operator * (const TransformationMatrix &left, const TransformationMatrix &right);
TransformationMatrix operator + (const TransformationMatrix &left, const TransformationMatrix &right);
TransformationMatrix operator - (const TransformationMatrix &left, const TransformationMatrix &right);
TransformationMatrix operator * (float scalarLeft, const TransformationMatrix &right);
TransformationMatrix operator * (const TransformationMatrix &left, float scalarRight);
TransformationVector operator * (const TransformationMatrix &left, const TransformationVector &right); 


/** \brief TransformationMatrixSteps define a sequence of affine transformations.
 *  \notes	Using a deque (1 sided list) the steps can identify a sequence
 *          of individual affine transformations.
*/
typedef boost::container::deque<TransformationMatrix> TransformationMatrixSteps;


/** \brief TransformationNodes are coordinate frames that can be organized in 
  *         a tree structure to describe the relative position and sequence of transformations
  *		   from a coordinate frame to another.
  *	\Notes For commodity, each node in the tree stores all of the TransformationMatrixSteps to 
  *		   convert its coordinates into each of its parent's coordinates.
  *		   The transformationNode can be used to facilitate transformation of coordinates in
  *		   a multiple axis "robot" (using the ToReferenceCoord() method).
  *		   After modification of a node's coordinates, RefreshGlobal() updates all the children's
  *		   TransformationMaxtrixSteps.
*/
class TransformationNode : public boost::enable_shared_from_this<TransformationNode>

{
public:
	typedef boost::shared_ptr<TransformationNode> Ptr;
	typedef boost::shared_ptr<TransformationNode> ConstPtr;
	typedef boost::container::vector<TransformationNode::Ptr> List;
	typedef TransformationNode::List *ListPtr;

public:
	TransformationNode(const CartesianCoord &inCartesian, const Orientation &inOrientation);
	TransformationNode(const RelativePosition &inRelativePosition);
	TransformationNode();

	void AddChild(TransformationNode::Ptr inChild);
	
	void RefreshGlobal();

	CartesianCoord ToReferenceCoord(eCoordLevel inLevel, const CartesianCoord & inCoord);
	CartesianCoord FromReferenceCoord(eCoordLevel inLevel, const CartesianCoord & inCoord);

public:
	TransformationNode::Ptr parent;
	TransformationNode::List children;
	

	RelativePosition relativePosition;
	TransformationMatrixSteps transformations;
};

// Class AWL Coordinates should be ported.
class AWLCoordinates
{
public:
	static AWLCoordinates *InitCoordinates();
	static AWLCoordinates *GetGlobalCoordinates();
	static TransformationNode::Ptr GetFirstNode();
	static TransformationNode::List GetReceivers();
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
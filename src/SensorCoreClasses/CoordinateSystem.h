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

#ifndef SENSORCORE_COORDINATESYSTEM_H
#define SENSORCORE_COORDINATESYSTEM_H

/* 
	This module manages 3D coordinate transformations between frames of reference.
	It also manages conversions from the 3D coordinates to the 2D camera pixel space.
*/

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/container/vector.hpp>
#include <boost/container/deque.hpp>

#include <cmath>
#include "SensorCoreClassesGlobal.h"


#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif
#ifndef M_SENSORCORE_2
#define M_SENSORCORE_2     1.57079632679489661923
#endif

SENSORCORE_BEGIN_NAMESPACE

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
class CameraCalibration;

/** \brief Identifies the "layer" of coordinate transformation.
  *        The coordinate transformations strart at Pixel (voxel) level, then Sensor (or camera) Level, 
  *        are transformed to vehicle coordinates and then 
  *        to world coordinates.
*/
typedef enum eCoordLevel
{
	/**Convert internal sensor (voxel) to Receiver coordinates*/
	eSensorToReceiverCoord = 2,
	/**Convert internal sensor (voxel) to Vehicle coordinates*/
	eSensorToVehicleCoord = 1,
	/**Convert internal sensor (voxel) to World coordinates*/
	eSensorToWorldCoord = 0,

	/**Convert Receiver to Vehicle coordinates */
	eReceiverToVehicleCoord = 1,
	/**Convert Receiver to World coordinates*/
	eReceiverToWorldCoord = 0,

	/**Convert Vehicle to World coordinates*/
	eVehicleToWorldCoord = 0,

	/**Convert Camera pixel to Vehicle coordinates*/
	eCameraToVehicleCoord = 1,
	/**Convert Camera pixel to World coordinates*/
	eCameraToWorldCoord = 0,

	/**Convert World coordinates back into vehicle coordinates*/
	eWorldToVehicleCoord = 0,
	/**Convert World coordinates back into receiver coordinates*/
	eWorldToReceiverCoord = 1,
	/**Convert World coordinates back into camera coordinates*/
	eWorldToCameraCoord = 1
}
eCoordLevel;

/** \brief Structure holding the values of a 4x4 homogeneous 3D transformation matrix. 
*/
 typedef float(TransformationArray)[4][4];

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
  * •x forward
  * •y left
  * •z up
  * 
  *  Note that, in the case of cameras, there is often a different frame which uses a slightly 
  *  different convention, NOT USED HERE, where:
  * •z backwards 
  * •x right
  * •y up
  * Excercice caution when specifying coordinates to make sure you always use the body standard, not the camera style reference.
  * X is always forward.
  * The only exception to this rule is when we will get to convert Coordinates to camera pixels in the CameraCalibration class.
  */
class CartesianCoord 
{
public:
	/** \brief Constructor */
	CartesianCoord();
	/** \brief Constructor based on individual axes values*/
	CartesianCoord(float inX, float inY, float inZ);
	/** \brief Copy Constructor */
	CartesianCoord(const CartesianCoord &inCartesian);

	/** \brief Constructor from spherical coordinates*/
	CartesianCoord(const SphericalCoord &inSpherical);

	/** \brief Constructor from a transformation vector*/
	CartesianCoord(const TransformationVector &inVect);

	/** \brief Set coordinates on individual axes */
	void Set(float x, float y, float z);

	/** \brief Convert  a given CartesianCoord spherical coordinates */
	static SphericalCoord ToSpherical(const CartesianCoord &inCartesian);

	/** \brief Convert the object to spherical coordinates */
	SphericalCoord ToSpherical() const;

	/** \brief Assign spherical coordinates to a CartesianCoord */
	CartesianCoord& operator=(const SphericalCoord &sourceSpherical);

	/** \brief Assign a transformation vector to a cartesianCoord */
	CartesianCoord& operator=(const TransformationVector &inVector);

//	operator SphericalCoord() const;

public:
	/** \brief Coordinates have two different naming conventions */
	union 
	{
		/** The standard "x,y,z" naming convention. */
		struct {
			float x;
			float y;
			float z;
		} cartesian;

		/** The alternate "relative to body" naming convention can be useful for 
		 * code that wants non-equivocal naming and avoid confusion,
		 * for example when when converting to 
		 * graphics X, Y coordinates.
		 */
		struct {
			float forward;
			float left;
			float up;
		} bodyRelative;

	};

};


/** \brief The SpericalCoord class defines a position, in spherical coordinates. 
 *  \notes spherical coordinates are using right-handed notation				
 *         rho is distance.												
 *         theta is angle from z axis, clockwise. 						
 *         phi is angle from x axis, counterclockwise					
 *  We use the notation that is common practice in physics, 
 *  As specified by ISO standard 31-11.
 *
 * Conversions from spherical to cartesian are performed using formulas indicated 
 * in http://mathworld.wolfram.com/SphericalCoordinates.html
 * But, since we are using physics convention instead of math convention (as Wolfram), 
 * swap theta and phi from document.
 * 
 * With x looking forward:
 * Theta: -: Up  +: Down
 * Phi:   + Left  -: Right
*/
class SphericalCoord 
{
public:
	/** \brief Constructor */
	SphericalCoord();
	/** \brief Constructor based on individual axes values*/
	SphericalCoord(float inRho, float inTheta, float inPhi);
	/** \brief Copy Constructor */
	SphericalCoord(const SphericalCoord &inSpherical);


	/** \brief Constructor from Cartesian coordinates*/
	SphericalCoord(const CartesianCoord &inCartesian);

	/** \brief Constructor from a transformation vector*/
	SphericalCoord(const TransformationVector &inVect);

	/** \brief Set coordinates on individual axes */
	void Set(float inRho, float inTheta, float inPhi);

	/** \brief Convert  a given SphericaCoord to cartesian coordinates */
	static CartesianCoord ToCartesian(const SphericalCoord &inSpherical);
	/** \brief Convert the object to cartesian coordinates */
	CartesianCoord ToCartesian() const;

	/** \brief Assign Cartesian coordinates to a SpherticalCoord */
	SphericalCoord & operator=(const CartesianCoord &sourceCartesian);
	/** \brief Assign a transformation Vector its Sperical coordinates */
	SphericalCoord& operator=(const TransformationVector &inVector);

public:
	/* \brief rho is distance.*/

	float rho;
	/* \brief theta is angle from z axis, clockwise*/
	float theta;
	/* \brief  phi is angle from x axis, counterclockwise*/
	float phi;
};

/** \brief The Orientation class defines the relative orientation of an object in a frame of reference. 
	*  \notes
	*  yaw is a counterclockwise rotation of $ \alpha$ about the $ z$-axis (looking right is yaw negative).
	*  pitch is a counterclockwise rotation of $ \beta$ about the $ y$-axis (looking down is pitch positive [since Y axis is oriented leftwards]). 
	 * roll is a counterclockwise rotation of $ \gamma$ about the $ x$-axis (rolling rightwards is roll negative).
	*  This meansd that, for a standard reference frame, where X axis is looking forward from the object:
	*  - looking right is yaw negative.
	*  - looking down is pitch positive. 
	 * - rolling to the right side is negative.

	 * \reference: http://planning.cs.uiuc.edu/node102.html
*/
class Orientation 
{
public:

	/** \brief Constructor */
	Orientation();
	
	/** \brief Constructor using roll, pich, yaw 
	    \Notes We use the roll, pitch, yaw order, to keep consistent with eigen */
	Orientation(float inRoll, float inPitch, float inYaw);

	/** \brief Copy Constructor */
	Orientation(const Orientation  &inOrientation);

	/** \brief Constructor  from a TransformationVector*/
	Orientation(const TransformationVector &inVect);

	/** \brief Assignment from a transformation vector */
	Orientation& operator=(const TransformationVector &inVector);

public:
	/** \brief  roll is a counterclockwise rotation of $ \gamma$ about the $ x$-axis (rolling rightwards is roll negative). */
	float roll;
	/** \brief  pitch is a counterclockwise rotation of $ \beta$ about the $ y$-axis (looking down is pitch positive [since Y axis is oriented leftwards]). */
	float pitch;
	/** \brief  yaw is a counterclockwise rotation of $ \alpha$ about the $ z$-axis (looking right is yaw negative) */
	float yaw;
};

/** \brief The RelativePosition Class defines the relative position AND orientation of an object or frame of reference from another. 
 *  \notes The class contains the relative position and orientation of a frame of reference, from the origin frame of reference. 
*/
class RelativePosition
{
public:
	/** \brief Constructor */
	RelativePosition();
	/** \brief Constructor from individual cartesian position and orientation components*/
	RelativePosition(float inX, float inY, float inZ, float inRoll, float inPitch, float inYaw);
	/** \brief Constructor from cartesian position and orientation */
	RelativePosition(const CartesianCoord &inPosition, const Orientation &inOrientation);
	/** \brief Constructor from spherical position and orientation */
	RelativePosition(const SphericalCoord &inPosition, const Orientation &inOrientation);
	/** \brief Copy Constructor */
	RelativePosition(const RelativePosition &inRelativePosition);
	
public:
	/** \brief Position component of the relative position */
	CartesianCoord	position;
	/** \brief Orientation component of the relative position */
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
	/** \brief Constructor */
	TransformationMatrix();	
	/** \brief Constructor from cartesian coordinates */
	TransformationMatrix(const CartesianCoord &inCartesian);
	/** \brief Constructor from spherical coordinates */
	TransformationMatrix(const SphericalCoord &inSpherical);
	/** \brief Constructor from an orientation */
	TransformationMatrix(const Orientation &inOrientation);

	/** \brief Constructor from cartesian coordinates and orientation */
	TransformationMatrix(const CartesianCoord &inCartesian, const Orientation &inOrientation);	
	/** \brief Constructor from spherical coordinates and orientation */
	TransformationMatrix(const SphericalCoord &inSpherical, const Orientation &inOrientation);
	/** \brief Constructor from a RelativePosition */
	TransformationMatrix(const RelativePosition &inRelativePosition);

	/** \brief Assignment of CartesianCoord into a TransformationMatrix */
	TransformationMatrix& operator=(const CartesianCoord &inCartesian);
	/** \brief Assignment of SphericalCoord into a TransformationMatrix */
	TransformationMatrix& operator=(const SphericalCoord &inSpherical);
	/** \brief Assignment of Orientation into a TransformationMatrix */
	TransformationMatrix& operator=(const Orientation &inOrientation);
	/** \brief Assignment of RelativePosition into a TransformationMatrix */
	TransformationMatrix& operator=(const RelativePosition &inRelativePosition);

	/** \brief Reverse the transformation 
	 * Ref: <a href="linkURL">http://www.cse.psu.edu/~rcollins/CSE486/lecture12.pdf</a>
	*/
	TransformationMatrix Reverse();

public:
	/** \brief Structure that holds the transformation information */
	TransformationArray matrix; 
};

/** \brief The TransformationVector class supports the basic operators on a 
 *          1x4 vector.
 *  \notes	The vectors can be used as operands in affine transformation operations
 *          or as convenient placeholders for sub-sections of TransformationMatrix.
*/
class TransformationVector
{
public:
	/** \brief Constructor */
	TransformationVector();	
	/** \brief Constructor from cartesian coordinates */
	TransformationVector(const CartesianCoord &inCartesian);
	/** \brief Constructor from spherical coordinates */
	TransformationVector(const SphericalCoord &inSpherical);
	/** \brief Constructor from an orientation */
	TransformationVector(const Orientation &inOrientation);

	
	/** \brief Assignment of CartesianCoord into a TransformationVector*/
	TransformationVector& operator=(const CartesianCoord &inCartesian);
	/** \brief Assignment of SphericalCoord into a TransformationVector */
	TransformationVector& operator=(const SphericalCoord &inSpherical);
	/** \brief Assignment of Orientation into a TransformationVector */
	TransformationVector& operator=(const Orientation &inOrientation);

public:
	/** \brief Structure that holds the transformation information */
	TransformationRow vect;
};



/** \brief Concatenating transformation vectors by vector multiplication */
TransformationVector operator * (float scalarLeft, const TransformationVector& right);
/** \brief Concatenating transformation vectors by vector multiplication */
TransformationVector operator * (const TransformationVector& left, float scalarRight);

/** \brief Concatenating transformation matrices by matricial multiplication */
TransformationMatrix operator * (const TransformationMatrix &left, const TransformationMatrix &right);
/** \brief Addition of two transformation matrices */
TransformationMatrix operator + (const TransformationMatrix &left, const TransformationMatrix &right);
/** \brief Substaction of two transformation matrices */
TransformationMatrix operator - (const TransformationMatrix &left, const TransformationMatrix &right);
/** \brief Multiplication of scalar by TransformationMatrix*/
TransformationMatrix operator * (float scalarLeft, const TransformationMatrix &right);
/** \brief Multiplication of TransformationMatrix by a scalar*/
TransformationMatrix operator * (const TransformationMatrix &left, float scalarRight);
/** \brief Concatenation of a TransformationMatrix and Transformation Vector*/
TransformationVector operator * (const TransformationMatrix &left, const TransformationVector &right); 
/** \brief Concatenation of a TransformationVector and Transformation matrix*/
TransformationVector operator * (const TransformationVector &left, const TransformationMatrix &right);

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
	/** \brief Pointer to a Transformation node */
	typedef boost::shared_ptr<TransformationNode> Ptr;
	/** \brief Const Pointer to a Transformation node */
	typedef boost::shared_ptr<TransformationNode> ConstPtr;
	/** \brief List of transformations applied to coordinates  */
	typedef boost::container::vector<TransformationNode::Ptr> List;
	/** \brief Pointer to a List of transformations applied to coordinates  */
	typedef TransformationNode::List *ListPtr;

public:
	/** \brief Constructor of TransformationNode based on Cartesian Coordinates and Orientation  */
	TransformationNode(const CartesianCoord &inCartesian, const Orientation &inOrientation);
	/** \brief Constructor of TransformationNode based on RelativePosition  */
	TransformationNode(const RelativePosition &inRelativePosition);

	/** \brief Constructor for an empty TransformationNode  */
	TransformationNode();

	/** \brief Add a transformation to ane xisting transformation node  */
	void AddChild(TransformationNode::Ptr inChild);
	

	/** \brief Calculate the resulting transformation across the trasformation chain  */
	void RefreshGlobal();

	/** \brief Convert the provided coordinates to the number of levels provided  */
	CartesianCoord ToReferenceCoord(eCoordLevel inLevel, const CartesianCoord & inCoord);
	/** \brief Convert the provided coordinates from the number of levels provided to the root coordinates  */
	CartesianCoord FromReferenceCoord(eCoordLevel inLevel, const CartesianCoord & inCoord);

public:
	/** \brief Parent in the transformation chain  */
	TransformationNode::Ptr parent;
	/** \brief List of children in the transformation chain  */
	TransformationNode::List children;
	
	/** \brief Relative position in current coordinate system*/
	RelativePosition relativePosition;

	/** \brief Computed list of affine transformations (used to optimize compute time)  */
	TransformationMatrixSteps transformations;
};

/** \brief CameraCalibration parameters are used to convert from CameraCoordinates to 
  *        pixel coordinates in the image.
  * 
  * \reference: http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html

  *	\Notes We use a calibration system that uses parameters similar to the OpenCV
  *        distorsion matrixes and camera calibration matrix.
  *		   This means we can use the results from OpenCV calibration procedures in configuration files.
*/
class CameraCalibration
{
public:	
	/** \brief Constriuctor, using the openCV standard parameters  */
	CameraCalibration(int inFrameWidthInPixels = 640, int inFrameHeightInPixels = 480, 
					  float inFovWidth = 0.0, float inFovHeight = 0.0, 
					  float inFocalLengthX = 0.0, float inFocalLengthY = 0.0,
					  float inCenterX = 0.0, float inCenterY = 0,  
					  float inRadialK1 = 0, float inRadialK2 = 0, float inRadialK3 = 0, 
					  float inTangentialP1 = 0, float inTangentialP2 = 0);

	/** \brief convert camera coordinates to image (XY) coordinates of the camera
	  *  \param[out] cameraX Position of the pixel in the X axis.
	  *  \param[out] cameraY Position of the pixel in the Y axis with (0, 0) being top left.
	  *  \returns Returns true if the point is in front of the camera.  Returns false for points behind the camera.
	  * 
	  */
	bool ToFrameXY(const CartesianCoord &coordInCameraCart, int &cameraX, int &cameraY) const;

	/** \brief Calculate the focalLengths calibration parameters from the FOVs that are already stored.
	  */
	void CalculateFocalLengthsFromFOVs();

public:
	/* \brief Effective FOV Width 
	 *  \remarks FOV is typically related to focal length.  However, the effective FOV is used here for display purposes, 
	 *           as FOV may be constrained by mechanical constraints on some devices.
	 */
	float fovWidth; 

	/* \brief Effective FOV Height 
	 *  \remarks FOV is typically related to focal length.  However, the effective FOV is used here for display purposes, 
	 *           as FOV may be constrained by mechanical constraints on some devices 
	 */
	float fovHeight;
	/** \brief Camera frame width in pixels */
	int frameWidthInPixels; 
	/** \brief Camera frame height in pixels */
	int frameHeightInPixels; 
	/** \brief Camera focal length and scaling (accounts for pixel size in x direction) */
	float focalLengthX; 
	/** \brief Camera focal length and scaling (accounts for pixel size in y direction) */
	float focalLengthY;
	/** \brief sensorCenter offset in x direction*/
	float centerX; 
	/** \brief sensorCenter offset in y direction*/
	float centerY; 
	/** \brief radial distorsion (barrel and pincushion) parameter 1*/
	float radialK1; 
	/** \brief radial distorsion (barrel and pincushion) parameter 2*/
	float radialK2; 
	/** \brief radial distorsion (barrel and pincushion) parameter 3*/
	float radialK3;
	/** \brief tangential distorsion (keystone) parameter 1*/
	float tangentialP1;
	/** \brief tangential distorsion (keystone) parameter 2*/
	float tangentialP2;
};

/** \brief Convert Camera pixel coordinates into a straigtened X-Y coordinates, accounting for camera distorsion*/
bool CameraCoordToFrameXY(float cameraFovWidthInRad, float cameraFovHeightInRad, int frameWidthInPixels, int frameHeightInPixels, const CartesianCoord &coordInCameraCart, int &cameraX, int &cameraY, float barrelK1 = 0.0, float barrelK2 = 0.0);

SENSORCORE_END_NAMESPACE
#endif // SENSORCORE_COORDINATESYSTEM_H
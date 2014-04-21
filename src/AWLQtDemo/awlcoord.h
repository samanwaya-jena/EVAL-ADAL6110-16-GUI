#ifndef AWL_COORD_H
#define AWL_COORD_H

#include <cmath>

using namespace std;

namespace awl
{


class PolarCoord;
class CartesianCoord;

/** \brief Structure containing relativePosition, in polar coordinates. */
/*  \notes polar coordinates are using left-hand notation				*/
/*         rho is distance.												*/
/*         theta is angle from z axis, clockwise.						*/
/*         phi is angle from x axis, counterclockwise					*/

class PolarCoord 
{
public:
	PolarCoord();
	PolarCoord(float inRho, float inTheta, float inPhi);
	PolarCoord(const PolarCoord &inPolar);	
	PolarCoord(const CartesianCoord &inCartesian);

	void Set(float inRho, float inTheta, float inPhi);

	static CartesianCoord ToCartesian(const PolarCoord &inPolar);
	CartesianCoord ToCartesian() const;
#if 0
	CartesianCoord& operator=(const PolarCoord &sourcePolar);
#endif
public:
	float rho;
	float theta;
	float phi;
};

/** \brief Structure containing relativePosition, in cartesian coordinates. */

class CartesianCoord 
{
public:
	CartesianCoord();
	CartesianCoord(float inX, float inY, float inZ);
	CartesianCoord(const CartesianCoord &inCartesian);
	CartesianCoord(const PolarCoord &inPolar);

	void Set(float x, float y, float z);

	static PolarCoord ToPolar(const CartesianCoord &inCartesian);
	PolarCoord ToPolar() const;
#if 0
	PolarCoord & operator=(const CartesianCoord &sourceCartesian);
#endif
//	operator PolarCoord() const;

public:
	float x;
	float y;
	float z;
};


void ConvertSensorToVehicleCoord(const PolarCoord &sensorCoord, 
					  const float pitch, const float yaw, const float roll, 
					  const float xOffset, const float yOffset, const float zOffset, 
					  CartesianCoord &vehicleCoord);
} // namespace awl
#endif // AWL_COORD_H
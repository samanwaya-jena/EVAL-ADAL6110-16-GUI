#ifndef AWL_COORD_H
#define AWL_COORD_H

#include <cmath>

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

class PolarCoord;
class CartesianCoord;

typedef enum eCoordLevel
{
	eVehicleCoord = 0,		// Vehicle position in world
	eReceiverCoord = 1,		// Receiver position on vehicle		
	eSensorCoord = 2		// Sensor posistion in receiver
}
eCoordLevel;






/** \brief Structure containing relativePosition, in polar coordinates. */
/*  \notes polar coordinates are using right-handed notation				*/
/*         rho is distance.												*/
/*         theta is angle from z axis, clockwise.						*/
/*         phi is angle from x axis, counterclockwise					*/
//  We use the notation that is common practice in physics, 
//  As specified by ISO standard 31-11.
//  This means use positive sign for azimuth angles that are measured in 
//  the counter-clockwise sense from the reference direction on the 
//  reference plane, as seen from the zenith side of the plane

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
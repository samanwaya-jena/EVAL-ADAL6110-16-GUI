

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
 
#define DEG2RAD(angle) (angle * 3.14159265358979323846 / 180.0)


 // following code compliments of Bibek Subedi
 // http://www.programming-techniques.com/2012/03/3d-rotation-algorithm-about-arbitrary.html

 float rotationMatrix[4][4]; 
 float inputMatrix[4][1] = {0.0, 0.0, 0.0, 0.0}; 
 float outputMatrix[4][1] = {0.0, 0.0, 0.0, 0.0};   
 
 void MultiplyMatrix() 
 {     
	 for(int i = 0; i < 4; i++ )
	 {         
		 for(int j = 0; j < 1; j++)
		 {             
			 outputMatrix[i][j] = 0;
			 for(int k = 0; k < 4; k++)
			 {                 
				 outputMatrix[i][j] += rotationMatrix[i][k] * inputMatrix[k][j];
			 }         
		 }     
	 } 
 } 


 void SetUpRotationMatrix(float angle, float u, float v, float w) 
 {     
	 float L = (u*u + v * v + w * w);     
	 float u2 = u * u;     
	 float v2 = v * v;     
	 float w2 = w * w;
	 float cosA = cos(angle);
	 float sinA = sin(angle);
	 float sqrtL = sqrt(L);

	 rotationMatrix[0][0] = (u2 + (v2 + w2) * cosA) / L;
	 rotationMatrix[0][1] = (u * v * (1 - cosA) - w * sqrtL * sinA) / L;
	 rotationMatrix[0][2] = (u * w * (1 - cosA) + v * sqrtL * sinA) / L;
	 rotationMatrix[0][3] = 0.0;       
	 rotationMatrix[1][0] = (u * v * (1 - cosA) + w * sqrtL * sinA) / L;
	 rotationMatrix[1][1] = (v2 + (u2 + w2) * cosA) / L;
	 rotationMatrix[1][2] = (v * w * (1 - cosA) - u * sqrtL * sinA) / L;     
	 rotationMatrix[1][3] = 0.0;       
	 rotationMatrix[2][0] = (u * w * (1 - cosA) - v * sqrtL * sinA) / L;     
	 rotationMatrix[2][1] = (v * w * (1 - cosA) + u * sqrtL * sinA) / L;     
	 rotationMatrix[2][2] = (w2 + (u2 + v2) * cosA) / L;     
	 rotationMatrix[2][3] = 0.0;       
	 rotationMatrix[3][0] = 0.0;     
	 rotationMatrix[3][1] = 0.0;     
	 rotationMatrix[3][2] = 0.0;     
	 rotationMatrix[3][3] = 1.0; 
 }   

 void SetUpInputMatrix(float originalX, float originalY, float originalZ)
 {
	 inputMatrix[0][0] = originalX;     
	 inputMatrix[1][0] = originalY;     
	 inputMatrix[2][0] = originalZ;     
	 inputMatrix[3][0] = 1.0;       
 }

  void SetUpInputMatrix(const CartesianCoord cart)
 {
	 inputMatrix[0][0] = cart.x;     
	 inputMatrix[1][0] = cart.y;     
	 inputMatrix[2][0] = cart.z;     
	 inputMatrix[3][0] = 1.0;       
 }

 void awl::ConvertSensorToVehicleCoord(const PolarCoord &sensorCoord, 
							 const float pitch, const float yaw, const float roll, 
							 const float xOffset, const float yOffset, const float zOffset, 
							CartesianCoord &vehicleCoord)
 {
	 vehicleCoord = sensorCoord;

 	 // Roll
	 SetUpInputMatrix(vehicleCoord);
	 SetUpRotationMatrix(DEG2RAD(roll), 0, 1, 0);
	 MultiplyMatrix();
	 vehicleCoord.x = outputMatrix[0] [0];
	 vehicleCoord.y = outputMatrix[1] [0];
	 vehicleCoord.z = outputMatrix[2] [0];

	 // Pitch
	 SetUpInputMatrix(vehicleCoord);
	 SetUpRotationMatrix(DEG2RAD(pitch), 1, 0, 0);
	 MultiplyMatrix();
	 vehicleCoord.x = outputMatrix[0] [0];
	 vehicleCoord.y = outputMatrix[1] [0];
	 vehicleCoord.z = outputMatrix[2] [0];

	 // Yaw
	 // Yaw is clockWise in degrees, counterClockWise in our coordinate system,  so we reverse the sign
	 SetUpInputMatrix(vehicleCoord);
	 SetUpRotationMatrix(DEG2RAD(yaw), 0, 0, 1);
	 MultiplyMatrix();
	 vehicleCoord.x = outputMatrix[0] [0];
	 vehicleCoord.y = outputMatrix[1] [0];
	 vehicleCoord.z = outputMatrix[2] [0];

	 vehicleCoord.x+=xOffset;
	 vehicleCoord.y+=yOffset;
	 vehicleCoord.z+=zOffset;
 }

	
PolarCoord::PolarCoord():
rho(0.0),
theta(0.0),
phi(0.0)
{
}

PolarCoord::PolarCoord(float inRho, float inTheta, float inPhi):
rho(inRho),
theta(inTheta),
phi(inPhi)
{
}

PolarCoord::PolarCoord(const PolarCoord &inPolar)
{
	rho = inPolar.rho;
	theta = inPolar.theta;
	phi = inPolar.phi;
}

PolarCoord::PolarCoord(const CartesianCoord &inCartesian)
{
	*this = inCartesian.ToPolar();
}

void PolarCoord::Set(float inRho, float inTheta, float inPhi)
{
	rho = inRho;
	theta = inTheta;
	phi = inPhi;
}


CartesianCoord PolarCoord::ToCartesian(const PolarCoord &inPolar)
{
	CartesianCoord cartesian;
	float sinTheta = sin(inPolar.theta);
	cartesian.x = inPolar.rho * sinTheta * cos(inPolar.phi);
	cartesian.y = inPolar.rho * sinTheta * sin(inPolar.phi);
	cartesian.z = inPolar.rho * cos(inPolar.theta);
	return(cartesian);
}


CartesianCoord PolarCoord::ToCartesian() const 
{
	return(ToCartesian(*this));
}

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

CartesianCoord::CartesianCoord(const PolarCoord &inPolar)

{
	*this = inPolar.ToCartesian();
}

void CartesianCoord::Set(float inX, float inY, float inZ)
{
	x = inX;
	y = inY;
	z = inZ;
}

PolarCoord CartesianCoord::ToPolar(const CartesianCoord &inCartesian) 
{
	PolarCoord polar;

	polar.rho = sqrt((inCartesian.x*inCartesian.x) + (inCartesian.y*inCartesian.y) + (inCartesian.z * inCartesian.z));

	if (polar.rho != 0.0)
	{
		polar.theta = acos(inCartesian.z/polar.rho);
	}

	if (inCartesian.x != 0.0) 
	{
		polar.phi = atan(inCartesian.y / inCartesian.x);
	}

	return(polar);
}

PolarCoord CartesianCoord::ToPolar() const
{
	return(ToPolar(*this));
}


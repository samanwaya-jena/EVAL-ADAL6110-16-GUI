#ifndef DETECTIONSTRUCT_H
#define DETECTIONSTRUCT_H

#include <QVector>

/** \brief Structure containing detected object information. */
typedef struct
{
    int id;									// Id of the detected object
    float distanceRadial;					// Distance from sensor (Radial)
	float distanceLongitudinal;				// Distance from bumper
    float angle;							// Angle where the object is detected (Center)
    float angleWidth;						// Size of the object (in fact, angle width of the sensor)
	int fromChannel;						// Channel where the object was detected
}DetectionData;

/** \brief Structure containing 2D View configuration. */
typedef struct
{
    float shortRangeDistance;				// Max range for short range sensor (Including limited range)
    float shortRangeDistanceStartLimited;	// Limited range for short range sensor
    float shortRangeAngle;					// Max angle width for short range sensor (Including limited angle)
    float shortRangeAngleStartLimited;		// Limited angle for short range sensor 

    float longRangeDistance;				// Max range for long range sensor (Including limited range)
    float longRangeDistanceStartLimited;	// Limited range for long range sensor
    float longRangeAngle;					// Max angle width for long range sensor (Including limited angle)
    float longRangeAngleStartLimited;		// Limited angle for long range sensor 

    float sensorDepth;				// Sensor distance from bumper 
    float sensorHeight;				// Sensor distance from ground
}ConfigSensor;

typedef QVector<DetectionData> DetectionDataVect;

#endif // DETECTIONSTRUCT_H

#ifndef DETECTIONSTRUCT_H
#define DETECTIONSTRUCT_H

#include <boost/container/vector.hpp>

#include "Tracker.h"

namespace awl
{

typedef boost::container::vector<Detection::Ptr> DetectionDataVect;

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



} // namespace awl
#endif // DETECTIONSTRUCT_H

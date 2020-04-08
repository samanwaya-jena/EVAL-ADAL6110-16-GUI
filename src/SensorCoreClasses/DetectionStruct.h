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

#ifndef SENSORCORE_DETECTION_STRUCT_H
#define SENSORCORE_DETECTION_STRUCT_H

#include <stdint.h>
#include <string>
#include <queue>
#include <boost/shared_ptr.hpp>
#include <boost/container/vector.hpp>

#include "SensorCoreClassesGlobal.h"
#include "CoordinateSystem.h"
#include "DetectionStruct.h"


SENSORCORE_BEGIN_NAMESPACE


//const float NAN = std::numeric_limits<float>::quiet_NaN ();
#ifdef _MSC_VER
#ifndef NAN
#define NAN std::numeric_limits<float>::quiet_NaN()
#endif // IFNDEF
#define isNAN(val) _isnan(val)
#else //_MSC_VER not defined
#define isNAN(val) (val == NAN)
#endif

typedef uint16_t TrackID;
typedef uint32_t FrameID;

class AlertCondition;
class Detection;
class AScan;
class Track;
class SensorFrame;
class VoxelFrame;


/** \brief TimeStamp is the standard format for storing time elapsed
* \author Jean-Yves Deschênes
*/

typedef double Timestamp;

/** \brief VoxelMask struct describes receiverVoxel bit mask used in most data structures
*        and communications
* \author Jean-Yves Deschênes
*/

typedef union
{
	uint16_t wordData;
	struct  {
		bool voxel0 : 1;
		bool voxel1 : 1;
		bool voxel2 : 1;
		bool voxel3 : 1;
		bool voxel4 : 1;
		bool voxel5 : 1;
		bool voxel6 : 1;
		bool voxel7 : 1;
	} bitFieldData;;
} VoxelMask, AlertVoxelMask;

/**\brief A CellID is a unique identifier of a "pixel" position (column, row) within a a receiver array.
*/

class CellID
{
public:
	CellID(int inColumn, int inRow) : column(inColumn), row(inRow) {};
	CellID() : column(0), row(0) {};

public:
	int column;
	int row;
};

/** \brief The Alert class defines alert conditions.
*/

class AlertCondition {
public: 
	typedef boost::shared_ptr<AlertCondition> Ptr;
	typedef boost::shared_ptr<AlertCondition> ConstPtr;
	typedef boost::container::vector<AlertCondition::Ptr> Vector;

	typedef enum ThreatLevel {
		eThreatNone = 0,  // No threat level assigned
		eThreatLow = 1,   // target detected, threat level low
		eThreatWarn = 2,  // Target may have collision course, but not acertained yet
		eThreatCritical = 3, // target is within collision range
		eThreatOutlineOnly = 4 // target is within collision range
	}
	ThreatLevel;

	typedef enum AlertType {
		eAlertInvalid = 0,
		eAlertDistanceWithin = 1, // Alert based on distance  within range specified
		eAlertDistanceOutside = 2, // Alert based on distance  outside range specified
		eAlertSpeed = 3, // Alert based on speed range;
		eAlertAcceleration = 4,    // Alert based on accel range;
		eAlertDecelerationToStop = 5, // Alert based on deceleration to stop
		eAlertTTC = 6		// Alert based on time to collision;

	}
	AlertType;

public:
	AlertCondition() {};
	AlertCondition(AlertCondition::AlertType inAlertType, int inReceiverID, VoxelMask inVoxelMask, float inMinRange, float inMaxRange, ThreatLevel inThreatLevel = eThreatNone);
	AlertCondition(AlertCondition &sourceCondition);

	static AlertCondition::ThreatLevel FindTrackThreat(int inReceiverID, boost::shared_ptr<Track> track);

	int	GetReceiverID() { return(receiverID); }
	AlertVoxelMask	GetVoxelMask() { return(alertVoxelMask); }
	float GetMinRange() { return (minRange); }
	float GetMaxRange() { return (maxRange); }
	ThreatLevel GetThreatLevel (){ return(threatLevel); }

	static void Store(AlertCondition::Ptr storedCondition) { globalAlertsVector.push_back(storedCondition); }

	AlertCondition::Vector GetAlertConditions() { return (globalAlertsVector); }
public:

	/** \brief Type of condition for the alert */
	AlertType alertType;

	/** \brief sensor ID of the sensor where detection origins from */
	int	  receiverID;

	/** \brief voxelMask of the voxelss where detection origins from */
	AlertVoxelMask	  alertVoxelMask;

	/** \brief minimum range for triggerting of the alert. Nature depends on AlertType */
	float	  minRange;

	/** \brief maximum range for triggerting of the alert. Nature depends on AlertType */
	float	  maxRange;

	ThreatLevel threatLevel;


protected:
	static AlertCondition::Vector globalAlertsVector;
};

/** \brief The Detection class corresponds to a single detectio returned by the receiver.
           It holds the distance for the detection, its intensity, threat level
*/

class Detection 
{
	friend class AlertCondition;

public:
	typedef boost::shared_ptr<Detection> Ptr;
    typedef boost::shared_ptr<Detection> ConstPtr;
	typedef boost::container::vector<Detection::Ptr> Vector;

public:
	Detection();
	Detection(int inReceiverID, CellID inCellID, int inDetectionID);
	Detection(int inReceiverID, CellID inCellID, int inDetectionID, float inDistance, float inIntensity, float inVelocity,
	          float inTimeStamp, float inFirstTimeStamp, TrackID inTrackID, AlertCondition::ThreatLevel inThreatLevel = AlertCondition::eThreatNone);
	
	int	GetReceiverID() {return(receiverID);}	
	int	GetDetectionID() {return(detectionID);}
	CellID	GetCellID() {return(cellID);}

public:
	/** \brief sensor ID of the sensor where detection origins from */
	int	  receiverID;

	/** \brief cellID of the voxel where detection origins from */
	CellID	  cellID;

	/** \brief index of the detection within the voxel */
	int	  detectionID;

	/** \brief distance, in meters */
	float distance;

	/** \brief intensity ratio exprtessed as SNR */
	float intensity;

	/** \brief Velocity, in m/s */
	float velocity;

	/** \brief acceleration, in m/s squared */
	float acceleration;

	/** \brief Time to collision, in seconds */

	float timeToCollision;

	/** \brief Required acceleration to zero speed, in m/s2seconds */
	float decelerationToStop;

	/** \brief Track Positive detection probability estimate */
	float probability;

	/** \brief Timestamp, in frames */
	Timestamp timeStamp;

	/** \brief First time stamp at which the target was acquired. */
	Timestamp firstTimeStamp;

	/** \brief Track ID from which the detection was generated, if that is the case. */
	TrackID	trackID;

	/** \brief Coordinates of detection relative to sensor */
	CartesianCoord relativeToSensorCart;
	SphericalCoord	   relativeToSensorSpherical;

	/** \brief Coordinates of detection relative to vehicule bumper */
	CartesianCoord relativeToVehicleCart;
	SphericalCoord	   relativeToVehicleSpherical;

	/** \brief Coordinates of detection relative to world */
	CartesianCoord relativeToWorldCart;
	SphericalCoord	   relativeToWorldSpherical;

	/** \brief Threat level associated to detection */
	AlertCondition::ThreatLevel	threatLevel;
};

class AScan
{
public:
	typedef boost::shared_ptr<AScan> Ptr;
    typedef boost::shared_ptr<AScan> ConstPtr;
	typedef boost::container::vector<AScan::Ptr> Vector;
public:
	AScan(int inReceiverID, CellID inCellID, int inChannelNo)
	{
		receiverID = inReceiverID;
		cellID = inCellID;
		channelNo = inChannelNo;

		sampleOffset = 0;
		sampleCount = 0;
		sampleSize = 0;
		sampleSigned = false;
		samples = 0;
	}
/*
	~AScan()
	{
		//if (samples) delete samples;
	}
*/
	float GetScaleFactorForRange(int range);
	void FindMinMaxMean(float *min, float *max, float *mean);
	bool operator < (const AScan& cmp)  const 
	{ 
		if (cellID.row > cmp.cellID.row) return false;
		if (cellID.row < cmp.cellID.row) return true;
		// same row
		if (cellID.column < cmp.cellID.column) return true;
		return(false);
	}
public:
	int receiverID;
	/** \brief cellIDID of the voxel where detection origins from */
	CellID cellID;

	/* Internal channelNo is used for display purposes */
	int channelNo;

	size_t sampleOffset; // Samples to ignore at the begining

	size_t sampleDrop;  // Samples to ignore at the end

	size_t sampleCount; // Samples to considere

	size_t sampleSize; // Size of a sample in bytes

	bool sampleSigned; // Are the sample signed ?

	uint8_t *samples;


};

/** \brief A track corresponds to an obstacle tracking based on a singl;e or multiple detections,
           and gives information on the evolution of the detected obstacle across multiple frames
*/

class Track
{
public:
	typedef boost::shared_ptr<Track> Ptr;
    typedef boost::shared_ptr<Track> ConstPtr;
	typedef boost::container::vector<Track::Ptr> Vector;

	Track(TrackID trackID);

	int	GetTrackID() {return(trackID);}

	// A track is built from 2 distinct CAN message sections.  Make sure all parts are entered before a track is completed.
	// Note: (Before 2018-02-18, it used to be 4 parts, but messages of part 3 and 4 were deprecated.
	bool IsComplete() { return (part1Entered && part2Entered); };
public:
	/** \brief Track ID. */
	TrackID	trackID;

	/** \brief distance, in meters */
	float distance;

	/** \brief Velocity, in m/s.  Positive velocity means target is moving away from sensor */
	float velocity;

	/** \brief acceleration, in m/s squared */
	float acceleration;

	/** \brief Time to collision, in seconds */
	float timeToCollision;

	/** \brief Required acceleration to zero speed, in m/s2seconds */
	float decelerationToStop;

	/** \brief Track Positive detection probability estimate */
	float probability;

	/** \bried Track relative intensity, expressed as SNR */
	float intensity;

	/** \brief Timestamp, in frames */
	Timestamp timeStamp;

	/** \brief First time stamp at which the target was acquired. */
	Timestamp firstTimeStamp;

	/** \brief Threat level associated to detection */
	AlertCondition::ThreatLevel	threatLevel;

	/** \brief Channels in which detections were made for the track **/
	VoxelMask trackChannels;

	/** \brief Main voxel in which detections is made for the track **/
	uint16_t trackMainVoxel;


	// A track is built from up to 4 message sections (in CAN).  Make sure all parts are entered before a track is completed.

	bool part1Entered;
	bool part2Entered;
	bool part3Entered;  // Not required anymore, message was deprecated.
	bool part4Entered;  // Note required anymore, message was deprecated.
protected:
};


/** \brief The SensorFrame class holds all voxel detections (voxelFrames),
		corresponding to a unique time frame.
*/
class SensorFrame
{
public:
	typedef boost::shared_ptr<SensorFrame> Ptr;
    typedef boost::shared_ptr<SensorFrame> ConstPtr;
	typedef std::deque<SensorFrame::Ptr> Queue;

public:
	SensorFrame(int inReceiverID, FrameID inFrameID, int inVoxelQty);
	virtual ~SensorFrame() {};

	int GetReceiverID() { return(receiverID);}
	FrameID	GetFrameID() {return(frameID);}


	Detection::Ptr MakeUniqueDetection(Detection::Vector &detectionVector, CellID inCellID, int detectionID);
	bool FindDetection(Detection::Vector &detectionVector, CellID inCellID, int inDetectionID, Detection::Ptr &outDetection);

	AScan::Ptr MakeUniqueAScan(AScan::Vector &detectionVector,  int receiverID, CellID inCellID, int inChannelID);
	bool FindAScan(AScan::Vector &detectionVector, int inReceiverID, CellID inCellID, AScan::Ptr &outAScan);

public:
	int receiverID;
	FrameID frameID;
	int voxelQty;

	Detection::Vector rawDetections;		// Raw detections as acquired from device
	AScan::Vector aScans;

	Track::Vector tracks; 

	// Timestamp im milliseconds, elapsed from start of thread.
	Timestamp timeStamp;

};

/** \brief The AcquisitionSequence is a list of all the information acquired from the receiver,
		across multiple time frames.
*/
class AcquisitionSequence
{
public:
	typedef boost::shared_ptr<AcquisitionSequence> Ptr;
    typedef boost::shared_ptr<AcquisitionSequence> ConstPtr;


public:
	AcquisitionSequence();

//	SensorFrame &operator[](int frameIndex) {return(*(sensorFrames[frameIndex]));}

	virtual ~AcquisitionSequence() {};
	
	/** \brief Get the frameID of the last complete frame
	*/
	FrameID	GetLastFrameID();

	FrameID AllocateFrameID();

	bool FindTrack(SensorFrame::Ptr currentFrame,TrackID trackID, Track::Ptr &outTrack);
	Track::Ptr MakeUniqueTrack(SensorFrame::Ptr currentFrame,TrackID trackID);

	bool FindSensorFrame(FrameID frameID, SensorFrame::Ptr &outSensorFrame);

public: 
	// Queue of the stored sensor frames.
	// The acquisitionSequence stores multiple frames, in support of asynchonous operations,
	// as well as to support tracking algorithms
	SensorFrame::Queue sensorFrames;
	
	// Frame ID counter.  Each SensorFrame within an acquisition sequence has a unique frame ID.
	FrameID frameID;
};

SENSORCORE_END_NAMESPACE
#endif // SENSORCORE_DETECTION_STRUCT_H

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

/** \brief TrackID is a unique identifier for a Track
*/
typedef uint16_t TrackID;
/** \brief FrameID is a unique identifier for a Frame
*/
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

/** \brief The AlertCondition class defines and manages alert conditions.
 *
 *         Within and application, there is a unique global alert storage.
 *         Tracks can be evaluated to see if they meet conditions for these alerts using AlertCondition::FindTrackThreat()
 *         The most critical matching alert level is reported.
*/
class AlertCondition {
public: 
	typedef boost::shared_ptr<AlertCondition> Ptr;
	typedef boost::shared_ptr<AlertCondition> ConstPtr;
	typedef boost::container::vector<AlertCondition::Ptr> Vector;


	/** \brief The AlertCondition threat level associates a threat level to the specified condition.
	*/
	typedef enum ThreatLevel {
		/**No threat level assigned */
		eThreatNone = 0, 		
		/**target detected, threat level low*/
		eThreatLow = 1,    
		/**Target may have collision course, but not acertained yet*/
		eThreatWarn = 2,  
		/**target is within collision range*/
		eThreatCritical = 3,  
		/**No relation to threat level, just used for internal manipulation purposes*/
		eThreatOutlineOnly = 4 
	}
	ThreatLevel;

	/** \brief Specify the type of logic that will be used to evaluate the alert.
	*/
	typedef enum AlertType {
		/**Invalid Alert - Use to document unused alerts or at end of alert list*/
		eAlertInvalid = 0,
		/**Alert based on distance  within range specified*/
		eAlertDistanceWithin = 1,  
		/**Alert based on distance  outside range specified*/
		eAlertDistanceOutside = 2,  
		/**Alert based on speed range*/
		eAlertSpeed = 3,
		/**Alert based on accel range*/
		eAlertAcceleration = 4,    
		/**Alert based on deceleration to stop*/
		eAlertDecelerationToStop = 5,  
		/**Alert based on time to collision*/
		eAlertTTC = 6		
	}
	AlertType;  

public:
	/** \brief Empty Constructor
	*/
	AlertCondition():
		alertType(eAlertInvalid),
		receiverID(0),
		minRange(0.0),
		maxRange(10.0),
	    threatLevel(eThreatNone)
	    {
		alertVoxelMask.wordData = 0;
	    };

	/** \brief Constructor based on detailed specs
	*/
	AlertCondition(AlertCondition::AlertType inAlertType, int inReceiverID, VoxelMask inVoxelMask, float inMinRange, float inMaxRange, AlertCondition::ThreatLevel inThreatLevel = eThreatNone);

	/** \brief Copy constructor.
	*/
	AlertCondition(AlertCondition &sourceCondition);

	/** \brief Scan the global alert storage to see if the track meets alert conditions.
	  *  The most critical alert level detected will be returned.
	*/
	static AlertCondition::ThreatLevel FindTrackThreat(int inReceiverID, boost::shared_ptr<Track> track);


	int	GetReceiverID() { return(receiverID); }
	AlertVoxelMask	GetVoxelMask() { return(alertVoxelMask); }
	float GetMinRange() { return (minRange); }
	float GetMaxRange() { return (maxRange); }
	ThreatLevel GetThreatLevel (){ return(threatLevel); }

	/** \brief Store the alert to the global alert storageFind the threat level of a track based based on highest altert detected.
	*/
	static void Store(AlertCondition::Ptr storedCondition) { globalAlertsVector.push_back(storedCondition); }

	/** \brief Return a pointer to the global alert storage.
	*/
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

	/** \brief Threat level for the current alert */
	ThreatLevel threatLevel;


protected:
	/** \brief Unique global alert storage for the application */
	static AlertCondition::Vector globalAlertsVector;
};

/** \brief The Detection class corresponds to a single detection returned by the receiver.
  *         It holds the distance for the detection, its intensity, threat level.
  *		   For optimization purposes, multiple transformations of the coordinates for the detection are
  *        pre-calculated and stored as members. 
*/
class Detection 
{
	friend class AlertCondition;

public:
	typedef boost::shared_ptr<Detection> Ptr;
    typedef boost::shared_ptr<Detection> ConstPtr;
	typedef boost::container::vector<Detection::Ptr> Vector;

public:
	/** \brief Empty constructor */
	Detection();
	/** \brief Constructor with cell identification only */
	Detection(int inReceiverID, CellID inCellID, int inDetectionID);

	/** \brief Constructor with cell identification and complete detection information */
	Detection(int inReceiverID, CellID inCellID, int inDetectionID, float inDistance, float inIntensity, float inVelocity,
	          float inTimeStamp, float inFirstTimeStamp, TrackID inTrackID, AlertCondition::ThreatLevel inThreatLevel = AlertCondition::eThreatNone);
	
	/** \brief Accessor to the receiverID */
	int	GetReceiverID() {return(receiverID);}

	/** \brief Accessor to the detectionID */
	int	GetDetectionID() {return(detectionID);}
	
	/** \brief Accessor to the cellID */
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

	/** \brief Cartesian Coordinates of detection relative to sensor */
	CartesianCoord relativeToSensorCart;
	/** \brief Spherical Coordinates of detection relative to sensor */
	SphericalCoord	   relativeToSensorSpherical;

	/** \brief Cartesian Coordinates of detection relative to vehicule bumper */
	CartesianCoord relativeToVehicleCart;
	/** \brief Spherical Coordinates of detection relative to vehicule bumper */
	SphericalCoord	   relativeToVehicleSpherical;

	/** \brief Cartesian Coordinates of detection relative to world */
	CartesianCoord relativeToWorldCart;
	/** \brief Spherical Coordinates of detection relative to world */
	SphericalCoord	   relativeToWorldSpherical;

	/** \brief Threat level associated to detection */
	AlertCondition::ThreatLevel	threatLevel;
};

/** \brief The AScan holds all the data points that correspond to a waveform
  *         acquired on a single voxel.
*/
class AScan
{
public:
	typedef boost::shared_ptr<AScan> Ptr;
    typedef boost::shared_ptr<AScan> ConstPtr;
	typedef boost::container::vector<AScan::Ptr> Vector;
public:
	/** \brief Constructor */

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
		sampleDrop = 0;
	}
/*
	~AScan()
	{
		//if (samples) delete samples;
	}
*/
	/** \brief Returns a scale factor for the vertical axis based on the min and max values contained in the waveform */
	float GetScaleFactorForRange(int range);
	/** \brief Within the A-Scan Waveform points, find the min, max and mean.  Used to adjust scale and display parameters*/
	void FindMinMaxMean(float *min, float *max, float *mean);

	/** \brief Comparator is used to order AScans in order of row and column */
	bool operator < (const AScan& cmp)  const 
	{ 
		if (cellID.row > cmp.cellID.row) return false;
		if (cellID.row < cmp.cellID.row) return true;
		// same row
		if (cellID.column < cmp.cellID.column) return true;
		return(false);
	}
public:
	/** \brief ReceiverID of the originating receiver */
	int receiverID;
	/** \brief cellID of the voxel where detection origins from */
	CellID cellID;

	/** \brief Internal channelNo  is the ADC channel from sensor . It used for display purposes */
	int channelNo;

	/**Samples to ignore at the begining - Some sensors have "dead zones" of unsused acquisition*/
	size_t sampleOffset;  
	/**Samples to ignore at the end - Some sensors have "dead zones" of unused acquisition */
	size_t sampleDrop;   
	/**Useful samples within the acquisred data*/
	size_t sampleCount; 

	/**Size of a sample in bytes*/
	size_t sampleSize;  

	/**True if samples are signed value.  False if unsigned*/
	bool sampleSigned;  

	/**Pointer to the waveform sample storage */
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

	
	/** \brief Constructor */
	Track(TrackID trackID);

	/** \brief Accesssor to the TrackkID */
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


	/** \brief A track is built from up to 4 message sections (in CAN).  Make sure all parts are entered before a track is completed. */
	bool part1Entered;
	/** \brief A track is built from up to 4 message sections (in CAN).  Make sure all parts are entered before a track is completed. */
	bool part2Entered;
	/** \brief A track is built from up to 4 message sections (in CAN).  Make sure all parts are entered before a track is completed. 
	  *         part3 is not required anymore, message was deprecated 
	  */
	bool part3Entered; 
	/** \brief A track is built from up to 4 message sections (in CAN).  Make sure all parts are entered before a track is completed. 
	  * part3 is not required anymore, message was deprecated
	  */
	bool part4Entered;
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
	/** \brief Constructor */
	SensorFrame(int inReceiverID, FrameID inFrameID, int inVoxelQty);
	/** \brief Destructor*/
	virtual ~SensorFrame() {};

	/** \brief Accessor to the receiverID */
	int GetReceiverID() { return(receiverID);}

	/** \brief Accesor to the frameID */
	FrameID	GetFrameID() {return(frameID);}

	/** \brief Create a detection with the CellID, detectionID within the Frame, if said detection does not exist.
	 *  \return a pointer to the existing detection, or the newly created detection.
	 */
	Detection::Ptr MakeUniqueDetection(Detection::Vector &detectionVector, CellID inCellID, int detectionID);


	/** \brief Find a detection within the SensorFrame, with the provided CellID, detectionID.
	  *  copy the   pointer to the detection in outDetection, if found.
	  * \return true if detection was found, false if detection was not found
	*/
	bool FindDetection(Detection::Vector &detectionVector, CellID inCellID, int inDetectionID, Detection::Ptr &outDetection);


	/** \brief Create a unque AScan with the receiverID, CellID, detectionID within the Frame, if said A-Scan does not exist.
	 * \return a pointer to the existing A-Scan, or the newly created A-Scan.
	 */
	AScan::Ptr MakeUniqueAScan(AScan::Vector &detectionVector,  int receiverID, CellID inCellID, int inChannelID);

	/** \brief Find an A-Scan within the SensorFrame, with the provided ReceiverID, CellID, detectionID.
	  *  if found, copy a pointer to the A-Scan in outAScan
	  * \return true if A-Scan was found, false if A-Scan was not found
	*/
	bool FindAScan(AScan::Vector &detectionVector, int inReceiverID, CellID inCellID, AScan::Ptr &outAScan);

public:
	/**ReceiverID if the originating receiver*/
	int receiverID;
	/**FrameID associated with the detections*/
	FrameID frameID;
	/**Number of voxels in the receiver*/
	int voxelQty;

	/**Raw detections as acquired from device*/
	Detection::Vector rawDetections;		

	/**A-Scans  as acquired from device*/
	AScan::Vector aScans;

	/**Tracks  acquired' resulting from transofrmation of detections by ReceiverPostProcessor.*/
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

	/** \brief Constructor */
	AcquisitionSequence();


	/** \brief Destructor */
	virtual ~AcquisitionSequence() {};
	
	/** \brief Get the frameID of the last complete frame
	*/
	FrameID	GetLastFrameID();

	/** \brief Allocate a new frame in the allocation sequence */
	FrameID AllocateFrameID();

	/** \brief Find A track within the current SensorFrame, using the provided TrackID.
	  *        If the tack is found, copy the pointer for that Track into outTrack;
	  *  \return true if Track was found, false if Track was not found
	*/
	bool FindTrack(SensorFrame::Ptr currentFrame,TrackID trackID, Track::Ptr &outTrack);
	/** \brief Create a unque Track with the specified TrackID, within the current sensorFrame, if said Track does not exist.
	 * \return a pointer to the existing Track, or the newly created Track.
	 */
	Track::Ptr MakeUniqueTrack(SensorFrame::Ptr currentFrame,TrackID trackID);


	/** \brief Find a SensorFrame with teh provided frameID.
	  *        If the SensorFrame is found, copy the pointer for that SensorFrame into outSensorFrame.
	  *  \return true if SensorFrame was found, false if SensorFrame was not found
	*/
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

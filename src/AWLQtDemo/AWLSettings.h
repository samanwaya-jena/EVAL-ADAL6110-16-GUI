#ifndef _AWLSettings__H
#define _AWLSettings__H

#include <QtCore/qlist.h>
#include <QtCore/qstring.h>
#include <QSettings>

#include <stdint.h>

#define VelocityToKmH(velocity) (velocity * 3.6)

#define ALGO_QTY 4
#define GLOBAL_PARAMETERS_INDEX 0

namespace awl
{

	typedef int ReceiverID;

	typedef struct RegisterSettings 
	{
		QString sIndex;
		uint16_t address;
		QString sDescription;
		uint32_t value;
		int pendingUpdates;
	}
	RegisterSettings;

	typedef enum  {
		eAlgoParamInt = 0,
		eAlgoParamFloat = 1
	}
	AlgoParamType;

	typedef struct AlgorithmParameters 
	{
		QString sIndex;
		uint16_t address;
		QString sDescription;
		AlgoParamType paramType;
		uint32_t intValue;
		float floatValue;
		int pendingUpdates;
	}
	AlgorithmParameters;

	typedef struct ChannelConfig 
	{
		int channelIndex;
		float fovX;
		float fovY;
		float centerX;
		float centerY;
		float maxRange;
		QString sMaskName;
		QString sFrameName;
		uint8_t displayColorRed;
		uint8_t displayColorGreen;
		uint8_t displayColorBlue;
	}
	ChannelConfig;


	typedef enum {
		eVelocityUnitsMS = 0,
		eVelocityUnitsKMH = 1
	}
	VelocityUnits;

	typedef struct ReceiverSettings
	{
	QList<RegisterSettings> registersFPGA;
	QList<RegisterSettings> registersADC;
	QList<RegisterSettings> registersGPIO;
	
	// Algorithms index start at 1. Algorithm 0 (GLOBAL_PARAMETERS_INDEX) is global parameters.
	QList<AlgorithmParameters> parametersAlgos[ALGO_QTY+1];

	// Channel configuration
	QList<ChannelConfig> channelsConfig;

	// Receiver
	QString sReceiverType;
	uint8_t receiverChannelMask;		// Indicates which channels are processed by unit
	uint8_t receiverFrameRate;			// Frame rate, in hertz

	// Receiver communications port config
	QString sCommPort;       // Default is "COM16"
	long serialPortRate;		// In bpschannelMask
	QString sCANBitRate;		// "S8" for 1Mbps.  Specific to the CAN driver used.
	uint16_t yearOffset;		   // All sensor Dates are offset from 1900
	uint16_t monthOffset;		// All sensor months start at 0.  Posix starts aty 1.
	
	
	// Messages enabled
	bool msgEnableObstacle;
	bool msgEnableDistance_1_4;
	bool msgEnableDistance_5_8;
	bool msgEnableIntensity_1_4;
	bool msgEnableIntensity_5_8;

	// Calibration parameters
	float sensorX;
	float sensorY;
	float sensorZ;

	float sensorPitch; // In degrees
	float sensorRoll;  // In degrees
	float sensorYaw;   // In degrees

	float displayedRangeMin;
	float displayedRangeMax;
	float rangeOffset;
	}
	ReceiverSettings;


class AWLSettings
{
public:
public:
	static AWLSettings *InitSettings();
	static AWLSettings *GetGlobalSettings();

	// Constructor
	AWLSettings();
	bool ReadSettings();

	/** \brief Return the index of the FPGA RegisterSettings for the object that
	           has the address specified.
    * \param[in] inReceiver the receiver for which we want the register read
	* \param[in] inAddress the register address
	* \return "index" of the found object in the list (this is NOT the sIndex field). -1 if no registers match that address.

      */
	int FindRegisterFPGAByAddress(ReceiverID receiverID, uint16_t inAddress);

	/** \brief Return the index of the FPGA RegisterSettings for the object that
	           has the address specified.
    * \param[in] inAddress the register address
	* \return "index" of the found object in the list (this is NOT the sIndex field). -1 if no registers match that address.

      */
	int FindRegisterADCByAddress(ReceiverID receiverID, uint16_t inAddress);

	/** \brief Return the index of the FPGA RegisterSettings for the object that
	           has the address specified.
    * \param[in] inAddress the register address
	* \return "index" of the found object in the list (this is NOT the sIndex field). -1 if no registers match that address.

      */
	int FindRegisterGPIOByAddress(ReceiverID receiverID, uint16_t inAddress);

	/** \brief Return the index of the FPGA RegisterSettings for the object that
	           has the address specified.
    * \param[in] inAddress the register address
	* \return "index" of the found object in the list (this is NOT the sIndex field). -1 if no registers match that address.

      */
	int FindAlgoParamByAddress(QList<AlgorithmParameters>&paramList, uint16_t inAddress);

public:
	// Default register configurations

	QList<RegisterSettings> defaultRegistersFPGA;
	QList<RegisterSettings> defaultRegistersADC;
	QList<RegisterSettings> defaultRegistersGPIO;

	// Algorithms index start at 1. Algorithm 0 (GLOBAL_PARAMETERS_INDEX) is global parameters.
	QList<AlgorithmParameters> defaultParametersAlgos[ALGO_QTY+1];


	// Receiver configuration
	QList<ReceiverSettings> receiverSettings;

	QString sAlgoNames[ALGO_QTY+1];
	// Default displayedAlgo
	int defaultAlgo;

	// Layout
	bool bDisplay3DWindow;
	bool bDisplay2DWindow;
	bool bDisplayTableViewWindow;
	bool bDisplayScopeWindow;
	bool bDisplayCameraWindow;

	bool bDisplayScopeDistance;
	bool bDisplayScopeVelocity;

	VelocityUnits velocityUnits; 

	QString sLogoFileName;
	QString sIconFileName;

	// Demo mode
	bool bEnableDemo;
	int  demoInjectType;

	// Calibration parameters
	float distanceScale;
	float targetHintDistance;
	float targetHintAngle;

	
	// Table view options
	int displayedDetectionsPerChannelInTableView;

	// 2D display options
	float carWidth;
	float carLength;
	float carHeight;

	float laneWidth;

	float shortRangeDistance;
	float shortRangeDistanceStartLimited;
	float shortRangeAngle;
	float shortRangeAngleStartLimited;

	float longRangeDistance;
	float longRangeDistanceStartLimited;
	float longRangeAngle;
	float longRangeAngleStartLimited;

	bool showPalette;
	int mergeDisplayMode;
	int measureMode;
	int displayDistanceMode2D;
	float mergeAcceptanceX;
	float mergeAcceptanceY;
	int colorCode2D;
	float maxVelocity2D;
	float zeroVelocity;


	// 3D display options
	int decimation;
	int pixelSize;
	int colorStyle;
	int cameraView;
	float viewerDepth; 
	float viewerHeight;
	float viewerMaxRange;



	// Scope
	int scopeTimerInterval;

	// Dynamic testing
	float threatLevelCriticalThreshold;
	float threatLevelWarnThreshold;
	float threatLevelLowThreshold;

	float brakingDeceleration;
	float travelSpeed;

	// Camera
	float cameraX;
	float cameraY; 
	float cameraZ; 
	float cameraPitch; 
	float cameraRoll; 
	float cameraYaw; 
	float cameraFovXDegrees;
	float cameraFovYDegrees;

	// Debug
	bool bWriteDebugFile;
	bool bWriteLogFile;


protected:
	QSettings settings;

	static AWLSettings *globalSettings;

};

} // namespace AWL          

#endif 
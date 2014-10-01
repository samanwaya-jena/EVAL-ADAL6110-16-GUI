#ifndef _AWLSettings__H
#define _AWLSettings__H

#include <string>
#include <boost/container/vector.hpp>
#include <stdint.h>

#define VelocityToKmH(velocity) (velocity * 3.6)

#define ALGO_QTY 4
#define GLOBAL_PARAMETERS_INDEX 0

namespace awl
{

	typedef int ReceiverID;

	typedef struct RegisterSetting 
	{
		std::string sIndex;
		uint16_t address;
		std::string sDescription;
		uint32_t value;
		int pendingUpdates;
	}
	RegisterSetting;
	
	typedef boost::container::vector<RegisterSetting> RegisterSet;


	typedef enum  {
		eAlgoParamInt = 0,
		eAlgoParamFloat = 1
	}
	AlgoParamType;

	typedef struct AlgorithmParameter 
	{
		std::string sIndex;
		uint16_t address;
		std::string sDescription;
		AlgoParamType paramType;
		uint32_t intValue;
		float floatValue;
		int pendingUpdates;
	}
	AlgorithmParameter;
	
	typedef boost::container::vector<AlgorithmParameter> AlgorithmParameterVector;

	typedef struct AlgorithmDescription
	{
		std::string sAlgoName;
		uint16_t	algoID;
		AlgorithmParameterVector parameters;
	}
	AlgorithmDescription;

	typedef struct AlgorithmSet
	{
		// Default displayedAlgo
		int defaultAlgo;
		boost::container::vector<AlgorithmDescription> algorithms;
	}
	AlgorithmSet;

	typedef struct ChannelConfig 
	{
		int channelIndex;
		float fovWidth;
		float fovHeight;
		float centerX;
		float centerY;
		float maxRange;
		uint8_t displayColorRed;
		uint8_t displayColorGreen;
		uint8_t displayColorBlue;
	}
	ChannelConfig;

	typedef boost::container::vector<ChannelConfig> ChannelConfigVector;


	typedef enum {
		eVelocityUnitsMS = 0,
		eVelocityUnitsKMH = 1
	}
	VelocityUnits;

	typedef struct ReceiverSettings
	{
	// Channel configuration
	ChannelConfigVector channelsConfig;

	// Receiver
	std::string sReceiverType;
	uint8_t receiverChannelMask;		// Indicates which channels are processed by unit
	uint8_t receiverFrameRate;			// Frame rate, in hertz

	// Receiver communications port config
	std::string sCommPort;       // Default is "COM16"
	
	// Messages enabled
	bool msgEnableObstacle;
	bool msgEnableDistance_1_4;
	bool msgEnableDistance_5_8;
	bool msgEnableIntensity_1_4;
	bool msgEnableIntensity_5_8;

	// Geometry parameters
	float sensorForward;
	float sensorLeft;
	float sensorUp;

	float sensorPitch; // In degrees
	float sensorRoll;  // In degrees
	float sensorYaw;   // In degrees

	float displayedRangeMin;
	float displayedRangeMax;
	float rangeOffset;
	}
	ReceiverSettings;

	typedef boost::container::vector<ReceiverSettings> ReceiverSettingsVector;

typedef struct CameraSettings
	{
	// Camera
	std::string sCameraName;
	bool cameraFlip;
	float cameraForward;
	float cameraLeft; 
	float cameraUp; 
	float cameraPitch; 
	float cameraRoll; 
	float cameraYaw; 
	float cameraFovWidthDegrees;
	float cameraFovHeightDegrees;
	}
	CameraSettings;

	typedef boost::container::vector<CameraSettings> CameraSettingsVector;

class AWLSettings
{
public:
	static AWLSettings *InitSettings();
	static AWLSettings *GetGlobalSettings();

	// Constructor
	AWLSettings();
	bool ReadSettings();


	/** \brief Stores the current receiver calibration settings
		* \return true if storage processe dwithout error. False in case of a storage error.
      */
	bool AWLSettings::StoreReceiverCalibration();

public:
	// Default register configurations

	RegisterSet defaultRegistersFPGA;
	RegisterSet defaultRegistersADC;
	RegisterSet defaultRegistersGPIO;

	// Algorithms index start at 1. Algorithm 0 (GLOBAL_PARAMETERS_INDEX) is global parameters.
	AlgorithmSet defaultParametersAlgos;

	// Receiver configuration
	ReceiverSettingsVector receiverSettings;

	// Camera configuration
	CameraSettingsVector cameraSettings;


	// Layout
	bool bDisplay3DWindow;
	bool bDisplay2DWindow;
	bool bDisplayTableViewWindow;
	bool bDisplayScopeWindow;
	bool bDisplayCameraWindow;

	bool bDisplayScopeDistance;
	bool bDisplayScopeVelocity;

	VelocityUnits velocityUnits; 

	std::string sLogoFileName;
	std::string sIconFileName;

	// Demo mode
	bool bEnableDemo;
	int  demoInjectType;

	// Calibration parameters
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

	// Video display options
	bool bDisplayVideoCrosshair;

	// Scope
	int scopeTimerInterval;

	// Dynamic testing
	float threatLevelCriticalThreshold;
	float threatLevelWarnThreshold;
	float threatLevelLowThreshold;

	float brakingDeceleration;
	float travelSpeed;

	// Debug
	bool bWriteDebugFile;
	bool bWriteLogFile;


protected:
	std::string sFileName;
	static AWLSettings *globalSettings;
};

} // namespace AWL          

#endif 
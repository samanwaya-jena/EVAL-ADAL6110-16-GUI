#ifndef _AWLSettings__H
#define _AWLSettings__H

#include <QtCore/qlist.h>
#include <QtCore/qstring.h>
#include <QtCore/qstringlist.h>
#include <QSettings>

#include <stdint.h>

namespace awl
{

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


#define ALGO_QTY 3
#define GLOBAL_PARAMETERS_INDEX 0

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
    * \param[in] inAddress the register address
	* \return "index" of the found object in the list (this is NOT the sIndex field). -1 if no registers match that address.

      */
	int FindRegisterFPGAByAddress(uint16_t inAddress);

	/** \brief Return the index of the FPGA RegisterSettings for the object that
	           has the address specified.
    * \param[in] inAddress the register address
	* \return "index" of the found object in the list (this is NOT the sIndex field). -1 if no registers match that address.

      */
	int FindRegisterADCByAddress(uint16_t inAddress);

	/** \brief Return the index of the FPGA RegisterSettings for the object that
	           has the address specified.
    * \param[in] inAddress the register address
	* \return "index" of the found object in the list (this is NOT the sIndex field). -1 if no registers match that address.

      */
	int FindRegisterGPIOByAddress(uint16_t inAddress);

	/** \brief Return the index of the FPGA RegisterSettings for the object that
	           has the address specified.
    * \param[in] inAddress the register address
	* \return "index" of the found object in the list (this is NOT the sIndex field). -1 if no registers match that address.

      */
	int FindAlgoParamByAddress(QList<AlgorithmParameters>&paramList, uint16_t inAddress);

public:
	// Registers

	QList<RegisterSettings> registersFPGA;
	QList<RegisterSettings> registersADC;
	QList<RegisterSettings> registersGPIO;

	// Algorithms index start at 1. Algorithm 0 (GLOBAL_PARAMETERS_INDEX) is global parameters.
	QList<AlgorithmParameters> parametersAlgos[ALGO_QTY+1];

	QString sAlgoNames[ALGO_QTY+1];


	// Default displayedAlgo
	int defaultAlgo;

	// Layout
	bool bDisplay3DWindow;
	bool bDisplay2DWindow;
	bool bDisplayScopeWindow;
	bool bDisplayCameraWindow;

	// Demo mode
	bool bEnableDemo;
	int  demoInjectType;

	// Calibration parameters
	float sensorHeight;
	float sensorDepth;
	float displayedRangeMin;
	float displayedRangeMax;
	float rangeOffset;

	// 3D display options
	int decimation;
	int pixelSize;
	int colorStyle;
	int cameraView;

	// 2D display options
	float shortRangeDistance;
	float shortRangeDistanceStartLimited;
	float shortRangeAngle;
	float shortRangeAngleStartLimited;

	float longRangeDistance;
	float longRangeDistanceStartLimited;
	float longRangeAngle;
	float longRangeAngleStartLimited;

	bool showPalette;
	int mergeDetectionMode;
	int mergeDisplayMode;
	int measureMode;
	float mergeAcceptance;

	// Receiver
	QString sReceiverType;

	// CAN Receiver config
	QString sCANCommPort;       // Default is "COM16"
	QString sCANBitRate;		// "S8" for 1Mbps.  Specific to the CAN driver used.
	long serialCANPortRate;		// In bps
	uint16_t yearOffsetCAN;		   // All CAN Dates are offset from 1900
	uint16_t monthOffsetCAN;		// All CAN months start at 0.  Posix starts aty 1.
	
	// BareMetal receiverConfig
	QString sBareMetalCommPort;       // Default is "COM16"
	long serialBareMetalPortRate;	 // In bps

	// Scope
	int scopeTimerInterval;

	// Camera
	float cameraFovXDegrees;
	float cameraFovYDegrees;


protected:
	QSettings settings;

	static AWLSettings *globalSettings;

};

} // namespace AWL          

#endif 
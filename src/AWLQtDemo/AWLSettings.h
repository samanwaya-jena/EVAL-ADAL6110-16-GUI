#ifndef _AWLSettings__H
#define _AWLSettings__H

#include <QtCore/qlist.h>
#include <QtCore/qstring.h>
#include <QtCore/qstringlist.h>
#include <QSettings>

#include <stdint.h>

namespace awl
{
class AWLSettings
{
public:
	typedef struct RegisterSettings 
	{
		QString sIndex;
		uint16_t address;
		QString sDescription;
		uint32_t value;
	}
	RegisterSettings;


public:
	static AWLSettings *InitSettings();
	static AWLSettings *GetGlobalSettings();

	// Constructor
	AWLSettings();
	bool ReadSettings();

public:
	// Registers

	QList<RegisterSettings> registersFPGA;
	QList<RegisterSettings> registersADC;

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

	// CAN
	QString sCANCommPort;       // Default is "COM16"
	QString sCANBitRate;		// "S8" for 1Mbps.  Specific to the CAN driver used.
	long serialCANPortRate;		// In bps
	uint16_t yearOffsetCAN;		   // All CAN Dates are offset from 1900
	uint16_t monthOffsetCAN;		// All CAN months start at 0.  Posix starts aty 1.


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
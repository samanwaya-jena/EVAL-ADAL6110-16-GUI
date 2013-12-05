#include "AWLSettings.h"

using namespace awl;


AWLSettings *AWLSettings::globalSettings=NULL;


AWLSettings::AWLSettings():
settings("AWLQTDemo.ini", QSettings::IniFormat),
registersFPGA(),
registersADC(),

sensorHeight(0.0),
sensorDepth(0.0),
displayedRangeMin(0.0),
displayedRangeMax(0.0),
rangeOffset(0.0),
targetHintDistance(0.0),
targetHintAngle(0.0),
decimation(3),
pixelSize(1),
colorStyle(0),
cameraView(3),

sCANCommPort("COM16"),
sCANBitRate("S8"),
msgEnableObstacle(false),
msgEnableDistance_1_4(true),
msgEnableDistance_5_8(true),
msgEnableIntensity_1_4(true),
msgEnableIntensity_5_8(true)

{
	cameraView = 3;
}

AWLSettings * AWLSettings::InitSettings()
{
	globalSettings = new AWLSettings();
	return(globalSettings);
}

AWLSettings *AWLSettings::GetGlobalSettings()
{
	return(globalSettings);
}



bool AWLSettings::ReadSettings()
{
	settings.beginGroup("registersAWL");
	int size = settings.beginReadArray("register");
	for (int i = 0; i < size; ++i) 
	{
		settings.setArrayIndex(i);
		RegisterSettings registerFPGA;
		registerFPGA.sIndex = settings.value("index").toString();
		registerFPGA.address = settings.value("address").toUInt();
		registerFPGA.sDescription = settings.value("description").toString();
		registerFPGA.value = 0L;
		registerFPGA.pendingUpdates = 0;
		
		registersFPGA.append(registerFPGA);
	}

	settings.endArray();
	settings.endGroup();

	settings.beginGroup("registersADC");
	size = settings.beginReadArray("register");
	for (int i = 0; i < size; ++i) 
	{
		settings.setArrayIndex(i);
		RegisterSettings registerADC;
		registerADC.sIndex = settings.value("index").toString();
		registerADC.address = settings.value("address").toUInt();
		registerADC.sDescription = settings.value("description").toString();
		registerADC.value = 0L;
		registerADC.pendingUpdates = 0;
		registersADC.append(registerADC);

	}
	settings.endArray();
	settings.endGroup();

	settings.beginGroup("GPIOs");
	size = settings.beginReadArray("register");
	for (int i = 0; i < size; ++i) 
	{
		settings.setArrayIndex(i);
		RegisterSettings registerGPIO;
		registerGPIO.sIndex = settings.value("index").toString();
		registerGPIO.address = settings.value("address").toUInt();
		registerGPIO.sDescription = settings.value("description").toString();
		registerGPIO.value = 0L;
		registerGPIO.pendingUpdates = 0;
		registersGPIO.append(registerGPIO);

	}
	settings.endArray();
	settings.endGroup();

	// Debug and log file control
	settings.beginGroup("debug");
	bWriteDebugFile = settings.value("enableDebugFile").toBool();
	bWriteLogFile = settings.value("enableLogFile").toBool();
	settings.endGroup();

	// Default algo
	settings.beginGroup("algos");
	defaultAlgo = settings.value("defaultAlgo").toInt();
	settings.endGroup();

	// Load all algorithm parameters for all algorithms and for global parameters
	for (int algoIndex = 0; algoIndex <= ALGO_QTY; algoIndex++)
	{
		QString sAlgoGroupName;
		sAlgoGroupName.sprintf("Algo%02d", algoIndex);

		settings.beginGroup(sAlgoGroupName);

		sAlgoNames[algoIndex] = settings.value("algoName").toString();

		size = settings.beginReadArray("parameter");
		for (int i = 0; i < size; ++i) 
		{
			settings.setArrayIndex(i);
			AlgorithmParameters parameters;
			parameters.sIndex = settings.value("index").toString();
			parameters.address = settings.value("address").toUInt();
			parameters.sDescription = settings.value("description").toString();
			QString sType = settings.value("type").toString();
			if (!sType.compare("int")) 
			{
				parameters.paramType = eAlgoParamInt;
				parameters.intValue = settings.value("default").toInt();
				parameters.floatValue = 0.0;
			}
			else if (!sType.compare("float")) 
			{
				parameters.paramType = eAlgoParamFloat;
				parameters.floatValue = 0;
				parameters.floatValue = settings.value("default").toFloat();
			}

			parameters.pendingUpdates = 0;
			parametersAlgos[algoIndex].append(parameters);

		}

		settings.endArray();
		settings.endGroup();
	}

	// Other settings

	settings.beginGroup("demoMode");
	bEnableDemo = settings.value("enableDemo").toBool();
	this->demoInjectType = settings.value("injectType").toInt();
	settings.endGroup();

	settings.beginGroup("layout");
	bDisplay3DWindow = settings.value("display3DWindow").toBool();
	bDisplay2DWindow = settings.value("display2DWindow").toBool();
	bDisplayScopeWindow = settings.value("displayScopeWindow").toBool();
	bDisplayCameraWindow = settings.value("displayCameraWindow").toBool();

	bDisplayScopeDistance = settings.value("displayScopeDistance").toBool();
	bDisplayScopeVelocity = settings.value("displayScopeVelocity").toBool();

	velocityUnits = (VelocityUnits) settings.value("velocityUnits").toInt();
	settings.endGroup();

	settings.beginGroup("calibration");
	sensorHeight = settings.value("sensorHeight").toFloat();
	sensorDepth = settings.value("sensorDepth").toFloat();
	displayedRangeMin = settings.value("displayedRangeMin").toFloat();
	displayedRangeMax = settings.value("displayedRangeMax").toFloat();
	rangeOffset = settings.value("rangeOffset").toFloat();
	distanceScale = settings.value("distanceScale").toFloat();
	targetHintDistance = settings.value("targetHintDistance").toFloat();
	targetHintAngle = settings.value("targetHintAngle").toFloat();
	settings.endGroup();

	settings.beginGroup("display3D");
	decimation = settings.value("decimation").toInt();
	pixelSize = settings.value("pixelSize").toInt();
	colorStyle =settings.value("colorStyle").toInt();
	cameraView = settings.value("cameraView").toInt();
	settings.endGroup();

	settings.beginGroup("display2D");
	shortRangeDistance = settings.value("shortRangeDistance").toFloat();
	shortRangeDistanceStartLimited = settings.value("shortRangeDistanceStartLimited").toFloat();
	shortRangeAngle = settings.value("shortRangeAngle").toFloat();
	shortRangeAngleStartLimited = settings.value("shortRangeAngleStartLimited").toFloat();

	longRangeDistance = settings.value("longRangeDistance").toFloat();
	longRangeDistanceStartLimited = settings.value("longRangeDistanceStartLimited").toFloat();
	longRangeAngle = settings.value("longRangeAngle").toFloat();
	longRangeAngleStartLimited = settings.value("longRangeAngleStartLimited").toFloat();

	showPalette = settings.value("showPalette").toInt();
	mergeDisplayMode = settings.value("mergeDisplayMode").toInt();
	measureMode = settings.value("measureMode").toInt();
	mergeAcceptance = settings.value("mergeAcceptance").toFloat();
	colorCode2D = settings.value("colorCode").toInt();
	maxVelocity2D = settings.value("maxVelocity").toFloat();
	zeroVelocity = settings.value("zeroVelocity").toFloat();
	settings.endGroup();

	
	settings.beginGroup("receiver");
	sReceiverType = settings.value("receiverType").toString();
	receiverChannelMask = settings.value("channelMask").toUInt();
	receiverFrameRate = settings.value("frameRate").toUInt();

	msgEnableObstacle = settings.value("msgEnableObstacle").toBool();
	msgEnableDistance_1_4 = settings.value("msgEnableDistance_1_4").toBool();
	msgEnableDistance_5_8 = settings.value("msgEnableDistance_5_8").toBool();
	msgEnableIntensity_1_4 = settings.value("msgEnableIntensity_1_4").toBool();
	msgEnableIntensity_5_8 = settings.value("msgEnableIntensity_5_8").toBool();


	settings.endGroup();

	settings.beginGroup("bareMetalComm");
	sBareMetalCommPort = settings.value("commPort").toString();
	serialBareMetalPortRate = settings.value("serialPortRate").toLongLong();
	settings.endGroup();

	settings.beginGroup("CAN");
	sCANCommPort = settings.value("commPort").toString();
	sCANBitRate = settings.value("bitRate").toString();
	serialCANPortRate = settings.value("serialPortRate").toLongLong();
	yearOffsetCAN = settings.value("yearOffset").toUInt();
	monthOffsetCAN = settings.value("monthOffset").toUInt();
	settings.endGroup();

	settings.beginGroup("scope");
	scopeTimerInterval = settings.value("timerInterval").toInt();
	settings.endGroup();

	settings.beginGroup("dynamicTesting");
	threatLevelCriticalThreshold = settings.value("threatLevelCriticalThreshold").toFloat();
	threatLevelWarnThreshold = settings.value("threatLevelWarnThreshold").toFloat();
	threatLevelLowThreshold = settings.value("threatLevelLowThreshold").toFloat();

	brakingDeceleration = settings.value("brakingDeceleration").toFloat();
	travelSpeed = settings.value("travelSpeed").toFloat();
	settings.endGroup();
	
	settings.beginGroup("camera");
	cameraFovXDegrees = settings.value("cameraFovX").toFloat();
	cameraFovYDegrees = settings.value("cameraFovY").toFloat();
	settings.endGroup();

	return(true);
}


int AWLSettings::FindRegisterFPGAByAddress(uint16_t inAddress)

{
	for (int i = 0; i < registersFPGA.count(); i++) 
	{
		if (registersFPGA[i].address == inAddress)
		{
			return(i);
		}
	}

	return(-1);
}

int AWLSettings::FindRegisterADCByAddress(uint16_t inAddress)

{
	for (int i = 0; i < registersADC.count(); i++) 
	{
		if (registersADC[i].address == inAddress)
		{
			return(i);
		}
	}

	return(-1);
}

int AWLSettings::FindRegisterGPIOByAddress(uint16_t inAddress)

{
	for (int i = 0; i < registersGPIO.count(); i++) 
	{
		if (registersGPIO[i].address == inAddress)
		{
			return(i);
		}
	}

	return(-1);
}

int AWLSettings::FindAlgoParamByAddress(QList<AlgorithmParameters>&paramList, uint16_t inAddress)
{
	for (int i = 0; i < paramList.count(); i++) 
	{
		if (paramList[i].address == inAddress)
		{
			return(i);
		}
	}

	return(-1);
}


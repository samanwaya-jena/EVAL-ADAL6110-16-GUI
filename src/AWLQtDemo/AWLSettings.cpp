#include "AWLSettings.h"

using namespace awl;


AWLSettings *AWLSettings::globalSettings=NULL;


AWLSettings::AWLSettings():
settings("AWLQTDemo.ini", QSettings::IniFormat),
targetHintDistance(0.0),
targetHintAngle(0.0),
decimation(3),
pixelSize(1),
colorStyle(0),
cameraView(3),
sLogoFileName(""),
sIconFileName("")

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
		
		defaultRegistersFPGA.append(registerFPGA);
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
		defaultRegistersADC.append(registerADC);

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
		defaultRegistersGPIO.append(registerGPIO);

	}
	settings.endArray();
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
			defaultParametersAlgos[algoIndex].append(parameters);

		}

		settings.endArray();
		settings.endGroup();
	}


	// Load the receiver configuration
	settings.beginGroup("receivers");
	size = settings.beginReadArray("receiver");
	for (int receiverID = 0; receiverID < size; receiverID++) 
	{
		settings.setArrayIndex(receiverID);
		ReceiverSettings receiver;

		// Type
		receiver.sReceiverType = settings.value("receiverType").toString();
		receiver.receiverChannelMask = settings.value("channelMask").toUInt();
		receiver.receiverFrameRate = settings.value("frameRate").toUInt();

		// Communication parameters
		receiver.sCommPort = settings.value("commPort").toString();
		receiver.serialPortRate = settings.value("serialPortRate").toLongLong();
		receiver.sCANBitRate = settings.value("bitRate").toString();
		receiver.yearOffset = settings.value("yearOffset").toUInt();
		receiver.monthOffset = settings.value("monthOffset").toUInt();

		// Messages enabled
		receiver.msgEnableObstacle = settings.value("msgEnableObstacle").toBool();
		receiver.msgEnableDistance_1_4 = settings.value("msgEnableDistance_1_4").toBool();
		receiver.msgEnableDistance_5_8 = settings.value("msgEnableDistance_5_8").toBool();
		receiver.msgEnableIntensity_1_4 = settings.value("msgEnableIntensity_1_4").toBool();
		receiver.msgEnableIntensity_5_8 = settings.value("msgEnableIntensity_5_8").toBool();

		// Calibration
		receiver.sensorX = settings.value("sensorX").toFloat();
		receiver.sensorY = settings.value("sensorY").toFloat();
		receiver.sensorZ = settings.value("sensorZ").toFloat();
		receiver.sensorPitch = settings.value("sensorPitch").toFloat();
		receiver.sensorRoll = settings.value("sensorRoll").toFloat();
		receiver.sensorYaw = settings.value("sensorYaw").toFloat();
		receiver.displayedRangeMin = settings.value("displayedRangeMin").toFloat();
		receiver.displayedRangeMax = settings.value("displayedRangeMax").toFloat();
		receiver.rangeOffset = settings.value("rangeOffset").toFloat();

		// Copy default register, adc, GPIO and algo settings
		receiver.registersFPGA = defaultRegistersFPGA;
		receiver.registersADC = defaultRegistersADC;
		receiver.registersGPIO = defaultRegistersGPIO;
		for (int algo = 0; algo <= ALGO_QTY; algo++) 
		{
			receiver.parametersAlgos[algo] = defaultParametersAlgos[algo];
		}

		// Store
		receiverSettings.append(receiver);
		ReceiverSettings *receiverPtr = &(receiverSettings[receiverID]);
		receiverPtr->channelsConfig.clear();
	}

	settings.endArray();
	settings.endGroup();

	// Read receiver channels for each receiver
	for (int receiverID = 0; receiverID < receiverSettings.count(); receiverID++)
	{
		QString sChannelGroupName;
		sChannelGroupName.sprintf("ReceiverChannels%02d", receiverID+1);

		settings.beginGroup(sChannelGroupName);
		size = settings.beginReadArray("channelConfig");
		for (int channelID = 0; channelID < size; channelID++) 
		{
			settings.setArrayIndex(channelID);
			ChannelConfig channelConfig;
			channelConfig.channelIndex = settings.value("index").toInt();
			channelConfig.fovX = settings.value("fovX").toFloat();
			channelConfig.fovY = settings.value("fovY").toFloat();
			channelConfig.centerX = settings.value("centerX").toFloat();
			channelConfig.centerY = settings.value("centerY").toFloat();
			channelConfig.maxRange = settings.value("maxRange").toFloat();
			channelConfig.sMaskName = settings.value("maskName").toString();
			channelConfig.sFrameName = settings.value("frameName").toString();
			channelConfig.displayColorRed = (uint8_t) settings.value("displayColorRed").toUInt();
			channelConfig.displayColorGreen = (uint8_t) settings.value("displayColorGreen").toUInt();
			channelConfig.displayColorBlue = (uint8_t) settings.value("displayColorBlue").toUInt();
		
			ReceiverSettings settingsCopy = receiverSettings[receiverID];
			receiverSettings[receiverID].channelsConfig.append(channelConfig);
		}
		settings.endArray();
		settings.endGroup();
	}


	// Debug and log file control
	settings.beginGroup("debug");
	bWriteDebugFile = settings.value("enableDebugFile").toBool();
	bWriteLogFile = settings.value("enableLogFile").toBool();
	settings.endGroup();

	// Default algo
	settings.beginGroup("algos");
	defaultAlgo = settings.value("defaultAlgo").toInt();
	settings.endGroup();



	// Other settings

	settings.beginGroup("demoMode");
	bEnableDemo = settings.value("enableDemo").toBool();
	this->demoInjectType = settings.value("injectType").toInt();
	settings.endGroup();

	settings.beginGroup("layout");
	bDisplay3DWindow = settings.value("display3DWindow").toBool();
	bDisplay2DWindow = settings.value("display2DWindow").toBool();
	bDisplayTableViewWindow = settings.value("displayTableViewWindow").toBool();
	bDisplayScopeWindow = settings.value("displayScopeWindow").toBool();
	bDisplayCameraWindow = settings.value("displayCameraWindow").toBool();

	velocityUnits = (VelocityUnits) settings.value("velocityUnits").toInt();

	sLogoFileName = settings.value("logoFileName").toString();
	sIconFileName = settings.value("iconFileName").toString();
	settings.endGroup();

	settings.beginGroup("calibration");
	distanceScale = settings.value("distanceScale").toFloat();
	targetHintDistance = settings.value("targetHintDistance").toFloat();
	targetHintAngle = settings.value("targetHintAngle").toFloat();
	settings.endGroup();

	settings.beginGroup("display3D");
	decimation = settings.value("decimation").toInt();
	pixelSize = settings.value("pixelSize").toInt();
	colorStyle =settings.value("colorStyle").toInt();
	cameraView = settings.value("cameraView").toInt();
	viewerDepth =  settings.value("viewerDepth").toFloat();
	viewerHeight =  settings.value("viewerHeight").toFloat();
	viewerMaxRange =  settings.value("viewerMaxRange").toFloat();
	settings.endGroup();

	settings.beginGroup("displayTableView");
	displayedDetectionsPerChannelInTableView = settings.value("displayedDetectionsPerChannelInTableView").toInt();
	settings.endGroup();

	settings.beginGroup("display2D");
	carWidth = settings.value("carWidth").toFloat();
	carLength = settings.value("carLength").toFloat();
	carHeight = settings.value("carHeight").toFloat();
	laneWidth = settings.value("LaneWidth").toFloat();
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
	mergeAcceptanceX = settings.value("mergeAcceptanceX").toFloat();
	mergeAcceptanceY = settings.value("mergeAcceptanceY").toFloat();
	colorCode2D = settings.value("colorCode").toInt();
	maxVelocity2D = settings.value("maxVelocity").toFloat();
	zeroVelocity = settings.value("zeroVelocity").toFloat();
	displayDistanceMode2D = settings.value("displayDistances").toInt();
	settings.endGroup();

	
	settings.beginGroup("scope");
	scopeTimerInterval = settings.value("timerInterval").toInt();
	bDisplayScopeDistance = settings.value("displayScopeDistance").toBool();
	bDisplayScopeVelocity = settings.value("displayScopeVelocity").toBool();
	settings.endGroup();

	settings.beginGroup("dynamicTesting");
	threatLevelCriticalThreshold = settings.value("threatLevelCriticalThreshold").toFloat();
	threatLevelWarnThreshold = settings.value("threatLevelWarnThreshold").toFloat();
	threatLevelLowThreshold = settings.value("threatLevelLowThreshold").toFloat();

	brakingDeceleration = settings.value("brakingDeceleration").toFloat();
	travelSpeed = settings.value("travelSpeed").toFloat();
	settings.endGroup();
	
	settings.beginGroup("camera");
	cameraX = settings.value("cameraX").toFloat();
	cameraY = settings.value("cameraY").toFloat(); 
	cameraZ = settings.value("cameraZ").toFloat(); 
	cameraPitch = settings.value("cameraPitch").toFloat(); 
	cameraRoll = settings.value("cameraRoll").toFloat(); 
	cameraYaw = settings.value("cameraYaw").toFloat(); 
	cameraFovXDegrees = settings.value("cameraFovX").toFloat();
	cameraFovYDegrees = settings.value("cameraFovY").toFloat();
	settings.endGroup();

	return(true);
}


int AWLSettings::FindRegisterFPGAByAddress(ReceiverID receiverID, uint16_t inAddress)

{
	if (receiverID >= receiverSettings.count()) return(-1);

	const QList<RegisterSettings> *registersFPGA  = &(receiverSettings.at(receiverID).registersFPGA);
	for (int i = 0; i < registersFPGA->count(); i++) 
	{
		if (registersFPGA->at(i).address == inAddress)
		{
			return(i);
		}
	}

	return(-1);
}

int AWLSettings::FindRegisterADCByAddress(ReceiverID receiverID, uint16_t inAddress)

{
	if (receiverID >= receiverSettings.count()) return(-1);

	const QList<RegisterSettings> *registersADC  = &(receiverSettings.at(receiverID).registersADC);
	for (int i = 0; i < registersADC->count(); i++) 
	{
		if (registersADC->at(i).address == inAddress)
		{
			return(i);
		}
	}

	return(-1);
}

int AWLSettings::FindRegisterGPIOByAddress(ReceiverID receiverID, uint16_t inAddress)

{
	if (receiverID >= receiverSettings.count()) return(-1);

	const QList<RegisterSettings> *registersGPIO  = &(receiverSettings.at(receiverID).registersGPIO);
	for (int i = 0; i < registersGPIO->count(); i++) 
	{
		if (registersGPIO->at(i).address == inAddress)
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


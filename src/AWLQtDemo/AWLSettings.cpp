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

decimation(3),
pixelSize(1),
colorStyle(0),
cameraView(3),

sCANCommPort("COM16"),
sCANBitRate("S8")

{
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
		registersADC.append(registerADC);

	}
	settings.endArray();
	settings.endGroup();

	settings.beginGroup("layout");
	bDisplay3DWindow = settings.value("display3DWindow").toBool();
	bDisplay2DWindow = settings.value("display2DWindow").toBool();
	bDisplayScopeWindow = settings.value("displayScopeWindow").toBool();
	bDisplayCameraWindow = settings.value("displayCameraWindow").toBool();
	settings.endGroup();

	settings.beginGroup("demoMode");
	bEnableDemo = settings.value("enableDemo").toBool();
	demoInjectType = settings.value("injectType").toInt();
	settings.endGroup();

	settings.beginGroup("calibration");
	sensorHeight = settings.value("sensorHeight").toFloat();
	sensorDepth = settings.value("sensorDepth").toFloat();
	displayedRangeMin = settings.value("displayedRangeMin").toFloat();
	displayedRangeMax = settings.value("displayedRangeMax").toFloat();
	rangeOffset = settings.value("rangeOffset").toFloat();
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

	settings.beginGroup("camera");
	cameraFovXDegrees = settings.value("cameraFovX").toFloat();
	cameraFovYDegrees = settings.value("cameraFovY").toFloat();
	settings.endGroup();


	return(true);
}

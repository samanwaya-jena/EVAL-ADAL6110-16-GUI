#include "AWLSettings.h"

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>


using namespace awl;
using namespace std;
const std::string sDefaultSettingsFileName("AWLDemoSettings.xml");


void GetPosition(boost::property_tree::ptree &node, float &forward, float &left, float&up);

void GetOrientation(boost::property_tree::ptree &node, float &pitch, float &yaw, float&roll);
void Get2DPoint(boost::property_tree::ptree &node, float &x, float &y);

void GetGeometry(boost::property_tree::ptree &geometryNode, float &forward, float &left, float &up, float &pitch, float &yaw, float &roll);
void GetColor(boost::property_tree::ptree &colorNodeNode, uint8_t &red, uint8_t &green, uint8_t &blue);

AWLSettings *AWLSettings::globalSettings=NULL;

AWLSettings::AWLSettings():
sFileName(sDefaultSettingsFileName),
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
	// Create an empty property tree object
    using boost::property_tree::ptree;
    ptree propTree;

	// Load the XML file into the property tree. If reading fails
    // (cannot open file, parse error), an exception is thrown.
    read_xml(sFileName, propTree);

	// Read all FPGA Registers default descriptions
	BOOST_FOREACH(ptree::value_type &registersFPGANode, propTree.get_child("config.registersFPGA"))
	{
		if( registersFPGANode.first == "register" ) {
			boost::property_tree::ptree &registerNode = registersFPGANode.second;

            RegisterSetting registerFPGA;
            registerFPGA.sIndex = registerNode.get<std::string>("index");
            registerFPGA.address = registerNode.get<uint16_t>("address");
		    registerFPGA.sDescription = registerNode.get<std::string>("description");
			registerFPGA.value = 0L;
			registerFPGA.pendingUpdates = 0;

			defaultRegistersFPGA.push_back(registerFPGA);
        }
    }
 
 
	// Read all ADC Registers default descriptions
	BOOST_FOREACH(ptree::value_type &registersADCNode, propTree.get_child("config.registersADC"))
	{
		if( registersADCNode.first == "register" ) 
		{
			boost::property_tree::ptree &registerNode = registersADCNode.second;

            RegisterSetting registerADC;
            registerADC.sIndex = registerNode.get<std::string>("index");
            registerADC.address  = registerNode.get<uint16_t>("address");
		    registerADC.sDescription = registerNode.get<std::string>("description");
			registerADC.value = 0L;
			registerADC.pendingUpdates = 0;

			defaultRegistersADC.push_back(registerADC);
        }
    }
 
 	BOOST_FOREACH(ptree::value_type &registersGPIONode, propTree.get_child("config.GPIOs"))
	{
		if( registersGPIONode.first == "register" ) 
		{
			boost::property_tree::ptree &gpioNode = registersGPIONode.second;

            RegisterSetting registerGPIO;
            registerGPIO.sIndex = gpioNode.get<std::string>("index");
            registerGPIO.address  = gpioNode.get<uint16_t>("address");
		    registerGPIO.sDescription = gpioNode.get<std::string>("description");
			registerGPIO.value = 0L;
			registerGPIO.pendingUpdates = 0;

			defaultRegistersGPIO.push_back(registerGPIO);
        }
    }

	// Load all algorithm parameters for all algorithms and for global parameters
	defaultParametersAlgos.defaultAlgo = propTree.get<uint16_t>("config.algos.defaultAlgo");
	BOOST_FOREACH(ptree::value_type &algosNode, propTree.get_child("config.algos"))
	{
		if (algosNode.first == "algo")
		{
			boost::property_tree::ptree &algoNode = algosNode.second;
			AlgorithmDescription algoDescription;	
			algoDescription.algoID = algoNode.get<uint16_t>("algoID");
			algoDescription.sAlgoName = algoNode.get<std::string>("algoName");

			// All channel info for the receiver
			BOOST_FOREACH(ptree::value_type &parametersNode, algoNode/*.get_child("parameter")*/)
			{
#if 1
				if( parametersNode.first == "parameter" ) 
				{
					boost::property_tree::ptree &parameterNode = parametersNode.second;
					AlgorithmParameter parameter;
					parameter.sIndex = parameterNode.get<std::string>("index");
					parameter.address = parameterNode.get<uint16_t>("address");
					parameter.sDescription = parameterNode.get<std::string>("description");
					std::string sType = parameterNode.get<std::string>("type");
					if (!sType.compare("int")) 
					{
						parameter.paramType = eAlgoParamInt;
						parameter.intValue = parameterNode.get<uint32_t>("default");
						parameter.floatValue = 0.0;
					}
					else if (!sType.compare("float")) 
					{
						parameter.paramType = eAlgoParamFloat;
						parameter.intValue = 0;
						parameter.floatValue = parameterNode.get<float>("default");
					}

					parameter.pendingUpdates = 0;
					algoDescription.parameters.push_back(parameter);
				} // if (parametersNode.first)
#else
					boost::property_tree::ptree &parameterNode = parametersNode.second;
					AlgorithmParameter parameter;
					parameter.sIndex = parameterNode.get<std::string>("index");
					parameter.sIndex = parameterNode.get<uint16_t>("address");
					parameter.sDescription = parameterNode.get<std::string>("description");
					std::string sType = parameterNode.get<std::string>("type");
					if (!sType.compare("int")) 
					{
						parameter.paramType = eAlgoParamInt;
						parameter.intValue = parameterNode.get<uint32_t>("default");
						parameter.floatValue = 0.0;
					}
					else if (!sType.compare("float")) 
					{
						parameter.paramType = eAlgoParamFloat;
						parameter.intValue = 0;
						parameter.floatValue = parameterNode.get<float>("default");
					}

					parameter.pendingUpdates = 0;
					algoDescription.parameters.push_back(parameter);
#endif
			} // BOOST_FOREACH (parametersNode)

			defaultParametersAlgos.algorithms.push_back(algoDescription);
		} //		if (algoNode.first == "algo")
	} // BOOST_FOREACH(algosNode)


	// Receiver Configurations
	BOOST_FOREACH(ptree::value_type &receiversNode, propTree.get_child("config.receivers"))
	{
		if( receiversNode.first == "receiver" ) 
		{
			boost::property_tree::ptree &receiverNode = receiversNode.second;
			ReceiverSettings receiver;

			receiver.sReceiverType = receiverNode.get<std::string>("receiverType");
			receiver.receiverChannelMask = receiverNode.get<uint8_t>("channelMask");
			receiver.receiverFrameRate = receiverNode.get<uint8_t>("frameRate");

			// Communication parameters
			receiver.sCommPort =  receiverNode.get<std::string>("commPort");
			receiver.serialPortRate =  receiverNode.get<long>("serialPortRate");
			receiver.sCANBitRate = receiverNode.get<std::string>("bitRate");
			receiver.yearOffset = receiverNode.get<uint16_t>("yearOffset");
			receiver.monthOffset = receiverNode.get<uint16_t>("monthOffset");

			// Messages enabled
			receiver.msgEnableObstacle = receiverNode.get<bool>("msgEnableObstacle");
			receiver.msgEnableDistance_1_4 = receiverNode.get<bool>("msgEnableDistance_1_4");
			receiver.msgEnableDistance_5_8 = receiverNode.get<bool>("msgEnableDistance_5_8");
			receiver.msgEnableIntensity_1_4 = receiverNode.get<bool>("msgEnableIntensity_1_4");
			receiver.msgEnableIntensity_5_8 = receiverNode.get<bool>("msgEnableIntensity_5_8");

			// Geometry
			boost::property_tree::ptree &geometryNode = receiverNode.get_child("sensorGeometry");
			GetGeometry(geometryNode, 
				        receiver.sensorForward, receiver.sensorLeft, receiver.sensorUp,
						receiver.sensorPitch, receiver.sensorYaw, receiver.sensorRoll);

			// Display
			receiver.displayedRangeMin = receiverNode.get<float>("displayedRangeMin");
			receiver.displayedRangeMax = receiverNode.get<float>("displayedRangeMax");
			receiver.rangeOffset = receiverNode.get<float>("rangeOffset");

			// All channel info for the receiver
			BOOST_FOREACH(ptree::value_type &channelsNode, receiverNode)
			{
				if( channelsNode.first == "channel" ) 
				{
					boost::property_tree::ptree &channelNode = channelsNode.second;
					ChannelConfig channelConfig;

					channelConfig.channelIndex = channelNode.get<int>("index");
					Get2DPoint(channelNode.get_child("fov"), channelConfig.fovWidth, channelConfig.fovHeight);
					float roll;
					GetOrientation(channelNode.get_child("orientation"), channelConfig.centerY, channelConfig.centerX, roll);
					channelConfig.maxRange = channelNode.get<float>("maxRange");
					
					GetColor(channelNode.get_child("displayColor"), 
						     channelConfig.displayColorRed, channelConfig.displayColorGreen, channelConfig.displayColorBlue);

					receiver.channelsConfig.push_back(channelConfig);
				}// if( receiversNode.first == "channel"
			} // BOOST_FOREACH(ptree::value_type &channelsNode

			// Copy default register, adc, GPIO and algo settings into each receiver
			receiver.registersFPGA = defaultRegistersFPGA;
			receiver.registersADC = defaultRegistersADC;
			receiver.registersGPIO = defaultRegistersGPIO;
			receiver.parametersAlgos = defaultParametersAlgos;
			// Store
			receiverSettings.push_back(receiver);
		} //if ( receiversNode.first == "receiver" 
	} // BOOST_FOREACH(receiversNode



	// Debug and log file control
	bWriteDebugFile = propTree.get<bool>("config.debug.enableDebugFile");
	bWriteLogFile = propTree.get<bool>("config.debug.enableLogFile");

	// Other settings

	bEnableDemo = propTree.get<bool>("config.demoMode.enableDemo");
	demoInjectType = propTree.get<int>("config.demoMode.injectType");


	bDisplay3DWindow = propTree.get<bool>("config.layout.display3DWindow");
	bDisplay2DWindow = propTree.get<bool>("config.layout.display2DWindow");
	bDisplayTableViewWindow = propTree.get<bool>("config.layout.displayTableViewWindow");;
	bDisplayScopeWindow = propTree.get<bool>("config.layout.displayScopeWindow");
	bDisplayCameraWindow = propTree.get<bool>("config.layout.displayCameraWindow");

	velocityUnits = (VelocityUnits) propTree.get<int>("config.layout.velocityUnits");

	sLogoFileName = propTree.get<std::string>("config.layout.logoFileName");
	sIconFileName = propTree.get<std::string>("config.layout.iconFileName");

	distanceScale =  propTree.get<float>("config.calibration.distanceScale");
	targetHintDistance = propTree.get<float>("config.calibration.targetHintDistance");
	targetHintAngle = propTree.get<float>("config.calibration.targetHintAngle");

	decimation = propTree.get<int>("config.display3D.decimation");
	pixelSize = propTree.get<int>("config.display3D.pixelSize");
	colorStyle =propTree.get<int>("config.display3D.colorStyle");
	cameraView =propTree.get<int>("config.display3D.cameraView");
	viewerDepth =  propTree.get<float>("config.display3D.viewerDepth");
	viewerHeight = propTree.get<float>("config.display3D.viewerHeight");
	viewerMaxRange =  propTree.get<float>("config.display3D.viewerMaxRange");

	displayedDetectionsPerChannelInTableView = propTree.get<int>("config.displayTableView.displayedDetectionsPerChannelInTableView");

	carWidth = propTree.get<float>("config.display2D.carWidth");
	carLength = propTree.get<float>("config.display2D.carLength");
	carHeight = propTree.get<float>("config.display2D.carHeight");
	laneWidth = propTree.get<float>("config.display2D.laneWidth");
	shortRangeDistance = propTree.get<float>("config.display2D.shortRangeDistance");
	shortRangeDistanceStartLimited = propTree.get<float>("config.display2D.shortRangeDistanceStartLimited");
	shortRangeAngle = propTree.get<float>("config.display2D.shortRangeAngle");
	shortRangeAngleStartLimited = propTree.get<float>("config.display2D.shortRangeAngleStartLimited");

	longRangeDistance = propTree.get<float>("config.display2D.longRangeDistance");
	longRangeDistanceStartLimited = propTree.get<float>("config.display2D.longRangeDistanceStartLimited");
	longRangeAngle = propTree.get<float>("config.display2D.longRangeAngle");
	longRangeAngleStartLimited = propTree.get<float>("config.display2D.longRangeAngleStartLimited");

	showPalette = propTree.get<int>("config.display2D.showPalette");
	mergeDisplayMode = propTree.get<int>("config.display2D.mergeDisplayMode");
	measureMode = propTree.get<int>("config.display2D.measureMode");
	Get2DPoint(propTree.get_child("config.display2D.mergeAcceptance"), mergeAcceptanceX, mergeAcceptanceY);
	colorCode2D = propTree.get<int>("config.display2D.colorCode");
	maxVelocity2D = propTree.get<float>("config.display2D.maxVelocity");
	zeroVelocity = propTree.get<float>("config.display2D.zeroVelocity");
	displayDistanceMode2D = propTree.get<int>("config.display2D.displayDistances");

	scopeTimerInterval = propTree.get<int>("config.scope.timerInterval");
	bDisplayScopeDistance = propTree.get<bool>("config.scope.displayScopeDistance");
	bDisplayScopeVelocity = propTree.get<bool>("config.scope.displayScopeVelocity");


	threatLevelCriticalThreshold = propTree.get<float>("config.dynamicTesting.threatLevelCriticalThreshold");
	threatLevelWarnThreshold = propTree.get<float>("config.dynamicTesting.threatLevelWarnThreshold");
	threatLevelLowThreshold = propTree.get<float>("config.dynamicTesting.threatLevelLowThreshold");
	brakingDeceleration = propTree.get<float>("config.dynamicTesting.brakingDeceleration");
	travelSpeed = propTree.get<float>("config.dynamicTesting.travelSpeed");

	sCameraName = propTree.get<std::string>("config.camera.cameraName");
	GetGeometry(propTree.get_child("config.camera"),
				cameraForward, cameraLeft, cameraUp,
				cameraPitch, cameraYaw, cameraRoll);
	Get2DPoint(propTree.get_child("config.camera.fov"), cameraFovWidthDegrees, cameraFovHeightDegrees);

	return(true);
}


int AWLSettings::FindRegisterFPGAByAddress(ReceiverID receiverID, uint16_t inAddress)

{
	if (receiverID >= receiverSettings.size()) return(-1);

	const RegisterSet *registersFPGA  = &(receiverSettings.at(receiverID).registersFPGA);
	for (int i = 0; i < registersFPGA->size(); i++) 
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
	if (receiverID >= receiverSettings.size()) return(-1);

	const RegisterSet *registersADC  = &(receiverSettings.at(receiverID).registersADC);
	for (int i = 0; i < registersADC->size(); i++) 
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
	if (receiverID >= receiverSettings.size()) return(-1);

	const RegisterSet *registersGPIO  = &(receiverSettings.at(receiverID).registersGPIO);
	for (int i = 0; i < registersGPIO->size(); i++) 
	{
		if (registersGPIO->at(i).address == inAddress)
		{
			return(i);
		}
	}

	return(-1);
}

AlgorithmParameter * AWLSettings::FindAlgoParamByAddress(int receiverID, int algoID, uint16_t inAddress)
{

	for (int i = 0; i < receiverSettings[receiverID].parametersAlgos.algorithms[algoID].parameters.size(); i++) 	
	{
		if ( receiverSettings[receiverID].parametersAlgos.algorithms[algoID].parameters[i].address == inAddress)
		{
			return(&receiverSettings[receiverID].parametersAlgos.algorithms[algoID].parameters[i]);
		}
	}

	return(NULL);
}

void GetPosition(boost::property_tree::ptree &node, float &forward, float &left, float&up)
{
	forward = node.get<float>("forward");
	left = node.get<float>("left");
	up = node.get<float>("up");
}

void GetOrientation(boost::property_tree::ptree &node, float &pitch, float &yaw, float &roll)
{
	pitch = node.get<float>("pitch");
	yaw = node.get<float>("yaw");
	roll = node.get<float>("roll");
}

void Get2DPoint(boost::property_tree::ptree &node, float &x, float &y)
{
	x = node.get<float>("x");
	y = node.get<float>("y");
}

void GetGeometry(boost::property_tree::ptree &geometryNode, float &forward, float &left, float &up, float &pitch, float &yaw, float &roll)
{
	boost::property_tree::ptree &positionNode = geometryNode.get_child("position");
	boost::property_tree::ptree &orientationNode = geometryNode.get_child("orientation");
	GetPosition(positionNode, forward, left, up);
	GetOrientation(orientationNode, pitch, yaw, roll);
}

void GetColor(boost::property_tree::ptree &colorNode, uint8_t &red, uint8_t &green, uint8_t &blue)
{
	red = colorNode.get<uint8_t>("red");
	green = colorNode.get<uint8_t>("green");
	blue = colorNode.get<uint8_t>("blue");
}



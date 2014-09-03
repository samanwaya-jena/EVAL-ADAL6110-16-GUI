#define CV_NO_BACKWARD_COMPATIBILITY
#if 0

include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif
#include "tracker.h"

#include "ReceiverCapture.h"
#include "ReceiverUDPCapture.h"
#include "Tracker.h"

using namespace std;
using namespace pcl;
using namespace awl;


ReceiverUDPCapture::ReceiverUDPCapture(int inReceiverChannelQtyl):
ReceiverCapture(inReceiverChannelQty),
canHandle(0),
bitRate("50"),
acceptanceCode("0x1FFFFFFF"),
acceptanceMask("0x1FFFFFFF"),
currentFrame(new SensorFrame(0, inReceiverChannelQty)),
lastMessageID(0)

{
	debugFile.open("UDPLog.dat");

	// Get the command line arguments and initialize socket data

	XXXXXXXX

	// initialize the UDP port
	InitUDPSocket(serverIP, serverUDPPort, int *receiveFD, sendData, servAddr)
}

ReceiverUDPCapture::~ReceiverUDPCapture()
{
	debugFile.close();
	Stop();

	if (receiveFD) CloseUDPSocket();
}

ReceiverUDPCapture::InitUDPSocket(std::string serverIP, int serverUDPPort, int *receiveFD,
struct sockaddr_in *sendData, struct sockaddr_in *servAddr)
{

#ifdef __linux__
	int flags;
	int optval;
	socklen_t optlen;
#endif
#ifdef __WIN32__
	char optval;
	int optlen;

#endif

	socklen_t optlen;
	char str[255]	
	sprintf(str, "Connecting to server %s on UDP port %d\n", serverIP,
		serverUDPPort);	
	std::string myString(str);
	debugFile << str;

	if ((*receiveFD = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		sprintf(str, "DatagramSocketError\n");	
		std::string myString(str);
		debugFile << str;		
		
		perror("datagram socket");
		return;
	}


#ifdef __linux__
	if (-1 == (flags = fcntl(*receiveFD, F_GETFL, 0))) 
	{
		sprintf(str, "fcntl 1 Error\n");	
		std::string myString(str);
		debugFile << str;			
		perror("fcntl 1");
		flags = 0;
		return;
	}

	if (-1 == (fcntl(*receiveD, F_SETFL, flags | O_NONBLOCK))) 
	{
		sprintf(str, "fcntl 2 Error\n");	
		std::string myString(str);
		debugFile << str;			
		perror("fcntl 2");
		return;
	}
#endif

	if (getsockopt(*receiveFD, SOL_SOCKET, SO_RCVBUF, &optval, &optlen) == -1) 
	{
		sprintf(str, "GtSockOpt 1 Error\n");	
		std::string myString(str);
		debugFile << str;			

		perror("getsockopt 1");
		return;
	} 

	sprintf(str, "RECVBUF = %d\n", optval);	
	std::string myString(str);
	debugFile << str;	

	optval *= 2;
	if (setsockopt(*receiveFD, SOL_SOCKET, SO_RCVBUF, &optval, optlen) == -1) 
	{
		sprintf(str, "SetSockOpt Error\n");	
		std::string myString(str);
		debugFile << str;			

		perror("setsockopt");
		return;
	} 

	
	if (getsockopt(*receiveFD, SOL_SOCKET, SO_RCVBUF, &optval, &optlen) == -1) 
	{
		sprintf(str, "GetSockOpt2 Error\n");	
		std::string myString(str);
		debugFile << str;		

		perror("getsockopt 2");
		return;
	} 

	sprintf(str, "RECVBUF = %d\n", optval);	
	std::string myString(str);
	debugFile << str;	


	memset(servAddr, 0, sizeof(*serv_addr));
	servAddr->sin_family = AF_INET;
	servAddr->sin_addr.s_addr = inet_addr(serverIP);
	servAddr->sin_port = htons(PORT_NUM);

	memset(sendData, 0, sizeof(*sendData));
	sendData->sin_family = AF_INET;
	sendData->sin_addr.s_addr = htonl(INADDR_ANY);
	sendData->sin_port = htons(0);

	if (bind(*receiveFD, (struct sockaddr *)sendData, sizeof(*sendData)) <	0)
	{
		sprintf(str, "Bind Error\n");	
		std::string myString(str);
		debugFile << str;			

		perror("bind");
		return;
	}
}

void ReceiverUDPCapture:CloseUDPSocket()

{
}



void  ReceiverCUDPCapture::Go(bool inIsThreaded) 
{
	bIsThreaded = inIsThreaded;
	assert(!mThread);
    mStopRequested = false;

	if (bIsThreaded) 
	{
		mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ReceiverCANCapture::DoThreadLoop, this)));
#if 0
		// Set the priority under windows.  This is the most critical display thread 
		// for user interaction
	

		HANDLE th = mThread->native_handle();
		//   SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
		//   SetThreadPriority(th, THREAD_PRIORITY_ABOVE_NORMAL);
#endif
	}
}
 

void  ReceiverUDPCapture::Stop() 
{
	if (mStopRequested) return;
    mStopRequested = true;
	if (bIsThreaded) {
		bIsThreaded = false;
		assert(mThread);
		mThread->join();
	}
}


bool  ReceiverUDPCapture::WasStopped()
{
	if (mStopRequested) return(true);
	return(false);
}

void ReceiverUDPCapture::DoThreadLoop()

{
	while (!WasStopped())
    {
		DoOneThreadIteration();
	} // while (!WasStoppped)
}

void ReceiverUDPCapture::DoThreadIteration()

{
	if (!bIsThreaded)
    {
		DoOneThreadIteration();
	} // while (!WasStoppped)
}


void ReceiverUDPCapture::DoOneThreadIteration()

{
	if (!WasStopped())
    {
		lastMessageID = 0;

		if (canHandle) 
		{
			AWLCANMessage msg;
			float distance;

			int result = canplus_Read(canHandle, &msg);

			if (result > 0) 
			{
				unsigned long msgID = msg.id;


				if (msgID >= 20 && msgID <= 26) 
				{
					ProcessChannelDistance(msg);
					lastMessageID = msgID;
				}
				else if (msgID >= 30 && msgID <= 36) 
				{
					ProcessChannelDistance(msg);
					lastMessageID = msgID;
					// On the last distance message, notify send the sensor frame to the application.
					if (msgID == 36) ProcessCompletedFrame();
				}
				else
				{

					char str[255];
					boost::posix_time::ptime myTime(boost::posix_time::microsec_clock::local_time());
					std::string timeStr(boost::posix_time::to_simple_string(myTime));
					sprintf(str, "Msg %d - %s- \n", msgID, timeStr.c_str());
					std::string myString(str);
					debugFile << myString;

					lastMessageID = msgID;
				}
			}  // If result
			else 
			{
				char str[255];
				boost::posix_time::ptime myTime(boost::posix_time::microsec_clock::local_time());
				std::string timeStr(boost::posix_time::to_simple_string(myTime));
				sprintf(str, "-   %s -  result %d, status %d\n", timeStr.c_str(), result, canplus_Status(canHandle));
				std::string myString(str);
				debugFile << myString;

				// If result == 0, we are just in a read time out.
				// if result < 0 it is a error.  Flush and restart
				if ((result == ERROR_CANPLUS_NO_MESSAGE) || (result == 0))
				{
					boost::this_thread::interruptible_wait(5);
				}
				else 
				{
					canplus_Flush(canHandle);
					boost::this_thread::interruptible_wait(20);
				}
			}

		}

	} // while (!WasStoppped)
}


void ReceiverCANCapture::ProcessChannelDistance(AWLCANMessage &inMsg)

{
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());

	int channel;
	int detectOffset = 0;
	uint16_t *distancePtr = (uint16_t *) inMsg.data;

	if (inMsg.id >= 30) 
	{
		channel = inMsg.id - 30;
		detectOffset = 4;
	}
	else 
	{
		channel = inMsg.id - 20;
	}

		float distance = (float)(distancePtr[0]);
		distance *= distanceScale;
		distance /= 100;
		distance += measurementOffset;
		distance += sensorDepth;


		currentFrame->channelFrames[channel]->timeStamp = GetElapsed();
		if (distance < minDistance  || distance > maxDistances[channel]) distance = 0.0;

		int detectionIndex = 0+detectOffset;
		Detection::Ptr detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
		
		detection->distance = distance;
		detection->firstTimeStamp = currentFrame->channelFrames[channel]->timeStamp;
		detection->timeStamp = currentFrame->channelFrames[channel]->timeStamp;
		detection->trackID = 0;
		detection->velocity = 0;

		distance = (float)(distancePtr[1]);
		distance *= distanceScale;
		distance /= 100;
		distance += measurementOffset;
		distance += sensorDepth;


		if (distance < minDistance  || distance > maxDistances[channel]) distance = 0.0;
		detectionIndex = 1+detectOffset;
		detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
		
		detection->distance = distance;
		detection->firstTimeStamp = currentFrame->channelFrames[channel]->timeStamp;
		detection->timeStamp = currentFrame->channelFrames[channel]->timeStamp;
		detection->trackID = 0;
		detection->velocity = 0;
	
		
		distance = (float)(distancePtr[2]);
		distance *= distanceScale;
		distance /= 100;
		distance += measurementOffset;
		distance += sensorDepth;

		if (distance < minDistance  || distance > maxDistances[channel]) distance = 0.0;
		detectionIndex = 2+detectOffset;
		detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
		
		detection->distance = distance;
		detection->firstTimeStamp = currentFrame->channelFrames[channel]->timeStamp;
		detection->timeStamp = currentFrame->channelFrames[channel]->timeStamp;
		detection->trackID = 0;
		detection->velocity = 0;
	

		distance = (float)(distancePtr[3]);
		distance *= distanceScale;
		distance /= 100;
		distance += measurementOffset;
		distance += sensorDepth;

		if (distance < minDistance  || distance > maxDistances[channel]) distance = 0.0;
		detectionIndex = 3+detectOffset;
		detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
		
		detection->distance = distance;
		detection->firstTimeStamp = currentFrame->channelFrames[channel]->timeStamp;
		detection->timeStamp = currentFrame->channelFrames[channel]->timeStamp;
		detection->trackID = 0;
		detection->velocity = 0;

	rawLock.unlock();

	char str[255];

	boost::posix_time::ptime myTime(boost::posix_time::microsec_clock::local_time());
	std::string timeStr(boost::posix_time::to_simple_string(myTime));

	sprintf(str, "Msg %d - %s - Val %d %d %d %d\n", inMsg.id, timeStr.c_str(), distancePtr[0], distancePtr[1], distancePtr[2], distancePtr[3]);
	std::string myString(str);
	debugFile << myString;
}

void ReceiverCANCapture::ProcessChannelIntensity(AWLCANMessage &inMsg)

{
	int channel;
	int detectOffset = 0;
	uint16_t *intensityPtr = (uint16_t *) inMsg.data;

	if (inMsg.id >= 50) 
	{
		channel = inMsg.id - 50;
		detectOffset = 4;
	}
	else 
	{
		channel = inMsg.id - 40;
	}

	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	float intensity = ((float) intensityPtr[0]) / maxIntensity;
	int detectionIndex = 0+detectOffset;
	Detection::Ptr detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
	detection->intensity = intensity;
	detection->trackID = 0;
	detection->velocity = 0;

	intensity = ((float) intensityPtr[1]) / maxIntensity;
	detectionIndex = 1+detectOffset;
	detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
	detection->intensity = intensity;
	detection->trackID = 0;
	detection->velocity = 0;

	intensity = ((float) intensityPtr[2]) / maxIntensity;
	detectionIndex = 2+detectOffset;
	detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
	detection->intensity = intensity;
	detection->trackID = 0;
	detection->velocity = 0;

	intensity = ((float) intensityPtr[3]) / maxIntensity;
	detectionIndex = 3+detectOffset;
	detection = currentFrame->MakeUniqueDetection(channel, detectionIndex);
	detection->intensity = intensity;
	detection->trackID = 0;
	detection->velocity = 0;
	rawLock.unlock();


	char str[255];

	boost::posix_time::ptime myTime(boost::posix_time::microsec_clock::local_time());
	std::string timeStr(boost::posix_time::to_simple_string(myTime));

	sprintf(str, "Msg %d - %s - Val %d %d %d %d\n", inMsg.id, timeStr.c_str(), intensityPtr[0], intensityPtr[1], intensityPtr[2], intensityPtr[3]);
	std::string myString(str);
	debugFile << myString;
}

void ReceiverCANCapture::ProcessCompletedFrame()

{
	
	boost::mutex::scoped_lock rawLock(currentReceiverCaptureSubscriptions->GetMutex());
	
	// Push the current frame in the frame buffer
	acquisitionSequence->sensorFrames.push(currentFrame);
	currentReceiverCaptureSubscriptions->PutNews(currentFrame->frameID);

	// Make sure we do not keep too many of those frames around.
	// Remove the older frame if we exceed the buffer capacity
	if (acquisitionSequence->sensorFrames.size() > maximumSensorFrames) 
	{
		acquisitionSequence->sensorFrames.pop();
	}

	// Create a new current frame.
	uint32_t frameID = acquisitionSequence->AllocateFrameID();

	currentFrame = SensorFrame::Ptr(new SensorFrame(frameID, receiverChannelQty));

	rawLock.unlock();
}
#endif
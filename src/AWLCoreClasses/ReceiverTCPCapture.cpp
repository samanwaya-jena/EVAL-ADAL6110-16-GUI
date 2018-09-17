/* ReceiverTCPCapture.cpp */
/*
	Copyright 2014, 2015 Phantom Intelligence Inc.

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>

// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

#define DEFAULT_ADDRESS "localhost"
#define DEFAULT_PORT "27011"


#include <stdio.h>
#include <string.h>
#include "ReceiverTCPCapture.h"


using namespace std;
using namespace awl;

const int reopenPortDelaylMillisec = 2000; // We try to repopen the conmm fds every repoenPortDelayMillisec, 
										   // To see if the system reconnects


ReceiverTCPCapture::ReceiverTCPCapture(int receiverID, boost::property_tree::ptree &propTree):
  ReceiverPolledCapture(receiverID, propTree),
  TCPServerAddress(DEFAULT_ADDRESS),
  TCPServerPort(DEFAULT_PORT)
{
  // Read the configuration from the configuration file
  ReadConfigFromPropTree(propTree);
  ReadRegistersFromPropTree(propTree);
}

ReceiverTCPCapture::~ReceiverTCPCapture()
{
}

int ReceiverTCPCapture::ReadBytes(uint8_t * pData, int num)
{
  int numRem = num;
  int received;

  while (numRem)
  {
    received = recv((SOCKET)handle, (char*)pData, numRem, 0);
    if (received <= 0)
      return -1;

    pData += received;
    numRem -= received;
  }

  return num;
}

int ReceiverTCPCapture::WriteBytes(uint8_t * pData, int num)
{
  int numRem = num;
  int transferred;

  while (numRem)
  {
    transferred = send((SOCKET)handle, (char*)pData, numRem, 0);
    if (transferred <= 0)
      return -1;

    pData += transferred;
    numRem -= transferred;
  }

  return num;
}

bool  ReceiverTCPCapture::OpenCANPort()
{
  int ret = 0;

  reconnectTime = boost::posix_time::microsec_clock::local_time() + boost::posix_time::milliseconds(reopenPortDelaylMillisec);

	SOCKET ConnectSocket = INVALID_SOCKET;
	WSADATA wsaData;
	struct addrinfo *result = NULL,
		*ptr = NULL,
		hints;
	int iResult;

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	ZeroMemory( &hints, sizeof(hints) );
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo(TCPServerAddress.c_str(), TCPServerPort.c_str(), &hints, &result);
	if ( iResult != 0 ) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 1;
	}

	// Attempt to connect to an address until one succeeds
	for(ptr=result; ptr != NULL ;ptr=ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, 
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return 1;
		}

		// Connect to server.
		iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return 1;
	}

	handle = (void*) ConnectSocket;

  if (handle)
  {
    int transferred = 0;
    int received = 0;

    boost::mutex::scoped_lock rawLock(m_Mutex);

    AWLCANMessage msg;
    msg.id = AWLCANMSG_ID_LIDARQUERY;

    AWLCANMessage resp;

		transferred = WriteBytes((uint8_t*) &msg, sizeof(msg));
		if (transferred != sizeof(AWLCANMessage))
			return false;

		received = ReadBytes((uint8_t*) &resp, sizeof(resp));
		if (received != sizeof(AWLCANMessage))
			return false;

		rawLock.unlock();

		SendSoftwareReset();

    m_FrameRate = 0;

    return true;
  }

  return false;
}

bool  ReceiverTCPCapture::CloseCANPort()
{
  boost::mutex::scoped_lock rawLock(m_Mutex);

  if (handle)
  {
		// shutdown the connection since no more data will be sent
		shutdown((SOCKET) handle, SD_SEND);

		// cleanup
		closesocket((SOCKET) handle);

    handle = NULL;
  }

	WSACleanup();

	reconnectTime = boost::posix_time::microsec_clock::local_time()+boost::posix_time::milliseconds(reopenPortDelaylMillisec);

	return(true);
}

bool ReceiverTCPCapture::ReadConfigFromPropTree(boost::property_tree::ptree &propTree)
{
	ReceiverCANCapture::ReadConfigFromPropTree(propTree);

	char receiverKeyString[32];
	sprintf(receiverKeyString, "config.receivers.receiver%d", receiverID);
	std::string receiverKey = receiverKeyString;

	boost::property_tree::ptree &receiverNode =  propTree.get_child(receiverKey);
	// Communication parameters

  TCPServerAddress =  receiverNode.get<string>("TCPServerAddress", DEFAULT_ADDRESS);
  TCPServerPort =  receiverNode.get<string>("TCPServerPort", DEFAULT_PORT);

	return(true);
}

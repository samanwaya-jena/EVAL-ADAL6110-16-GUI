#ifndef AWLSIMPLEDEMO_H
#define AWLSIMPLEDEMO_H
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

#include <boost/container/vector.hpp>


#include "AWLSettings.h"
#include "ReceiverCapture.h"

namespace awl
{



class AWLSimpleDemo
{
//		public types and enums:
public:
public:
	AWLSimpleDemo();
	~AWLSimpleDemo();
	int exec();


protected:
	void DisplayReceiverStatus();
	void DisplayReceiverStatus(int receiverID);

	bool DoOneLoopIteration();
	bool GetLatestDetections(Detection::Vector &detectionData);
	bool OutputDetections(const Detection::Vector &detectionData);

private:
	ReceiverCapture::List receiverCaptures;

	/** \brief Our subscription identifier to access to lidar data. */
	boost::container::vector<Publisher::SubscriberID> receiverCaptureSubscriberIDs;
};
} // namespace AWL          

#endif // AWLSimpleDEMO_H

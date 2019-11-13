/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the BareDemo application of the
** LiDAR Sensor Toolkit.
**
** $PHANTOM_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding a valid commercial license granted by Phantom Intelligence
** may use this file in  accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the terms
** contained in a written agreement between you and Phantom Intelligence.
** For licensing terms and conditions contact directly
** Phantom Intelligence using the contact informaton supplied above.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.LGPL3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License  version 3 or any later version approved by
** Phantom Intelligence. The licenses are as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $PHANTOM_END_LICENSE$
**
****************************************************************************/

#ifndef AWLSIMPLEDEMO_H
#define AWLSIMPLEDEMO_H

#include <boost/container/vector.hpp>


#include "AWLSimpleSettings.h"
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
	bool GetLatestDetections(SensorCoreScope::Detection::Vector &detectionData);
	bool OutputDetections(const SensorCoreScope::Detection::Vector &detectionData);

private:
	SensorCoreScope::ReceiverCapture::List receiverCaptures;

	/** \brief Our subscription identifier to access to lidar data. */
	boost::container::vector<Publisher::SubscriberID> receiverCaptureSubscriberIDs;
};
} // namespace AWL          

#endif // AWLSimpleDEMO_H

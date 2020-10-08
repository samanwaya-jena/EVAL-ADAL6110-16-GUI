/* LoopedWorker.cpp */
/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the SensorCoreClasses library of the
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

#include "SensorCoreClassesGlobal.h"
#include "LoopedWorker.h"

SENSORCORE_USE_NAMESPACE

LoopedWorker::LoopedWorker():
mWorkerRunning(false)
{

}

LoopedWorker::~LoopedWorker()
{
	if (mWorkerRunning) Stop();
}

void LoopedWorker::Go() 
	
{
    mWorkerRunning = true;
}
 

void  LoopedWorker::Stop() 
{
	if (!mWorkerRunning) return;
    mWorkerRunning = false;
}


bool  LoopedWorker::WasStopped()
{
	if (!mWorkerRunning) return(true);
	return(false);
}

#if 0
// Sample code for a LoopedWorker loop iteration
bool LoopedWorker::SpinOnce()

{

	if (WasStopped()) return(false);
	...
	...
	if (someStopCondition)
	{
		Stop();
		return(false);
	}

	return(true);
}
#endif

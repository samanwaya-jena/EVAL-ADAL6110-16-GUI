/* ThreadedWorker.cpp : Basic management of a thread loop*/
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
#include "ThreadedWorker.h"

SENSORCORE_USE_NAMESPACE

ThreadedWorker::ThreadedWorker():
mWorkerRunning(false)

{

}

ThreadedWorker::~ThreadedWorker()
{
	if (mWorkerRunning) Stop();
}

void ThreadedWorker::Go() 
	
{
	assert(!mThread);
    mWorkerRunning = true;

	mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ThreadedWorker::DoThreadLoop, this)));
}
 

void  ThreadedWorker::Stop() 
{
	StopThread();
}


bool  ThreadedWorker::WasStopped()
{
	if (!mWorkerRunning) return(true);
	return(false);
}

void  ThreadedWorker::StopThread()
{
	if (!mWorkerRunning) return;
    mWorkerRunning = false;

	assert(mThread);
	mThread->join();
}


#if 0
// Sample code for a threadedworker loop
void ThreadedWorker::DoThreadLoop()

{

	while (!WasStopped())
    {
		DoOneThreadIteration();
		if (someCondition)
		{
			StopThread();
		}

	} // while (!WasStoppped)
}
#endif

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

#ifndef _LOOPEDWORKER_H
#define _LOOPEDWORKER_H

#ifndef Q_MOC_RUN
#include <boost/thread/mutex.hpp>
#endif

#include "SensorCoreClassesGlobal.h"

SENSORCORE_BEGIN_NAMESPACE

/** \brief Abstract class defining common behavior for the all worker classes that are not threaded and are
  *        "pumped" by the application's main loop.
  *        
  * \Notes:	To operate a looped worker, the caller should usually follow a sequence similar to:
  *             LoopedWorker *worker = new ThreadedWorker(...)
  *				worker->Go();  // Initializes the worker objects before the loop
  *             while (!worker->IsStopped())
  *             {
  *					if !worker->SpinOnce() break;
  *                 if (someUserAction) worker->Stop();
  *             }
  *             delete (worker);
  *
  *          The worker class must implement a SpinOnce() method.
  *
  * \author Jean-Yves Deschênes
  */
class LoopedWorker
{
public:

	LoopedWorker();
	/** \brief ThreadedWorker Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~LoopedWorker();

	/** \brief Initialize the worker loop, before loop start.
      */
	virtual void  Go();

	/** \brief Stop worker. Free all objects created during for or during the loop.
	  * \notes  To avoid recursion, Stop() shoudl never call WasStopped().
	  *         Descendants should always use the mWorkerRunning member directly, when in the Stop() method.
      */
	virtual void  Stop(); 

	/** \brief Return worker loop  running status. The loop may Stop() if there are workin ellements mission or Stop()ped. 
      * \return true if the loop is stopped.
      */
	virtual bool  WasStopped();

	/** \brief Do a single iteration of the worker loop.
	  * \return Returns true is the loop is pursuing.  
	  *         Returns false if the loop should be stopped.
      */
	virtual void  SpinOnce() = 0;


protected:
   /** \brief Local flag indicating the running status of the thread. */
	volatile bool mWorkerRunning;
	}; // LoopedWorker

SENSORCORE_END_NAMESPACE

#endif //_LOOPEDWORKER_H
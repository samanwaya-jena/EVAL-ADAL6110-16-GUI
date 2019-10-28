#ifndef _THREADEDWORKER_H
#define _THREADEDWORKER_H

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

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

#include "SensorCoreClassesGlobal.h"

SENSORCORE_BEGIN_NAMESPACE



/** \brief Abstract class defining common behavior for the all threaded worker classes.
  *        
  * \Notes:	To operate a threaded worker, the caller should usually follow a sequence similar to:
  *             ThreadedWorker *worker = new ThreadedWorker(...)
  *				worker->Go();
  *             while (!worker->IsStopped())
  *             {
  *                 if (someUserAction) worker->Stop();
  *             }
  *             delete (worker);
  *
  *          The worker class must implement a DoThreadLoop() method of the style
  *			    while (!WasStopped())
  *			    {
  *		            DoOneThreadIteration();
  *				} // while (!WasStoppped)
  *
  * \author Jean-Yves Deschênes
  */
class ThreadedWorker
{
public:

	ThreadedWorker();
	/** \brief ThreadedWorker Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~ThreadedWorker();

	/** \brief Start the worker thread.  Initialize all objects before thread start.
      */
	virtual void  Go();

	/** \brief Stop worker thread. Free all objects created during the thread.
	  * \notes  To avoid recursion, Stop() shoudl never call WasStopped().
	  *         Descendants should always use the mWorkerRunning member directly, when in the Stop() method.
      */
	virtual void  Stop(); 

	/** \brief Return worker thread  running status
      * \return true if the thread is stopped.
      */
	virtual bool  WasStopped();


protected:
	/** \brief Return the lidar data rendering thread status
      * \return true if the lidar data rendering thread is stoppped.
      */
	virtual void  DoThreadLoop() = 0;

protected:
	/** \brief Thread management object. */
    boost::shared_ptr<boost::thread> mThread;

   /** \brief Local flag indicating the running status of the thread. */
	volatile bool mWorkerRunning;

	/** \brief Stop the lidar data projection thread
	 *         The Stop() method should call StopThread()
	 
	 *         In some threads, the internal Stopping management may be different
	 *		   depending on the calling thread.
	 *		   (That is the case with some OpenCV objects).
     */
	virtual void  StopThread(); 
	}; // ThreadedWorker

SENSORCORE_END_NAMESPACE

#endif //_THREADEDWORKER_H
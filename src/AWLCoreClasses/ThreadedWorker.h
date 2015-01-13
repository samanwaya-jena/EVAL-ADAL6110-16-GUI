#ifndef _THREADEDWORKER_H
#define _THREADEDWORKER_H

/*
	Copyright 2014 Aerostar R&D Canada Inc.

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

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

namespace awl
{


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
  * \author Jean-Yves Desch�nes
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
	 *         The Stop() method should call StopThread(false)
	 *         All calls to stop from within the thread should call
	 *		   StopThread(true).
	 *         In some threads, the internal Stopping management may be different
	 *		   depending on the calling thread.
	 *		   (That is the case with some OpenCV objects).
     */
	virtual void  StopThread(bool bCalledFromThread = false); 
	}; // ThreadedWorker

} // namespace awl

#endif //_THREADEDWORKER_H
#ifndef _THREADEDWORKER_H
#define _THREADERWORKER_H

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
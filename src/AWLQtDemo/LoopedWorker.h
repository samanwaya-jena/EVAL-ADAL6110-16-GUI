#ifndef _LOOPEDWORKER_H
#define _LOOPEDWORKER_H

#ifndef Q_MOC_RUN
#include <boost/thread/mutex.hpp>
#endif

namespace awl
{


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

} // namespace awl

#endif //_LOOPEDWORKER_H
#ifndef AWL_RECEIVER_FILE_CAPTURE_H
#define AWL_RECEIVER_FILE_CAPTURE_H

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include <stdint.h>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#endif

#include "Subscription.h"
#include "Tracker.h"
#include "ReceiverCapture.h"
using namespace std;
using namespace pcl;

namespace awl
{


/** \brief Threaded ReceiveFileCapture class is used to simulate data acquisition from LIDAr using a file stream.
  *        The receiver file capture buffers up a few frames to faciulitae processing afterwards.
  * \author Jean-Yves Deschênes
  */
class ReceiverFileCapture: public ReceiverCapture
{
// Public types
public:
	typedef boost::shared_ptr<ReceiverFileCapture> Ptr;
    typedef boost::shared_ptr<ReceiverFileCapture> ConstPtr;

// public Methods
public:

	/** \brief ReceiverFileCapture constructor.
 	    * \param[in] inSequenceID  unique sequence ID (for documentation)
	    * \param[in] inReceiverChannelQty index of the required channel
 	    * \param[in] inDetectionsPerChannel number of detections per channel
      */

	ReceiverFileCapture(int inReceiverID, int inSequenceID, int inReceiverChannelQty, int inDetectionsPerChannel, std::string inFileName);

	/** \brief ReceiverFileCapture Destructor.  Insures that all threads are stopped before destruction.
      */
	virtual ~ReceiverFileCapture();

	/** \brief Start the lidar Data Projection  thread
      */
	virtual void  Go(bool inIsThreaded = false); 

	/** \brief Stop the lidar data projection thread
      */
	virtual void  Stop(); 

	/** \brief Return the video acquisition thread status
      * \return true if the video acquisition thread is stoppped.
      */
	virtual bool  WasStopped();

	// public variables
public:

	/** \brief Do one iteration of the thread loop.
      */
	virtual void DoThreadIteration();

// Protected methods
protected:
	/** \brief Return the lidar data rendering thread status
      * \return true if the lidar data rendering thread is stoppped.
      */
	void  DoThreadLoop();

	/** \brief Do one iteration of the thread loop.
      */
	virtual void DoOneThreadIteration();

// Protected variables
protected:
};


} // namespace AWL

#endif
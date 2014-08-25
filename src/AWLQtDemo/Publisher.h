#ifndef AWL_PUBLISHER_H
#define AWL_PUBLISHER_H

#include <vector> 

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/container/vector.hpp>
#include <boost/thread/thread.hpp>
#endif

using namespace std;

/** \brief Simple thread-safe class for a object that needs to inform
  * other objects of changes to its contents..
  * Just helps determine when a Publisher has an updated "publication" in store for 
  * a given subscriber.
  * \Notes:
  *    To use the class, a subscriber does:
  *        Publisher::SubscriberID subscriberID= publisher->Subscribe();  // Subscribe to a publisher
  *        if (publisher->HasNews(subscriberID))
  *        {
  *            if (publisher->LockNews(subscriberID));  // Informs the publisher and locks the mutex; 
  *			   {
  *                ... // Process the publisher news
  *			       publisher->FreeNews(subscriberID);  // Frees the lock on the news.
  *            }
  *        }
  * \author Jean-Yves Deschênes
  */


namespace awl
{

class Publisher
{
public:
	typedef  int SubscriberID;

	typedef boost::shared_ptr<Publisher> Ptr;
	typedef boost::container::vector<Publisher::Ptr> List;

	Publisher();
	Publisher::SubscriberID Subscribe();

	// Returns the number of publications issued since.
	// Note that the publications are not kept in store
	// the qty is for informational purposes.

	int HasNews(SubscriberID inSubscriber);
	
	// Locks the publisher's mutex for accessing the news.
	// and clears the number of publications missed for the given subscriber
	// \return Returns true if successfully locked.  Otherwise returns false.
	//         (Lock may fail if subscriber ID is invalid).
	bool LockNews(SubscriberID inSubscriber);

	// Unlocks the publisher's mutex for accessing the news.
	void UnlockNews(SubscriberID inSubscriber);

protected:
	// Increments the publications counter for all subscribers
	void PutNews();

	boost::mutex &GetMutex() {return(mMutex);};

	// Fir each subscriber, the vector contains the quantity of unpublished news.
	boost::container::vector<int> subscribers;

   boost::mutex mMutex;

}; // Publisher

} // namespace awl
#endif  // AWL_PUBLISHER_H
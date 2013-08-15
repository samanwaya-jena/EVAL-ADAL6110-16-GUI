#ifndef AWL_SUBSCRIPTION_H
#define AWL_SUBSCRIPTION_H

#include <vector> 

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#endif

using namespace std;

/** \brief Simple threaded subscription management class.
  * Just helps determine when an object has an updated "publication" in store for 
  * a given subscriber.
  * A subscribet
  * \author Jean-Yves Deschênes
  */


namespace awl
{

class Subscription
{
public:
	typedef  int SubscriberID;

	typedef boost::shared_ptr<Subscription> Ptr;

	Subscription();
	Subscription::SubscriberID Subscribe();

	// Returns the number of publications issued since.
	// Note that the publications are not kept in store
	// the qty is for informational purposes.

	int HasNews(SubscriberID inSubscriber);
	
	// Clears the number of publications missed for the given subscriber
	void GetNews(SubscriberID inSubscriber);

	// Increments the publications counter for all subscribers
	void PutNews();

	// Gets the mutex used to access the data
	boost::mutex& GetMutex() {return (mMutex);};

protected:
	std::vector<int> subscribers;

   boost::mutex mMutex;

}; // Subscription

} // namespace awl
#endif
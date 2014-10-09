#ifndef AWL_SUBSCRIPTION_H
#define AWL_SUBSCRIPTION_H

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

#include <vector> 

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/container/vector.hpp>
#include <boost/thread/thread.hpp>
#endif

using namespace std;

/** \brief Simple threaded subscription management class.
  * Just helps determine when an object has an updated "publication" in store for 
  * a given subscriber.
  * A subscriber just 
  * \author Jean-Yves Deschênes
  */


namespace awl
{

class Subscription
{
public:
	typedef  int SubscriberID;

	typedef boost::shared_ptr<Subscription> Ptr;
	typedef boost::container::vector<Subscription::Ptr> List;

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

	// Fir each subscriber, the vector contains the quantity of unpublished news.
	boost::container::vector<int> subscribers;

   boost::mutex mMutex;

}; // Subscription
} // namespace awl
#endif
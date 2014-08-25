#include <iostream>

#include "Publisher.h"

using namespace std;
using namespace awl;



Publisher::Publisher()
{
	subscribers.clear();
}

Publisher::SubscriberID Publisher::Subscribe()
{
	int publicationQty = 0;
	subscribers.push_back(publicationQty);
	
	// The subscriberID is actually the index in the subscriber array
	return ((SubscriberID) subscribers.size()-1);
}

int Publisher::HasNews(SubscriberID inSubscriber)

{
	if (inSubscriber < 0 || inSubscriber >= (int) subscribers.size())
	{
		return(0);
	}
	else
	{
		return(subscribers.at(inSubscriber));                           
	}
}

bool Publisher::LockNews(SubscriberID inSubscriber)

{
	if (inSubscriber < 0 || inSubscriber >= (int) subscribers.size())
	{
		return(false);
	}

	subscribers.at(inSubscriber) = 0;
	mMutex.lock();
	return(true);
}

void Publisher::UnlockNews(SubscriberID inSubscriber)

{
	if (inSubscriber < 0 || inSubscriber >= (int) subscribers.size())
	{
		return;
	}

	mMutex.unlock();
}


void Publisher::PutNews()
{
	int subscriberQty = subscribers.size();
	for (int i = 0; i < subscriberQty; i++) 
	{
		mMutex.lock();
		int publications = subscribers.at(i);
		publications++;
		subscribers.at(i) = publications;
		mMutex.unlock();
	}
}

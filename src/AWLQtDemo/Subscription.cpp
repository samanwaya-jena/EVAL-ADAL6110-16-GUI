#include <iostream>

#include "Subscription.h"

using namespace std;
using namespace awl;



Subscription::Subscription()
{
	subscribers.clear();
}

Subscription::SubscriberID Subscription::Subscribe()
{
	int publicationQty = 0;
	subscribers.push_back(publicationQty);
	
	return ((SubscriberID) subscribers.size()-1);
}

int Subscription::HasNews(SubscriberID inSubscriber)

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

void Subscription::GetNews(SubscriberID inSubscriber)

{
	if (inSubscriber < 0 || inSubscriber >= (int) subscribers.size())
	{
		return;
	}

	subscribers.at(inSubscriber) = 0;
}

void Subscription::PutNews()
{
	int subscriberQty = subscribers.size();
	for (int i = 0; i < subscriberQty; i++) 
	{
		int publications = subscribers.at(i);
		publications++;
		subscribers.at(i) = publications;
	}

}

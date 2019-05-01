/* Subscription.cpp */
/*
	Copyright 2014, 2015 Phantom Intelligence Inc.

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

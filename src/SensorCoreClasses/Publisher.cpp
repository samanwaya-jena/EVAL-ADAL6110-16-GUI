/* Publisher.cpp */
/****************************************************************************
**
** Copyright (C) 2014-2019 Phantom Intelligence Inc.
** Contact: https://www.phantomintelligence.com/contact/en
**
** This file is part of the SensorCoreClasses library of the
** LiDAR Sensor Toolkit.
**
** $PHANTOM_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding a valid commercial license granted by Phantom Intelligence
** may use this file in  accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the terms
** contained in a written agreement between you and Phantom Intelligence.
** For licensing terms and conditions contact directly
** Phantom Intelligence using the contact informaton supplied above.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.LGPL3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License  version 3 or any later version approved by
** Phantom Intelligence. The licenses are as published by the Free Software
** Foundation and appearing in the file PHANTOM_LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $PHANTOM_END_LICENSE$
**
****************************************************************************/

#include <iostream>

#include "SensorCoreClassesGlobal.h"
#include "Publisher.h"

SENSORCORE_USE_NAMESPACE

Publisher::Publisher()
{
	currentPublications.clear();
	consumedPublications.clear();}

Publisher::SubscriberID Publisher::Subscribe()
{
	IssueID IssueID = 0;
	currentPublications.push_back(IssueID);
	consumedPublications.push_back(IssueID);	

	// The subscriberID is actually the index in the subscriber array
	return ((SubscriberID) currentPublications.size()-1);
}

bool Publisher::HasNews(SubscriberID inSubscriber)

{
	if (inSubscriber < 0 || inSubscriber >= (int) currentPublications.size())
	{
		return(false);
	}
	else
	{
		return(currentPublications.at(inSubscriber)!= consumedPublications.at(inSubscriber));                           
	}
}

bool Publisher::LockNews(SubscriberID inSubscriber)

{
	if (inSubscriber < 0 || inSubscriber >= (int) currentPublications.size())
	{
		return(false);
	}

	consumedPublications.at(inSubscriber) = currentPublications.at(inSubscriber);
	mMutex.lock();
	return(true);
}

bool Publisher::LockNews(SubscriberID inSubscriber, IssueID inIssueID)

{
	if (inSubscriber < 0 || inSubscriber >= (int) currentPublications.size())
	{
		return(false);
	}

	consumedPublications.at(inSubscriber) = inIssueID;
	mMutex.lock();
	return(true);
}

void Publisher::UnlockNews(SubscriberID inSubscriber)

{
	if (inSubscriber < 0 || inSubscriber >= (int) currentPublications.size())
	{
		return;
	}

	mMutex.unlock();
}


void Publisher::PutNews()
{
	size_t subscriberQty = currentPublications.size();
	for (size_t i = 0; i < subscriberQty; i++) 
	{
		mMutex.lock();
		IssueID issueID = currentPublications.at(i);
		issueID++;
		currentPublications.at(i) = issueID;
		mMutex.unlock();
	}
}

void Publisher::PutNews(IssueID inIssueID)
{
	size_t subscriberQty = currentPublications.size();
	for (size_t i = 0; i < subscriberQty; i++) 
	{
		mMutex.lock();
		currentPublications.at(i) = inIssueID;
		mMutex.unlock();
	}
}

Publisher::IssueID Publisher::GetCurrentIssueID(SubscriberID inSubscriber)

{
	if (inSubscriber < 0 || inSubscriber >= (int) currentPublications.size())
	{
		return(0);
	}

	return(currentPublications.at(inSubscriber));
}

Publisher::IssueID Publisher::GetConsumedIssueID(SubscriberID inSubscriber)
{
	if (inSubscriber < 0 || inSubscriber >= (int) currentPublications.size())
	{
		return(0);
	}

	return(consumedPublications.at(inSubscriber));
}

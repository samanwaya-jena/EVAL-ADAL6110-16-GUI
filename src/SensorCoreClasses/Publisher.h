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

#ifndef SENSORCORE_PUBLISHER_H
#define SENSORCORE_PUBLISHER_H

#include <vector> 

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/container/vector.hpp>
#include <boost/thread/thread.hpp>
#endif

#include "SensorCoreClassesGlobal.h"




SENSORCORE_BEGIN_NAMESPACE


/** \brief Simple thread-safe class for a object that needs to inform
  * other objects of changes to its contents..
  * Just helps determine when a Publisher has an updated "publication" in store for
  * a given subscriber.
  * A Publisher keeps track of its current publication number, and also keeps track of the
  * latest issue that was consumed (Locked) by the subscriber.
  * This way, it can tell the subscriber which issue was the last consumed.
  *
  * \Notes:
  *    For a simple use of the class, a subscriber does:
  *        PublisherDerivateClass publisher();
  *        Publisher::SubscriberID subscriberID= publisher->Subscribe();  // Subscribe to a publisher
  *        // In application event loop
  *        if (publisher->HasNews(subscriberID))
  *        {
  *            if (publisher->LockNews(subscriberID));  // Informs the publisher and locks the mutex;
  *			   {
  *                ... // Process the publisher news
  *                publisher->GetIssueDerivateClassMethod(latestIssueNumber);
  *			       publisher->UnlockNews(subscriberID);  // Frees the lock on the news.
  *            }
  *        }
  *
  *    Since some publishers also have non-incremental Publication ID, a more complex subscriber could:
  *        PublisherDerivateClass publisher();
  *        Publisher::SubscriberID subscriberID= publisher->Subscribe();  // Subscribe to a publisher
  *
  *        // In application event loop
  *        if (publisher->HasNews())
  *        {
  *            Publisher::IssueID latestIssue = publisher->GetCurrentIssueID();
  *            Publisher::IssueID issueOnHand =  publisher->GetConsumedIssueID();
  *            Publisher::IssueID issueRequested =  issueOnHand;
  *            ... // Process all of the back issues
  *            do
  *			   {
  *                issueRequested++;  // Request the next issue.
  *                if (publisher->LockNews(subscriberID, issueRequested);  // Informs the publisher and locks the mutex;
  *			       {
  *                     publisher->GetIssueDerivateClassMethod(issueRequested);
  *			            publisher->UnlockNews(subscriberID);  // Frees the lock on the news.
  *				   }
  *            } while (issueRequested != lastIssue);
  *        }
  *
  *        ....
  *        // You may reuse the IssueID to request a re-issue from the publisher.
  *        publisher->GetIssueDerivateClassMethod(issueOnHandNumber);
  * \author Jean-Yves Deschênes
  */
  class Publisher
{
public:
    /** \brief Unique ID assigned to a susbsriber to a specific publication. */
	typedef  int SubscriberID;

    /** \brief Unique ID assigned to a specific issue of a publication. */
    typedef  uint32_t IssueID;

	typedef boost::shared_ptr<Publisher> Ptr;
	typedef boost::container::vector<Publisher::Ptr> List;

    /** \brief Constructor. */
	Publisher();

    /** \brief Subscribe to a publisher object. 
     * \return Unique SubscriberID, used in further iteractions with the publisher.
     */
	Publisher::SubscriberID Subscribe();

    /** \brief Determine if there is any new issue from the publisher, for the given Subscriber.
     * \Return true if the current publication ID is differerent from the latest accessed.
	 * \Note The publications are not kept in store, if the subscribers don't pick them up, they are lost.
     *       Only the atest publication is available
     */
	bool HasNews(SubscriberID inSubscriber);
	
    /** \brief Locks the publisher's mutex for accessing the news.
	  * and marks the current publication ID as consumed for the given subscriber
	  * \return Returns true if successfully locked.  Otherwise returns false.
	  * \note  (Lock may fail if subscriber ID is invalid).
      */
	bool LockNews(SubscriberID inSubscriber);

    /** \brief Locks the publisher's mutex for accessing the news.
	 * and marks the provided publication ID as the last consumed for the given subscriber
	 * \return Returns true if successfully locked.  Otherwise returns false.
	 * \note Lock may fail if subscriber ID is invalid.
     */
	bool LockNews(SubscriberID inSubscriber, IssueID inIssueID);

    /** \brief Unlocks the publisher's mutex for accessing the news. */
	void UnlockNews(SubscriberID inSubscriber);

    /** \brief Get the current publication ID for this subscriber */
	IssueID GetCurrentIssueID(SubscriberID inSubscriber);

    /** \brief Get the publication ID of the latest issue that was Locked() */
	IssueID GetConsumedIssueID(SubscriberID inSubscriber);

protected:
    /** \brief Increments the publications number for all subscribers */
	void PutNews();

    /** \brief Sets the latest publication number for all subscribers */
	void PutNews(IssueID newPublication);

    /** \brief returns Mutex to guyaratee thread-safe access to the publication */
	boost::mutex &GetMutex() {return(mMutex);};

    /** \brief For each subscriber, the vector contains the issue number of latest news.
	 *  \note: There is no guarantee from the publisher that the IssueIDs will be consecutive.
	 *       The only guarantee is that consecutive publications will not have the same number for a reasonable amoount of time.
	 *       (For example, there may be wraparound of the IssueIDs)
     */
	boost::container::vector<IssueID> currentPublications;

    /** \brief For each subscriber, the vector contains the issue number last Locked() news. */
	boost::container::vector<IssueID> consumedPublications;

    /** \brief mutex used for thread-safe access to the news */
   boost::mutex mMutex;

}; // Publisher

SENSORCORE_END_NAMESPACE
#endif  // SENSORCORE_PUBLISHER_H
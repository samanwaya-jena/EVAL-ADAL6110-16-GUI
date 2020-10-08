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

#ifndef SENSORCORE_SUBSCRIPTION_H
#define SENSORCORE_SUBSCRIPTION_H

#include <vector> 

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/container/vector.hpp>
#include <boost/thread/thread.hpp>
#endif
#include "SensorCoreClassesGlobal.h"





SENSORCORE_BEGIN_NAMESPACE

/** \brief Simple threaded subscription management class.
  * Just helps determine when an object has an updated "publication" in store for
  * a given subscriber.
  * A subscriber just
  * \author Jean-Yves Deschênes
  */
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
SENSORCORE_END_NAMESPACE
#endif
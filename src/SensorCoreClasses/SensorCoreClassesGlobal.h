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

#ifndef SENSORCORE_SENSORCLASSESESGLOBAL_H
#define SENSORCORE_SENSORCLASSESESGLOBAL_H


#ifdef __cplusplus


#if !defined(SENSORCORE_NAMESPACE) /* user namespace */

# define SENSORCORE_USE_NAMESPACE
# define SENSORCORE_BEGIN_NAMESPACE
# define SENSORCORE_END_NAMESPACE
# define SENSORCORE_NAMESPACE_PREFIX 
# define SensorCoreScope

#else /* user namespace */

# define SENSORCORE_USE_NAMESPACE using namespace ::SENSORCORE_NAMESPACE;
# define SENSORCORE_BEGIN_NAMESPACE namespace SENSORCORE_NAMESPACE {
# define SENSORCORE_END_NAMESPACE }
# define SENSORCORE_NAMESPACE_PREFIX SENSORCORE_NAMESPACE 
# define SensorCoreScope SENSORCORE_NAMESPACE

namespace SENSORCORE_NAMESPACE {}

#endif /* user namespace */

#else /* __cplusplus */

# define SENSORCORE_USE_NAMESPACE
# define SENSORCORE_BEGIN_NAMESPACE
# define SENSORCORE_END_NAMESPACE
	
# define SENSORCORE_BEGIN_INCLUDE_NAMESPACE
# define SENSORCORE_END_INCLUDE_NAMESPACE
# define SENSORCORE_NAMESPACE_PREFIX 
# define SensorCoreScope

#endif /* __cplusplus */

#endif //SENSORCORE_SENSORCLASSESESGLOBAL_H

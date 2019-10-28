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

#ifndef _DEBUGPRINTF_H
#define _DEBUGPRINTF_H

#include <iostream>
#include <fstream>
#include "SensorCoreClassesGlobal.h"

SENSORCORE_BEGIN_NAMESPACE


/** \brief Appends formatted information to the default debug stream ("debug.dbg").
  *        The debug uses the same syntax as Printf.
  *        Internally uses vsprintf();
  *		   This version of the DebugFilePrintf opens and closes the file at
  *		   each call.  Buffers are flushed at every call, making robust in 
  *        case of a crash, at the cost of some efficiency. 
  * \param[in] debugFile the steam to print the data to.
  * \param[in] format  format string.
  * \param[in] ...	varialble argument string, similar to printf.
  * \note Debugs only when compiled in debug mode (that is, when _DEBUG is defined)
  *       Otherwise, the call is empty.
  */
void DebugFilePrintf(const char *format, ...);


/** \brief Appends debug information to the caller specified stream.
  *        The debug uses the same syntax as Printf.
  *        Internally uses vsprintf();
  *        Difference with the DebugPrintf without file argument is that the output
  *        debug file is assumed open all the time.
  *        This makes for faster I/O, but is susceptible to loss of buffer in case of a crash.
  * \param[in] debugFile the steam to print the data to.
  * \param[in] format  format string.
  * \param[in] ...	varialble argument string, similar to printf.
  * \returns true if the file is sucessfully opened. false otherwise.
  * \note Debugs only when compiled in debug mode (that is, when _DEBUG is defined)
  *       Otherwise, the call is empty.
  */
void DebugFilePrintf(std::ofstream &debugFile, const char *format, ...);


/** \brief Opens a debug file specified by name.
  *			if (bAppend) is equal to true, the file is opened in "append mode", 
  *        otherwise, the file is flushed.
  *        If the stream is already opened, it is closed, then reopened.
  * \param[in,out] debugFile the stream to open.
  * \param[in] format  format string.
  * \param[in] bAppend  set to true to open the file in append mode.  
  *                     Otherwise, set to false (default).
  */

bool OpenDebugFile(std::ofstream &debugFile, const char *fileName, bool bAppend = false);

/** \brief Closes a previously opened debug file.
  * \param[in,out] debugFile the stream to close.
  */

bool CloseDebugFile(std::ofstream &debugFile);



/** \brief Appends formatted information to the default debug stream ("debug.dbg").
  *        The debug uses the same syntax as Printf.
  *        Internally uses vsprintf();
  *		   This version of the DebugFilePrintf opens and closes the file at
  *		   each call.  Buffers are flushed at every call, making robust in 
  *        case of a crash, at the cost of some efficiency. 
  * \param[in] debugFile the steam to print the data to.
  * \param[in] format  format string.
  * \param[in] ...	varialble argument string, similar to printf.
  * \note Debugs only when compiled in debug mode (that is, when _DEBUG is defined)
  *       Otherwise, the call is empty.
  */
void DebugFilePrintf(const char *format, ...);


/** \brief Appends log information to the caller specified stream.
  *        The log uses the same syntax as Printf.
  *        Internally uses vsprintf();
  *        Output log file is assumed open all the time.
  *        This makes for faster I/O, but is susceptible to loss of buffer in case of a crash.
  * \param[in] debugFile the steam to print the data to.
  * \param[in] format  format string.
  * \param[in] ...	varialble argument string, similar to printf.
  * \returns true if the file is sucessfully opened. false otherwise.
  * \note  Output log file is assumed open all the time. Must be opened using OpenLogFile();
  *        This makes for faster I/O, but is susceptible to loss of buffer in case of a crash.
  */
void LogFilePrintf(std::ofstream &logFile, const char *format, ...);


/** \brief Opens a log file specified by name.
  *			if (bAppend) is equal to true, the file is opened in "append mode", 
  *        otherwise, the file is flushed.
  *        If the stream is already opened, it is closed, then reopened.
  * \param[in,out] logFile the stream to open.
  * \param[in] format  format string.
  * \param[in] bAppend  set to true to open the file in append mode.  
  *                     Otherwise, set to false (default).
  */

bool OpenLogFile(std::ofstream &logFile, const char *fileName, bool bAppend = false);

/** \brief Closes a previously opened log file.
  * \param[in,out] logFile the stream to close.
  */

bool CloseLogFile(std::ofstream &logFile);

/** \brief Changes the default path for the log and debug files.
* \param[in] new path.
* \note By default the log and debug files are set to the application directory.
*/

bool SetLogAndDebugFilePath(std::string newFilePath);

/** \brief Returns the current path for the log and debug files.
* \note By default the log and debug files are set to the application directory.
*/
std::string GetLogAndDebugFilePath();


/** \brief Changes the default fileName for the log file.
* \param[in] new fileName.
*/

bool SetLogFileName(std::string newFileName);

/** \brief Returns the current fileName for the log file.
*/
std::string GetLogFileName();

SENSORCORE_END_NAMESPACE          

#endif
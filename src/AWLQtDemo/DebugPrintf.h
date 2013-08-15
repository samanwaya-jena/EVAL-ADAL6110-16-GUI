#ifndef _DEBUGPRINTF_H
#define _DEBUGPRINTF_H

#include <iostream>
#include <fstream>

using namespace std;

namespace awl
{

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
void DebugFilePrintf(ofstream &debugFile, const char *format, ...);


/** \brief Opens a debug file specified by name.
  *			if (bAppend) is equal to true, the file is opened in "append mode", 
  *        otherwise, the file is flushed.
  *        If the stream is already opened, it is closed, then reopened.
  * \param[in,out] debugFile the stream to open.
  * \param[in] format  format string.
  * \param[in] bAppend  set to true to open the file in append mode.  
  *                     Otherwise, set to false (default).
  */

bool OpenDebugFile(ofstream &debugFile, const char *fileName, bool bAppend = false);

/** \brief Closes a previously opened debug file.
  * \param[in,out] debugFile the stream to close.
  */

bool CloseDebugFile(ofstream &debugFile);


} // namespace AWL          

#endif
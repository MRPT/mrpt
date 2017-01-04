/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARLOG_H
#define ARLOG_H

#ifndef WIN32
#include <stdio.h>
#endif
#include <string>
#include "ariaTypedefs.h"
#include "ArMutex.h"
#include "ArFunctor.h"

class ArConfig;

/// Logging utility class
/**
   ArLog is a utility class to log all messages from Aria to a choosen
   destintation. Messages can be logged to stdout, stderr, a file, and
   turned off completely. Logging by default is set to stdout. The level
   of logging can be changed as well. Allowed levels are Terse, Normal,
   and Verbose. By default the level is set to Normal.
*/
class ArLog
{
public:

  typedef enum {
    StdOut, ///< Use stdout for logging
    StdErr, ///< Use stderr for logging
    File, ///< Use a file for logging
    Colbert, ///< Use a Colbert stream for logging
    None ///< Disable logging
  } LogType;
  typedef enum {
    Terse, ///< Use terse logging
    Normal, ///< Use normal logging
    Verbose ///< Use verbose logging
  } LogLevel;

#ifndef SWIG
  /** @brief Log a message, with formatting and variable number of arguments
   *  @swignote In Java and Python, this function only takes one 
   *    string argument. Use Java or Python's native facities
   *    for constructing a formatted string, e.g. the % and + string
   *    operators in Python, and the methods of the Java String class.
   */
  AREXPORT static void log(LogLevel level, const char *str, ...);
#endif
  /// Log a message containing just a plain string
  AREXPORT static void logPlain(LogLevel level, const char *str);
  /// Initialize the logging utility with options
  AREXPORT static bool init(LogType type, LogLevel level,
			    const char *fileName="",
			    bool logTime = false, bool alsoPrint = false, 
			    bool printThisCall = true);
  /// Close the logging utility
  AREXPORT static void close();

#ifndef SWIG // this is internal we don't need to wrap it
  // Do not use this unless you know what you are doing...
  /** @internal
   * @swigomit */
  AREXPORT static void logNoLock(LogLevel level, const char *str, ...);
#endif 
  // We use this to print to a Colbert stream, if available
  AREXPORT static void (* colbertPrint)(int i, const char *str);

  /// Use an ArConfig object to control ArLog's options
  AREXPORT static void addToConfig(ArConfig *config);
protected:
  AREXPORT static bool processFile(void);

  static ArLog *ourLog;
  static ArMutex ourMutex;
  static LogType ourType;
  static LogLevel ourLevel;
  static bool ourLoggingTime;
  static FILE *ourFP;
  static int ourColbertStream;
  static std::string ourFileName;
  static bool ourAlsoPrint;
  
  static LogType ourConfigLogType;
  static LogLevel ourConfigLogLevel;
  static char ourConfigFileName[1024];
  static bool ourConfigLogTime;
  static bool ourConfigAlsoPrint;
  static ArGlobalRetFunctor<bool> ourConfigProcessFileCB;
};


#endif

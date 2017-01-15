/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArLog.h"
#include "ArConfig.h"
#include <time.h>
#include <stdarg.h>
#include <ctype.h>


/* VS Express 2005 doesn't have this, and it's only used for debug logging which I've never
   really seen work.   Maybe we could check  _MSC_VER and/or _ATL_VER?
#if defined(_DEBUG) && defined(ARIA_MSVC)
#include <atlbase.h>
#endif
*/

ArMutex ArLog::ourMutex;
ArLog::LogType ArLog::ourType=StdOut;
ArLog::LogLevel ArLog::ourLevel=ArLog::Normal;
FILE * ArLog::ourFP=0;
std::string ArLog::ourFileName;
int ArLog::ourColbertStream = -1;
bool ArLog::ourLoggingTime = false;
bool ArLog::ourAlsoPrint = false;
AREXPORT void (* ArLog::colbertPrint)(int i, const char *str);

ArLog::LogType ArLog::ourConfigLogType = ArLog::StdOut;
ArLog::LogLevel ArLog::ourConfigLogLevel = ArLog::Normal;
char ArLog::ourConfigFileName[1024] = "log.txt";
bool ArLog::ourConfigLogTime = false;
bool ArLog::ourConfigAlsoPrint = false;
ArGlobalRetFunctor<bool> ArLog::ourConfigProcessFileCB(&ArLog::processFile);

AREXPORT void ArLog::logPlain(LogLevel level, const char *str)
{
  log(level, str);
}

/**
   This function is used like printf(). If the supplied level is less than
   or equal to the set level, it will be printed.
   @param level level of logging
   @param str printf() like formating string
*/
AREXPORT void ArLog::log(LogLevel level, const char *str, ...)
{
  char buf[2048];
  char *bufPtr;
  char *timeStr;
  int timeLen = 20; // this is a value based on the standard length of
                       // ctime return
  time_t now;


  if (level <= ourLevel)
  {
    ourMutex.lock();
    // put our time in if we want it
    if (ourLoggingTime)
    {
      now = time(NULL);
      timeStr = ctime(&now);
      // get take just the portion of the time we want
      strncpy(buf, timeStr, timeLen);
      buf[timeLen] = '\0';
      bufPtr = &buf[timeLen];
    }
    else
      bufPtr = buf;
    va_list ptr;
    va_start(ptr, str);
    vsprintf(bufPtr, str, ptr);
    // can do whatever you want with the buf now
    if (ourType == Colbert)
    {
      if (colbertPrint)		// check if we have a print routine
	(*colbertPrint)(ourColbertStream, buf);
    }
    else if (ourFP)
    {
      fprintf(ourFP, "%s\n", buf);
      fflush(ourFP);
    }
    else if (ourType != None)
    {
      printf("%s\n", buf);
      fflush(stdout);
    }
    if (ourAlsoPrint)
      printf("%s\n", buf);

	/* rh: this does not build in VS Express 2005, and I don't think I've ever seen it actually 
	       do anything, so removing it for now.
	       We could also check _ATL_VER and/or _MSC_VER preprocessor symbols.
// Also send it to the VC++ debug output window...
#if defined(_DEBUG) && defined(_WIN32)
ATLTRACE2("%s\n", buf);
#endif
    */

    va_end(ptr);
	ourMutex.unlock();
  }

}

/**
   Initialize the logging utility by supplying the type of logging and the
   level of logging. If the type is File, the fileName needs to be supplied.
   @param type type of Logging
   @param level level of logging
   @param fileName the name of the file for File type of logging
   @param logTime if this is true then the time a message is given will be logged
   @param alsoPrint if this is true then in addition to whatever other logging (to a file for instance) the results will also be printed
   @param printThisCall if this is true the new settings will be printed otherwise they won't
*/
AREXPORT bool ArLog::init(LogType type, LogLevel level, const char *fileName,
			  bool logTime, bool alsoPrint, bool printThisCall)
{
  ourMutex.lock();
  
  // if we weren't or won't be doing a file then close any old file
  if (ourType != File || type != File)
  {
    close();
  }
  
  if (type == StdOut)
    ourFP=stdout;
  else if (type == StdErr)
    ourFP=stderr;
  else if (type == File)
  {
    if (fileName != NULL)
    {
      if (strcmp(ourFileName.c_str(), fileName) == 0)
      {
	ArLog::logNoLock(ArLog::Terse, "ArLog::init: Continuing to log to the same file.");
      }
      else
      {
	close();
	if ((ourFP = fopen(fileName, "w")) == NULL)
	{
	  ArLog::logNoLock(ArLog::Terse, "ArLog::init: Could not open file %s for logging.", fileName);
	  ourMutex.unlock();
	  return(false);
	}
	ourFileName=fileName;
      }
    }
  }
  else if (type == Colbert)
  {
    colbertPrint = NULL;
    ourColbertStream = -1;	// use default stream
    if (fileName)
    {  // need to translate fileName to integer index
    }
  }
  else if (type == None)
  {

  }
  ourType=type;
  ourLevel=level;

  // environment variable overrides level
  {
    char* lev = getenv("ARLOG_LEVEL");
    if(lev)
    {
      switch(toupper(lev[0]))
      {
        case 'N':
          ourLevel = Normal;
          break;
        case 'T':
          ourLevel = Terse;
          break;
        case 'V':
          ourLevel = Verbose;
          break;
       }
    }
  }

  ourLoggingTime = logTime;
  ourAlsoPrint = alsoPrint;

  if (printThisCall)
  {
    printf("ArLog::init: ");
    
    if (ourType == StdOut)
      printf(" StdOut\t");
    else if (ourType == StdErr)
      printf(" StdErr\t");
    else if (ourType == File)
      printf(" File(%s)\t", ourFileName.c_str());
    else if (ourType == Colbert)
      printf(" Colbert\t");
    else if (ourType == None)
      printf(" None\t");
    else
      printf(" BadType\t");
    
    if (ourLevel == Terse)
      printf(" Terse\t");
    else if (ourLevel == Normal)
      printf(" Normal\t");
    else if (ourLevel == Verbose)
      printf(" Verbose\t");
    else
      printf(" BadLevel\t");
    
    if (ourLoggingTime)
      printf(" Logging Time\t");
    else
      printf(" Not logging time\t");
    
    if (ourAlsoPrint)
      printf(" Also printing\n");
    else
      printf(" Not also printing\n");
  }
  ourMutex.unlock();
  return(true);
}

AREXPORT void ArLog::close()
{
  if (ourFP && (ourType == File))
  {
    fclose(ourFP);
    ourFP=0;
    ourFileName="";
  }
}

AREXPORT void ArLog::logNoLock(LogLevel level, const char *str, ...)
{
  char buf[2048];
  char *bufPtr;
  char *timeStr;
  int timeLen = 20; // this is a value based on the standard length of
                       // ctime return
  time_t now;

  if (level <= ourLevel)
  {
    // put our time in if we want it
    if (ourLoggingTime)
    {
      now = time(NULL);
      timeStr = ctime(&now);
      // get take just the portion of the time we want
      strncpy(buf, timeStr, timeLen);
      buf[timeLen] = '\0';
      bufPtr = &buf[timeLen];
    }
    else
      bufPtr = buf;
    va_list ptr;
    va_start(ptr, str);
    vsprintf(bufPtr, str, ptr);
    // can do whatever you want with the buf now
    if (ourType == Colbert)
    {
      if (colbertPrint)		// check if we have a print routine
	(*colbertPrint)(ourColbertStream, buf);
    }
    else if (ourFP)
      fprintf(ourFP, "%s\n", buf);
    else if (ourType != None)
      printf("%s\n", buf);
    if (ourAlsoPrint)
      printf("%s\n", buf);
    va_end(ptr);
  }
}

AREXPORT void ArLog::addToConfig(ArConfig *config)
{
  std::string section = "LogConfig";
  config->addParam(
	  ArConfigArg("LogType", (int *)&ourConfigLogType,
		      "The type of log we'll be using, 0 for StdOut, 1 for StdErr, 2 for File (and give it a file name), 3 for colbert (don't use that), and 4 for None", 
		      ArLog::StdOut, ArLog::None), 
	  section.c_str(), ArPriority::TRIVIAL);
  config->addParam(
	  ArConfigArg("LogLevel", (int *)&ourConfigLogLevel,
		      "The level of logging to do, 0 for Terse, 1 for Normal, and 2 for Verbose", 
		      ArLog::Terse, ArLog::Verbose), 
	  section.c_str(), ArPriority::TRIVIAL);
  config->addParam(
	  ArConfigArg("LogFileName", ourConfigFileName,
		      "File to log to", sizeof(ourConfigFileName)),
	  section.c_str(), ArPriority::TRIVIAL);
  config->addParam(
	  ArConfigArg("LogTime", &ourConfigLogTime,
		      "True to prefix log messages with time and date, false not to"),
	  section.c_str(), ArPriority::TRIVIAL);
  config->addParam(
	  ArConfigArg("LogAlsoPrint", &ourConfigAlsoPrint,
		      "True to also printf the message, false not to"),	  
	  section.c_str(), ArPriority::TRIVIAL);
  ourConfigProcessFileCB.setName("ArLog");
  config->addProcessFileCB(&ourConfigProcessFileCB, 200);
}

AREXPORT bool ArLog::processFile(void)
{
  if (ourConfigLogType != ourType || ourConfigLogLevel != ourLevel ||
      strcmp(ourConfigFileName, ourFileName.c_str()) != 0 || 
      ourConfigLogTime != ourLoggingTime || ourConfigAlsoPrint != ourAlsoPrint)
  {
    ArLog::log(ArLog::Normal, "Initializing log from config");
    return ArLog::init(ourConfigLogType, ourConfigLogLevel, ourConfigFileName, 
		       ourConfigLogTime, ourConfigAlsoPrint, true);
  }
  return true;
}

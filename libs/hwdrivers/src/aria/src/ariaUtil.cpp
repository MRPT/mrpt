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
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <ctype.h>
#ifndef WIN32
#include <sys/time.h>
#include <stdarg.h>
#include <unistd.h>
#endif

#include "ariaInternal.h"
#include "ariaTypedefs.h"
#include "ariaUtil.h"

#ifdef WIN32
AREXPORT const char *ArUtil::COM1 = "COM1";
AREXPORT const char *ArUtil::COM2 = "COM2";
AREXPORT const char *ArUtil::COM3 = "COM3";
AREXPORT const char *ArUtil::COM4 = "COM4";
#else // ifndef WIN32
const char *ArUtil::COM1 = "/dev/ttyS0";
const char *ArUtil::COM2 = "/dev/ttyS1";
const char *ArUtil::COM3 = "/dev/ttyS2";
const char *ArUtil::COM4 = "/dev/ttyS3";
#endif  // WIN32

AREXPORT const char *ArUtil::TRUESTRING = "true";
AREXPORT const char *ArUtil::FALSESTRING = "false";

/**
   This sleeps for the given number of milliseconds... Note in linux it
   tries to sleep for 10 ms less than the amount given, which should wind up
   close to correct...
   Linux is broken in this regard and sleeps for too long...
   it sleeps for the ceiling of the current 10 ms range,
   then for an additional 10 ms... so:
   11 to 20 ms sleeps for 30 ms...
   21 to 30 ms sleeps for 40 ms...
   31 to 40 ms sleeps for 50 ms...
   this continues on up to the values we care about of..
   81 to 90 ms sleeps for 100 ms...
   91 to 100 ms sleeps for 110 ms...
   so we'll sleep for 10 ms less than we want to, which should put us about
   right... guh
   @param ms the number of milliseconds to sleep for
*/
AREXPORT void ArUtil::sleep(unsigned int ms)
{
#ifdef WIN32
  Sleep(ms);
#endif // WIN32
#ifdef linux
  if (ms > 10)
    ms -= 10;
  usleep(ms * 1000);
#endif // linux

}

/**
   Get the time in milliseconds, counting from some arbitrary point.
   This time is only valid within this run of the program.
   @return millisecond time
*/
AREXPORT unsigned int ArUtil::getTime(void)
{
#ifdef WIN32
  return timeGetTime();
#endif // WIN32

#ifndef WIN32
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_usec/1000 + (tv.tv_sec % 1000000)*1000;
#endif
}

/*
   Takes a string and splits it into a list of words. It appends the words
   to the outList. If there is nothing found, it will not touch the outList.
   @param inString the input string to split
   @param outList the list in which to store the words that are found
*/
/*
AREXPORT void ArUtil::splitString(std::string inString,
				  std::list<std::string> &outList)
{
  const char *start, *end;

  // Strip off leading white space
  for (end=inString.c_str(); *end && isspace(*end); ++end)
    ;
  while (*end)
  {
    // Mark start of the word then find end
    for (start=end; *end && !isspace(*end); ++end)
      ;
    // Store the word
    if (*start && ((*end && isspace(*end)) || !*end))
      outList.push_back(std::string(start, (int)(end-start)));
    for (; *end && isspace(*end); ++end)
      ;
  }
}
*/
#ifdef WIN32

/**
   @return size in bytes. -1 on error.
   @param fileName name of the file to size
*/
AREXPORT long ArUtil::sizeFile(std::string fileName)
{
  struct _stat buf;

  if (_stat(fileName.c_str(), &buf) < 0)
    return(-1);

  if (!(buf.st_mode | _S_IFREG))
    return(-1);

  return(buf.st_size);
}

/**
   @return size in bytes. -1 on error.
   @param fileName name of the file to size
*/
AREXPORT long ArUtil::sizeFile(const char * fileName)
{
  struct _stat buf;

  if (_stat(fileName, &buf) < 0)
    return(-1);

  if (!(buf.st_mode | _S_IFREG))
    return(-1);

  return(buf.st_size);
}

#else // WIN32

AREXPORT long ArUtil::sizeFile(std::string fileName)
{
  struct stat buf;

  if (stat(fileName.c_str(), &buf) < 0)
  {
    perror("stat");
    return(-1);
  }

  if (!S_ISREG(buf.st_mode))
    return(-1);

  return(buf.st_size);
}


/**
   @return size in bytes. -1 on error.
   @param fileName name of the file to size
*/
AREXPORT long ArUtil::sizeFile(const char * fileName)
{
  struct stat buf;

  if (stat(fileName, &buf) < 0)
  {
    perror("stat");
    return(-1);
  }

  if (!S_ISREG(buf.st_mode))
    return(-1);

  return(buf.st_size);
}

#endif // WIN32

/**
   @return true if file is found
   @param fileName name of the file to size
*/
AREXPORT bool ArUtil::findFile(const char *fileName)
{
  FILE *fp;

  if ((fp=fopen(fileName, "r")))
  {
    fclose(fp);
    return(true);
  }
  else
    return(false);
}

/*
   Works for \ and /. Returns true if something was actualy done. Sets
   fileOut to be what ever the answer is.
   @return true if the path contains a file
   @param fileIn input path/fileName
   @param fileOut output fileName
*/
/*AREXPORT bool ArUtil::stripDir(std::string fileIn, std::string &fileOut)
{
  const char *ptr;

  for (ptr=fileIn.c_str(); *ptr; ++ptr)
    ;
  for (--ptr; (ptr > fileIn.c_str()) && (*ptr != '/') && (*ptr != '\\'); --ptr)
    ;
  if ((*ptr == '/') || (*ptr == '\\'))
  {
    fileOut=ptr+1;
    return(true);
  }
  else
  {
    fileOut=fileIn;
    return(false);
  }
}
*/
/*
   Works for \ and /. Returns true if something was actualy done. Sets
   fileOut to be what ever the answer is.
   @return true if the file contains a path
   @param fileIn input path/fileName
   @param fileOut output path
*/
/*
AREXPORT bool ArUtil::stripFile(std::string fileIn, std::string &fileOut)
{
  const char *start, *end;

  for (start=end=fileIn.c_str(); *end; ++end)
  {
    if ((*end == '/') || (*end == '\\'))
    {
      start=end;
      for (; *end && ((*end == '/') || (*end == '\\')); ++end)
        ;
    }
  }

  if (start < end)
  {
    fileOut.assign(fileIn, 0, start-fileIn.c_str());
    return(true);
  }

  fileOut=fileIn;
  return(false);
}
*/
AREXPORT bool ArUtil::stripQuotes(char *dest, const char *src, size_t destLen)
{
  size_t srcLen = strlen(src);
  if (destLen < srcLen + 1)
  {
    ArLog::log(ArLog::Normal, "ArUtil::stripQuotes: destLen isn't long enough to fit copy its %d should be %d", destLen, srcLen + 1);
    return false;
  }
  // if there are no quotes to strip just copy and return
  if (srcLen < 2 ||
      (src[0] != '"' || src[srcLen - 1] != '"'))
  {
    strcpy(dest, src);
    return true;
  }
  // we have quotes so chop of the first and last char
  strncpy(dest, &src[1], srcLen - 1);
  dest[srcLen - 2] = '\0';
  return true;
}

/** Append a directory separator character to the given path string, depending on the
 * platform.  On Windows, a backslash ('\\') is added. On other platforms, a
 * forward slash ('/') is appended. If there is no more allocated space in the
 * path string, no character will be appended.
   @param path the path string to append a slash to
   @param pathLength maximum length allocated for path string
*/
AREXPORT void ArUtil::appendSlash(char *path, size_t pathLength)
{
  // first check boundary
  size_t len;
  len = strlen(path);
  if (len > pathLength - 2)
    return;

  if (len == 0 || (path[len - 1] != '\\' && path[len - 1] != '/'))
  {
#ifdef WIN32
    path[len] = '\\';
#else
    path[len] = '/';
#endif
    path[len + 1] = '\0';
  }
}

/**
   @param path the path in which to fix the orientation of the slashes
   @param pathLength the maximum length of path
*/
AREXPORT void ArUtil::fixSlashes(char *path, size_t pathLength)
{
#ifdef WIN32
  fixSlashesBackward(path, pathLength);
#else
  fixSlashesForward(path, pathLength);
#endif
}

/**
   @param path the path in which to fix the orientation of the slashes
   @param pathLength how long that path is at max
*/
AREXPORT void ArUtil::fixSlashesBackward(char *path, size_t pathLength)
{
  for (size_t i=0; path[i] != '\0' && i < pathLength; i++)
  {
    if (path[i] == '/')
      path[i]='\\';
  }
}

/**
   @param path the path in which to fix the orientation of the slashes
   @param pathLength how long that path is at max
*/
AREXPORT void ArUtil::fixSlashesForward(char *path, size_t pathLength)
{

  for (size_t i=0; path[i] != '\0' && i < pathLength; i++)
  {
    if (path[i] == '\\')
      path[i]='/';
  }
}

/**
   This function will take the 'baseDir' and put the 'insideDir' after
   it so that it winds up with 'baseDir/insideDir/'.  It will take
   care of slashes, making sure there is one between them and one at
   the end, and the slashes will match what the operating system
   expects.

   @param dest the place to put the result
   @param destLength the length of the place to put the results
   @param baseDir the directory to start with
   @param insideDir the directory to place after the baseDir
**/
AREXPORT void ArUtil::addDirectories(char *dest, size_t destLength,
				     const char *baseDir,
				     const char *insideDir)
{
  // start it off
  strncpy(dest, baseDir, destLength - 1);
  // make sure we have a null term
  dest[destLength - 1] = '\0';
  // toss on that slash
  appendSlash(dest, destLength);
  // put on the inside dir
  strncat(dest, insideDir, destLength - strlen(dest) - 1);
  // now toss on that slash
  appendSlash(dest, destLength);
  // and now fix up all the slashes
  fixSlashes(dest, destLength);
}

/**
    This compares two strings, it returns an integer less than, equal to,
    or greater than zero  if  str  is  found, respectively, to be less than, to
    match, or be greater than str2.
    @param str the string to compare
    @param str2 the second string to compare
    @return an integer less than, equal to, or greater than zero if str is
    found, respectively, to be less than, to match, or be greater than str2.
*/
AREXPORT int ArUtil::strcmp(std::string str, std::string str2)
{
  return ::strcmp(str.c_str(), str2.c_str());
}

/**
    This compares two strings, it returns an integer less than, equal to,
    or greater than zero  if  str  is  found, respectively, to be less than, to
    match, or be greater than str2.
    @param str the string to compare
    @param str2 the second string to compare
    @return an integer less than, equal to, or greater than zero if str is
    found, respectively, to be less than, to match, or be greater than str2.
*/
AREXPORT int ArUtil::strcmp(std::string str, const char *str2)
{
  return ::strcmp(str.c_str(), str2);
}

/**
    This compares two strings, it returns an integer less than, equal to,
    or greater than zero  if  str  is  found, respectively, to be less than, to
    match, or be greater than str2.
    @param str the string to compare
    @param str2 the second string to compare
    @return an integer less than, equal to, or greater than zero if str is
    found, respectively, to be less than, to match, or be greater than str2.
*/
AREXPORT int ArUtil::strcmp(const char *str, std::string str2)
{
  return ::strcmp(str, str2.c_str());
}

/**
    This compares two strings, it returns an integer less than, equal to,
    or greater than zero  if  str  is  found, respectively, to be less than, to
    match, or be greater than str2.
    @param str the string to compare
    @param str2 the second string to compare
    @return an integer less than, equal to, or greater than zero if str is
    found, respectively, to be less than, to match, or be greater than str2.
*/
AREXPORT int ArUtil::strcmp(const char *str, const char *str2)
{
  return ::strcmp(str, str2);
}


/**
    This compares two strings ignoring case, it returns an integer
    less than, equal to, or greater than zero if str is found,
    respectively, to be less than, to match, or be greater than str2.
    @param str the string to compare @param str2 the second string to
    compare @return an integer less than, equal to, or greater than
    zero if str is found, respectively, to be less than, to match, or
    be greater than str2.  */
AREXPORT int ArUtil::strcasecmp(std::string str, std::string str2)
{
  return ::strcasecmp(str.c_str(), str2.c_str());
}

/**
    This compares two strings ignoring case, it returns an integer
    less than, equal to, or greater than zero if str is found,
    respectively, to be less than, to match, or be greater than str2.
    @param str the string to compare @param str2 the second string to
    compare @return an integer less than, equal to, or greater than
    zero if str is found, respectively, to be less than, to match, or
    be greater than str2.  */
AREXPORT int ArUtil::strcasecmp(std::string str, const char *str2)
{
  return ::strcasecmp(str.c_str(), str2);
}

/**
    This compares two strings ignoring case, it returns an integer
    less than, equal to, or greater than zero if str is found,
    respectively, to be less than, to match, or be greater than str2.
    @param str the string to compare @param str2 the second string to
    compare @return an integer less than, equal to, or greater than
    zero if str is found, respectively, to be less than, to match, or
    be greater than str2.  */
AREXPORT int ArUtil::strcasecmp(const char *str, std::string str2)
{
  return ::strcasecmp(str, str2.c_str());
}

/**
    This compares two strings ignoring case, it returns an integer
    less than, equal to, or greater than zero if str is found,
    respectively, to be less than, to match, or be greater than str2.
    @param str the string to compare @param str2 the second string to
    compare @return an integer less than, equal to, or greater than
    zero if str is found, respectively, to be less than, to match, or
    be greater than str2.  */
AREXPORT int ArUtil::strcasecmp(const char *str, const char *str2)
{
  return ::strcasecmp(str, str2);
}

/**
   This copies src into dest but puts a \ before any spaces in src,
   escaping them... its mostly for use with ArArgumentBuilder...
   make sure you have at least maxLen spaces in the arrays that you're passing
   as dest... this allocates no memory
**/
AREXPORT void ArUtil::escapeSpaces(char *dest, const char *src, size_t maxLen)
{
  size_t i, adj, len;

  len = strlen(src);
  // walk it, when we find one toss in the slash and incr adj so the
  // next characters go in the right space
  for (i = 0, adj = 0; i < len && i + adj < maxLen; i++)
  {
    if (src[i] == ' ')
    {
      dest[i+adj] = '\\';
      adj++;
    }
    dest[i+adj] = src[i];
  }
  // make sure its null terminated
  dest[i+adj] = '\0';
}

/**
   This copies src into dest but makes it lower case make sure you
   have at least maxLen arrays that you're passing as dest... this
   allocates no memory
**/
AREXPORT void ArUtil::lower(char *dest, const char *src, size_t maxLen)
{
  size_t i;
  size_t len;

  len = strlen(src);
  for (i = 0; i < len && i < maxLen; i++)
    dest[i] = tolower(src[i]);
  dest[i] = '\0';

}


AREXPORT bool ArUtil::isOnlyAlphaNumeric(const char *str)
{
  unsigned int ui;
  unsigned int len;
  if (str == NULL)
    return true;
  for (ui = 0, len = sizeof(str); ui < len; ui++)
  {
    if (!isalpha(str[ui]) && !isdigit(str[ui]) && str[ui] != '\0')
      return false;
  }
  return true;
}

AREXPORT bool ArUtil::isStrEmpty(const char *str)
{
	if (str == NULL) {
		return true;
	}
	if (str[0] == '\0') {
		return true;
	}
	return false;

} // end method isStrEmpty


AREXPORT const char *ArUtil::convertBool(int val)
{
  if (val)
    return TRUESTRING;
  else
    return FALSESTRING;
}

AREXPORT double ArUtil::atof(const char *nptr)
{
  if (strcasecmp(nptr, "inf") == 0)
    return HUGE_VAL;
  else if (strcasecmp(nptr, "-inf") == 0)
    return -HUGE_VAL;
  else
	return ::atof(nptr);
}

AREXPORT void ArUtil::functorPrintf(ArFunctor1<const char *> *functor,
				    const char *str, ...)
{
  char buf[2048];
  va_list ptr;
  va_start(ptr, str);
  vsprintf(buf, str, ptr);
  functor->invoke(buf);
  va_end(ptr);
}

AREXPORT void ArUtil::writeToFile(const char *str, FILE *file)
{
  fprintf(file, "%s", str);
}


/**
   This function reads a string from a file.
   The file can contain spaces or tabs, but a '\\r'
   or '\\n' will be treated as the end of the string, and the string
   cannot have more characters than the value given by strLen.  This is mostly for internal use
   with Linux to determine the Aria directory from a file in /etc, but
   will work with Linux or Windows.

   @param fileName name of the file in which to look
   @param str the string to copy the file contents into
   @param strLen the maximum allocated length of str
**/
AREXPORT bool ArUtil::getStringFromFile(const char *fileName,
					char *str, size_t strLen)
{
  FILE *strFile;
  unsigned int i;

  if ((strFile = fopen(fileName, "r")) != NULL)
  {
    char * dumm_ret =fgets(str, strLen, strFile);
    if (!dumm_ret) return false;
    for (i = 0; i < strLen; i++)
    {
      if (str[i] == '\r' || str[i] == '\n' || str[i] == '\0')
      {
	str[i] = '\0';
	break;
      }
    }
  }
  else
  {
    str[0] = '\0';
    return false;
  }
  return true;
}

/**
 * Look up the given value under the given key, within the given registry root
 * key.

   @param root the root key to use, one of the REGKEY enum values

   @param key the name of the key to find

   @param value the value name in which to find the string

   @param str where to put the string found, or if it could not be
   found, an empty (length() == 0) string

   @param len the length of the allocated memory in str

   @return true if the string was found, false if it was not found or if there was a problem such as the string not being long enough
 **/

AREXPORT bool ArUtil::getStringFromRegistry(REGKEY root,
						   const char *key,
						   const char *value,
						   char *str,
						   int len)
{
#ifndef WIN32
  return false;
#else // WIN32

  HKEY hkey;
  int err;
  int i;
  unsigned long numKeys;
  unsigned long longestKey;
  unsigned long numValues;
  unsigned long longestValue;
  unsigned long longestDataLength;
  char *valueName;
  unsigned long valueLength;
  unsigned long type;
  char *data;
  unsigned long dataLength;
  HKEY rootKey;


  switch (root)
  {
  case REGKEY_CLASSES_ROOT:
    rootKey = HKEY_CLASSES_ROOT;
    break;
  case REGKEY_CURRENT_CONFIG:
    rootKey = HKEY_CURRENT_CONFIG;
    break;
  case REGKEY_CURRENT_USER:
    rootKey = HKEY_CURRENT_USER;
    break;
  case REGKEY_LOCAL_MACHINE:
    rootKey = HKEY_LOCAL_MACHINE;
    break;
  case REGKEY_USERS:
    rootKey=HKEY_USERS;
    break;
  default:
    ArLog::log(ArLog::Terse,
	       "ArUtil::getStringFromRegistry: Bad root key given.");
    return false;
  }


  if ((err = RegOpenKeyExA(rootKey, key, 0, KEY_READ, &hkey)) == ERROR_SUCCESS)
  {
    //printf("Got a key\n");
    if (RegQueryInfoKey(hkey, NULL, NULL, NULL, &numKeys, &longestKey, NULL,
			&numValues, &longestValue, &longestDataLength, NULL, NULL) == ERROR_SUCCESS)
    {
	/*
      printf("Have %d keys longest is %d, have %d values longest name is %d, longest data is %d\n",
	     numKeys, longestKey, numValues, longestValue, longestDataLength);
	*/
      data = new char[longestDataLength+2];
      valueName = new char[longestValue+2];
      for (i = 0; i < int(numValues); ++i)
      {
	dataLength = longestDataLength+1;
	valueLength = longestValue+1;
	if ((err = RegEnumValueA(hkey, i, valueName, &valueLength, NULL,
				&type, (unsigned char *)data, &dataLength)) == ERROR_SUCCESS)
	{
		//printf("Enumed value %d, name is %s, value is %s\n", i, valueName, data);
	  if (strcmp(value, valueName) == 0)
	  {
	    if (len < int(dataLength))
	    {
	      ArLog::log(ArLog::Terse,"ArUtil::getStringFromRegistry: str passed in not long enough for data.");
	      delete data;
	      delete valueName;
	      return false;
	    }
	    strncpy(str, data, len);
	    delete data;
	    delete valueName;
	    return true;
	  }
	}
	/*
	else
		printf("Couldn't enum value %d cause %d\n",i,  err);
		*/
	    }
      delete data;
      delete valueName;
    }
	/*
    else
      printf("QueryInfoKey failed\n");
	  */
  }
  /*
  else
    printf("No key %d\n", err);
  */
  return false;
#endif
}

AREXPORT ArRunningAverage::ArRunningAverage(size_t numToAverage)
{
  myNumToAverage = numToAverage;
  myTotal = 0;
  myNum = 0;
}

AREXPORT ArRunningAverage::~ArRunningAverage()
{

}

AREXPORT double ArRunningAverage::getAverage(void) const
{
  return myTotal / myNum;
}

AREXPORT void ArRunningAverage::add(double val)
{
  myTotal += val;
  myNum++;
  myVals.push_front(val);
  if (myVals.size() > myNumToAverage || myNum > myNumToAverage)
  {
    myTotal -= myVals.back();
    myNum--;
    myVals.pop_back();
  }
}

AREXPORT void ArRunningAverage::clear(void)
{
  while (myVals.size() > 0)
    myVals.pop_back();
  myNum = 0;
  myTotal = 0;
}

AREXPORT size_t ArRunningAverage::getNumToAverage(void) const
{
  return myNumToAverage;
}

AREXPORT void ArRunningAverage::setNumToAverage(size_t numToAverage)
{
  myNumToAverage = numToAverage;
  while (myVals.size() > myNumToAverage)
  {
    myTotal -= myVals.back();
    myNum--;
    myVals.pop_back();
  }
}

#ifndef WIN32

AREXPORT ArDaemonizer::ArDaemonizer(int *argc, char **argv) :
  myParser(argc, argv),
  myLogOptionsCB(this, &ArDaemonizer::logOptions)
{
  myIsDaemonized = false;
  Aria::addLogOptionsCB(&myLogOptionsCB);
}

AREXPORT ArDaemonizer::~ArDaemonizer()
{

}

AREXPORT bool ArDaemonizer::daemonize(void)
{
  if (myParser.checkArgument("-daemonize") ||
      myParser.checkArgument("-d"))
  {
    return forceDaemonize();
  }
  else
    return true;

}

/**
   This returns true if daemonizing worked, returns false if it
   didn't... the parent process exits here if forking worked.
 **/
AREXPORT bool ArDaemonizer::forceDaemonize(void)
{
    switch (fork())
    {
    case 0: // child process just return
      myIsDaemonized = true;
      fclose(stdout);
      fclose(stderr);
      return true;
    case -1: // error.... fail
      printf("Can't fork");
      ArLog::log(ArLog::Terse, "ArDaemonizer: Can't fork");
      return false;
    default: // parent process
      printf("Daemon started\n");
      exit(0);
    }
}

AREXPORT void ArDaemonizer::logOptions(void) const
{
  ArLog::log(ArLog::Terse, "Options for Daemonizing:");
  ArLog::log(ArLog::Terse, "-daemonize");
  ArLog::log(ArLog::Terse, "-d");
  ArLog::log(ArLog::Terse, "");
}

#endif // WIN32


std::map<ArPriority::Priority, std::string> ArPriority::ourPriorityNames;
std::string ArPriority::ourUnknownPriorityName;
bool ArPriority::ourStringsInited = false;

AREXPORT const char *ArPriority::getPriorityName(Priority priority)
{

  if (!ourStringsInited)
  {
    ourPriorityNames[IMPORTANT] = "Important";
    ourPriorityNames[NORMAL] = "Normal";
    ourPriorityNames[TRIVIAL] = "Detailed";
    ourPriorityNames[DETAILED] = "Detailed";
    ourUnknownPriorityName = "Unknown";
    ourStringsInited = true;
  }
  return ourPriorityNames[priority].c_str();
}

AREXPORT void ArUtil::putCurrentYearInString(char* s, size_t len)
{
  struct tm t;
  ArUtil::localtime(&t);
  snprintf(s, len, "%4d", 1900 + t.tm_year);
  s[len-1] = '\0';
}
AREXPORT void ArUtil::putCurrentMonthInString(char* s, size_t len)
{

  struct tm t;
  ArUtil::localtime(&t);
  snprintf(s, len, "%02d", t.tm_mon);
  s[len-1] = '\0';
}
AREXPORT void ArUtil::putCurrentDayInString(char* s, size_t len)
{
  struct tm t;
  ArUtil::localtime(&t);
  snprintf(s, len, "%02d", t.tm_mday);
  s[len-1] = '\0';
}
AREXPORT void ArUtil::putCurrentHourInString(char* s, size_t len)
{
  struct tm t;
  ArUtil::localtime(&t);
  snprintf(s, len, "%02d", t.tm_hour);
  s[len-1] = '\0';
}
AREXPORT void ArUtil::putCurrentMinuteInString(char* s, size_t len)
{
  struct tm t;
  ArUtil::localtime(&t);
  snprintf(s, len, "%02d", t.tm_min);
  s[len-1] = '\0';
}
AREXPORT void ArUtil::putCurrentSecondInString(char* s, size_t len)
{
  struct tm t;
  ArUtil::localtime(&t);
  snprintf(s, len, "%02d", t.tm_sec);
  s[len-1] = '\0';
}


AREXPORT bool ArUtil::localtime(const time_t *timep, struct tm *result)
{
  // TODO make this threadsafe
#ifdef WIN32
  struct tm *r = ::localtime(timep);
  if(r == NULL) return false;
  *result = *r;
  return true;
#else
  return (::localtime_r(timep, result) != NULL);
#endif
}


/** Call ArUtil::localtime() with the current time obtained by calling
* time(NULL).
*  @return false on error (e.g. invalid input), otherwise true.
*/
AREXPORT bool ArUtil::localtime(struct tm *result)
{
  time_t now = time(NULL);
  return ArUtil::localtime(&now, result);
}

AREXPORT ArCallbackList::ArCallbackList(const char *name,
					ArLog::LogLevel logLevel)
{
  myName = name;
  setLogLevel(logLevel);
}

AREXPORT  ArCallbackList::~ArCallbackList()
{

}

AREXPORT void ArCallbackList::addCallback(
	ArFunctor *functor, int position)
{
  myDataMutex.lock();
  myList.insert(
	  std::pair<int, ArFunctor *>(-position,
				      functor));
  myDataMutex.unlock();
}

AREXPORT void ArCallbackList::remCallback(ArFunctor *functor)
{
  myDataMutex.lock();
  std::map<int, ArFunctor *>::iterator it;

  for (it = myList.begin(); it != myList.end(); it++)
  {
    if ((*it).second == functor)
    {
      myList.erase(it);
      myDataMutex.unlock();
      remCallback(functor);
      return;
    }
  }
  myDataMutex.unlock();
}


AREXPORT void ArCallbackList::setName(const char *name)
{
  myDataMutex.lock();
  myName = name;
  myDataMutex.unlock();
}

AREXPORT void ArCallbackList::setLogLevel(ArLog::LogLevel logLevel)
{
  myDataMutex.lock();
  myLogLevel = logLevel;
  myDataMutex.unlock();
}

AREXPORT void ArCallbackList::invoke(void)
{
  myDataMutex.lock();

  std::map<int, ArFunctor *>::iterator it;
  ArFunctor *functor;

  ArLog::log(myLogLevel, "%s: Starting calls", myName.c_str());

  for (it = myList.begin();
       it != myList.end();
       it++)
  {
    functor = (*it).second;
    if (functor == NULL)
      continue;

    if (functor->getName() != NULL && functor->getName()[0] != '\0')
      ArLog::log(myLogLevel, "%s: Calling functor '%s' at %d",
		 myName.c_str(), functor->getName(), -(*it).first);
    else
      ArLog::log(myLogLevel, "%s: Calling unnamed functor at %d",
		 myName.c_str(), -(*it).first);
    functor->invoke();
  }

  ArLog::log(myLogLevel, "%s: Ended calls", myName.c_str());
  myDataMutex.unlock();
}

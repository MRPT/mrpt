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
#include "ArArgumentBuilder.h"
#include "ArArgumentParser.h"
#include "ArLog.h"
#include "ariaUtil.h"
#include <stdarg.h>

std::list<std::string> ArArgumentParser::ourDefaultArgumentLocs;
std::list<bool> ArArgumentParser::ourDefaultArgumentLocIsFile;
/**
   @param argc pointer to program argument count (e.g. @a argc from main())
   @param argv array of program arguments (e.g. @a arcv from main())
**/
AREXPORT ArArgumentParser::ArArgumentParser(int *argc, char **argv)
{
  myArgc = argc;
  myArgv = argv;
  myUsingBuilder = false;
  myBuilder = NULL;
  myOwnBuilder = false;

  myEmptyArg[0] = '\0';
}

/**
 * @param builder an ArArgumentBuilder object containing arguments
**/
AREXPORT ArArgumentParser::ArArgumentParser(ArArgumentBuilder *builder)
{
  myUsingBuilder = true;
  myBuilder = builder;
  myOwnBuilder = false;
  myEmptyArg[0] = '\0';
}

AREXPORT ArArgumentParser::~ArArgumentParser()
{
  if (myOwnBuilder)
  {
    delete myBuilder;
    myBuilder = NULL;
  }
}

AREXPORT bool ArArgumentParser::checkArgumentVar(const char *argument, ...)
{
  char arg[2048];
  va_list ptr;
  va_start(ptr, argument);
  vsnprintf(arg, sizeof(arg), argument, ptr);
  va_end(ptr);
  return checkArgument(arg);
}


/**
   @param argument the string to check for, if the argument is found
   its pulled from the list of arguments

   @param ... the extra string to feed into the argument for parsing
   (like printf)

   @return true if the argument was found, false otherwise
**/
AREXPORT bool ArArgumentParser::checkArgument(const char *argument)
{
  size_t i;
  std::string extraHyphen;
  extraHyphen = "-";
  extraHyphen += argument;
  for (i = 0; i < getArgc(); i++)
  {
    if (strcasecmp(argument, getArgv()[i]) == 0 ||
	strcasecmp(extraHyphen.c_str(), getArgv()[i]) == 0)
    {
      removeArg(i);
      // MPL took this out so you could add the same arg multiple times
      //checkArgument(argument);
      return true;
    }
  }
  return false;
}

/**
   This is like checkParameterArgument but lets you fail out if the
   argument is there but the parameter for it is not

   @param argument the string to check for, if the argument is found
   its pulled from the list of arguments

   @param dest if the parameter to the argument is found then the dest
   is set to the parameter

   @param wasReallySet the target of this pointer is set to true if
     @a argument was found followed by a valid value,
     and @a dest was set to the value,
     or false if @a argument was not found and @a dest was not changed.

   @param ... the extra string to feed into the argument for
   parsing (like printf)

   @return true if either this argument wasn't there or if the
   argument was there with a valid parameter
**/
AREXPORT bool ArArgumentParser::checkParameterArgumentStringVar(
	bool *wasReallySet, const char **dest, const char *argument, ...)
{
  char arg[2048];
  va_list ptr;
  va_start(ptr, argument);
  vsnprintf(arg, sizeof(arg), argument, ptr);
  va_end(ptr);
  return checkParameterArgumentString(arg, dest, wasReallySet);
}

/**
   This is like checkParameterArgument but lets you fail out if the
   argument is there but the parameter for it is not

   @param argument the string to check for, if the argument is found
   its pulled from the list of arguments

   @param dest if the parameter to the argument is found then the dest
   is set to the parameter

   @param wasReallySet the target of this pointer is set to true if
     @a argument was found followed by a valid value,
     and @a dest was set to the value,
     or false if @a argument was not found and @a dest was not changed.

   @param returnFirst if we should go just take the first argument
   (true) or if we should through the list and pull up the last one
   (default is false, use true if you want to use the same parameter
   multiple times)

   @return true if either this argument wasn't there or if the
   argument was there with a valid parameter
**/
AREXPORT bool ArArgumentParser::checkParameterArgumentString(
	const char *argument, const char **dest, bool *wasReallySet,
	bool returnFirst)
{
  char *param;
  param = checkParameterArgument(argument, returnFirst);

  if (param == NULL)
  {
    if (wasReallySet)
      *wasReallySet = false;
    return true;
  }
  else if (param[0] != '\0')
  {
    *dest = param;
    if (wasReallySet)
      *wasReallySet = true;
    return true;
  }
  else
  {
    ArLog::log(ArLog::Normal, "No argument given to %s", argument);
    return false;
  }
}

/**
   This is like checkParameterArgument but lets you fail out if the
   argument is there but the parameter for it is not

   @param wasReallySet the target of this pointer is set to true if
     @a argument was found followed by a valid value,
     and @a dest was set to the boolean value,
     or false if @a argument was not found and @a dest was not changed.

   @param dest if @a argument is found and is followed by a
    recognized string representation of a boolean value
    ("true", "false", "1", or "0"), then @a dest is set to the appropriate value

   @param argument the argument string to search for. If the argument is found,
     it is removed from the list of arguments

   @param ... if @a argument contains format codes (like printf()), provide
    the values to substitute following @a argument.

   @return false if @a argument was found but was not followed by a recognized
     value (an error), or true if @a argument was not found, or if the
     argument was there with a valid parameter
*/
AREXPORT bool ArArgumentParser::checkParameterArgumentBoolVar(
	bool *wasReallySet, bool *dest, const char *argument, ...)
{
  char arg[2048];
  va_list ptr;
  va_start(ptr, argument);
  vsnprintf(arg, sizeof(arg), argument, ptr);
  va_end(ptr);
  return checkParameterArgumentBool(arg, dest, wasReallySet);
}

/**
   This is like checkParameterArgument but lets you fail out if the
   argument is there but the parameter for it is not

   @param dest if @a argument is found and is followed by a
    recognized string representation of a boolean value
    ("true", "false", "1", or "0"), then @a dest is set to the appropriate value

   @param wasReallySet the target of this pointer is set to true if
     @a argument was found followed by a valid value,
     and @a dest was set to the value,
     or false if @a argument was not found and @a dest was not changed.

   @param returnFirst if we should go just take the first argument
   (true) or if we should through the list and pull up the last one
   (default is false, use true if you want to use the same parameter
   multiple times)

   @param argument the argument string to search for. If the argument is found,
     it is removed from the list of arguments

   @return false if @a argument was found but was not followed by a recognized
     value (an error), or true if @a argument was not found, or if the
     argument was there with a valid parameter
*/
AREXPORT bool ArArgumentParser::checkParameterArgumentBool(char *argument,
							   bool *dest,
							   bool *wasReallySet,
							   bool returnFirst)
{
  char *param;
  param = checkParameterArgument(argument, returnFirst);

  if (param == NULL)
  {
    if (wasReallySet)
      *wasReallySet = false;
    return true;
  }
  else if (param[0] != '\0')
  {
    if (strcasecmp(param, "true") == 0 || strcmp(param, "1") == 0)
    {
      *dest = true;
      if (wasReallySet)
	*wasReallySet = true;
      return true;
    }
    else if (strcasecmp(param, "false") == 0 || strcmp(param, "0") == 0)
    {
      *dest = false;
      if (wasReallySet)
	*wasReallySet = true;
      return true;
    }
    else
    {
      ArLog::log(ArLog::Normal,
		"Argument given to %s was not a bool (true, false, 1, 0) it was the string %s",
		 argument, param);
      return false;
    }

  }
  else
  {
    ArLog::log(ArLog::Normal, "No argument given to %s", argument);
    return false;
  }

}

/**
   This is like checkParameterArgument but lets you fail out if the
   argument is there but the parameter for it is not

   @param argument the string to check for, if the argument is found
   its pulled from the list of arguments

   @param dest if the parameter to the argument is found and is a
   valid integer then dest is set to the integer value

   @param wasReallySet the target of this pointer is set to true if
     @a argument was found followed by a valid value,
     and @a dest was set to the value,
     or false if @a argument was not found and @a dest was not changed.

   @param ... the extra string to feed into the argument for
   parsing (like printf)

   @return true if either this argument wasn't there or if the
   argument was there with a valid parameter
**/
AREXPORT bool ArArgumentParser::checkParameterArgumentIntegerVar(
	bool *wasReallySet, int *dest, const char *argument, ...)
{
  char arg[2048];
  va_list ptr;
  va_start(ptr, argument);
  vsnprintf(arg, sizeof(arg), argument, ptr);
  va_end(ptr);
  return checkParameterArgumentInteger(arg, dest, wasReallySet);
}

/**
   This is like checkParameterArgument but lets you fail out if the
   argument is there but the parameter for it is not

   @param argument the string to check for, if the argument is found
   its pulled from the list of arguments

   @param dest if the parameter to the argument is found and is a
   valid integer then dest is set to the integer value

   @param returnFirst if we should go just take the first argument
   (true) or if we should through the list and pull up the last one
   (default is false, use true if you want to use the same parameter
   multiple times)

   @param wasReallySet the target of this pointer is set to true if
     @a argument was found followed by a valid value,
     and @a dest was set to the value,
     or false if @a argument was not found and @a dest was not changed.

   @return true if either this argument wasn't there or if the
   argument was there with a valid parameter
**/
AREXPORT bool ArArgumentParser::checkParameterArgumentInteger(
	const char *argument, int *dest, bool *wasReallySet, bool returnFirst)
{
  char *param;
  char *endPtr;
  int intVal;

  param = checkParameterArgument(argument, returnFirst);

  if (param == NULL)
  {
    if (wasReallySet)
      *wasReallySet = false;
    return true;
  }
  else if (param[0] != '\0')
  {
    intVal = strtol(param, &endPtr, 10);
    if (endPtr[0] == '\0')
    {
      *dest = intVal;
      if (wasReallySet)
	*wasReallySet = true;
      return true;
    }
    else
    {
      ArLog::log(ArLog::Normal,
		"Argument given to %s was not an integer it was the string %s",
		 argument, param);
      return false;
    }

  }
  else
  {
    ArLog::log(ArLog::Normal, "No argument given to %s", argument);
    return false;
  }

}

/**
   @param argument the argument to check for. if found, it is removed from the list of arguments
   @param dest if the parameter given after the argument is found and is a
   valid float, then the target of this pointer is assigned to the found value.
   @param returnFirst true if only the first instance of the argument given should be used, false if only the last (and all preceding instances discarded).

   @param wasReallySet the target of this pointer is set to true if
     @a argument was found followed by a valid value,
     and @a dest was set to the value,
     or false if @a argument was not found and @a dest was not changed.
   @return false if the argument was given but an error occurred while parsing the float parameter (i.e. no parameter given, or it was not a parsable float).
*/
AREXPORT bool ArArgumentParser::checkParameterArgumentFloat(
	char *argument, float *dest, bool *wasReallySet, bool returnFirst)
{
  char *param = checkParameterArgument(argument, returnFirst);
  if (param == NULL)
  {
    if (wasReallySet) *wasReallySet = false;
    return true;
  }
  else if (param[0] != '\0')
  {
    char *endPtr;
    float floatVal = strtod(param, &endPtr);
    if(endPtr == param)
    {
      ArLog::log(ArLog::Normal, "Argument given with %s was not a valid number", argument);
      return false;
    }
    else
    {
      *dest = floatVal;
      if (wasReallySet) *wasReallySet = true;
      return true;
    }
  }
  else
  {
    ArLog::log(ArLog::Normal, "No argument given with %s", argument);
    return false;
  }
}


/**
   @param argument the string to check for, if the argument is found
   its pulled from the list of arguments

   @param ... the extra string to feed into the argument for
   parsing (like printf)

   @return NULL if the argument wasn't found, the argument after the
   one given if the argument was found, or a string with the first
   char as NULL again if the argument after the one given isn't there
 **/
AREXPORT char *ArArgumentParser::checkParameterArgumentVar(char *argument, ...)
{
  char arg[2048];
  va_list ptr;
  va_start(ptr, argument);
  vsnprintf(arg, sizeof(arg), argument, ptr);
  va_end(ptr);
  return checkParameterArgument(arg);
}
/**
   @param argument the string to check for. If the argument is found,
   then it is removed from the argument list

   @param returnFirst true if we should just take the first matching argument,
   or false if we should iterate through the list and only use the last one
   (default is false, use true if you want to use the same parameter
   multiple times)

   @return NULL if the argument wasn't found; the value given after the
   found argument; or at empty string (with a NULL first character) if
   the argument was found but no value followed the argument flag.
**/
AREXPORT char * ArArgumentParser::checkParameterArgument(const char *argument,
							 bool returnFirst)
{
  char *ret;
  char *retRecursive;
  size_t i;
  std::string extraHyphen;

  extraHyphen = "-";
  extraHyphen += argument;

  for (i = 0; i < getArgc(); i++)
  {
    if (strcasecmp(argument, getArgv()[i]) == 0 ||
	strcasecmp(extraHyphen.c_str(), getArgv()[i]) == 0)
    {
      // see if we have a ret, we don't if the ret would be beyond argc
      if (getArgc() > i+1)
      {
	ret = getArgv()[i+1];
      }
      else
      {
	ret = myEmptyArg;
      }
      // remove our argument
      removeArg(i);
      // if we have a return remove that one too
      if (ret != NULL && ret != myEmptyArg)
	removeArg(i);
      // now see if there are any more, if so return that
      if (returnFirst)
      {
	return ret;
      }
      else if ((retRecursive = checkParameterArgument(argument)) != NULL)
      {
	return retRecursive;
      }
      // otherwise return what we found
      else
      {
	return ret;
      }
    }
  }
  return NULL;
}

void ArArgumentParser::removeArg(size_t which)
{
  if (which >= getArgc())
  {
    ArLog::log(ArLog::Terse, "ArArgumentParser::removeArg: %d is greater than the number of arguments which is %d", which, getArgc());
    return;
  }
  if (myUsingBuilder)
    {
      myBuilder->removeArg(which);
    }
  else
    {
      size_t i;
      for (i = which; i < getArgc() - 1; i++)
	myArgv[i] = myArgv[i+1];
      *myArgc -= 1;
    }
}

AREXPORT size_t ArArgumentParser::getArgc(void) const
{
  if (myUsingBuilder)
    return myBuilder->getArgc();
  else
    return *myArgc;
}

AREXPORT char** ArArgumentParser::getArgv(void) const
{
  if (myUsingBuilder)
    return myBuilder->getArgv();
  else
    return myArgv;
}

AREXPORT const char* ArArgumentParser::getArg(size_t whichArg) const
{
  if (whichArg >= getArgc())
    return NULL;
  else
    return getArgv()[whichArg];
}

AREXPORT void ArArgumentParser::log(void) const
{
  size_t i;
  ArLog::log(ArLog::Terse, "Num arguments: %d", getArgc());
  for (i = 0; i < getArgc(); ++i)
    ArLog::log(ArLog::Terse, "Arg %d: %s", i, getArgv()[i]);
}

AREXPORT void ArArgumentParser::addDefaultArgument(const char *argument, int position)
{
  if (!myUsingBuilder)
    {
      myBuilder = new ArArgumentBuilder;
      myBuilder->addStringsAsIs(*myArgc, myArgv);
      myOwnBuilder = true;
      myUsingBuilder = true;
    }
  myBuilder->addPlain(argument, position);
}

/**
 * Search all locations for argument defaults and parse them.
 * These locations may be environment variables to read argument varues
 * from, or files to read.
 * @sa addDefaultArgumentLocation
 *
 * @note If you use this function your normal argc (passed into main()) will
 * have been modified, and won't reflect reality anymore. You'll have to use
 * getArgc() to get the actual original argument count.  This is a little wierd but is
 * this way so lots of people don't have to change lots of code.
 */
AREXPORT void ArArgumentParser::loadDefaultArguments(int position)
{
  std::list<std::string>::iterator it;
  std::list<bool>::iterator bIt;
  const char *str;
  char *argumentsPtr;
  char arguments[1024];

  if (!myUsingBuilder)
    {
      myBuilder = new ArArgumentBuilder;
      myBuilder->addStringsAsIs(*myArgc, myArgv);
      myOwnBuilder = true;
      myUsingBuilder = true;
    }

  for (it = ourDefaultArgumentLocs.begin(),
        bIt = ourDefaultArgumentLocIsFile.begin();
       it != ourDefaultArgumentLocs.end();
       it++, bIt++)
  {
    str = (*it).c_str();
    // see if its an environmental variable
    if (!(*bIt) && (argumentsPtr = getenv(str)) != NULL)
    {
      ArArgumentBuilder compressed;
      compressed.add(argumentsPtr);
      compressed.compressQuoted(true);
      myBuilder->addStringsAsIs(compressed.getArgc(), compressed.getArgv(),
                              position);
      ArLog::log(ArLog::Normal,
		 "Added arguments from environmental variable '%s'", str);
    }
    // see if we have a file
    else if ((*bIt) &&
	     ArUtil::getStringFromFile(str, arguments, sizeof(arguments)))
    {
      ArArgumentBuilder compressed;
      compressed.add(arguments);
      compressed.compressQuoted(true);
      myBuilder->addStringsAsIs(compressed.getArgc(), compressed.getArgv(),
                              position);
      ArLog::log(ArLog::Normal, "Added arguments from file '%s'",
		 str);
    }
    // the file or env didn't exit
    // this'll return true otherwise it'll return false)
    else
    {
      ArLog::log(ArLog::Verbose,
		 "Could not load from environmental variable or file '%s'",
		 str);
    }
  }
}

/**
   This adds a file to the list of default argument locations.
   @param file Name of the file
 **/
AREXPORT void ArArgumentParser::addDefaultArgumentFile(const char *file)
{
  ourDefaultArgumentLocs.push_back(file);
  ourDefaultArgumentLocIsFile.push_back(true);
}


/**
   This adds an environment variable to the list of default argument
   locations.
   @param env Name of the environment variable
 **/
AREXPORT void ArArgumentParser::addDefaultArgumentEnv(const char *env)
{
  ourDefaultArgumentLocs.push_back(env);
  ourDefaultArgumentLocIsFile.push_back(false);
}

AREXPORT void ArArgumentParser::logDefaultArgumentLocations(void)
{
  std::list<std::string>::iterator it;
  std::list<bool>::iterator bIt;

  ArLog::log(ArLog::Normal,
	     "Default argument files or environmental variables:");
  for (it = ourDefaultArgumentLocs.begin(),
        bIt = ourDefaultArgumentLocIsFile.begin();
       it != ourDefaultArgumentLocs.end();
       it++, bIt++)
  {
    if (*bIt)
      ArLog::log(ArLog::Normal, "%10s%-10s%s", "", "file", (*it).c_str());
    else
      ArLog::log(ArLog::Normal, "%10s%-10s%s", "", "envVar", (*it).c_str());
  }
}
/**
 * Check whether a special "help" flag was given in the arguments, and also
 * print a warning (Using ArLog at Normal log level) if any unparsed arguments were found.
 * The following are the help flags: -help, -h, --help, /?, /h.
 * @return false if a help flag was found or unparsed arguments
 * were found, true otherwise.
 * @param numArgsOkay If you plan on checking for additional
 *  arguments later in the program, you can specify the number of arguments
 *  expected here, which prevents this method from warning about them being unparsed
 *  yet.
 */
AREXPORT bool ArArgumentParser::checkHelpAndWarnUnparsed(
	unsigned int numArgsOkay)
{
  if (checkArgument("-help") || checkArgument("-h") || checkArgument("/?") ||
      checkArgument("/h"))
    return false;

  if (getArgc() <= 1 + numArgsOkay)
    return true;

  size_t i;
  char buf[2048];
  sprintf(buf, "Unhandled arguments to program:");
  for (i = 1 + (int)numArgsOkay; i < getArgc(); i++)
    sprintf(buf, "%s %s", buf, getArg(i));
  ArLog::log(ArLog::Normal, buf);
  ArLog::log(ArLog::Normal,
	   "Program will continue but to see the help listing type '%s -help'",
	     getArg(0));
  return true;
}


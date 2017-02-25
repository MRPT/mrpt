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
#include "ArLog.h"

#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <stdarg.h>
#include <ctype.h>
#include <cmath>
#include <stdio.h>
#include <mrpt/utils/mrpt_macros.h>

/**
   @param argvLen the largest number of arguments we'll parse

   @param extraSpaceChar if not NULL, then this character will also be
    used to break up arguments (in addition to whitespace)
**/
AREXPORT ArArgumentBuilder::ArArgumentBuilder(size_t argvLen,
					      char extraSpaceChar)
{
  myArgc = 0;
  myOrigArgc = 0;
  myArgvLen = argvLen;
  myArgv = new char *[myArgvLen];
  myFirstAdd = true;
  myExtraSpace = extraSpaceChar;
}

AREXPORT ArArgumentBuilder::ArArgumentBuilder(const ArArgumentBuilder & builder)
{
  size_t i;
  myFullString = builder.myFullString;
  myExtraString = builder.myExtraString;
  myArgc = builder.getArgc();
  myArgvLen = builder.getArgvLen();
  myOrigArgc = myArgc;
  myArgv = new char *[myArgvLen];
  for (i = 0; i < myArgc; i++)
    myArgv[i] = strdup(builder.getArg(i));
}

AREXPORT ArArgumentBuilder::~ArArgumentBuilder()
{
  size_t i;
  if (myOrigArgc > 0)
  {
    for (i = 0; i < myOrigArgc; ++i)
      delete[] myArgv[i];
  }
  delete[] myArgv;
}


AREXPORT void ArArgumentBuilder::removeArg(size_t which)
{
  size_t i;
  char *temp;

  if (which > myArgc - 1)
  {
    ArLog::log(ArLog::Terse, "ArArgumentBuilder::removeArg: %d is greater than the number of arguments which is %d", which, myArgc);
    return;
  }

  temp = myArgv[which];
  //delete[] myArgv[which];
  for (i = which; i < myArgc - 1; i++)
    myArgv[i] = myArgv[i+1];
  myArgc -= 1;
  myArgv[i] = temp;
  // delete the one off the end
}

AREXPORT void ArArgumentBuilder::add(const char *str, ...)
{
  char buf[2048];
  va_list ptr;
  va_start(ptr, str);
  vsprintf(buf, str, ptr);
  internalAdd(buf, -1);
  va_end(ptr);
}

/**
   Internal function that adds a string starting at some given space

   @param str the string to add

   @param position the position to add the string at, a position less
   than 0 means to add at the end, if this number is greater than how
   many positions exist then it will also be added at the end
 **/
AREXPORT void ArArgumentBuilder::internalAdd(const char *str, int position)
{
  char buf[2048];
  int i;
  int j;
  size_t k;
  bool findingSpace = true;
  int startNonSpace;
  int len;
  bool addAtEnd;
  //size_t startingArgc = getArgc();

  if (position < 0 || (size_t)position > myArgc)
    addAtEnd = true;
  else
    addAtEnd = false;

  strncpy(buf, str, sizeof(buf));
  len = strlen(buf);

  // can do whatever you want with the buf now
  // first we advance to non-space
  for (i = 0; i < len; ++i)
  {
    if (!isspace(buf[i]) || (myExtraSpace != '\0' && buf[i] == myExtraSpace))
      break;
  }
  // see if we're done
  if (i == len)
  {
    ArLog::log(ArLog::Verbose, "All white space add for argument builder.");
    return;
  }


  // walk through the line until we get to the end of the buffer...
  // we keep track of if we're looking for white space or non-white...
  // if we're looking for white space when we find it we have finished
  // one argument, so we toss that into argv, reset pointers and moveon
  for (startNonSpace = i; ; ++i)
  {
    // take out the slash of escaped spaces
    if (buf[i] == '\\' && i + 1 < len && buf[i + 1] == ' ')
    {
      for (j = i; j < len && j != '\0'; j++)
      {
	buf[j] = buf[j + 1];
      }
      --len;
    }
    // if we're not finding space and we see a non space (or the end),
    // set us into finding space mode and denote where it started
    else if (!findingSpace &&
	     !(i == len || isspace(buf[i]) || buf[i] == '\0' ||
	       (myExtraSpace != '\0' && buf[i] == myExtraSpace)))
    {
      startNonSpace = i;
      findingSpace = true;
    }
    // if we're finding space, and we see it (or the end), we're at
    // the end of one arg, so toss it into the list
    else if (findingSpace &&
	     (i == len || isspace(buf[i]) || buf[i] == '\0' ||
	      (myExtraSpace != '\0' && buf[i] == myExtraSpace)))
    {
      // see if we have room in our argvLen
      if (myArgc + 1 >= myArgvLen)
      {
	ArLog::log(ArLog::Terse, "ArArgumentBuilder::Add: could not add argument since argc (%u) has grown beyond the argv given in the conbufuctor (%u)", myArgc, myArgvLen);
      }
      else
      {
	// if we're adding at the end just put it there, also put it
	// at the end if its too far out
	if (addAtEnd)
	{
	  myArgv[myArgc] = new char[i - startNonSpace + 1];
	  strncpy(myArgv[myArgc], &buf[startNonSpace], i - startNonSpace);
	  myArgv[myArgc][i - startNonSpace] = '\0';
	  // add to our full string
	  // if its not our first add a space (or whatever our space char is)
	  if (!myFirstAdd && myExtraSpace == '\0')
	    myFullString += " ";
	  else if (!myFirstAdd)
	    myFullString += myExtraSpace;

	  myFullString += myArgv[myArgc];
	  myFirstAdd = false;

	  myArgc++;
	  myOrigArgc = myArgc;
	}
        // otherwise stick it where we wanted it if we can or just
	else
	{
	  // first move things down
	  for (k = myArgc + 1; k > (size_t)position; k--)
	  {
	    myArgv[k] = myArgv[k - 1];
	  }
	  myArgc++;
	  myOrigArgc = myArgc;

	  myArgv[position] = new char[i - startNonSpace + 1];
	  strncpy(myArgv[position], &buf[startNonSpace], i - startNonSpace);
	  myArgv[position][i - startNonSpace] = '\0';
	  position++;
	  // now rebuild the full string
	  myFullString = "";
	  for (k = 0; k < myArgc; k++)
	  {
	    myFullString += myArgv[k];
	    myFullString += " ";
	  }
	  myFirstAdd = false;
	}

      }
      findingSpace = false;
    }
    // if we're at the end or its a null, we're at the end of the buf
    if (i == len || buf[i] == '\0')
      break;
  }
}

/**
   @param str the string to add

   @param position the position to add the string at, a position less
   than 0 means to add at the end, if this number is greater than how
   many positions exist then it will also be added at the end
**/
AREXPORT void ArArgumentBuilder::addPlain(const char *str, int position)
{
  internalAdd(str, position);
}

/**
   @param argc how many arguments to add
   @param argv the strings to add
   @param position the position to add the string at, a position less
   than 0 means to add at the end, if this number is greater than how
   many positions exist then it will also be added at the end
**/
AREXPORT void ArArgumentBuilder::addStrings(char **argv, int argc,
					    int position)
{
  addStrings(argc, argv, position);
}

/**
   @param argc how many arguments to add
   @param argv the strings to add
   @param position the position to add the string at, a position less
   than 0 means to add at the end, if this number is greater than how
   many positions exist then it will also be added at the end
**/
AREXPORT void ArArgumentBuilder::addStrings(int argc, char **argv,
					    int position)
{
  int i;
  for (i = 0; i < argc; i++)
    add(argv[i], position + i);
}

/**
   @param argc how many arguments to add
   @param argv the strings to add
   @param position the position to add the string at, a position less
   than 0 means to add at the end, if this number is greater than how
   many positions exist then it will also be added at the end
**/
AREXPORT void ArArgumentBuilder::addStringsAsIs(int argc, char **argv,
						int position)
{
  int i;
  for (i = 0; i < argc; i++)
    internalAddAsIs(argv[i], position + i);
}

/**
   @param str the string to add

   @param position the position to add the string at, a position less
   than 0 means to add at the end, if this number is greater than how
   many positions exist then it will also be added at the end
**/
AREXPORT void ArArgumentBuilder::addPlainAsIs(const char *str, int position)
{
  internalAddAsIs(str, position);
}

AREXPORT void ArArgumentBuilder::internalAddAsIs(const char *str, int position)
{
  MRPT_UNUSED_PARAM(position);
  myArgv[myArgc] = new char[strlen(str) + 1];
  strcpy(myArgv[myArgc], str);
  myArgv[myArgc][strlen(str)] = '\0';

  // add to our full string
  // if its not our first add a space (or whatever our space char is)
  if (!myFirstAdd && myExtraSpace == '\0')
    myFullString += " ";
  else if (!myFirstAdd)
    myFullString += myExtraSpace;

  myFullString += myArgv[myArgc];
  myFirstAdd = false;

  myArgc++;
  myOrigArgc = myArgc;
}

AREXPORT size_t ArArgumentBuilder::getArgc(void) const
{
  return myArgc;
}

AREXPORT char** ArArgumentBuilder::getArgv(void) const
{
  return myArgv;
}

AREXPORT const char *ArArgumentBuilder::getFullString(void) const
{
  return myFullString.c_str();
}

AREXPORT const char *ArArgumentBuilder::getExtraString(void) const
{
  return myExtraString.c_str();
}

AREXPORT void ArArgumentBuilder::setExtraString(const char *str)
{
  myExtraString = str;
}

AREXPORT void ArArgumentBuilder::setFullString(const char *str)
{
  myFullString = str;
}

AREXPORT const char* ArArgumentBuilder::getArg(size_t whichArg) const
{
  if (whichArg >= myArgc)
    return NULL;
  else
    return myArgv[whichArg];
}

AREXPORT void ArArgumentBuilder::log(void) const
{
  size_t i;
  ArLog::log(ArLog::Terse, "Num arguments: %d", myArgc);
  for (i = 0; i < myArgc; ++i)
    ArLog::log(ArLog::Terse, "Arg %d: %s", i, myArgv[i]);
}

AREXPORT bool ArArgumentBuilder::isArgBool(size_t whichArg) const
{
  if (whichArg > myArgc || getArg(whichArg) == NULL)
    return false;

  if (strcasecmp(getArg(whichArg), "true") == 0 ||
      strcasecmp(getArg(whichArg), "1") == 0 ||
      strcasecmp(getArg(whichArg), "false") == 0 ||
      strcasecmp(getArg(whichArg), "0") == 0)
    return true;
  else
    return false;
}

AREXPORT bool ArArgumentBuilder::getArgBool(size_t whichArg) const
{
  if (whichArg > myArgc || getArg(whichArg) == NULL)
    return false;

  if (strcasecmp(getArg(whichArg), "true") == 0 ||
      strcasecmp(getArg(whichArg), "1") == 0)
    return true;
  else
    return false;
}

AREXPORT bool ArArgumentBuilder::isArgInt(size_t whichArg) const
{
  const char *str;
  //int ret;
  char *endPtr;
  if (whichArg > myArgc || getArg(whichArg) == NULL)
    return false;

  str = getArg(whichArg);
  int ret = strtol(str, &endPtr, 10);
  (void)(ret); // Avoid unused var warning
  if (endPtr[0] == '\0' && endPtr != str)
    return true;
  else
    return false;
}

AREXPORT int ArArgumentBuilder::getArgInt(size_t whichArg) const
{
  const char *str;
  int ret;
  char *endPtr;
  if (whichArg > myArgc || getArg(whichArg) == NULL)
    return 0;

  str = getArg(whichArg);
  ret = strtol(str, &endPtr, 10);
  if (endPtr[0] == '\0' && endPtr != str)
    return ret;
  else
    return 0;
}

AREXPORT bool ArArgumentBuilder::isArgDouble(size_t whichArg) const
{
  const char *str;
  //double ret;
  char *endPtr;
  if (whichArg > myArgc || getArg(whichArg) == NULL)
    return false;

  str = getArg(whichArg);
  if (strcmp(str, "-INF") == 0)
  {
	return true;
  }
  else if (strcmp(str, "INF") == 0)
  {
	return true;
  }
  else
  {
    double ret = strtod(str, &endPtr);
    (void)(ret); // Avoid unused var warning
    if (endPtr[0] == '\0' && endPtr != str)
      return true;
    else
      return false;
  }

}

AREXPORT double ArArgumentBuilder::getArgDouble(size_t whichArg) const
{
  const char *str;
  double ret;
  char *endPtr;
  if (whichArg > myArgc || getArg(whichArg) == NULL)
    return 0;

  str = getArg(whichArg);
  if (strcmp(str, "-INF") == 0)
  {
	ret = -HUGE_VAL;
	return ret;
  }
  else if (strcmp(str, "INF") == 0)
  {
	ret = HUGE_VAL;
	return ret;
  }
  else
  {
	ret = strtod(str, &endPtr);
    if (endPtr[0] == '\0' && endPtr != str)
      return ret;
    else
      return 0;
  }
}

AREXPORT void ArArgumentBuilder::compressQuoted(bool stripQuotationMarks)
{
  size_t argLen;
  size_t i;
  std::string myNewArg;

  for (i = 0; i < myArgc; i++)
  {
    argLen = strlen(myArgv[i]);
    if (stripQuotationMarks && argLen >= 2 &&
	myArgv[i][0] == '"' && myArgv[i][argLen - 1] == '"')
    {
      myNewArg = &myArgv[i][1];
      myNewArg[myNewArg.size() - 1] = '\0';
      delete myArgv[i];
      // but replacing ourself with the new arg
      myArgv[i] = strdup(myNewArg.c_str());
      continue;
    }
    // if this arg begins with a quote but doesn't end with one
    if (argLen >= 2 && myArgv[i][0] == '"' && myArgv[i][argLen - 1] != '"')
    {
      // start the new value for this arg, if stripping quotations
      // then start after the quote
      if (stripQuotationMarks)
	myNewArg = &myArgv[i][1];
      else
	myNewArg = myArgv[i];

      bool isEndQuoteFound = false;

      // now while the end char of the next args isn't the end of our
      // start quote we toss things into this arg
      while ((i + 1 < myArgc) && !isEndQuoteFound) {

        int nextArgLen = strlen(myArgv[i+1]);

        // Check whether the next arg contains the ending quote...
        if ((nextArgLen > 0) &&
	    (myArgv[i+1][nextArgLen - 1] == '"'))
	{
	      isEndQuoteFound = true;
        }

        // Concatenate the next arg to this one...
        myNewArg += " ";
        myNewArg += myArgv[i+1];
	// if we are striping quotes off then replace the quote
	if (stripQuotationMarks && myNewArg.size() > 0 && isEndQuoteFound)
	  myNewArg[myNewArg.size() - 1] = '\0';
        // removing those next args
        removeArg(i+1);
        // and ourself
        delete myArgv[i];

        // but replacing ourself with the new arg
        myArgv[i] = strdup(myNewArg.c_str());
      }
    }
  }
}

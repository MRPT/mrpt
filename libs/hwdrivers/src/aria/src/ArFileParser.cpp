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
#include "ArFileParser.h"
#include "ArLog.h"
#include "ariaUtil.h"
#include <ctype.h>

AREXPORT ArFileParser::ArFileParser(const char *baseDirectory)
{
  myRemainderHandler.reset();
  setBaseDirectory(baseDirectory);
  resetCounters();
}

AREXPORT ArFileParser::~ArFileParser(void)
{

}

AREXPORT bool ArFileParser::addHandler(
	const char *keyword, ArRetFunctor1<bool, ArArgumentBuilder *> *functor)
{
  if (keyword == NULL)
  {
    if (myRemainderHandler != NULL)
    {
      ArLog::log(ArLog::Verbose, "There is already a functor to handle unhandled lines");
      return false;
    }
    else
    {
      myRemainderHandler.reset( new HandlerCBType(functor) );
      return true;
    }
  }

  auto it = myMap.find(keyword);
  if (it!= myMap.end())
  {
    ArLog::log(ArLog::Verbose, "There is already a functor to handle keyword '%s'", keyword);
    return false;
  }
  ArLog::log(ArLog::Verbose, "keyword '%s' handler added", keyword);
  myMap[keyword].reset( new HandlerCBType(functor) );
  return true;
}

/**
   This function has a different name than addProcessFileCB just so
   that if you mean to get this function but have the wrong functor
   you'll get an error.  The rem's are the same though since that
   shouldn't matter.
**/
AREXPORT bool ArFileParser::addHandlerWithError(
	const char *keyword,
	ArRetFunctor3<bool, ArArgumentBuilder *, char *, size_t> *functor)
{
  if (keyword == NULL)
  {
    if (myRemainderHandler != NULL)
    {
      ArLog::log(ArLog::Verbose, "There is already a functor to handle unhandled lines");
      return false;
    }
    else
    {
      myRemainderHandler.reset( new HandlerCBType(functor) );
      return true;
    }
  }
  auto it = myMap.find(keyword);
  if (it != myMap.end())
  {
    ArLog::log(ArLog::Verbose, "There is already a functor to handle keyword '%s'", keyword);
    return false;
  }
  ArLog::log(ArLog::Verbose, "keyword '%s' handler added", keyword);
  myMap[keyword].reset( new HandlerCBType(functor) );
  return true;
}

AREXPORT bool ArFileParser::remHandler(const char *keyword,
				       bool logIfCannotFind)
{
  std::shared_ptr<HandlerCBType> handler;

  if (myRemainderHandler != NULL && keyword == NULL)
  {
    myRemainderHandler.reset();
    ArLog::log(ArLog::Verbose, "Functor for remainder handler removed");
    return true;
  }

  auto it = myMap.find(keyword);
  if (it == myMap.end())
  {
    if (logIfCannotFind)
      ArLog::log(ArLog::Normal, "There is no keyword '%s' to remove.",
		 keyword);
    return false;
  }
  ArLog::log(ArLog::Verbose, "keyword '%s' removed", keyword);
  handler = (*it).second;
  myMap.erase(it);
  handler.reset();
  remHandler(keyword, false);
  return true;

}

AREXPORT bool ArFileParser::remHandler(
	ArRetFunctor1<bool, ArArgumentBuilder *> *functor)
{
  if (myRemainderHandler != NULL && myRemainderHandler->haveFunctor(functor))
  {
    myRemainderHandler.reset();
    ArLog::log(ArLog::Verbose, "Functor for remainder handler removed");
    return true;
  }

  for (auto it = myMap.begin(); it != myMap.end(); it++)
  {
    if ((*it).second->haveFunctor(functor))
    {
      ArLog::log(ArLog::Verbose, "Functor for keyword '%s' removed.",
		 (*it).first.c_str());
      (*it).second.reset();
      myMap.erase(it);
      remHandler(functor);
      return true;
    }
  }
  return false;

}

AREXPORT bool ArFileParser::remHandler(
	ArRetFunctor3<bool, ArArgumentBuilder *, char *, size_t> *functor)
{
  if (myRemainderHandler != NULL && myRemainderHandler->haveFunctor(functor))
  {
    myRemainderHandler.reset();
    ArLog::log(ArLog::Verbose, "Functor for remainder handler removed");
    return true;
  }

  for (auto it = myMap.begin(); it != myMap.end(); it++)
  {
    if ((*it).second->haveFunctor(functor))
    {
      ArLog::log(ArLog::Verbose, "Functor for keyword '%s' removed.",
		 (*it).first.c_str());
      (*it).second.reset();
      myMap.erase(it);
      remHandler(functor);
      return true;
    }
  }
  return false;

}

/*
AREXPORT ArRetFunctor1<bool, ArArgumentBuilder *> *ArFileParser::getHandler(const char *keyword)
{
  std::map<std::string, ArRetFunctor1<bool, ArArgumentBuilder *> *, ArStrCaseCmpOp>::iterator it;

  if ((it = myMap.find(keyword)) == myMap.end())
  {
    ArLog::log(ArLog::Normal, "There is no keyword handler for '%s'", keyword);
    return NULL;
  }

  return (*it).second;
}
*/
AREXPORT void ArFileParser::setBaseDirectory(const char *baseDirectory)
{
  if (baseDirectory != NULL && strlen(baseDirectory) > 0)
    myBaseDir = baseDirectory;
  else
    myBaseDir = "";
}

AREXPORT const char *ArFileParser::getBaseDirectory(void) const
{
  return myBaseDir.c_str();
}

AREXPORT void ArFileParser::resetCounters(void)
{
  myLineNumber = 0;
}

AREXPORT bool ArFileParser::parseLine(char *line,
				      char *errorBuffer, size_t errorBufferLen)
{
  char keyword[512];
  char *choppingPos;
  char *valueStart=NULL;
  size_t textStart=0;
  size_t len;
  size_t i;
  bool noArgs;
  std::shared_ptr<HandlerCBType> handler;

  myLineNumber++;
  noArgs = false;

  // chop out the comments
  if ((choppingPos = strstr(line, ";")) != NULL)
    line[choppingPos-line] = '\0';
  if ((choppingPos = strstr(line, "#")) != NULL)
    line[choppingPos-line] = '\0';



  // chop out the new line if its there
  if ((choppingPos = strstr(line, "\n")) != NULL)
    line[choppingPos-line] = '\0';
  // chop out the windows new line if its there
  while ((choppingPos = strstr(line, "\r")) != NULL)
    memmove(choppingPos, choppingPos + 1, strlen(line));

  // see how long the line is
  len = strlen(line);

  // find the keyword
  // if this is 0 then we have an empty line so we continue
  if (len == 0)
  {
    ArLog::log(ArLog::Verbose, "line %d: empty line", myLineNumber);
    return true;
  }
  // first find the start of the text
  for (i = 0; i < len; i++)
  {
    // if its not a space we're done
    if (!isspace(line[i]))
    {
      textStart = i;
      break;
    };
  }
  // if we reached the end of the line then continue
  if (i == len)
  {
    ArLog::log(ArLog::Verbose, "line %d: just white space at start of line", myLineNumber);
    return true;
  }
  // now we chisel out the keyword
  // adding it so that if the text is quoted it pulls the whole keyword
  bool quoted = false;
  for (i = textStart;
       i < len && i < sizeof(keyword) + textStart - 3;
       i++)
  {
    // if we're on the start and its a quote just note that and continue
    if (!quoted && i == textStart && line[i] == '"')
    {
      // set quoted to true since we're going to move textStart ahead
      // and don't want to loop this
      quoted = true;
      // note that our text starts on the next char really
      textStart++;
      continue;
    }
    // if we're not looking for the end quote and its a space we're done
    if (!quoted && isspace(line[i]))
    {
      break;
    }
    // if we are looking for the end quote and its a quote we're done
    // (so put the null terminator in the keyword and advance the line
    // iterator beyond the end quote
    else if (quoted && line[i] == '"')
    {
      keyword[i-textStart] = '\0';
      i++;
      break;
    }
    // if not its part of the keyword
    else
      keyword[i-textStart] = line[i];
  }

  keyword[i-textStart] = '\0';
  //ArLog::log(ArLog::Verbose, "line %d: keyword %s", lineNumber, keyword);
  // now find the start of the value (first non whitespace)
  for (; i < len; i++)
  {
    // if its not a space we're done
    if (!isspace(line[i]))
    {
      valueStart = &line[i];
      break;
    };
  }
  // lower that keyword
  ArUtil::lower(keyword, keyword, 512);

  // a variable for if we're using the remainder handler or not (don't
  // do a test just because someone could set the remainder handler to
  // some other handler they're using)
  bool usingRemainder = false;
  // see if we have a handler for the keyword
  auto it = myMap.find(keyword);
  if ( it != myMap.end())
  {
    //printf("have handler for keyword %s\n", keyword);
    // we have a handler, so pull that out
    handler = (*it).second;
    // valueStart was set above but make sure there's an argument
    if (i == len)
      noArgs = true;
  }
  // if we don't then check for a remainder handler
  else
  {
    //printf("no handler for keyword %s\n", keyword);
    // if we have one set it
    if (myRemainderHandler != NULL)
    {
      usingRemainder = true;
      handler = myRemainderHandler;
      // reset the value to the start of the text
      valueStart = &line[textStart];
    }
    // if we don't just keep going
    else
    {
      ArLog::log(ArLog::Verbose,
		 "line %d: unknown keyword '%s' line '%s', continuing",
		 myLineNumber, keyword, &line[textStart]);
      return true;
    }
  }
  /*
  if (noArgs)
    ArLog::log(ArLog::Verbose, "line %d: firstword '%s' no argument",
	       myLineNumber, keyword);
  else
    ArLog::log(ArLog::Verbose, "line %d: firstword '%s' argument '%s'",
	       myLineNumber, keyword, valueStart);
  */
  // now toss the rest of the argument into an argument builder then
  // form it up to send to the functor

  ArArgumentBuilder builder;
  // if we have arguments add them
  if (!noArgs)
    builder.add(valueStart);
  // if not we still set the name of whatever we parsed (unless we
  // didn't have a param of course)
  if (!usingRemainder)
    builder.setExtraString(keyword);

  // make sure we don't overwrite any errors
  if (errorBuffer != NULL && errorBuffer[0] != '\0')
  {
    errorBuffer = NULL;
    errorBufferLen = 0;
  }

  // call the functor and see if there are errors;
  // if we had an error and aren't continuing on errors then we keep going
  if (!handler->call(&builder, errorBuffer, errorBufferLen))
  {
    // put the line number in the error message (this won't overwrite
    // anything because of the check above
    if (errorBuffer != NULL)
    {
      std::string errorString = errorBuffer;
      snprintf(errorBuffer, errorBufferLen, "Line %d: %s", myLineNumber,
	       errorString.c_str());

    }
    return false;
  }
  return true;
}

/**
   @param fileName the file to open

   @param continueOnErrors whether to continue or immediately bail upon an error

   @param noFileNotFoundMessage whether or not to log if we find a
   file (we normally want to but for robot param files that'd be too
   annoying since we test for a lot of files)

   @param errorBuffer buffer to put errors into if not NULL. Only the
   first error is saved, and as soon as this function is called it
   immediately empties the errorBuffer

   @param errorBufferLen the length of @a errorBuffer
*/
AREXPORT bool ArFileParser::parseFile(const char *fileName,
				      bool continueOnErrors,
				      bool noFileNotFoundMessage,
				      char *errorBuffer,
				      size_t errorBufferLen)
{
  FILE *file;

  char line[10000];
  bool ret = true;

  if (errorBuffer)
    errorBuffer[0] = '\0';

  std::string realFileName;
  if (fileName[0] == '/' || fileName[0] == '\\')
  {
    realFileName = fileName;
  }
  else
  {
    realFileName = myBaseDir;
    realFileName += fileName;
  }

  ArLog::log(ArLog::Verbose, "Opening file %s from fileName given %s and base directory %s", realFileName.c_str(), fileName, myBaseDir.c_str());

  //char *buf = new char[4096];

  if ((file = fopen(realFileName.c_str(), "r")) == NULL)
  {
    if (errorBuffer != NULL)
      snprintf(errorBuffer, errorBufferLen, "cannot open file %s", fileName);
    if (!noFileNotFoundMessage)
      ArLog::log(ArLog::Terse, "ArFileParser::parseFile: Could not open file %s to parse file.", realFileName.c_str());
    return false;
  }
/**
   if( setvbuf( file, buf, _IOFBF, sizeof( buf ) ) != 0 )
         printf( "Incorrect type or size of buffer for file\n" );
     //else
     //    printf( "'file' now has a buffer of 1024 bytes\n" );
**/

  resetCounters();
  // read until the end of the file
  while (fgets(line, sizeof(line), file) != NULL)
  {
    if (!parseLine(line, errorBuffer, errorBufferLen))
    {
      ArLog::log(ArLog::Terse, "## Last error on line %d of file '%s'",
		 myLineNumber, realFileName.c_str());
      ret = false;
      if (!continueOnErrors)
	break;
    }
  }

  fclose(file);
  return ret;
}


/**
 * @param file File pointer for a file to be parsed. The file must be open for
 *  reading (e.g. with fopen()) and this pointer must not be NULL.
 * @param buffer a non-NULL char array in which to read the file
 * @param bufferLength the number of chars in the buffer; must be greater than 0
 * @param continueOnErrors a bool set to true if parsing should continue
 *  even after an error is detected
 * @param errorBuffer buffer to put errors into if not NULL. Only the
 *  first error is saved, and as soon as this function is called it
 *  immediately empties the errorBuffer
 * @param errorBufferLen the length of @a errorBuffer
*/
AREXPORT bool ArFileParser::parseFile(FILE *file, char *buffer,
				      int bufferLength,
				      bool continueOnErrors,
				      char *errorBuffer,
				      size_t errorBufferLen)
{
  if (errorBuffer)
    errorBuffer[0] = '\0';

  if ((file == NULL) || (buffer == NULL) || (bufferLength <= 0))
  {
    if (errorBuffer != NULL)
      snprintf(errorBuffer, errorBufferLen, "parseFile: bad setup");
    return false;
  }

  bool ret = true;
  resetCounters();

  // read until the end of the file
  while (fgets(buffer, bufferLength, file) != NULL)
  {
    if (!parseLine(buffer, errorBuffer, errorBufferLen))
    {
      ret = false;
      if (!continueOnErrors)
        break;
    }
  }
  return ret;
}

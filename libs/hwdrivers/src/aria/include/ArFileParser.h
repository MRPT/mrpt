/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARFILEPARSER_H
#define ARFILEPARSER_H

#include "ariaTypedefs.h"
#include "ArArgumentParser.h"
#include "ArFunctor.h"
#include "ariaUtil.h"
#include <memory>


/// Class for parsing files more easily
/**
   This class helps parse text files based on keywords followed by various
   values.
   To use it, add functors of different types
   of arguments with addHandler(), then call parseFile() to parse the file
   and invoke the various functors as items are read from the file.
   parseFile() returns true if there were no errors parsing and false if
   there were errors.

   One side feature is that you can have ONE handler for the keyword
   NULL, and if there is a line read that isn't entirely comments or
   whitespace that handler will be given the line.  There isn't an
   explicit set for them since then there'd be another set of 5 adds.

   There should be some whitespace after keywords in the file, and any
   semicolon (;) will act as a comment with the rest of the line
   ignored.  If no handler exists for the first word the line is
   passed to the handler above for NULL.  You can't have any lines
   longer than 10000 characters or keywords longer than 512 characters
   (though I don't know why you'd have either).  If you have more than
   2048 words on a line you'll have problems as well.
 **/
class ArFileParser
{
public:
  /// Constructor
  AREXPORT ArFileParser(const char *baseDirectory = "./");
  /// Destructor
  AREXPORT ~ArFileParser(void);
  /// Adds a functor to handle a keyword that wants an easily parsable string
  AREXPORT bool addHandler(const char *keyword,
			   ArRetFunctor1<bool, ArArgumentBuilder *> *functor);
  /// Adds a functor to handle a keyword that wants an easily parsable string and returns error messages
  AREXPORT bool addHandlerWithError(const char *keyword,
			   ArRetFunctor3<bool, ArArgumentBuilder *,
				    char *, size_t> *functor);
  /// Removes a handler for a keyword
  AREXPORT bool remHandler(const char *keyword, bool logIfCannotFind = true);
  /// Removes any handlers with this functor
  AREXPORT bool remHandler(ArRetFunctor1<bool, ArArgumentBuilder *> *functor);
  /// Removes any handlers with this functor
  AREXPORT bool remHandler(
	  ArRetFunctor3<bool, ArArgumentBuilder *, char *, size_t> *functor);
  /* this shouldn't be needed and would be inelegant with the new scheme,
     if someone needs it let us know and I'll update it somehow
  /// Gets handler data for some keyword
  AREXPORT ArRetFunctor1<bool, ArArgumentBuilder *> *getHandler(const char *keyword);
  */

  /// The function to parse a file
  AREXPORT bool parseFile(const char *fileName, bool continueOnErrors = true,
			  bool noFileNotFoundMessage = false,
			  char *errorBuffer = NULL, size_t errorBufferLen = 0);

  /// Parses an open file; the file is not closed by this method.
  /**
   * @param file the open FILE* to be parsed; must not be NULL
   * @param buffer a non-NULL char array in which to read the file
   * @param bufferLength the number of chars in the buffer; must be greater than 0
   * @param continueOnErrors a bool set to true if parsing should continue
   * even after an error is detected
  **/
  AREXPORT bool parseFile(FILE *file, char *buffer, int bufferLength,
			  bool continueOnErrors = true,
			  char *errorBuffer = NULL, size_t errorBufferLen = 0);

  /// Gets the base directory
  AREXPORT const char *getBaseDirectory(void) const;
  /// Sets the base directory
  AREXPORT void setBaseDirectory(const char *baseDirectory);
  /// Function to parse a single line
  AREXPORT bool parseLine(char *line, char *errorBuffer = NULL,
			  size_t errorBufferLen = 0);
  /// Function to reset counters
  AREXPORT void resetCounters(void);
protected:
  class HandlerCBType
  {
    public:
    HandlerCBType(
	    ArRetFunctor3<bool, ArArgumentBuilder *, char *, size_t> *functor)
    {
      myCallbackWithError = functor;
      myCallback = NULL;
    }
    HandlerCBType(ArRetFunctor1<bool, ArArgumentBuilder *> *functor)
    {
      myCallbackWithError = NULL;
      myCallback = functor;
    }
    ~HandlerCBType() {}
    bool call(ArArgumentBuilder *arg, char *errorBuffer,
	      size_t errorBufferLen)
    {
      if (myCallbackWithError != NULL)
	return myCallbackWithError->invokeR(arg, errorBuffer, errorBufferLen);
      else if (myCallback != NULL)
	return myCallback->invokeR(arg);
      // if we get here there's a problem
      ArLog::log(ArLog::Terse, "ArFileParser: Horrible problem with process callbacks");
      return false;
    }
    bool haveFunctor(
	    ArRetFunctor3<bool, ArArgumentBuilder *, char *, size_t> *functor)
    {
      if (myCallbackWithError == functor)
	return true;
      else
	return false;
    }
    bool haveFunctor(ArRetFunctor1<bool, ArArgumentBuilder *> *functor)
    {
      if (myCallback == functor)
	return true;
      else
	return false;
    }
    const char *getName(void)
    {
      if (myCallbackWithError != NULL)
	return myCallbackWithError->getName();
      else if (myCallback != NULL)
	return myCallback->getName();
      // if we get here there's a problem
      ArLog::log(ArLog::Terse, "ArFileParser: Horrible problem with process callback names");
      return NULL;
    }
    protected:
    ArRetFunctor3<bool, ArArgumentBuilder *, char *, size_t> *myCallbackWithError;
    ArRetFunctor1<bool, ArArgumentBuilder *> *myCallback;
  };
  int myLineNumber;
  std::string myBaseDir;
  std::map<std::string, std::shared_ptr<HandlerCBType>, ArStrCaseCmpOp> myMap;
  // handles that NULL case
  std::shared_ptr<HandlerCBType> myRemainderHandler;
};

#endif // ARFILEPARSER_H

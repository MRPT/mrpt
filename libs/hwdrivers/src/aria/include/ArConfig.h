/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARCONFIG_H
#define ARCONFIG_H

#include "ArConfigArg.h"
#include "ArFileParser.h"
#include <set>

class ArArgumentBuilder;
class ArConfigSection;

/// Stores configuration information which may be read to and from files or other sources
/**
 * The configuration is a set of parameters (or config arguments), organized into
 * sections.  Parameters contain a key or name (a short string), a value
 * (which may be one of several types), a priority (important, normal, or trivial
 * to most users), a longer description, and a display hint which suggests
 * what kind of UI control might be most appropriate for the parameter.
 * After adding a parameter to the configuration, its value may be changed
 * when the configuration is loaded or reloaded from a file.
 * Various program modules may register callbacks to be notified
 * when a shared global configuration (such as the static ArConfig object kept
 * by the Aria class) is loaded or otherwise changed.
 *
 * Classes dealing with more specialized kinds of config files
 * inherit from this one.
 *
 * Important methods in this class are: addParam(), addProcessFileCB(),
 * remProcessFileCB(), parseFile(), writeFile().
*/
class ArConfig
{
public:
  /// Constructor
  AREXPORT ArConfig(const char *baseDirectory = NULL,
		                bool noBlanksBetweenParams = false,
		                bool ignoreBounds = false,
		                bool failOnBadSection = false,
		                bool saveUnknown = true);

  /// Copy constructor
  AREXPORT ArConfig(const ArConfig &config);

  AREXPORT ArConfig &operator=(const ArConfig &config);

  /// Destructor
  AREXPORT virtual ~ArConfig();

  /// Stores an optional config and robot name to be used in log messages.
  AREXPORT virtual void setConfigName(const char *configName,
                                      const char *robotName = NULL);

  /// Parse a config file
  AREXPORT bool parseFile(const char *fileName,
                          bool continueOnError = false,
			                    bool noFileNotFoundMessage = false,
			                    char *errorBuffer = NULL,
			                    size_t errorBufferLen = 0,
                          std::list<std::string> *sectionsToParse = NULL);

  /// Write out a config file
  AREXPORT bool writeFile(const char *fileName,
                         bool append = false,
			                   std::set<std::string> *alreadyWritten = NULL,
			                   bool writePriorities = false,
                         std::list<std::string> *sectionsToWrite = NULL);

  /// Command to add a parameter to the given section with given priority
  AREXPORT bool addParam(const ArConfigArg &arg,
                         const char *sectionName = "",
			 ArPriority::Priority priority = ArPriority::NORMAL,
                         const char *displayHint = NULL);

  /// Command to add a new comment to the given section with given priority
  AREXPORT bool addComment(const char *comment, const char *sectionName = "",
			   ArPriority::Priority priority = ArPriority::NORMAL);
  /// Sets the comment for a section
  AREXPORT void setSectionComment(const char *sectionName,
				  const char *comment);
  /// Uses this argument parser after it parses a file before it processes
  AREXPORT void useArgumentParser(ArArgumentParser *parser);
  /// for inheritors this is called after the file is processed
  /**
     For classes that inherit from ArConfig this function is called
     after parseFile and all of the processFileCBs are called... If
     you want to call something before the processFileCBs then just
     add a processFileCB... this is only called if there were no
     errors parsing the file or continueOnError was set to false when
     parseFile was called

     @return true if the config parsed was good (parseFile will return
     true) false if the config parsed wasn't (parseFile will return false)
   **/
  /*AREXPORT*/ virtual bool processFile(void) { return true; }
  /// Adds a callback to be invoked when the configuration is loaded or
  /// reloaded.
  AREXPORT void addProcessFileCB(ArRetFunctor<bool> *functor,
				 int priority = 0);
  /// Adds a callback to be invoked when the configuration is loaded or
  /// reloaded, which my also receive error messages
  AREXPORT void addProcessFileWithErrorCB(
	  ArRetFunctor2<bool, char *, size_t> *functor,
	  int priority = 0);
  /// Removes a processedFile callback
  AREXPORT void remProcessFileCB(ArRetFunctor<bool> *functor);
  /// Removes a processedFile callback
  AREXPORT void remProcessFileCB(
	  ArRetFunctor2<bool, char *, size_t> *functor);
  /// Call the processFileCBs
  AREXPORT bool callProcessFileCallBacks(bool continueOnError,
					 char *errorBuffer = NULL,
					 size_t errorBufferLen = 0);
  /// This parses the argument given (for parser or other use)
  AREXPORT bool parseArgument(ArArgumentBuilder *arg,
			      char *errorBuffer = NULL,
			      size_t errorBufferLen = 0);
  /// This parses the section change (for parser or other use)
  AREXPORT bool parseSection(ArArgumentBuilder *arg,
			      char *errorBuffer = NULL,
			      size_t errorBufferLen = 0);
  /// This parses an unknown argument (so we can save it)
  AREXPORT bool parseUnknown(ArArgumentBuilder *arg,
			     char *errorBuffer = NULL,
			     size_t errorBufferLen = 0);
  /// Get the base directory
  AREXPORT const char *getBaseDirectory(void) const;
  /// Set the base directory
  AREXPORT void setBaseDirectory(const char *baseDirectory);
  /// Get the file name we loaded
  AREXPORT const char *getFileName(void) const;
  /// Set whether we have blanks between the params or not
  AREXPORT void setNoBlanksBetweenParams(bool noBlanksBetweenParams);

  /// Get whether we have blanks between the params or not
  AREXPORT bool getNoBlanksBetweenParams(void);

  /// Use an argument parser to change the config
  AREXPORT bool parseArgumentParser(ArArgumentParser *parser,
				    bool continueOnError = false,
				    char *errorBuffer = NULL,
				    size_t errorBufferLen = 0);

  /// Get the sections themselves (use only if you know what to do)
  AREXPORT std::list<ArConfigSection *> *getSections(void);

  /// Find the section with the given name.
  /// @return section object, or NULL if not found.
  AREXPORT ArConfigSection *findSection(const char *sectionName) const;
  /// Set the log level used when loading or reloading the configuration
  /*AREXPORT*/void setProcessFileCallbacksLogLevel(ArLog::LogLevel level)
    {  myProcessFileCallbacksLogLevel = level; }
  /// Get the log level used when loading or reloading the configuration
  /*AREXPORT*/ArLog::LogLevel getProcessFileCallbacksLogLevel(void)
    {  return myProcessFileCallbacksLogLevel; }
  /// Sets whether we save unknown items (if we don't save 'em we ignore 'em)
  /*AREXPORT*/void setSaveUnknown(bool saveUnknown)
    { mySaveUnknown = saveUnknown; }
  /// Gets whether we save unknowns (if we don't save 'em we ignore 'em)
  /*AREXPORT*/ bool getSaveUnknown(void) { return mySaveUnknown; }
  /// Clears out all the section information
  AREXPORT void clearSections(void);
  /// Clears out all the section information and the processFileCBs
  AREXPORT void clearAll(void);
  /// adds a flag to a section
  AREXPORT bool addSectionFlags(const char *sectionName,
				const char *flags);
  /// Removes a flag from a section
  AREXPORT bool remSectionFlag(const char *sectionName,
			       const char *flag);

  /// calls clearValueSet on the whole config (internal for default configs)
  AREXPORT void clearAllValueSet(void);
  /// Removes all unset values from the config (internal for default configs)
  AREXPORT void removeAllUnsetValues(void);

  AREXPORT void log(bool isSummary = true);

protected:
  /// Write out a section
  AREXPORT void writeSection(ArConfigSection *section, FILE *file,
			     std::set<std::string> *alreadyWritten,
			     bool writePriorities);

  void copySectionsToParse(std::list<std::string> *from);

  /**
     This class's job is to make the two functor types largely look
     like the same one from the code's perspective, this is so we can
     store them both in the same map for order of operations purposes.

     The funkiness with the constructor is because the retfunctor2
     looks like the retfunctor and winds up always falling into that
     constructor.
  **/
  class ProcessFileCBType
  {
    public:
    ProcessFileCBType(
	    ArRetFunctor2<bool, char *, size_t> *functor)
    {
      myCallbackWithError = functor;
      myCallback = NULL;
    }
    ProcessFileCBType(ArRetFunctor<bool> *functor)
    {
      myCallbackWithError = NULL;
      myCallback = functor;
    }
    ~ProcessFileCBType() {}
    bool call(char *errorBuffer, size_t errorBufferLen)
    {
      if (myCallbackWithError != NULL)
	return myCallbackWithError->invokeR(errorBuffer, errorBufferLen);
      else if (myCallback != NULL)
	return myCallback->invokeR();
      // if we get here there's a problem
      ArLog::log(ArLog::Terse, "ArConfig: Horrible problem with process callbacks");
      return false;
    }
    bool haveFunctor(ArRetFunctor2<bool, char *, size_t> *functor)
    {
      if (myCallbackWithError == functor)
	return true;
      else
	return false;
    }
    bool haveFunctor(ArRetFunctor<bool> *functor)
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
      ArLog::log(ArLog::Terse, "ArConfig: Horrible problem with process callback names");
      return NULL;
    }
    protected:
    ArRetFunctor2<bool, char *, size_t> *myCallbackWithError;
    ArRetFunctor<bool> *myCallback;
  };
  void addParserHandlers(void);

  /// Optional name of the robot with which the config is associated.
  std::string myRobotName;
  /// Optional name of the config instance.
  std::string myConfigName;
  /// Prefix to be inserted in log messages (contains the robot and config names).
  std::string myLogPrefix;

  ArArgumentParser *myArgumentParser;
  std::multimap<int, ProcessFileCBType *> myProcessFileCBList;
  bool myNoBlanksBetweenParams;
  std::string mySection;
  std::list<std::string> *mySectionsToParse;
  bool mySectionBroken;
  bool mySectionIgnored;
  bool myUsingSections;
  std::string myFileName;
  std::string myBaseDirectory;
  ArFileParser myParser;
  bool myIgnoreBounds;
  bool myFailOnBadSection;
  bool myDuplicateParams;
  bool mySaveUnknown;
  ArLog::LogLevel myProcessFileCallbacksLogLevel;
  // our list of sections which has in it the argument list for each
  std::list<ArConfigSection *> mySections;
  // callback for the file parser
  ArRetFunctor3C<bool, ArConfig, ArArgumentBuilder *, char *, size_t> myParserCB;
  // callback for the section in the file parser
  ArRetFunctor3C<bool, ArConfig, ArArgumentBuilder *, char *, size_t> mySectionCB;
  // callback for the unknown param (or wahtever) file parser
  ArRetFunctor3C<bool, ArConfig, ArArgumentBuilder *, char *, size_t> myUnknownCB;
};


/** Represents a section in the configuration. Sections are used to
 *  group items used by separate parts of Aria.
 */
class ArConfigSection
{
public:
  AREXPORT ArConfigSection(const char *name = NULL,
						   const char *comment = NULL);
  AREXPORT virtual ~ArConfigSection();
  AREXPORT ArConfigSection(const ArConfigSection &section);
  AREXPORT ArConfigSection &operator=(const ArConfigSection &section);

  /** @return The name of this section */
  const char *getName(void) const { return myName.c_str(); }

  /** @return A comment describing this section */
  const char *getComment(void) const { return myComment.c_str(); }
  const char *getFlags(void) const { return myFlags->getFullString(); }
  AREXPORT bool hasFlag(const char *flag) const;
  std::list<ArConfigArg> *getParams(void) { return &myParams; }
  void setName(const char *name) { myName = name; }
  void setComment(const char *comment) { myComment = comment; }
  bool addFlags(const char *flags) { myFlags->add(flags); return true; }
  AREXPORT bool remFlag(const char *dataFlag);
  /// Finds a parameter item in this section with the given name.  Returns NULL if not found.
  AREXPORT ArConfigArg *findParam(const char *paramName);
  /// Removes a string holder for this param, returns true if it found one
  AREXPORT bool remStringHolder(const char *paramName);

protected:
  std::string myName;
  std::string myComment;
  ArArgumentBuilder *myFlags;
  std::list<ArConfigArg> myParams;
};

#endif // ARCONFIG

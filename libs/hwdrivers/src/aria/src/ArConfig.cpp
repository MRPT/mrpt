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
#include "ArConfig.h"
#include "ArArgumentBuilder.h"
#include "ArLog.h"

#include <mrpt/utils/mrpt_macros.h>

// Comment out this line to hide the "(probably unimportant)" messages.
#define ARDEBUG_CONFIG

#if (defined(ARDEBUG_CONFIG))
//#if (defined(_DEBUG) && defined(ARDEBUG_CONFIG))
#define IFDEBUG(code) {code;}
#else
#define IFDEBUG(code)
#endif

// IFDEBUG2 provides even more detailed level of logging (e.g. when configs are copied)
// #define ARDEBUG_CONFIG2

#if (defined(_DEBUG) && defined(ARDEBUG_CONFIG2))
#define IFDEBUG2(code) {code;}
#else
#define IFDEBUG2(code)
#endif


/**
   @param baseDirectory a directory to search for files in
   @param noBlanksBetweenParams if there should not be blank lines between params in output
   @param ignoreBounds if this is true bounds checking will be ignored when the file is read in. this should ONLY be used by developers debugging
   @param failOnBadSection if this is true and there is a bad section, then parseFile will fail
   @param saveUnknown if this is true and there are unknown parameters or sections then they will be saved, if false then they will be ignored (can also be set with setSaveUnknown())
 **/
AREXPORT ArConfig::ArConfig(const char *baseDirectory,
			    bool noBlanksBetweenParams,
			    bool ignoreBounds,
			    bool failOnBadSection,
			    bool saveUnknown) :
  myRobotName(""),
  myConfigName(""),
  myLogPrefix("ArConfig: "),

  myProcessFileCBList(),
  myNoBlanksBetweenParams(noBlanksBetweenParams),
  mySectionsToParse(NULL),
  myBaseDirectory(),
  myParser(NULL),
  mySections(),
  myParserCB(this, &ArConfig::parseArgument),
  mySectionCB(this, &ArConfig::parseSection),
  myUnknownCB(this, &ArConfig::parseUnknown)
{
  /* taking this part out and just calling 'addParsrHandler' so all the codes in one spot
  if (!myParser.addHandlerWithError("section", &mySectionCB))
  {
    ArLog::log(ArLog::Normal, "Could not add section to file parser, horrific failure");
    }*/
  addParserHandlers();

  myArgumentParser = NULL;
  mySaveUnknown = saveUnknown;
  setBaseDirectory(baseDirectory);
  myIgnoreBounds = ignoreBounds;
  myFailOnBadSection = failOnBadSection;
  myProcessFileCallbacksLogLevel = ArLog::Verbose;
  myUsingSections = false;
  mySectionBroken = false;
  mySectionIgnored = false;
  myDuplicateParams = false;

  myParserCB.setName("ArConfig::parseArgument");
  mySectionCB.setName("ArConfig::parseSection");
  myUnknownCB.setName("ArConfig::parseUnknown");

  mySection = "";
}

AREXPORT ArConfig::~ArConfig()
{
  clearSections();
}


AREXPORT ArConfig::ArConfig(const ArConfig &config) :
  myRobotName(""),
  myConfigName(""),
  myLogPrefix("ArConfig: "),

  myProcessFileCBList(),    // TODO: Copy? (don't think we need to)
  myNoBlanksBetweenParams(config.myNoBlanksBetweenParams),
  myBaseDirectory(),
  myParser(NULL),
  mySections(),
  myParserCB(this, &ArConfig::parseArgument),
  mySectionCB(this, &ArConfig::parseSection),
  myUnknownCB(this, &ArConfig::parseUnknown)
{
  myArgumentParser = NULL;
  setBaseDirectory(config.getBaseDirectory());
  myIgnoreBounds = config.myIgnoreBounds;
  myFailOnBadSection = config.myFailOnBadSection;
  myProcessFileCallbacksLogLevel = config.myProcessFileCallbacksLogLevel;
  mySectionBroken = config.mySectionBroken;
  mySectionIgnored = config.mySectionIgnored;
  mySection = config.mySection;
  myUsingSections = config.myUsingSections;
  myDuplicateParams = config.myDuplicateParams;


  std::list<ArConfigSection *>::const_iterator it;
  for (it = config.mySections.begin();
       it != config.mySections.end();
       it++)
  {
    mySections.push_back(new ArConfigSection(*(*it)));
  }
  copySectionsToParse(config.mySectionsToParse);

  myParserCB.setName("ArConfig::parseArgument");
  mySectionCB.setName("ArConfig::parseSection");
  myUnknownCB.setName("ArConfig::parseUnknown");

  myParser.remHandler(&myParserCB);
  myParser.remHandler(&mySectionCB);
  myParser.remHandler(&myUnknownCB);
  addParserHandlers();

}


AREXPORT ArConfig &ArConfig::operator=(const ArConfig &config)
{
  if (this != &config) {

    // Note that the name is not copied...
    IFDEBUG2(ArLog::log(ArLog::Verbose,
                       "%soperator= (from %s) begins",
                       myLogPrefix.c_str(),
                       config.myLogPrefix.c_str()));

    // TODO: The following attributes also need to be copied (or
    // something) -- ditto for copy ctor:
    //     myProcessFileCBList
    //     myParser
    //     myParserCB
    //     mySectionCB
    myArgumentParser = NULL;
    setBaseDirectory(config.getBaseDirectory());
    myNoBlanksBetweenParams = config.myNoBlanksBetweenParams;
    myIgnoreBounds = config.myIgnoreBounds;
    myFailOnBadSection = config.myFailOnBadSection;
    mySection = config.mySection;
    mySectionBroken = config.mySectionBroken;
    mySectionIgnored = config.mySectionIgnored;
    myUsingSections = config.myUsingSections;
    myDuplicateParams = config.myDuplicateParams;
    mySections.clear();
    std::list<ArConfigSection *>::const_iterator it;
    for (it = config.mySections.begin();
	 it != config.mySections.end();
	 it++)
    {
      mySections.push_back(new ArConfigSection(*(*it)));
    }

    copySectionsToParse(config.mySectionsToParse);

    myParser.remHandler(&myParserCB);
    myParser.remHandler(&mySectionCB);
    myParser.remHandler(&myUnknownCB);
    addParserHandlers();

    IFDEBUG2(ArLog::log(ArLog::Verbose,
                       "%soperator= (from %s) ends",
                       myLogPrefix.c_str(),
                       config.myLogPrefix.c_str()));
  }
  return *this;

}

/**
 * These names are used solely for log messages.
 *
 * @param configName a char * descriptive name of the ArConfig instance
 * (e.g. Server or Default)
 * @param robotName an optional char * identifier of the robot that has
 * the config
**/
AREXPORT void ArConfig::setConfigName(const char *configName,
                                      const char *robotName)
{
  myConfigName = ((configName != NULL) ? configName : "");
  myRobotName = ((robotName != NULL) ? robotName : "");
  myLogPrefix = "";

  if (!myRobotName.empty()) {
    myLogPrefix = myRobotName + ": ";
  }
  myLogPrefix += "ArConfig";

  if (!myConfigName.empty()) {
    myLogPrefix += " (" + myConfigName + ")";
  }

  myLogPrefix += ": ";
}

AREXPORT void ArConfig::log(bool isSummary)
{
  std::list<ArConfigArg> *params = NULL;

  ArLog::log(ArLog::Normal,
	           "%slog",
             myLogPrefix.c_str());

  for (std::list<ArConfigSection *>::const_iterator it = mySections.begin();
       it != mySections.end();
       it++)
  {
    params = (*it)->getParams();

    if (params == NULL) {
      ArLog::log(ArLog::Normal, "    Section %s has NULL params",
                 (*it)->getName());

      continue;
    }

    if (isSummary) {
      ArLog::log(ArLog::Normal, "    Section %s has %i params",
                 (*it)->getName(), params->size());
      continue;
    }

    ArLog::log(ArLog::Normal, "    Section %s:",
                 (*it)->getName());

    for (std::list<ArConfigArg>::iterator pit = params->begin(); pit != params->end(); ++pit)
    {
       (*pit).log();
    }

  } // end for each section

    ArLog::log(ArLog::Normal,
	           "%send",
             myLogPrefix.c_str());

} // end method log


AREXPORT void ArConfig::clearSections(void)
{
  IFDEBUG2(ArLog::log(ArLog::Verbose,
                     "%sclearSections() begin",
                     myLogPrefix.c_str()));

  while (mySections.begin() != mySections.end())
  {
    delete mySections.front();
    mySections.pop_front();
  }
  // Clear this just in case...
  delete mySectionsToParse;
  mySectionsToParse = NULL;

  IFDEBUG2(ArLog::log(ArLog::Verbose,
                     "%sclearSections() end",
                     myLogPrefix.c_str()));

}

AREXPORT void ArConfig::clearAll(void)
{
  clearSections();
  myProcessFileCBList.clear();
}

void ArConfig::addParserHandlers(void)
{
  std::list<ArConfigSection *>::const_iterator it;
  std::list<ArConfigArg> *params;
  std::list<ArConfigArg>::iterator pit;

  if (!myParser.addHandlerWithError("section", &mySectionCB)) {
    IFDEBUG(ArLog::log(ArLog::Verbose,
	                      "%sCould not add section parser (probably unimportant)",
                        myLogPrefix.c_str()));
  }
  for (it = mySections.begin();
       it != mySections.end();
       it++)
  {
    params = (*it)->getParams();
    if (params == NULL)
      continue;
    for (pit = params->begin(); pit != params->end(); pit++)
    {
      if (!myParser.addHandlerWithError((*pit).getName(), &myParserCB)) {
	IFDEBUG(ArLog::log(ArLog::Verbose,
		   "%sCould not add keyword %s (probably unimportant)",
       myLogPrefix.c_str(),
		   (*pit).getName()));
      }
    }
  }

  // add our parser for unknown params
  if (!myParser.addHandlerWithError(NULL, &myUnknownCB)) {
    IFDEBUG(ArLog::log(ArLog::Verbose,
	        "%sCould not add unknown param parser (probably unimportant)",
          myLogPrefix.c_str()));
  }
}

/**
   Set the comment string associated with a section. If the section doesn't
   exist then it is created.
**/
AREXPORT void ArConfig::setSectionComment(const char *sectionName,
					  const char *comment)
{
  ArConfigSection *section = findSection(sectionName);

  if (section == NULL)
  {
    ArLog::log(ArLog::Verbose, "%sMaking new section '%s' (for comment)",
               myLogPrefix.c_str(), sectionName);

    section = new ArConfigSection(sectionName, comment);
    mySections.push_back(section);
  }
  else
    section->setComment(comment);
}


/**
   Add a flag to a section. If the section doesn't
   exist then it is created.
**/
AREXPORT bool ArConfig::addSectionFlags(const char *sectionName,
					const char *flags)
{
  ArConfigSection *section = findSection(sectionName);

  if (section == NULL)
  {
    ArLog::log(ArLog::Verbose, "%sMaking new section '%s' (flags)",
               myLogPrefix.c_str(), sectionName);
    section = new ArConfigSection(sectionName);
    section->addFlags(flags);

    mySections.push_back(section);
  }
  else
    section->addFlags(flags);
  return true;
}

/**
   Add a flag to a section. If the section doesn't
   exist then it is created.
**/
AREXPORT bool ArConfig::remSectionFlag(const char *sectionName,
				       const char *flag)
{
  ArConfigSection *section = findSection(sectionName);

  if (section == NULL)
    return false;

  section->remFlag(flag);
  return true;
}

/** Add a parameter.
 * @param arg Object containing key, description and value type of this parameter. This object will be copied.
 * @param sectionName Name of the section to put this parameter in.
 * @param priority Priority or importance of this parameter to a user.
 * @param displayHint Suggest an appropriate UI control for editing this parameter's value. See ArConfigArg::setDisplayHint() for description of display hint format.
 */
AREXPORT bool ArConfig::addParam(const ArConfigArg &arg,
				 const char *sectionName,
				 ArPriority::Priority priority,
                                 const char *displayHint)
{
  ArConfigSection *section = findSection(sectionName);

  //printf("SECTION '%s' name '%s' desc '%s'\n", sectionName, arg.getName(), arg.getDescription());
  if (section == NULL)
  {
    ArLog::log(ArLog::Verbose, "ArConfigArg %s: Making new section '%s' (for param)",
               myLogPrefix.c_str(), sectionName);
    section = new ArConfigSection(sectionName);
    mySections.push_back(section);
  }

  std::list<ArConfigArg> *params = section->getParams();

  if (params == NULL)
  {
    ArLog::log(ArLog::Terse, "%sSomething has gone hideously wrong in ArConfig::addParam",
               myLogPrefix.c_str());
    return false;
  }

  if (arg.getType() == ArConfigArg::SEPARATOR &&
      ( !params->empty() && params->back().getType() == ArConfigArg::SEPARATOR) )
  {
    //ArLog::log(ArLog::Verbose, "Last parameter a sep, so is this one, ignoring it");
    return true;
  }

  // see if we have this parameter in another section so we can require sections
  std::list<ArConfigSection *>::iterator sectionIt;

  for (sectionIt = mySections.begin();
       sectionIt != mySections.end();
       sectionIt++)
  {
    // if we have an argument of this name but we don't have see if
    // this section is our own, if its not then note we have
    // duplicates
    if (strlen(arg.getName()) > 0 &&
	(*sectionIt)->findParam(arg.getName()) != NULL &&
	strcasecmp((*sectionIt)->getName(), section->getName()) != 0)
    {
      ArLog::log(ArLog::Verbose,
        "%sParameter %s duplicated in section %s and %s",
        myLogPrefix.c_str(),
		    arg.getName(), (*sectionIt)->getName(), section->getName());
      myDuplicateParams = true;
    }
  }

  // now make sure we can add it to the file parser (with the section
  // stuff its okay if we can't)
  if (!myParser.addHandlerWithError(arg.getName(), &myParserCB))
  {
    IFDEBUG(ArLog::log(ArLog::Verbose, "%sCould not add parameter '%s' to file parser, probably already there.",
	                      myLogPrefix.c_str(), arg.getName()));
    //return false;
  }

  // remove any string holders for this param
  section->remStringHolder(arg.getName());


  // we didn't have a parameter with this name so add it
  params->push_back(arg);
  params->back().setConfigPriority(priority);
  params->back().setDisplayHint(displayHint);
  params->back().setIgnoreBounds(myIgnoreBounds);

  IFDEBUG2(ArLog::log(ArLog::Verbose, "%sAdded parameter '%s' to section '%s'",
                      myLogPrefix.c_str(), arg.getName(), section->getName()));
  //arg.log();
  return true;
}

/**
 * Add a comment to a section.
   @param comment Text of the comment.
   @param sectionName Name of the section to add the comment to. If the section does not exist, it will be created.
   @param priority Priority or importance.
**/
AREXPORT bool ArConfig::addComment(const char *comment, const char *sectionName,
				   ArPriority::Priority priority)
{
  return addParam(ArConfigArg(comment), sectionName, priority);
}



/**
   The extra string of the parser should be set to the 'Section'
   command while the rest of the arg should be the arguments to the
   section command. Its case insensitive.

   @param arg Should contain the 'Section' keyword as its "extra" string, and section name as argument(s).

   @param errorBuffer if this is NULL it is ignored, otherwise the
   string for the error is put into the buffer, the first word should
   be the parameter that has trouble

   @param errorBufferLen the length of the error buffer
 **/
AREXPORT bool ArConfig::parseSection(ArArgumentBuilder *arg,
				      char *errorBuffer,
				      size_t errorBufferLen)
{
  if (myFailOnBadSection && errorBuffer != NULL)
    errorBuffer[0] = '\0';

  std::list<ArConfigSection *>::iterator sectionIt;
  ArConfigSection *section = NULL;

  if (myFailOnBadSection && errorBuffer != NULL)
    errorBuffer[0] = '\0';
  for (sectionIt = mySections.begin();
       sectionIt != mySections.end();
       sectionIt++)
  {
    section = (*sectionIt);
    if (ArUtil::strcasecmp(section->getName(), arg->getFullString()) == 0)
    {
      bool isParseSection = true;
      if (mySectionsToParse != NULL) {
        isParseSection = false;
        for (std::list<std::string>::iterator sIter = mySectionsToParse->begin();
             sIter != mySectionsToParse->end();
             sIter++) {
          std::string sp = *sIter;
          if (ArUtil::strcasecmp(section->getName(), sp.c_str()) == 0) {
            isParseSection = true;
            break;
          } // end if section
        } // end for each section to parse

      } // end else sections to parse specified

      if (isParseSection) {

        ArLog::log(ArLog::Verbose, "%sConfig switching to section '%s'",
                   myLogPrefix.c_str(),
		               arg->getFullString());
        //printf("Config switching to section '%s'\n",
        //arg->getFullString());
        mySection = arg->getFullString();
        mySectionBroken = false;
        mySectionIgnored = false;
        myUsingSections = true;
        return true;
      }
      else { // section is valid but shouldn't be parsed

        ArLog::log(ArLog::Verbose, "%signoring section '%s'",
                   myLogPrefix.c_str(),
		               arg->getFullString());
        //printf("Config switching to section '%s'\n",
        //arg->getFullString());
        mySection = arg->getFullString();
        mySectionBroken = false;
        mySectionIgnored = true;
        myUsingSections = true;
        return true;

      } // end else don't parse section
    } // end if section found
  } // end for each section


  if (myFailOnBadSection)
  {
    mySection = "";
    mySectionBroken = true;
    mySectionIgnored = false;
    snprintf(errorBuffer, errorBufferLen,  "ArConfig: Could not find section '%s'",
	     arg->getFullString());

    ArLog::log(ArLog::Terse,
	       "%sCould not find section '%s', failing",
         myLogPrefix.c_str(),
	       arg->getFullString());
    return false;
  }
  else
  {
    if (mySaveUnknown)
    {
      ArLog::log(ArLog::Verbose, "%smaking new section '%s' to save unknown",
		             myLogPrefix.c_str(), arg->getFullString());
      mySection = arg->getFullString();
      mySectionBroken = false;
      mySectionIgnored = false;
      section = new ArConfigSection(arg->getFullString());
      mySections.push_back(section);
    }
    else
    {
      mySection = "";
      mySectionBroken = false;
      mySectionIgnored = true;
      ArLog::log(ArLog::Normal,
		             "%sIgnoring section '%s'",
                 myLogPrefix.c_str(),
		             arg->getFullString());
    }
    return true;
  }
}

/**
   The extra string of the parser should be set to the command wanted,
   while the rest of the arg should be the arguments to the command.
   Its case insensitive.

   @param arg Obtain parameter name from this object's "extra string"
   and value(s) from its argument(s).

   @param errorBuffer If this is NULL it is ignored, otherwise the
   string for the error is put into the buffer, the first word should
   be the parameter that has trouble

   @param errorBufferLen the length of @a errorBuffer
 **/
AREXPORT bool ArConfig::parseArgument(ArArgumentBuilder *arg,
				      char *errorBuffer,
				      size_t errorBufferLen)
{
  std::list<ArConfigSection *>::iterator sectionIt;
  std::list<ArConfigArg>::iterator paramIt;
  ArConfigSection *section = NULL;
  std::list<ArConfigArg> *params = NULL;
  ArConfigArg *param = NULL;
  int valInt = 0;
  double valDouble = 0;
  bool valBool = false;
  bool ret = true;

  if (mySectionBroken)
  {
    ArLog::log(ArLog::Verbose, "%sSkipping parameter %s because section broken",
               myLogPrefix.c_str(),
	             arg->getExtraString());
    if (myFailOnBadSection)
    {
      snprintf(errorBuffer, errorBufferLen,
	       "Failed because broken config section");
      return false;
    }
    else
    {
      return true;
    }
  }
  else if (mySectionIgnored) {
    return true;
  }

  // if we have duplicate params and don't have sections don't trash anything
  if (myDuplicateParams && myUsingSections && mySection.size() <= 0)
  {
    snprintf(errorBuffer, errorBufferLen,
	     "%s not in section, client needs an upgrade",
	     arg->getExtraString());

    ArLog::log(ArLog::Normal,
               "%s%s not in a section, client needs an upgrade",
               myLogPrefix.c_str(),
	             arg->getExtraString());
    return false;
  }

  // see if we found this parameter
  bool found = false;

  if (errorBuffer != NULL)
    errorBuffer[0] = '\0';
  for (sectionIt = mySections.begin();
       sectionIt != mySections.end();
       sectionIt++)
  {
    section = (*sectionIt);
    // if we have a section make sure we're in it, otherwise do the
    // normal thing

    // MPL took out the part where if the param wasn't in a section at
    // all it checked all the sections, I took this out since
    // everything is generally in sections these days
    if (ArUtil::strcasecmp(mySection, section->getName()))
      continue;
    params = section->getParams();
    // find this parameter
    for (paramIt = params->begin(); paramIt != params->end(); paramIt++)
    {
      if (strcasecmp((*paramIt).getName(), arg->getExtraString()) == 0)
      {
	// we found it
	found = true;
	param = &(*paramIt);
	if (param->getType() != ArConfigArg::STRING &&
	    param->getType() != ArConfigArg::FUNCTOR &&
	    arg->getArg(0) == NULL)
	{
	  IFDEBUG(ArLog::log(ArLog::Verbose, "%sparameter '%s' has no argument.",
                        myLogPrefix.c_str(),
		                    param->getName()));
	  continue;
	}
	if ((param->getType() == ArConfigArg::DESCRIPTION_HOLDER) ||
      (param->getType() == ArConfigArg::SEPARATOR))
	{

	}
	// see if we're an int
	else if (param->getType() == ArConfigArg::INT)
	{
	  // if the param isn't an int fail
	  if (!arg->isArgInt(0))
	  {
	    ArLog::log(ArLog::Terse,
                 "%sparameter '%s' is an integer parameter but was given non-integer argument of '%s'",
                 myLogPrefix.c_str(), param->getName(), arg->getArg(0));
	    ret = false;
	    if (errorBuffer != NULL)
	      snprintf(errorBuffer, errorBufferLen, "%s is an integer parameter but was given non-integer argument of '%s'", param->getName(), arg->getArg(0));
	    continue;
	  }
	  valInt = arg->getArgInt(0);
	  if (param->setInt(valInt, errorBuffer, errorBufferLen))
	  {
      IFDEBUG2(ArLog::log(ArLog::Verbose, "%sSet parameter '%s' to '%d'",
		                      myLogPrefix.c_str(), param->getName(), valInt));
	    continue;
	  }
	  else
	  {
      ArLog::log(ArLog::Verbose, "%sCould not set parameter '%s' to '%d'",
		             myLogPrefix.c_str(), param->getName(), valInt);
	    ret = false;
	    continue;
	  }
	}
	else if (param->getType() == ArConfigArg::DOUBLE)
	{
	  // if the param isn't an in tfail
	  if (!arg->isArgDouble(0))
	  {
	    ArLog::log(ArLog::Terse, "%sparameter '%s' is a double parameter but was given non-double argument of '%s'",
                 myLogPrefix.c_str(), param->getName(), arg->getArg(0));
	    if (errorBuffer != NULL)
	      snprintf(errorBuffer, errorBufferLen, "%s is a double parameter but was given non-double argument of '%s'", param->getName(), arg->getArg(0));

	    ret = false;
	    continue;
	  }
	  valDouble = arg->getArgDouble(0);
	  if (param->setDouble(valDouble, errorBuffer, errorBufferLen))
	  {
      IFDEBUG2(ArLog::log(ArLog::Verbose, "%sSet parameter '%s' to '%.10f'",
		       myLogPrefix.c_str(), param->getName(), valDouble));
	    continue;
	  }
	  else
	  {
      ArLog::log(ArLog::Verbose, "%sCould not set parameter '%s' to '%.10f'",
		       myLogPrefix.c_str(), param->getName(), valDouble);
	    ret = false;
	    continue;
	  }
	}
	else if (param->getType() == ArConfigArg::BOOL)
	{
	  // if the param isn't an in tfail
	  if (!arg->isArgBool(0))
	  {
	    ArLog::log(ArLog::Terse, "%sparameter '%s' is a bool parameter but was given non-bool argument of '%s'",
                 myLogPrefix.c_str(), param->getName(), arg->getArg(0));
	    ret = false;
	    if (errorBuffer != NULL)
	      snprintf(errorBuffer, errorBufferLen, "%s is a bool parameter but was given non-bool argument of '%s'", param->getName(), arg->getArg(0));
	    continue;
	  }
	  valBool = arg->getArgBool(0);
	  if (param->setBool(valBool, errorBuffer, errorBufferLen))
	  {
      IFDEBUG2(ArLog::log(ArLog::Verbose, "%sSet parameter '%s' to %s",
		       myLogPrefix.c_str(), param->getName(), valBool ? "true" : "false" ));
	    continue;
	  }
	  else
	  {
      ArLog::log(ArLog::Verbose, "%sCould not set parameter '%s' to %s",
		            myLogPrefix.c_str(), param->getName(), valBool ? "true" : "false" );
	    ret = false;
	    continue;
	  }
	}
	else if (param->getType() == ArConfigArg::STRING)
	{
	  if (param->setString(arg->getFullString()))
	  {
      IFDEBUG2(ArLog::log(ArLog::Verbose, "%sSet parameter string '%s' to '%s'",
                          myLogPrefix.c_str(),
		                      param->getName(), param->getString()));
	    continue;
	  }
	  else
	  {
      ArLog::log(ArLog::Verbose, "%sCould not set string parameter '%s' to '%s'",
           myLogPrefix.c_str(),
		       param->getName(), param->getString());
	    if (errorBuffer != NULL && errorBuffer[0] == '\0')
	      snprintf(errorBuffer, errorBufferLen, "%s could not be set to '%s'.", param->getName(), arg->getFullString());

	    ret = false;
	    continue;
	  }
	}
	else if (param->getType() == ArConfigArg::FUNCTOR)
	{
	  if (param->setArgWithFunctor(arg))
	  {
      IFDEBUG2(ArLog::log(ArLog::Verbose, "%sSet arg '%s' with '%s'",
		       myLogPrefix.c_str(), param->getName(), arg->getFullString()));
	    continue;
	  }
	  else
	  {
      ArLog::log(ArLog::Verbose, "ArConfig: Could not set parameter '%s' to '%s'",
                 myLogPrefix.c_str(),
		              param->getName(), arg->getFullString());
	    // if it didn't put in an error message make one
	    if (errorBuffer != NULL && errorBuffer[0] == '\0')
	      snprintf(errorBuffer, errorBufferLen, "%s could not be set to '%s'.", param->getName(), arg->getFullString());
	    ret = false;
	    continue;
	  }
	}
	else
	{
    ArLog::log(ArLog::Terse, "%sHave no argument type for config '%s' in section, got string '%s', in section '%s'.",
      myLogPrefix.c_str(), arg->getExtraString(), arg->getFullString(), mySection.c_str());
	}
      }
    }
  }
  // if we didn't find this param its because its a parameter in another section, so pass this off to the parser for unknown things
  if (!found)
  {
    ArArgumentBuilder unknown;
    unknown.add(arg->getExtraString());
    unknown.add(arg->getFullString());
    return parseUnknown(&unknown, errorBuffer, errorBufferLen);
  }
  return ret;
}

/**
   The extra string of the parser should be set to the command wanted,
   while the rest of the arg should be the arguments to the command.
   Its case insensitive.

   @param arg Obtain parameter name from this argument builder's "exra" string and value(s) from its argument(s).

   @param errorBuffer if this is NULL it is ignored, otherwise the
   string for the error is put into the buffer, the first word should
   be the parameter that has trouble

   @param errorBufferLen the length of the error buffer
 **/
AREXPORT bool ArConfig::parseUnknown(ArArgumentBuilder *arg,
				     char *errorBuffer,
				     size_t errorBufferLen)
{
  MRPT_UNUSED_PARAM(errorBuffer);
  MRPT_UNUSED_PARAM(errorBufferLen);
  if (arg->getArgc() < 1)
  {
    IFDEBUG(ArLog::log(ArLog::Verbose, "%sEmpty arg in section '%s', ignoring it",
                       myLogPrefix.c_str(), mySection.c_str()));
    return true;
  }
  if (mySaveUnknown)
  {
    if (arg->getArgc() < 2)
    {
      ArLog::log(ArLog::Verbose, "%sNo arg for param '%s' in section '%s', saving it anyways",
                 myLogPrefix.c_str(), arg->getArg(0), mySection.c_str());
      addParam(ArConfigArg(arg->getArg(0), ""),
	       mySection.c_str());
    }
    else
    {
      ArLog::log(ArLog::Verbose, "%sUnknown '%s %s' in section '%s', saving it",
		             myLogPrefix.c_str(), arg->getArg(0), arg->getArg(1), mySection.c_str());
      addParam(ArConfigArg(arg->getArg(0), arg->getArg(1)),
	       mySection.c_str());
    }
  }
  else
  {
    ArLog::log(ArLog::Verbose, "%sUnknown '%s' in section '%s', ignoring it",
	             myLogPrefix.c_str(), arg->getFullString(), mySection.c_str());
  }
  return true;
}
/**
   @param fileName the file to load

   @param continueOnErrors whether to continue parsing if we get
   errors (or just bail)

   @param noFileNotFoundMessage if the file isn't found and this param
   is true it won't complain, otherwise it will

   @param errorBuffer If an error occurs and this is not NULL, copy a description of the error into this buffer

   @param errorBufferLen the length of @a errorBuffer

   @param sectionsToParse if NULL, then parse all sections; otherwise,
   a list of the section names that should be parsed (sections not in
   the list are ignored)
 **/
AREXPORT bool ArConfig::parseFile(const char *fileName,
                                  bool continueOnErrors,
				                          bool noFileNotFoundMessage,
				                          char *errorBuffer,
				                          size_t errorBufferLen,
                                  std::list<std::string> *sectionsToParse)
{
  bool ret = true;

  if(fileName)
    myFileName = fileName;
  else
    myFileName = "";

  if (errorBuffer != NULL)
    errorBuffer[0] = '\0';

  delete mySectionsToParse; // Should be null, but just in case...
  mySectionsToParse = NULL;

  copySectionsToParse(sectionsToParse);

  // parse the file (errors will go into myErrorBuffer from the functors)
  ret = myParser.parseFile(fileName, continueOnErrors, noFileNotFoundMessage);

  // set our pointers so we don't copy anymore into/over it
  if (errorBuffer != NULL && errorBuffer[0] != '\0')
  {
    errorBuffer = NULL;
    errorBufferLen = 0;
  }
  //printf("file %s\n", ArUtil::convertBool(ret));

  // if we have a parser and haven't failed (or we continue on errors)
  // then parse the arguments from the parser
  if (myArgumentParser != NULL && (ret || continueOnErrors))
    ret = parseArgumentParser(myArgumentParser, continueOnErrors,
			      errorBuffer, errorBufferLen) && ret;

  // set our pointers so we don't copy anymore into/over it
  if (errorBuffer != NULL && errorBuffer[0] != '\0')
  {
    errorBuffer = NULL;
    errorBufferLen = 0;
  }
  //printf("parser %s\n", ArUtil::convertBool(ret));

  // if we haven't failed (or we continue on errors) then call the
  // process file callbacks
  if (ret || continueOnErrors)
    ret = callProcessFileCallBacks(continueOnErrors, errorBuffer,
					  errorBufferLen) && ret;
  // copy our error if we have one and haven't copied in yet
  // set our pointers so we don't copy anymore into/over it
  if (errorBuffer != NULL && errorBuffer[0] != '\0')
  {
    errorBuffer = NULL;
    errorBufferLen = 0;
  }

  // Done with the temp parsing info, so delete it...
  delete mySectionsToParse;
  mySectionsToParse = NULL;

  //printf("callbacks %s\n", ArUtil::convertBool(ret));
  return ret;
}

/**
   @param fileName the name of the file to write out

   @param append if true then text will be appended to the file if it exists, otherwise any existing file will be overwritten.

   @param alreadyWritten if non-NULL, a list of strings that have already been
   written out, don't write it again if it's in this list; when
   something is written by this function, then it is put it into this list

   @param writePriorities if this is true then priorities will be written, if it
   is false they will not be

   @param sectionsToWrite if NULL, then write all sections; otherwise,
   a list of the section names that should be written
 **/
AREXPORT bool ArConfig::writeFile(const char *fileName,
                                  bool append,
				                          std::set<std::string> *alreadyWritten,
				                          bool writePriorities,
                                  std::list<std::string> *sectionsToWrite)
{
  FILE *file;

  std::set<std::string> writtenSet;
  std::set<std::string> *written;
  if (alreadyWritten != NULL)
    written = alreadyWritten;
  else
    written = &writtenSet;
  //std::list<ArConfigArg>::iterator it;

  // later this'll have a prefix
  std::string realFileName;
  if (fileName[0] == '/' || fileName[0] == '\\')
  {
    realFileName = fileName;
  }
  else
  {
    realFileName = myBaseDirectory;
    realFileName += fileName;
  }

  std::string mode;

  if (append)
    mode = "a";
  else
    mode = "w";

  if ((file = fopen(realFileName.c_str(), mode.c_str())) == NULL)
  {
    ArLog::log(ArLog::Terse, "%sCannot open file '%s' for writing",
	       myLogPrefix.c_str(), realFileName.c_str());
    return false;
  }

  bool firstSection = true;
  std::list<ArConfigSection *>::iterator sectionIt;
  ArConfigSection *section = NULL;

  // first write out the generic section (ie sectionless stuff, mostly
  // for backwards compatibility)
  if ( ((section = findSection("")) != NULL) &&
       (sectionsToWrite == NULL) )
  {
    if (!firstSection)
      fprintf(file, "\n");
    firstSection = false;
    writeSection(section, file, written, writePriorities);
  }

  // then write out the rest (skip the generic section if we have one)
  for (sectionIt = mySections.begin();
       sectionIt != mySections.end();
       sectionIt++)
  {
    section = (*sectionIt);
    if (strcmp(section->getName(), "") == 0)
      continue;

    if (sectionsToWrite != NULL) {
      bool isSectionFound = false;
      for (std::list<std::string>::iterator swIter = sectionsToWrite->begin();
           swIter != sectionsToWrite->end();
           swIter++) {
        std::string sp = *swIter;
        if (ArUtil::strcasecmp(section->getName(), sp.c_str()) == 0) {
          isSectionFound = true;
          break;
        }
      } // end for each section to write

      if (!isSectionFound) {
        continue;
      }

    } // end if sections specified

    if (!firstSection)
      fprintf(file, "\n");
    firstSection = false;

    writeSection(section, file, written, writePriorities);
  }
  fclose(file);
  return true;
}

AREXPORT void ArConfig::writeSection(ArConfigSection *section, FILE *file,
				     std::set<std::string> *alreadyWritten,
				     bool writePriorities)
{
  // holds each line
  char line[1024];
  // holds the fprintf
  char startLine[128];
  //bool commented = false;
  unsigned int startCommentColumn = 25;

  std::list<ArArgumentBuilder *>::const_iterator argIt;
  const std::list<ArArgumentBuilder *> *argList = NULL;


  std::list<ArConfigArg>::iterator paramIt;
  std::list<ArConfigArg> *params = NULL;
  ArConfigArg *param = NULL;

  /// clear out our written ones between sections
  alreadyWritten->clear();

  if (section->getName() != NULL && strlen(section->getName()) > 0)
  {
    //fprintf(file, "; %s\n", section->getName());
    fprintf(file, "Section %s\n", section->getName());
  }
  sprintf(line, "; ");
  if (section->getComment() != NULL && strlen(section->getComment()) > 0)
  {
    ArArgumentBuilder descr;
    descr.add(section->getComment());
    for (unsigned int i = 0; i < descr.getArgc(); i++)
    {
      // see if we're over, if we are write this line out and start
      // the next one
      if (strlen(line) + strlen(descr.getArg(i)) > 78)
      {
	      fprintf(file, "%s\n", line);
	      sprintf(line, "; ");
	      sprintf(line, "%s %s", line, descr.getArg(i));
      }
      // if its not the end of the line toss this word in
      else
      {
	      sprintf(line, "%s %s", line, descr.getArg(i));
      }
    }
    // put the last line into the file
    fprintf(file, "%s\n", line);
  }

  params = section->getParams();
  // find this parameter
  for (paramIt = params->begin(); paramIt != params->end(); paramIt++)
  {
    //commented = false;
    param = &(*paramIt);

    if (param->getType() == ArConfigArg::SEPARATOR) {
      continue;
    }

    if (alreadyWritten != NULL &&
	      param->getType() != ArConfigArg::DESCRIPTION_HOLDER &&
	      alreadyWritten->find(param->getName()) != alreadyWritten->end())
      continue;
    else if (alreadyWritten != NULL &&
	     param->getType() != ArConfigArg::DESCRIPTION_HOLDER)
    {
      alreadyWritten->insert(param->getName());
    }

    // if the type is a functor then we need to handle all of it up
    // here since its a special case both in regards to comments and values
    if (param->getType() == ArConfigArg::FUNCTOR)
    {
      // put the comments in the file first
      sprintf(line, "; ");
      if (param->getDescription() != NULL &&
        strlen(param->getDescription()) > 0)
      {
        ArArgumentBuilder descr;
        descr.add(param->getDescription());
        for (unsigned int i = 0; i < descr.getArgc(); i++)
        {
          // see if we're over, if we are write this line out and start
          // the next one
          if (strlen(line) + strlen(descr.getArg(i)) > 78)
          {
            fprintf(file, "%s\n", line);
            sprintf(line, "; ");
            sprintf(line, "%s %s", line, descr.getArg(i));
          }
          // if its not the end of the line toss this word in
          else
          {
            sprintf(line, "%s %s", line, descr.getArg(i));
          }
        }
        // put the last line into the file
        fprintf(file, "%s\n", line);
      }
      // now put in the values
      argList = param->getArgsWithFunctor();
	  if (argList != NULL)
        for (argIt = argList->begin(); argIt != argList->end(); argIt++)
        {
          // if there's a space in the name then quote the param name
          if (strchr(param->getName(), ' ') != NULL ||
            strchr(param->getName(), '\t') != NULL)
            fprintf(file, "\"%s\" %s\n", param->getName(),
            (*argIt)->getFullString());
          else
            fprintf(file, "%s %s\n", param->getName(),
            (*argIt)->getFullString());
        }

	  // okay, we did it all, now continue
	  continue;
    }

    // if its a string holder just write the name without quotes
    // (since its the value not really the name)
    if (param->getType() == ArConfigArg::STRING_HOLDER)
    {
      fprintf(file, "%s %s\n", param->getName(), param->getString());
      // now put a blank line between params if we should
      if (!myNoBlanksBetweenParams)
        fprintf(file, "\n");
      continue;
    }

    // if there's a space in the name then quote the param name
    if (strchr(param->getName(), ' ') != NULL ||
      strchr(param->getName(), '\t') != NULL)
      sprintf(line, "\"%s\"", param->getName());
    else
      sprintf(line, "%s", param->getName());
    if (param->getType() == ArConfigArg::INT)
    {
      sprintf(line, "%s %d", line, param->getInt());
    }
    else if (param->getType() == ArConfigArg::DOUBLE)
    {
      sprintf(line, "%s %g", line, param->getDouble());
    }
    else if (param->getType() == ArConfigArg::STRING)
    {
      sprintf(line, "%s %s", line, param->getString());
    }
    else if (param->getType() == ArConfigArg::BOOL)
    {
      sprintf(line, "%s %s", line, param->getBool() ? "true" : "false");
    }
    else if (param->getType() == ArConfigArg::DESCRIPTION_HOLDER)
    {
      if (strlen(param->getDescription()) == 0)
      {
        fprintf(file, "\n");
        continue;
      }
    }
    else
    {
      ArLog::log(ArLog::Terse, "%sno argument type", myLogPrefix.c_str());
    }
    // configure our start of line part
    if (param->getType() == ArConfigArg::DESCRIPTION_HOLDER)
      sprintf(startLine, "; ");
    else
      sprintf(startLine, "%%-%ds;", startCommentColumn);
    // if our line is already longer then where we want to go put in
    // an extra space
    if (strlen(line) >= startCommentColumn)
      sprintf(line, "%-s ;", line);
    // if its not then just put the start in
    else
      sprintf(line, startLine, line);
    // if its an int we want to put in ranges if there are any
    if (param->getType() == ArConfigArg::INT)
    {
      if (param->getMinInt() != INT_MIN && param->getMaxInt() != INT_MAX)
      {
        sprintf(line, "%s range [%d, %d], ", line,
          param->getMinInt(), param->getMaxInt());
      }
      else if (param->getMinInt() != INT_MIN)
        sprintf(line, "%s minumum %d, ", line, param->getMinInt());
      else if (param->getMaxInt() != INT_MAX)
        sprintf(line, "%s maximum %d, ", line, param->getMaxInt());
    }
    if (param->getType() == ArConfigArg::DOUBLE)
    {
      if (param->getMinDouble() != -HUGE_VAL &&
        param->getMaxDouble() != HUGE_VAL)
        sprintf(line, "%s range [%g, %g], ", line, param->getMinDouble(),
        param->getMaxDouble());
      else if (param->getMinDouble() != -HUGE_VAL)
        sprintf(line, "%s minimum %g, ", line,
        param->getMinDouble());
      else if (param->getMaxDouble() != HUGE_VAL)
        sprintf(line, "%s Maximum %g, ", line,
        param->getMaxDouble());
    }

    // if we have a description to put in, put it in with word wrap
    if (param->getDescription() != NULL &&
      strlen(param->getDescription()) > 0)
    {
      ArArgumentBuilder descr;
      descr.add(param->getDescription());
      for (unsigned int i = 0; i < descr.getArgc(); i++)
      {
        // see if we're over, if we are write this line out and start
        // the next one
        if (strlen(line) + strlen(descr.getArg(i)) > 78)
        {
          fprintf(file, "%s\n", line);
          sprintf(line, startLine, "");
          sprintf(line, "%s %s", line, descr.getArg(i));
        }
        // if its not the end of the line toss this word in
        else
        {
          sprintf(line, "%s %s", line, descr.getArg(i));
        }
      }
      // put the last line into the file
      fprintf(file, "%s\n", line);
    }
    // if they're no description just end the line
    else
      fprintf(file, "%s\n", line);
    // if they have a config priority put that on its own line
    if (writePriorities)
    {
      sprintf(line, startLine, "");
      fprintf(file, "%s Priority: %s\n", line,
        ArPriority::getPriorityName(param->getConfigPriority()));
    }
    // now put a blank line between params if we should
    if (!myNoBlanksBetweenParams)
      fprintf(file, "\n");
  }
}

AREXPORT const char *ArConfig::getBaseDirectory(void) const
{
  return myBaseDirectory.c_str();
}

AREXPORT void ArConfig::setBaseDirectory(const char *baseDirectory)
{
  if (baseDirectory != NULL && strlen(baseDirectory) > 0)
    myBaseDirectory = baseDirectory;
  else
    myBaseDirectory = "";

  myParser.setBaseDirectory(baseDirectory);
}

AREXPORT const char *ArConfig::getFileName(void) const
{
  return myFileName.c_str();
}

/**
   After a file has been read all the way these processFileCBs are
   called in the priority (higher numbers first)... these are only
   called if there were no errors parsing the file or
   continueOnError was set to false when parseFile was called

   The functor should return true if the config parsed was good
   (parseFile will return true) false if the config parsed wasn't
   (parseFile will return false)

   @param functor the functor to call

   @param priority the functors are called in descending order, if two
   things have the same number the first one added is the first one
   called
**/
AREXPORT void ArConfig::addProcessFileCB(ArRetFunctor<bool> *functor,
					 int priority)
{
  myProcessFileCBList.insert(
	  std::pair<int, ProcessFileCBType *>(-priority,
					      new ProcessFileCBType(functor)));
}

/**
    Removes a processFileCB, see addProcessFileCB for details
 **/
AREXPORT void ArConfig::remProcessFileCB(ArRetFunctor<bool> *functor)
{
  std::multimap<int, ProcessFileCBType *>::iterator it;
  ProcessFileCBType *cb;

  for (it = myProcessFileCBList.begin(); it != myProcessFileCBList.end(); ++it)
  {
    if ((*it).second->haveFunctor(functor))
    {
      cb = (*it).second;
      myProcessFileCBList.erase(it);
      delete cb;
      remProcessFileCB(functor);
      return;
    }
  }
}

/**
   This function has a different name than addProcessFileCB just so
   that if you mean to get this function but have the wrong functor
   you'll get an error.  The rem's are the same though since that
   shouldn't matter.

   After a file has been read all the way these processFileCBs are
   called in the priority (higher numbers first)... these are only
   called if there were no errors parsing the file or
   continueOnError was set to false when parseFile was called

   The functor should return true if the config parsed was good
   (parseFile will return true) false if the config parsed wasn't
   (parseFile will return false)

   @param functor the functor to call (the char * and unsigned int
   should be used by the functor to put an error message into that
   buffer)

   @param priority the functors are called in descending order, if two
   things have the same number the first one added is the first one
   called
**/
AREXPORT void ArConfig::addProcessFileWithErrorCB(
	ArRetFunctor2<bool, char *, size_t> *functor,
	int priority)
{
  myProcessFileCBList.insert(
	  std::pair<int, ProcessFileCBType *>(-priority,
				      new ProcessFileCBType(functor)));
}

/**
    Removes a processFileCB, see addProcessFileCB for details
 **/
AREXPORT void ArConfig::remProcessFileCB(
	ArRetFunctor2<bool, char *, size_t> *functor)
{
  std::multimap<int, ProcessFileCBType *>::iterator it;
  ProcessFileCBType *cb;

  for (it = myProcessFileCBList.begin(); it != myProcessFileCBList.end(); ++it)
  {
    if ((*it).second->haveFunctor(functor))
    {
      cb = (*it).second;
      myProcessFileCBList.erase(it);
      delete cb;
      remProcessFileCB(functor);
    }
  }
}

AREXPORT bool ArConfig::callProcessFileCallBacks(bool continueOnErrors,
						 char *errorBuffer,
						 size_t errorBufferLen)
{
  bool ret = true;
  std::multimap<int, ProcessFileCBType *>::iterator it;
  ProcessFileCBType *callback;
  ArLog::LogLevel level = myProcessFileCallbacksLogLevel;

  // reset our section to nothing again
  mySection = "";

  // in windows, can't simply declare an array of errorBufferLen -- so
  // allocate one.

  // empty the buffer, we're only going to put the first error in it
  if (errorBuffer != NULL && errorBufferLen > 0)
    errorBuffer[0] = '\0';

  ArLog::log(level, "%sProcessing file", myLogPrefix.c_str());
  for (it = myProcessFileCBList.begin();
       it != myProcessFileCBList.end();
       ++it)
  {
    callback = (*it).second;
    if (callback->getName() != NULL && callback->getName()[0] != '\0')
      ArLog::log(level, "%sProcessing functor '%s' (%d)",
                 myLogPrefix.c_str(),
		             callback->getName(), -(*it).first);
    else
      ArLog::log(level, "%sProcessing unnamed functor (%d)",
                 myLogPrefix.c_str(),
                 -(*it).first);
    if (!(*it).second->call(errorBuffer, errorBufferLen))
    {
      //printf("# %s\n", scratchBuffer);

      // if there is an error buffer and it got filled get rid of our
      // pointer to it
      if (errorBuffer != NULL && errorBuffer[0] != '\0')
      {
	errorBuffer = NULL;
	errorBufferLen = 0;
      }
      ret = false;
      if (!continueOnErrors)
      {
	if (callback->getName() != NULL && callback->getName()[0] != '\0')
	  ArLog::log(ArLog::Normal, "ArConfig: Failed, stopping because the '%s' process file callback failed",
		     callback->getName());
	else
	  ArLog::log(ArLog::Normal, "ArConfig: Failed, stopping because unnamed process file callback failed");
	break;
      }
      else
      {
	if (callback->getName() != NULL && callback->getName()[0] != '\0')
	  ArLog::log(ArLog::Normal, "ArConfig: Failed but continuing, the '%s' process file callback failed",
		     callback->getName());
	else
	  ArLog::log(ArLog::Normal, "ArConfig: Failed but continuing, an unnamed process file callback failed");
      }

    }
  }
  if (ret || continueOnErrors)
  {
    ArLog::log(level, "%sProcessing with own processFile",
                 myLogPrefix.c_str());
    if (!processFile())
      ret = false;
  }
  ArLog::log(level, "%sDone processing file, ret is %s", myLogPrefix.c_str(),
	     ArUtil::convertBool(ret));

  return ret;
}


AREXPORT std::list<ArConfigSection *> *ArConfig::getSections(void)
{
  return &mySections;
}

AREXPORT void ArConfig::setNoBlanksBetweenParams(bool noBlanksBetweenParams)
{
  myNoBlanksBetweenParams = noBlanksBetweenParams;
}

AREXPORT bool ArConfig::getNoBlanksBetweenParams(void)
{
  return myNoBlanksBetweenParams;
}

/**
   This argument parser the arguments in to check for parameters of
   this name, note that ONLY the first parameter of this name will be
   used, so if you have duplicates only the first one will be set.
 **/
AREXPORT void ArConfig::useArgumentParser(ArArgumentParser *parser)
{
  myArgumentParser = parser;
}

AREXPORT bool ArConfig::parseArgumentParser(ArArgumentParser *parser,
					    bool continueOnError,
					    char *errorBuffer,
					    size_t errorBufferLen)
{
  std::list<ArConfigSection *>::iterator sectionIt;
  std::list<ArConfigArg>::iterator paramIt;
  ArConfigSection *section = NULL;
  ArConfigArg *param = NULL;
  std::list<ArConfigArg> *params = NULL;

  bool ret;
  size_t i;
  std::string strArg;
  std::string strUndashArg;
  ArArgumentBuilder builder;
  bool plainMatch=false;

  for (i = 0; i < parser->getArgc(); i++)
  {
    if (parser->getArg(i) == NULL)
    {
      ArLog::log(ArLog::Terse, "%sset up wrong (parseArgumentParser broken).",
                 myLogPrefix.c_str());
      if (errorBuffer != NULL)
        strncpy(errorBuffer,
        "ArConfig set up wrong (parseArgumentParser broken).",
        errorBufferLen);
      return false;
    }
    strArg = parser->getArg(i);
    if (parser->getArg(i)[0] == '-')
      strUndashArg += &parser->getArg(i)[1];
    else
      strUndashArg = "";
    //printf("normal %s undash %s\n", strArg.c_str(), strUndashArg.c_str());
    for (sectionIt = mySections.begin();
         sectionIt != mySections.end();
         sectionIt++)
    {
      section = (*sectionIt);
      params = section->getParams();

      for (paramIt = params->begin(); paramIt != params->end(); paramIt++)
      {
        param = &(*paramIt);
        /*
        printf("### normal %s undash %s %d %d\n", strArg.c_str(),
        strUndashArg.c_str(),
        ArUtil::strcasecmp(param->getName(), strArg),
        ArUtil::strcasecmp(param->getName(), strUndashArg));
        */
        if (strlen(param->getName()) > 0 &&
          ((plainMatch = ArUtil::strcasecmp(param->getName(),strArg)) == 0 ||
          ArUtil::strcasecmp(param->getName(), strUndashArg) == 0))
        {
          if (plainMatch == 0)
            builder.setExtraString(strArg.c_str());
          else
            builder.setExtraString(strUndashArg.c_str());
          if (i+1 < parser->getArgc())
          {
            builder.add(parser->getArg(i+1));
            parser->removeArg(i+1);
          }
          parser->removeArg(i);

          // set us to use the section this is in and then parse the argument
          std::string oldSection = mySection;
          bool oldSectionBroken = mySectionBroken;
          bool oldSectionIgnored = mySectionIgnored;
          bool oldUsingSections = myUsingSections;
          bool oldDuplicateParams = myDuplicateParams;
          mySection = section->getName();
          mySectionBroken = false;
          mySectionIgnored = false; // ?? TODO
          myUsingSections = true;
          myDuplicateParams = false;
          ret = parseArgument(&builder, errorBuffer, errorBufferLen);
          mySection = oldSection;
          mySectionBroken = oldSectionBroken;
          mySectionIgnored = oldSectionIgnored;
          myUsingSections = oldUsingSections;
          myDuplicateParams = oldDuplicateParams;

          // if we parsed the argument right or are continuing on
          // errors call ourselves again (so we don't hose iterators up above)
          if (ret || continueOnError)
          {
            //printf("Ret %s\n", ArUtil::convertBool(ret));
            return ret && parseArgumentParser(parser, continueOnError,
              errorBuffer, errorBufferLen);
          }
          else
            return false;
        }
      }
    }
  }

  return true;
}

AREXPORT ArConfigSection *ArConfig::findSection(const char *sectionName) const
{
  ArConfigSection *section = NULL;
  ArConfigSection *tempSection = NULL;

  for (std::list<ArConfigSection *>::const_iterator sectionIt = mySections.begin();
       sectionIt != mySections.end();
       sectionIt++)
  {
    tempSection = (*sectionIt);
    if (ArUtil::strcasecmp(tempSection->getName(), sectionName) == 0)
    {
      section = tempSection;
    }
  }
  return section;

} // end method findSection

void ArConfig::copySectionsToParse(std::list<std::string> *from)
{
  mySectionsToParse = NULL;
  if (from != NULL) {
    mySectionsToParse = new std::list<std::string>();
    for (std::list<std::string>::const_iterator spIter = from->begin();
         spIter != from->end();
         spIter++) {
      mySectionsToParse->push_back(*spIter);
    } // end for each section to parse
  } // end if copy sections to parse

} // end method copySectionsToParse



AREXPORT void ArConfig::clearAllValueSet(void)
{
  std::list<ArConfigSection *> *sections;
  ArConfigSection *section;
  std::list<ArConfigSection *>::iterator sectionIt;

  std::list<ArConfigArg> *params;
  ArConfigArg *param;
  std::list<ArConfigArg>::iterator paramIt;

  sections = getSections();
  for (sectionIt = sections->begin();
       sectionIt != sections->end();
       sectionIt++)
  {
    section = (*sectionIt);
    params = section->getParams();
    for (paramIt = params->begin(); paramIt != params->end(); paramIt++)
    {
      param = &(*paramIt);
      param->clearValueSet();
    }
  }
}

AREXPORT void ArConfig::removeAllUnsetValues(void)
{
  std::list<ArConfigSection *> *sections;
  ArConfigSection *section;
  std::list<ArConfigSection *>::iterator sectionIt;

  std::list<ArConfigArg> *params;
  ArConfigArg *param;
  std::list<ArConfigArg>::iterator paramIt;
  std::list<std::list<ArConfigArg>::iterator> removeParams;
  std::list<std::list<ArConfigArg>::iterator>::iterator removeParamsIt;

  sections = getSections();
  for (sectionIt = sections->begin();
       sectionIt != sections->end();
       sectionIt++)
  {
    section = (*sectionIt);
    params = section->getParams();
    for (paramIt = params->begin(); paramIt != params->end(); paramIt++)
    {
      param = &(*paramIt);
      if (!param->isValueSet() &&
	  param->getType() != ArConfigArg::SEPARATOR &&
	  param->getType() != ArConfigArg::STRING_HOLDER &&
	  param->getType() != ArConfigArg:: DESCRIPTION_HOLDER)
      {
	removeParams.push_back(paramIt);
      }
    }
    while ((removeParamsIt = removeParams.begin()) != removeParams.end())
    {
      ArLog::log(ArLog::Verbose,
		 "%s:removeAllUnsetValues: Removing %s:%s",
     myLogPrefix.c_str(),
		 section->getName(), (*(*removeParamsIt)).getName());
      section->getParams()->erase((*removeParamsIt));
      removeParams.pop_front();
    }
  }
}


AREXPORT ArConfigSection::ArConfigSection(const char *name,
					  const char *comment)
{
  myName = name;
  if (comment != NULL)
    myComment = comment;
  else
    myComment = "";
  myFlags = new ArArgumentBuilder(512, '|');
}


AREXPORT ArConfigSection::ArConfigSection(const ArConfigSection &section)
{
  myName = section.myName;
  myComment = section.myComment;
  myFlags = new ArArgumentBuilder(512, '|');
  myFlags->add(section.myFlags->getFullString());

  for (std::list<ArConfigArg>::const_iterator it = section.myParams.begin();
       it != section.myParams.end();
       it++)
  {
    myParams.push_back(*it);
  }
}

AREXPORT ArConfigSection &ArConfigSection::operator=(const ArConfigSection &section)
{
  if (this != &section)
  {

    myName = section.getName();
    myComment = section.getComment();
    delete myFlags;
    myFlags = new ArArgumentBuilder(512, '|');
    myFlags->add(section.myFlags->getFullString());

    for (std::list<ArConfigArg>::const_iterator it = section.myParams.begin();
	 it != section.myParams.end();
	 it++)
    {
      myParams.push_back(*it);
    }
  }
  return *this;
}



AREXPORT ArConfigSection::~ArConfigSection()
{
  delete myFlags;
}



AREXPORT ArConfigArg *ArConfigSection::findParam(const char *paramName)
{
  ArConfigArg *param = NULL;
  ArConfigArg *tempParam = NULL;

  for (std::list<ArConfigArg>::iterator pIter = myParams.begin();
       pIter != myParams.end();
       pIter++)
  {
    // TODO: Does this do what I'm hoping it does?
    tempParam = &(*pIter);
    // ignore string holders
    if (tempParam->getType() == ArConfigArg::STRING_HOLDER)
      continue;
    if (ArUtil::strcasecmp(tempParam->getName(), paramName) == 0)
    {
      param = tempParam;
    }
  }
  return param;

} // end method findParam


AREXPORT bool ArConfigSection::remStringHolder(const char *paramName)
{
  ArConfigArg *tempParam = NULL;

  for (std::list<ArConfigArg>::iterator pIter = myParams.begin();
       pIter != myParams.end();
       pIter++)
  {
    // TODO: Does this do what I'm hoping it does?
    tempParam = &(*pIter);
    // pay attention to only string holders
    if (tempParam->getType() != ArConfigArg::STRING_HOLDER ||
	paramName == NULL || paramName[0] == '\0')
      continue;
    if (ArUtil::strcasecmp(tempParam->getName(), paramName) == 0)
    {
      myParams.erase(pIter);
      remStringHolder(paramName);
      return true;
    }
  }
  return false;
}

AREXPORT bool ArConfigSection::hasFlag(const char *flag) const
{
  size_t i;
  for (i = 0; i < myFlags->getArgc(); i++)
  {
    if (strcmp(myFlags->getArg(i), flag) == 0)
    {
      return true;
    }
  }
  return false;
}

AREXPORT bool ArConfigSection::remFlag(const char *flag)
{
  size_t i;
  for (i = myFlags->getArgc(); i < myFlags->getArgc(); i++)
  {
    if (strcmp(myFlags->getArg(i), flag) == 0)
    {
      myFlags->removeArg(i);
      return true;
    }
  }

  return false;
}


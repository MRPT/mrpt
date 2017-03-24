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
#include "ArArg.h"
#include "ArLog.h"
#include "ArArgumentBuilder.h"

AREXPORT ArArg::ArArg()
{
  myType = INVALID;
  myName = "";
  myDescription = "";
  clear();
}

AREXPORT ArArg::ArArg(const char * name, int *pointer, 
		      const char * description, int minInt, int maxInt) 
{ 
  myType = INT;
  myName = name; 
  myDescription = description;
  clear();
  myMinInt = minInt;
  myMaxInt = maxInt;
  myIntPointer = pointer;
}

AREXPORT ArArg::ArArg(const char * name, double *pointer,
		      const char * description, double minDouble, 
		      double maxDouble) 
{ 
  myType = DOUBLE;
  myName = name; 
  myDescription = description;
  clear();
  myMinDouble = minDouble;
  myMaxDouble = maxDouble;
  myDoublePointer = pointer;
}

AREXPORT ArArg::ArArg(const char * name, bool *pointer, 
		      const char * description) 
{ 
  myType = BOOL;
  myName = name; 
  myDescription = description;
  clear();
  myBoolPointer = pointer;
}

AREXPORT ArArg::ArArg(const char * name, ArPose *pointer, 
		      const char * description) 
{ 
  myType = POSE;
  myName = name; 
  myDescription = description;
  clear();
  myPosePointer = pointer;
}

AREXPORT ArArg::ArArg(const char * name, char *pointer, 
		      const char * description, size_t maxStrLen) 
{ 
  myType = STRING;
  myName = name; 
  myDescription = description;
  clear();
  myStringPointer = pointer;
  myMaxStrLen = maxStrLen;
}

/**
   This constructor is for the functor type of argument, this is for
   cases that need to be complicated and have more than one argument
   value per name, such as the sonar in a config file, and where this data
   needs to be used to construct compound data structures rather than
   single variables.

   @param name argument name
   @param description argument description
   
   @param setFunctor when an argument is read it is passed to this
   functor which should set up whatever it needs to from the data
   
   @param getFunctor since parameter files need to be written too,
   this get functor will get a list of strings to be written to the file
**/
AREXPORT ArArg::ArArg(const char *name, 
		      ArRetFunctor1<bool, ArArgumentBuilder *> *setFunctor, 
	      ArRetFunctor<const std::list<ArArgumentBuilder *> *> *getFunctor,
		      const char *description)
{
  myType = FUNCTOR;
  myName = name;
  myDescription = description;
  clear();
  mySetFunctor = setFunctor;
  myGetFunctor = getFunctor;
}

AREXPORT ArArg::ArArg(const char * description)
{ 
  myType = DESCRIPTION_HOLDER;
  myDescription = description;
  clear();
}

AREXPORT ArArg::ArArg(const ArArg & arg) 
{
  myType = arg.myType;
  myName = arg.myName;
  myDescription = arg.myDescription;
  myIntPointer = arg.myIntPointer;
  myDoublePointer = arg.myDoublePointer;
  myPosePointer = arg.myPosePointer;
  myBoolPointer = arg.myBoolPointer;
  myStringPointer = arg.myStringPointer;
  myMinInt = arg.myMinInt;
  myMaxInt = arg.myMaxInt;
  myMinDouble = arg.myMinDouble;
  myMaxDouble = arg.myMaxDouble;
  myMaxStrLen = arg.myMaxStrLen;
  mySetFunctor = arg.mySetFunctor;
  myGetFunctor = arg.myGetFunctor;
  myConfigPrioritySet = arg.myConfigPrioritySet;
  myConfigPriority = arg.myConfigPriority;
}

AREXPORT ArArg &ArArg::operator=(const ArArg & arg) 
{
	if (this != &arg) {
		myType = arg.myType;
		myName = arg.myName;
		myDescription = arg.myDescription;
		myIntPointer = arg.myIntPointer;
		myDoublePointer = arg.myDoublePointer;
		myPosePointer = arg.myPosePointer;
		myBoolPointer = arg.myBoolPointer;
		myStringPointer = arg.myStringPointer;
		myMinInt = arg.myMinInt;
		myMaxInt = arg.myMaxInt;
		myMinDouble = arg.myMinDouble;
		myMaxDouble = arg.myMaxDouble;
		myMaxStrLen = arg.myMaxStrLen;
		mySetFunctor = arg.mySetFunctor;
		myGetFunctor = arg.myGetFunctor;
		myConfigPrioritySet = arg.myConfigPrioritySet;
		myConfigPriority = arg.myConfigPriority;
	}
	return *this;
}


AREXPORT ArArg::~ArArg()
{
}

AREXPORT void ArArg::clear(void)
{
  myIntPointer = NULL;
  myDoublePointer = NULL;
  myBoolPointer = NULL;
  myPosePointer = NULL;
  myStringPointer = NULL;
  myMinInt = INT_MIN;
  myMaxInt = INT_MAX;
  myMinDouble = -HUGE_VAL;
  myMaxDouble = HUGE_VAL;
  myMaxStrLen = 0;
  mySetFunctor = NULL;
  myGetFunctor = NULL;  
  myConfigPrioritySet = false;
  myConfigPriority = ArPriority::NORMAL;
}

/**
   @see INVALID
   @see INT
   @see DOUBLE
   @see BOOL
   @see POSE */
AREXPORT ArArg::Type ArArg::getType(void) const
{
  return myType;
}

AREXPORT int ArArg::getMinInt(void) const
{
  return myMinInt;
}

AREXPORT int ArArg::getMaxInt(void) const
{
  return myMaxInt;
}

AREXPORT double ArArg::getMinDouble(void) const
{
  return myMinDouble;
}

AREXPORT double ArArg::getMaxDouble(void) const
{
  return myMaxDouble;
}

AREXPORT const char *ArArg::getName(void) const
{
  return myName.c_str();
}

AREXPORT const char *ArArg::getDescription(void) const
{
  return myDescription.c_str();
}

AREXPORT int ArArg::getInt(void) const
{ 
  if (myIntPointer != NULL)
    return *myIntPointer;
  else
    return 0;
}

AREXPORT double ArArg::getDouble(void) const 
{
  if (myDoublePointer != NULL)
    return *myDoublePointer; 
  else
    return 0;
}

AREXPORT bool ArArg::getBool(void) const
{
  if (myBoolPointer != NULL)
    return *myBoolPointer;
  else
    return false;
}

AREXPORT const char *ArArg::getString(void) const
{
  if (myStringPointer != NULL)
    return myStringPointer;
  else
    return NULL;
}

AREXPORT ArPose ArArg::getPose(void) const
{
  ArPose pose;
  if (myPosePointer != NULL)
    return *myPosePointer;
  else
    return pose;
}

AREXPORT const std::list<ArArgumentBuilder *> *ArArg::getArgsWithFunctor(void) const
{
  if (myGetFunctor == NULL)
    return NULL;
  else
    return myGetFunctor->invokeR();
}

AREXPORT bool ArArg::setInt(int val)
{
  if (val < myMinInt)
  {
    ArLog::log(ArLog::Normal, "ArArg of %s: setInt value %d below range [%d, %d]", getName(), val, myMinInt, myMaxInt);
    return false;
  }
  if (val > myMaxInt)
  {
    ArLog::log(ArLog::Normal, "ArArg of %s: setInt value %d above range [%d, %d]", getName(), val, myMinInt, myMaxInt);
    return false;
  }
  if (myIntPointer == NULL)
  {
    ArLog::log(ArLog::Normal, "ArArg of %s: setInt called with NULL int pointer.", getName());
  }
  // if we got to here we're good
  *myIntPointer = val;
  return true;
}

AREXPORT bool ArArg::setDouble(double val)
{ 
  if (val < myMinDouble)
  {
    ArLog::log(ArLog::Normal, "ArArg of %s: setDouble value %g below range [%g, %g]", getName(), val, myMinDouble, myMaxDouble);
    return false;
  }
  if (val > myMaxDouble)
  {
    ArLog::log(ArLog::Normal, "ArArg of %s: setDouble value %g above range [%g, %g]", getName(), val, myMinDouble, myMaxDouble);
    return false;
  }
  if (myDoublePointer == NULL)
  {
    ArLog::log(ArLog::Normal, "ArArg of %s: setDouble called with NULL pointer.", getName());
    return false;
  }
  // if we got to here we're good
  *myDoublePointer = val;
  return true;
}


AREXPORT bool ArArg::setBool(bool val)
{
  if (myBoolPointer == NULL)
  {
    ArLog::log(ArLog::Normal, "ArArg of %s: setBool called with NULL pointer.", getName());
    return false;
  }
  *myBoolPointer = val;
  return true;
}

AREXPORT bool ArArg::setString(const char *str)
{
  size_t len;
  if (myStringPointer == NULL)
  {
    ArLog::log(ArLog::Normal, "ArArg of %s: setString called with NULL pointer.", getName());
    return false;
  }
  // this is >= so that if it wouldn't have room with NULL that's
  // taken care of too
  if ((len = strlen(str)) >= myMaxStrLen)
  {
    ArLog::log(ArLog::Normal, "ArArg of %s: setString called with argument %d long, when max length is %d.", getName(), len, myMaxStrLen);
    return false;
  }
  strcpy(myStringPointer, str);
  return true;
}

AREXPORT bool ArArg::setPose(ArPose pose)
{
  if (myPosePointer == NULL)
  {
    ArLog::log(ArLog::Normal, "ArArg of %s: setPose called with NULL pointer.", getName());
    return false;
  }
  *myPosePointer = pose;
  return true;

}

AREXPORT bool ArArg::setArgWithFunctor(ArArgumentBuilder *argument)
{
  if (mySetFunctor == NULL)
  {
    ArLog::log(ArLog::Normal, "ArArg of %s: setArgWithFunctor called with NULL pointer.", getName());
    return false;
  }
  return mySetFunctor->invokeR(argument);
}


AREXPORT void ArArg::log(void) const
{
  std::list<ArArgumentBuilder *>::const_iterator it;
  const std::list<ArArgumentBuilder *> *argList;

  switch (getType()) 
  {
  case ArArg::INVALID:
    ArLog::log(ArLog::Terse, 
	       "\tType: %10s.  This argument was not created properly.", 
	       "invalid");
  case ArArg::INT:
    ArLog::log(ArLog::Terse, "\tType: %10s name: %12s value: %d", "int", 
	       getName(), getInt());
    if (strlen(getDescription()) != 0)
      ArLog::log(ArLog::Terse, "\t\tDescription: %s",
		 getDescription());
    break;
  case ArArg::DOUBLE:
    ArLog::log(ArLog::Terse, "\tType: %10s name: %12s value: %f", "double",
	       getName(), getDouble());
    if (strlen(getDescription()) != 0)
      ArLog::log(ArLog::Terse, "\t\tDescription: %s",
		 getDescription());
    break; 
  case ArArg::STRING:
    ArLog::log(ArLog::Terse, "\tType: %10s name: %12s value: %s", "string", 
               getName(), getString());
    if (strlen(getDescription()) != 0)
      ArLog::log(ArLog::Terse, "\t\tDescription: %s",
                 getDescription());
    break;
  case ArArg::BOOL:
    ArLog::log(ArLog::Terse, "\tType: %10s name: %12s value: %d", "bool",
	       getName(), getBool());
    if (strlen(getDescription()) != 0)
      ArLog::log(ArLog::Terse, "\t\tDescription: %s",
		 getDescription());
    break;
  case ArArg::POSE:
    ArLog::log(ArLog::Terse, "\tType: %10s name: %12s value: (%.1f %.1f %.1f)",
	       "pose", getName(), getPose().getX(), getPose().getY(),
	       getPose().getTh());
    if (strlen(getDescription()) != 0)
      ArLog::log(ArLog::Terse, "\t\tDescription: %s",
		 getDescription());
    break;
  case ArArg::FUNCTOR:
    ArLog::log(ArLog::Terse, "\tType: %10s name: %12s", 
	       "functor", getName(), getPose().getX(), getPose().getY(),
	       getPose().getTh());
    if (strlen(getDescription()) != 0)
      ArLog::log(ArLog::Terse, "\t\tDescription: %s",
		 getDescription());
    ArLog::log(ArLog::Terse, "\t\tValues:");
    argList = myGetFunctor->invokeR();
    for (it = argList->begin(); it != argList->end(); it++)
      ArLog::log(ArLog::Terse, "\t\t\t%s", (*it)->getFullString());
    break;
  case ArArg::DESCRIPTION_HOLDER:
    ArLog::log(ArLog::Terse, "\tType: %20s Description: %s", 
	       "description_holder", getDescription());

  default:
    ArLog::log(ArLog::Terse, 
	       "\tType: %10s.  This type doesn't have a case in ArArg::print.",
	       "unknown");
    break;
  }

  if (myConfigPrioritySet)
    ArLog::log(ArLog::Terse, "\t\tPriority: %s", 
	       ArPriority::getPriorityName(myConfigPriority));
}

/**
   If this is true then the config priority is set and you can use
   getConfigPriority.
**/
AREXPORT bool ArArg::getConfigPrioritySet(void) const
{
  return myConfigPrioritySet;
}

/**
   The priority of this argument when used in ArConfig.
 **/
AREXPORT ArPriority::Priority ArArg::getConfigPriority(void) const
{
  return myConfigPriority;
}

/**
   The priority of this argument when used in ArConfig.
 **/

AREXPORT void ArArg::setConfigPriority(ArPriority::Priority priority)
{
  myConfigPriority = priority;
  myConfigPrioritySet = true;
}

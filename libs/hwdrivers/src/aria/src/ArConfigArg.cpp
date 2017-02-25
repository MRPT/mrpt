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
#include "ArConfigArg.h"
#include "ArLog.h"
#include "ArArgumentBuilder.h"

AREXPORT ArConfigArg::ArConfigArg()
{
  clear();
}

/** @swigomit */
AREXPORT ArConfigArg::ArConfigArg(const char * name, int *pointer,
		      const char * description, int minInt, int maxInt)
{
  clear();
  set(INT, name, description);

  myIntType = INT_INT;
  myMinInt = minInt;
  myMaxInt = maxInt;
  myIntPointer = pointer;
}

/** @swigomit */
AREXPORT ArConfigArg::ArConfigArg(const char * name, short *pointer,
		      const char * description, int minInt, int maxInt)

{
  clear();
  set(INT, name, description);

  myIntType = INT_SHORT;
  myMinInt = minInt;
  myMaxInt = maxInt;
  myIntShortPointer = pointer;
}

/** @swigomit */
AREXPORT ArConfigArg::ArConfigArg(const char * name, unsigned short *pointer,
		      const char * description, int minInt, int maxInt)
{
  clear();
  set(INT, name, description);

  myIntType = INT_UNSIGNED_SHORT;
  myMinInt = minInt;
  myMaxInt = maxInt;
  myIntUnsignedShortPointer = pointer;
}

/** @swigomit */
AREXPORT ArConfigArg::ArConfigArg(const char * name, unsigned char *pointer,
		      const char * description, int minInt, int maxInt)
{
  clear();
  set(INT, name, description);

  myIntType = INT_UNSIGNED_CHAR;
  myMinInt = minInt;
  myMaxInt = maxInt;
  myIntUnsignedCharPointer = pointer;
}

/** @swigomit */
AREXPORT ArConfigArg::ArConfigArg(const char * name, double *pointer,
		      const char * description, double minDouble,
		      double maxDouble)
{
  clear();
  set(DOUBLE, name, description);

  myMinDouble = minDouble;
  myMaxDouble = maxDouble;
  myDoublePointer = pointer;
}

/** @swigomit */
AREXPORT ArConfigArg::ArConfigArg(const char * name, bool *pointer,
		      const char * description)

{
  clear();
  set(BOOL, name, description);

  myBoolPointer = pointer;
}


/** @swigomit Use ArConfigArg_Int subclass instead. */
AREXPORT ArConfigArg::ArConfigArg(const char * name, int val,
		      const char * description, int minInt, int maxInt)
{
  clear();
  set(INT, name, description);

  myIntType = INT_INT;
  myMinInt = minInt;
  myMaxInt = maxInt;
  myIntPointer = new int;
  *myIntPointer = val;
  myOwnPointedTo = true;
}

/** @swigomit Use ArConfigArg_Double subclass instead. */
AREXPORT ArConfigArg::ArConfigArg(const char * name, double val,
		      const char * description, double minDouble,
		      double maxDouble)
{
  clear();
  set(DOUBLE, name, description);

  myMinDouble = minDouble;
  myMaxDouble = maxDouble;
  myDoublePointer = new double;
  *myDoublePointer = val;
  myOwnPointedTo = true;
}

/** @swigomit Use ArConfigArg_Bool subclass instead. */
AREXPORT ArConfigArg::ArConfigArg(const char * name, bool val,
		      const char * description)
{
  clear();
  set(BOOL, name, description);

  myBoolPointer = new bool;
  *myBoolPointer = val;
  myOwnPointedTo = true;
}

/**
 * This constructor can accept both an already-allocated string,
 * or ArConfigArg can to the memory managment itself (reallocation
 * and finally deletion). If @a maxStrLen is 0, then ArConfigArg will
 * do its own memory management, with the contents of @a str copied
 * as the initial value of the internally held string. Otherwise,
 * @a str must point to an allocated string, with its size given by @a
 * maxStrLen.
 *
 *  @swigomit Use ArConfigArg_String subclass instead (which has no maxStrLen parameter in its constructor)
*/
AREXPORT ArConfigArg::ArConfigArg(const char * name, char *str,
		      const char * description, size_t maxStrLen)
{
  clear();
  set(STRING, name, description);

  if (maxStrLen == 0)
  {
    myUsingOwnedString = true;
    myString = str;
  }
  else
  {
    myStringPointer = str;
    myMaxStrLen = maxStrLen;
  }
}

/**
   This constructor is for the functor type of argument, this is for
   cases that need to be complicated and have more than one argument
   per name... such as the sonar in a config file.  Where this data
   needs to be used to construct internal data structures.

   @param name Name of this argument

   @param description Description of the purpose of this argument

   @param setFunctor When an argument is read it is passed to this
   functor in an ArArgumentBuilder object. The functor should return
   false if there is an error or problem handling the argument, or
   true otherwise.

   @param getFunctor Since configuration needs to be serialized to
   save files on disk or send data over the network etc., this
   functor will be called to get a list of strings to represent this
   argument and its value as text in the file etc.
**/
AREXPORT ArConfigArg::ArConfigArg(const char *name,
		      ArRetFunctor1<bool, ArArgumentBuilder *> *setFunctor,
	      ArRetFunctor<const std::list<ArArgumentBuilder *> *> *getFunctor,
		      const char *description)
{
  clear();
  set(FUNCTOR, name, description);

  mySetFunctor = setFunctor;
  myGetFunctor = getFunctor;
}

AREXPORT ArConfigArg::ArConfigArg(const char * str, Type type)
{
  clear();
  if (type == DESCRIPTION_HOLDER)
  {
    myType = DESCRIPTION_HOLDER;
    myDescription = str;
  }
  else
  {
    ArLog::log(ArLog::Terse, "ArConfigArg: Bad type %d for '%s'", type, str);
  }
}

/**
 * This constructor is useful for creating separators within a config
 * section.
**/
AREXPORT ArConfigArg::ArConfigArg(Type type)
{
  clear();
  set(type, "", "");
}

AREXPORT ArConfigArg::ArConfigArg(const char *name, const char *str)
{
  clear();
  set(STRING_HOLDER, name, "");
  myUsingOwnedString = true;
  myString = str;
}

AREXPORT ArConfigArg::ArConfigArg(const ArConfigArg & arg)
{
  copy(arg);
}

AREXPORT ArConfigArg &ArConfigArg::operator=(const ArConfigArg & arg)
{
  if (this != &arg)
  {
    copy(arg);
  }
  return *this;
}

void ArConfigArg::copy(const ArConfigArg &arg)
{
  clear();
  set(arg.myType,
      arg.myName.c_str(),
      arg.myDescription.c_str());

  myIntType = arg.myIntType;
  myOwnPointedTo = arg.myOwnPointedTo;
  if (arg.myOwnPointedTo && arg.myIntPointer != NULL)
  {
    myIntPointer = new int;
    *myIntPointer = *arg.myIntPointer;
  }
  else
  {
    myIntPointer = arg.myIntPointer;
  }
  if (arg.myOwnPointedTo && arg.myIntShortPointer != NULL)
  {
    myIntShortPointer = new short;
    *myIntShortPointer = *arg.myIntShortPointer;
  }
  else
  {
    myIntShortPointer = arg.myIntShortPointer;
  }
  if (arg.myOwnPointedTo && arg.myIntUnsignedShortPointer != NULL)
  {
    myIntUnsignedShortPointer = new unsigned short;
    *myIntUnsignedShortPointer = *arg.myIntUnsignedShortPointer;
  }
  else
  {
    myIntUnsignedShortPointer = arg.myIntUnsignedShortPointer;
  }
  if (arg.myOwnPointedTo && arg.myIntUnsignedCharPointer != NULL)
  {
    myIntUnsignedCharPointer = new unsigned char;
    *myIntUnsignedCharPointer = *arg.myIntUnsignedCharPointer;
  }
  else
  {
    myIntUnsignedCharPointer = arg.myIntUnsignedCharPointer;
  }
  if (arg.myOwnPointedTo && arg.myDoublePointer != NULL)
  {
    myDoublePointer = new double;
    *myDoublePointer = *arg.myDoublePointer;
  }
  else
  {
    myDoublePointer = arg.myDoublePointer;
  }
  if (arg.myOwnPointedTo && arg.myBoolPointer != NULL)
  {
    myBoolPointer = new bool;
    *myBoolPointer = *arg.myBoolPointer;
  }
  else
  {
    myBoolPointer = arg.myBoolPointer;
  }
  myStringPointer = arg.myStringPointer;
  myMinInt = arg.myMinInt;
  myMaxInt = arg.myMaxInt;
  myMinDouble = arg.myMinDouble;
  myMaxDouble = arg.myMaxDouble;
  myMaxStrLen = arg.myMaxStrLen;
  mySetFunctor = arg.mySetFunctor;
  myGetFunctor = arg.myGetFunctor;
  myUsingOwnedString = arg.myUsingOwnedString;
  myString = arg.myString;
  myConfigPriority = arg.myConfigPriority;
  myIgnoreBounds = arg.myIgnoreBounds;
  myDisplayHint = arg.myDisplayHint;
}

AREXPORT ArConfigArg::~ArConfigArg()
{
  clear();
}

void ArConfigArg::clear(void)
{
  set(INVALID, "", "");

  myIntType = INT_NOT;
  myOwnPointedTo = false;
  if (myOwnPointedTo && myIntPointer != NULL)
    delete myIntPointer;
  myIntPointer = NULL;
  if (myOwnPointedTo && myIntShortPointer != NULL)
    delete myIntShortPointer;
  myIntShortPointer = NULL;
  if (myOwnPointedTo && myIntUnsignedShortPointer != NULL)
    delete myIntUnsignedShortPointer;
  myIntUnsignedShortPointer = NULL;
  if (myOwnPointedTo && myIntUnsignedCharPointer != NULL)
    delete myIntUnsignedCharPointer;
  myIntUnsignedCharPointer = NULL;
  if (myOwnPointedTo && myDoublePointer != NULL)
    delete myDoublePointer;
  myDoublePointer = NULL;
  if (myOwnPointedTo && myBoolPointer != NULL)
    delete myBoolPointer;
  myBoolPointer = NULL;
  myStringPointer = NULL;
  myUsingOwnedString = false;
  myString = "";
  myMinInt = INT_MIN;
  myMaxInt = INT_MAX;
  myMinDouble = -HUGE_VAL;
  myMaxDouble = HUGE_VAL;
  myMaxStrLen = 0;
  mySetFunctor = NULL;
  myGetFunctor = NULL;
  myConfigPriority = ArPriority::NORMAL;
  myIgnoreBounds = false;
  myDisplayHint = "";
  myValueSet = false;
}

void ArConfigArg::set(ArConfigArg::Type type,
                      const char *name,
                      const char *description)
{
  myType = type;
  myName = name;
  myDescription = description;
}

/**
   @see INVALID
   @see INT
   @see DOUBLE
   @see BOOL
   @see POSE */
AREXPORT ArConfigArg::Type ArConfigArg::getType(void) const
{
  return myType;
}

AREXPORT int ArConfigArg::getMinInt(void) const
{
  return myMinInt;
}

AREXPORT int ArConfigArg::getMaxInt(void) const
{
  return myMaxInt;
}

AREXPORT double ArConfigArg::getMinDouble(void) const
{
  return myMinDouble;
}

AREXPORT double ArConfigArg::getMaxDouble(void) const
{
  return myMaxDouble;
}

AREXPORT const char *ArConfigArg::getName(void) const
{
  return myName.c_str();
}

AREXPORT const char *ArConfigArg::getDescription(void) const
{
  return myDescription.c_str();
}

AREXPORT int ArConfigArg::getInt(void) const
{
  // only one of these will be valid
  if (myIntPointer != NULL)
    return *myIntPointer;
  else if (myIntShortPointer != NULL)
    return *myIntShortPointer;
  else if (myIntUnsignedShortPointer != NULL)
    return *myIntUnsignedShortPointer;
  else if (myIntUnsignedCharPointer != NULL)
    return *myIntUnsignedCharPointer;
  else
    return 0;
}

AREXPORT double ArConfigArg::getDouble(void) const
{
  if (myDoublePointer != NULL)
    return *myDoublePointer;
  else
    return 0;
}

AREXPORT bool ArConfigArg::getBool(void) const
{
  if (myBoolPointer != NULL)
    return *myBoolPointer;
  else
    return false;
}

AREXPORT const char *ArConfigArg::getString(void) const
{
  if (myUsingOwnedString)
    return myString.c_str();
  else if (myStringPointer != NULL)
    return myStringPointer;
  else
    return NULL;
}

AREXPORT const std::list<ArArgumentBuilder *> *ArConfigArg::getArgsWithFunctor(void) const
{
  if (myGetFunctor == NULL)
    return NULL;
  else
    return myGetFunctor->invokeR();
}

AREXPORT bool ArConfigArg::setInt(int val, char *errorBuffer,
				  size_t errorBufferLen, bool doNotSet)
{
  myValueSet = true;
  if (!myIgnoreBounds && val < myMinInt)
  {
    ArLog::log(ArLog::Normal, "ArConfigArg of %s: setInt value %d below range [%d, %d]", getName(), val, myMinInt, myMaxInt);
    if (errorBuffer != NULL)
      snprintf(errorBuffer, errorBufferLen, "%s value of %d is below minimum of %d.", getName(), val, myMinInt);
    return false;
  }
  if (!myIgnoreBounds && val > myMaxInt)
  {
    ArLog::log(ArLog::Normal, "ArConfigArg of %s: setInt value %d above range [%d, %d]", getName(), val, myMinInt, myMaxInt);
    if (errorBuffer != NULL)
      snprintf(errorBuffer, errorBufferLen, "%s value of %d is above maximum of %d.", getName(), val, myMaxInt);
    return false;
  }
  if ((myIntType == INT_INT && myIntPointer == NULL) ||
      (myIntType == INT_SHORT && myIntShortPointer == NULL) ||
      (myIntType == INT_UNSIGNED_SHORT &&
       myIntUnsignedShortPointer == NULL) ||
      (myIntType == INT_UNSIGNED_CHAR && myIntUnsignedCharPointer == NULL))
  {
    ArLog::log(ArLog::Normal, "ArConfigArg of %s: setInt called with NULL int pointer.", getName());
    if (errorBuffer != NULL)
      snprintf(errorBuffer, errorBufferLen, "%s pointer is NULL.", getName());
    return false;
  }

  if (!doNotSet)
  {
    if (myIntType == INT_INT)
      *myIntPointer = val;
    else if (myIntType == INT_SHORT)
      *myIntShortPointer = val;
    else if (myIntType == INT_UNSIGNED_SHORT)
      *myIntUnsignedShortPointer = val;
    else if (myIntType == INT_UNSIGNED_CHAR)
      *myIntUnsignedCharPointer = val;
    else
    {
      ArLog::log(ArLog::Normal, "ArConfigArg of %s: int is bad type.", getName());
      if (errorBuffer != NULL)
	snprintf(errorBuffer, errorBufferLen, "%s int is bad type (%d).", getName(), myIntType);
      return false;
    }
  }
  return true;
}

AREXPORT bool ArConfigArg::setDouble(double val, char *errorBuffer,
				     size_t errorBufferLen, bool doNotSet)
{
  myValueSet = true;
  if (!myIgnoreBounds && val < myMinDouble)
  {
    ArLog::log(ArLog::Normal, "ArConfigArg of %s: setDouble value %g below range [%g, %g]", getName(), val, myMinDouble, myMaxDouble);
    if (errorBuffer != NULL)
      snprintf(errorBuffer, errorBufferLen, "%s value of %g is below minimum of %g.", getName(), val, myMinDouble);
    return false;
  }
  if (!myIgnoreBounds && val > myMaxDouble)
  {
    ArLog::log(ArLog::Normal, "ArConfigArg of %s: setDouble value %g above range [%g, %g]", getName(), val, myMinDouble, myMaxDouble);
    if (errorBuffer != NULL)
      snprintf(errorBuffer, errorBufferLen, "%s value of %g is above maximum of %g.", getName(), val, myMaxDouble);
    return false;
  }
  if (myDoublePointer == NULL)
  {
    ArLog::log(ArLog::Normal, "ArConfigArg of %s: setDouble called with NULL pointer.", getName());
    if (errorBuffer != NULL)
      snprintf(errorBuffer, errorBufferLen, "%s pointer is NULL.", getName());
    return false;
  }
  // if we got to here we're good
  if (!doNotSet)
    *myDoublePointer = val;
  return true;
}


AREXPORT bool ArConfigArg::setBool(bool val, char *errorBuffer,
				   size_t errorBufferLen, bool doNotSet)
{
  myValueSet = true;
  if (myBoolPointer == NULL)
  {
    ArLog::log(ArLog::Normal, "ArConfigArg of %s: setBool called with NULL pointer.", getName());
    if (errorBuffer != NULL)
      snprintf(errorBuffer, errorBufferLen, "%s pointer is NULL.", getName());
    return false;
  }
  if (!doNotSet)
    *myBoolPointer = val;
  return true;
}

AREXPORT bool ArConfigArg::setString(const char *str, char *errorBuffer,
				     size_t errorBufferLen, bool doNotSet)
{
  myValueSet = true;
  size_t len;
  if (myUsingOwnedString)
  {
    myString = str;
    return true;
  }
  if (myStringPointer == NULL)
  {
    ArLog::log(ArLog::Normal, "ArConfigArg of %s: setString called with NULL pointer.", getName());
    if (errorBuffer != NULL)
      snprintf(errorBuffer, errorBufferLen, "%s pointer is NULL.", getName());
    return false;
  }
  // this is >= so that if it wouldn't have room with NULL that's
  // taken care of too
  if ((len = strlen(str)) >= myMaxStrLen)
  {
    ArLog::log(ArLog::Normal, "ArConfigArg of %s: setString called with argument %d long, when max length is %d.", getName(), len, myMaxStrLen);
    if (errorBuffer != NULL)
      snprintf(errorBuffer, errorBufferLen, "%s string is %d long when max length is %d.", getName(), (int)len, (int)myMaxStrLen);
    return false;
  }
  if (!doNotSet)
    strcpy(myStringPointer, str);
  return true;
}

AREXPORT bool ArConfigArg::setArgWithFunctor(ArArgumentBuilder *argument,
					     char *errorBuffer,
					     size_t errorBufferLen,
					     bool doNotSet)
{
  myValueSet = true;
  bool ret = true;
  if (mySetFunctor == NULL)
  {
    ArLog::log(ArLog::Normal, "ArConfigArg of %s: setArgWithFunctor called with NULL pointer.", getName());
    if (errorBuffer != NULL)
      snprintf(errorBuffer, errorBufferLen, "%s pointer is NULL.", getName());
    return false;
  }
  if (!doNotSet)
    ret = mySetFunctor->invokeR(argument);
  return ret;
}


AREXPORT void ArConfigArg::log(bool verbose) const
{
  std::list<ArArgumentBuilder *>::const_iterator it;
  const std::list<ArArgumentBuilder *> *argList;
  std::string intType;

  switch (getType())
  {
  case ArConfigArg::INVALID:
    ArLog::log(ArLog::Terse,
	       "\tType: %10s.  This argument was not created properly.",
	       "invalid");
  case ArConfigArg::INT:
    if (myIntType == INT_NOT)
      intType = "Not";
    else if (myIntType == INT_INT)
      intType = "Int";
  else if (myIntType == INT_SHORT)
      intType = "Short";
    else if (myIntType == INT_UNSIGNED_SHORT)
      intType = "Unsigned Short";
    else if (myIntType == INT_UNSIGNED_CHAR)
      intType = "Unsigned Short";
    else
      intType = "Unknown";
    ArLog::log(ArLog::Terse, "\tType: %10s name: %12s value: %d intType: %s",
	       "int", getName(), getInt(), intType.c_str());
    if (strlen(getDescription()) != 0)
      ArLog::log(ArLog::Terse, "\t\tDescription: %s",
		 getDescription());
    if (verbose)
      ArLog::log(ArLog::Terse, "\t\tMin: %10d     Max: %10d",
		 myMinInt, myMaxInt);
    break;
  case ArConfigArg::DOUBLE:
    ArLog::log(ArLog::Terse, "\tType: %10s name: %12s value: %f", "double",
	       getName(), getDouble());
    if (strlen(getDescription()) != 0)
      ArLog::log(ArLog::Terse, "\t\tDescription: %s",
		 getDescription());
    if (verbose)
      ArLog::log(ArLog::Terse, "\t\tMin: %10g     Max: %10g",
		 myMinDouble, myMaxDouble);
    break;
  case ArConfigArg::STRING:
    ArLog::log(ArLog::Terse, "\tType: %10s name: %12s value: %s", "string",
               getName(), getString());
    if (strlen(getDescription()) != 0)
      ArLog::log(ArLog::Terse, "\t\tDescription: %s",
                 getDescription());
    if (verbose)
      ArLog::log(ArLog::Terse, "\t\tLength: %d", myMaxStrLen);
    break;
  case ArConfigArg::BOOL:
    ArLog::log(ArLog::Terse, "\tType: %10s name: %12s value: %d", "bool",
	       getName(), getBool());
    if (strlen(getDescription()) != 0)
      ArLog::log(ArLog::Terse, "\t\tDescription: %s",
		 getDescription());
    break;
  case ArConfigArg::FUNCTOR:
    ArLog::log(ArLog::Terse, "\tType: %10s name: %12s",
	       "functor", getName());
    if (strlen(getDescription()) != 0)
      ArLog::log(ArLog::Terse, "\t\tDescription: %s",
		 getDescription());
    ArLog::log(ArLog::Terse, "\t\tValues:");
    argList = myGetFunctor->invokeR();
    for (it = argList->begin(); it != argList->end(); it++)
      ArLog::log(ArLog::Terse, "\t\t\t%s", (*it)->getFullString());
    break;
  case ArConfigArg::DESCRIPTION_HOLDER:
    ArLog::log(ArLog::Terse, "\tType: %20s Description: %s",
	       "description_holder", getDescription());

  default:
    ArLog::log(ArLog::Terse,
	       "\tType: %10s.  This type doesn't have a case in ArConfigArg::print.",
	       "unknown");
    break;
  }

  ArLog::log(ArLog::Terse, "\t\tPriority: %s",
	     ArPriority::getPriorityName(myConfigPriority));
}

/**
   The priority of this argument when used in ArConfig.
 **/
AREXPORT ArPriority::Priority ArConfigArg::getConfigPriority(void) const
{
  return myConfigPriority;
}

/**
   The priority of this argument when used in ArConfig.
 **/

AREXPORT void ArConfigArg::setConfigPriority(ArPriority::Priority priority)
{
  myConfigPriority = priority;
}


AREXPORT const char *ArConfigArg::getDisplayHint() const
{
  if (myDisplayHint.length() > 0) {
    return myDisplayHint.c_str();
  }
  else {
    return NULL;
  }
} // end method getDisplayHint


/**
 * The "display hint" is a text string that may be used by the client
 * to improve the display of the argument.  The following display
 * hints are currently supported by MobileEyes and Mapper3.
 *
 * For type STRING:
 * <ul>
 *
 * <li><code>Choices:</code><i>choice1</i>(<code>;;</code><i>choice#i</i>)*
 *   <ul><li>For example, <code>"Choices:North;;South;;East;;West"</code></li></ul>
 * </li>
 * <li><code>MapItem:</code><i>type</i><code>|</code>(<code>Parent</code>|<code>SubType</code>)<code>|</code>(<code>Optional</code>|<code>Required</code>)
 *   <ul><li>For example, <code>"MapItem:Dock|Parent|Optional"</code></li></ul>
 * </li>
 * <li>  <code>Macro:</code><i>taskClass1</i>(<code>;;</code><i>taskClass#i</i>)*|(<code>Optional</code>|<code>Required</code>)
 *   <ul><li>For example, "Macro:Audio;;Camera;;Video|Optional"</li></ul>
 * </li>
 * <li>  <code>Macro</code>
 *   <ul><li>When used with no qualifiers, there are no task class restrictions and the macro is optional</li></ul>
 * </li>
 *
 * <li><code>RobotFile:</code><i>fileFilterNames</i><code>|</code><i>fileFilters</i>
 *   <ul>
 *      <li>where:
 *        <ul>
 *          <li><i>fileFilterNames</i> is a string that contains the complete displayable
 *                    names for all of the file filters
 *                    (e.g. <code>Map files (*.map);;World files
 *                    (*.wld)</code>), or,
 *                    empty if no filter needs to be applied, and</li>
 *          <li><i>fileFilters</i> is a string that contains just file extensions
 *                    for each of the file filters (e.g. <code>*.map;;*.wld"</code>).
 *                    The number and order of these filters must match
 *                    that of the fileFilterNames parameter.</li>
 *          </ul>
 *        </li>
 *      <li>For example, <code>"RobotFile:Map Files (*.map);;World Files
 *      (*.wld)|*.map;;*.wld"</code></li>
 *   </ul>
 * </li>
 * </ul>
 *
 *
 * For type INT:
 * <ul>
 *  <li><code>Color</code></li>
 * </ul>
**/
AREXPORT void ArConfigArg::setDisplayHint(const char *hintText)
{
  if (hintText != NULL) {
    myDisplayHint = hintText;
  }
  else {
    myDisplayHint = "";
  }
} // end method setDisplayHint



/**
   This is for debugging and will prevent the bounds checking from
   happening, you shouldn't normally use it
 **/
AREXPORT void ArConfigArg::setIgnoreBounds(bool ignoreBounds)
{
  myIgnoreBounds = ignoreBounds;
}

AREXPORT bool ArConfigArg::isValueEqual(const ArConfigArg &other) const
{
  if (strcmp(getName(), other.getName()) != 0) {
    return false;
  }
  Type t = getType();
  if (t != other.getType()) {
    return false;
  }
  bool isEqual = false;

  switch (t) {
  case INVALID:
    isEqual = true; // Seems logical that two invalid args are equal...
    break;

  case INT:
    isEqual = (getInt() == other.getInt());
    break;

  case DOUBLE:
    isEqual = (getDouble() == other.getDouble());
    break;

  case STRING:
    isEqual = (strcmp(getString(), other.getString()) == 0);
    break;

  case BOOL:
    isEqual = (getBool() == other.getBool());
    break;

  case SEPARATOR:
    isEqual = true;

  case FUNCTOR:
    // Hmmm... Equal if they are the same functors??  Not sure how much
    // sense this makes...
    isEqual = ((mySetFunctor == other.mySetFunctor) &&
               (myGetFunctor == other.myGetFunctor));
    break;

  case DESCRIPTION_HOLDER:
    isEqual = (strcmp(getDescription(), other.getDescription()) == 0);
    break;

  default:
    isEqual = false;
    break;

  } // end switch type

  return isEqual;

} // end method isValueEqual

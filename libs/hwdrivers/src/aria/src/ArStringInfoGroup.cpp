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
#include "ArStringInfoGroup.h"

/* 
   TODO the stringInt functions and such'll leak memory on a failed attempt 
*/

AREXPORT ArStringInfoGroup::ArStringInfoGroup()
{

}

AREXPORT ArStringInfoGroup::~ArStringInfoGroup()
{

}

AREXPORT bool ArStringInfoGroup::addString(
	const char *name, ArTypes::UByte2 maxLength,
	ArFunctor2<char *, ArTypes::UByte2> *functor)
{
  myDataMutex.lock();
  if (myAddedStrings.find(name) != myAddedStrings.end())
  {
    ArLog::log(ArLog::Normal, "ArStringInfoGroups: Cannot add info '%s', duplicate", name);
    myDataMutex.unlock();
    return false;
  }

  std::list<ArFunctor3<const char *, ArTypes::UByte2,
  ArFunctor2<char *, ArTypes::UByte2> *> *>::iterator it;
  ArLog::log(ArLog::Verbose, "ArStringInfoGroups: Adding info '%s'", name);
  myAddedStrings.insert(name);
  for (it = myAddStringCBList.begin(); it != myAddStringCBList.end(); it++)
  {
    (*it)->invoke(name, maxLength, functor);
  }
  ArLog::log(ArLog::Verbose, "ArStringInfoGroups: Added info '%s'", name);
  myDataMutex.unlock();
  return true;
}

AREXPORT bool ArStringInfoGroup::addStringInt(
	const char *name, ArTypes::UByte2 maxLength,
	ArRetFunctor<int> *functor, const char *format)
{
  return addString(name, maxLength, 
	    (new ArGlobalFunctor4<char *, ArTypes::UByte2, 
	     ArRetFunctor<int> *, 
	     const char *>(&ArStringInfoHolderFunctions::intWrapper, 
			   (char *)NULL, (ArTypes::UByte2) 0, 
			   functor, format)));
}

AREXPORT bool ArStringInfoGroup::addStringDouble(
	const char *name, ArTypes::UByte2 maxLength,
	ArRetFunctor<double> *functor, const char *format)
{
  return addString(name, maxLength, 
	    (new ArGlobalFunctor4<char *, ArTypes::UByte2, 
	     ArRetFunctor<double> *, 
	     const char *>(&ArStringInfoHolderFunctions::doubleWrapper, 
			   (char *)NULL, (ArTypes::UByte2) 0, 
			   functor, format)));
}


AREXPORT bool ArStringInfoGroup::addStringBool(
	const char *name, ArTypes::UByte2 maxLength,
	ArRetFunctor<bool> *functor, const char *format)
{
  return addString(name, maxLength, 
	    (new ArGlobalFunctor4<char *, ArTypes::UByte2, 
	     ArRetFunctor<bool> *, 
	     const char *>(&ArStringInfoHolderFunctions::boolWrapper, 
			   (char *)NULL, (ArTypes::UByte2) 0, 
			   functor, format)));
}

AREXPORT bool ArStringInfoGroup::addStringString(
	const char *name, ArTypes::UByte2 maxLength,
	ArRetFunctor<const char *> *functor, const char *format)
{
  return addString(name, maxLength, 
	    (new ArGlobalFunctor4<char *, ArTypes::UByte2, 
	     ArRetFunctor<const char *> *, 
	     const char *>(&ArStringInfoHolderFunctions::stringWrapper, 
			   (char *)NULL, (ArTypes::UByte2) 0, 
			   functor, format)));
}


AREXPORT void ArStringInfoGroup::addAddStringCallback(
	ArFunctor3<const char *, ArTypes::UByte2,
	ArFunctor2<char *, ArTypes::UByte2> *> *functor,
	ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myAddStringCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myAddStringCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse, 
	       "ArStringInfoGroup::addAddStringCallback: Invalid position.");
}

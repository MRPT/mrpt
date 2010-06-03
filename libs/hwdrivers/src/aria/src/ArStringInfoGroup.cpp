/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 19 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/

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

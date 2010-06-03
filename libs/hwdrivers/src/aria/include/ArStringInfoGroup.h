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

#ifndef ARSTRINGINFOGROUP_H
#define ARSTRINGINFOGROUP_H

#include "ariaUtil.h"
#include "ArMutex.h"
#include <string>
#include <set>
#include <list>

/**
   This class takes callbacks from different classes that want this
   string information and then lets you just add the information here
   instead of to each individual class.
 **/
class ArStringInfoGroup
{
public:
  /// Constructor
  AREXPORT ArStringInfoGroup();
  /// Destructor
  AREXPORT virtual ~ArStringInfoGroup();
  /// Adds a string to the list in the raw format
  AREXPORT bool addString(const char *name, ArTypes::UByte2 maxLen, 
			  ArFunctor2<char *, ArTypes::UByte2> *functor);

  /// Adds an int to the list in the helped way
  AREXPORT bool addStringInt(const char *name, ArTypes::UByte2 maxLen, 
			     ArRetFunctor<int> *functor, 
			     const char *format = "%d");

  /// Adds a double to the list in the helped way
  AREXPORT bool addStringDouble(const char *name, ArTypes::UByte2 maxLen, 
				ArRetFunctor<double> *functor, 
				const char *format = "%g");

  /// Adds a bool to the list in the helped way
  AREXPORT bool addStringBool(const char *name, ArTypes::UByte2 maxLen, 
			      ArRetFunctor<bool> *functor,
			      const char *format = "%s");

  /// Adds a string to the list in the helped way
  AREXPORT bool addStringString(const char *name, ArTypes::UByte2 maxLen, 
			      ArRetFunctor<const char *> *functor,
			      const char *format = "%s");
  /// This is the function to add a callback to be called by addString
  AREXPORT void addAddStringCallback(
	  ArFunctor3<const char *, ArTypes::UByte2,
	  ArFunctor2<char *, ArTypes::UByte2> *> *functor,
	  ArListPos::Pos position = ArListPos::LAST);

protected:
  ArMutex myDataMutex;
  std::set<std::string, ArStrCaseCmpOp> myAddedStrings;
  std::list<ArFunctor3<const char *, ArTypes::UByte2,
	      ArFunctor2<char *, ArTypes::UByte2> *> *> myAddStringCBList;
};


#endif // ARSTRINGINFOHELPER_H

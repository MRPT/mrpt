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


#ifndef ARIA_wrapper_ExtraClasses_h
#define ARIA_wrapper_ExtraClasses_h

/** @cond INCLUDE_SWIG_ONLY_CLASSES */

/* ArConfigArg subclasses for specific types, since for some target languages
 * (Python) Swig can't differentiate booleans, integers, short integers,
 * unsigned integers, etc.  Furthermore, ArConfig can't change program variables
 * via pointers in most languages, so you need to only use the constructors that take
 * an initial value for an internally held variable instead of pointers anyway.
 */

class ArConfigArg_Bool : public ArConfigArg
{
public:
  ArConfigArg_Bool(const char *name, bool b, const char *desc = "") :
    ArConfigArg(name, b, desc)
  { }
};

class ArConfigArg_Int : public ArConfigArg
{
public:
  ArConfigArg_Int(const char *name, int i, const char *desc = "", int min = INT_MIN, int max = INT_MAX) :
    ArConfigArg(name, i, desc, min, max)
  { }
};

class ArConfigArg_Double : public ArConfigArg
{
public:
  ArConfigArg_Double(const char *name, double d, const char *desc = "", double min = -HUGE_VAL, double max = HUGE_VAL) :
    ArConfigArg(name, d, desc, min, max)
  { }
};

class ArConfigArg_String : public ArConfigArg
{
public:
  ArConfigArg_String(const char *name, char *str, const char *desc) :
    ArConfigArg(name, str, desc, 0)
  { 
  }
};

/** @endcond INCLUDE_SWIG_ONLY_CLASSES */

#endif // wrapperExtraClasses.h

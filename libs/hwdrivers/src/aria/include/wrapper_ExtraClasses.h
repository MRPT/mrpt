/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


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

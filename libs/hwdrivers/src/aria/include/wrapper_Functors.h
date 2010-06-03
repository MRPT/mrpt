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


#ifndef ARIA_wrapper_Functors_h
#define ARIA_wrapper_Functors_h

/* Define ArFunctor subclasses to hold target-language callable function
 * object, and define typemaps to give instances of these subclasses to Aria
 * when it wants ArFunctors.
 *
 * These are used internally by the wrapper library, and typemaps convert
 * target-language function objects to these ArFunctor subclasses.
 */

#include "ArFunctor.h"

/* Functiors for Python: */

#ifdef SWIGPYTHON
class ArPyFunctor : public ArFunctor 
{
protected:
  PyObject* pyFunction;
public:
  ArPyFunctor(PyObject* _m) : pyFunction(_m) {
    Py_INCREF(pyFunction);
  }

  virtual void invoke() { 
    PyObject* r = PyObject_CallObject(pyFunction, NULL);
    if(!r) {
      fputs("** ArPyFunctor: Error calling Python function: ", stderr);
      PyErr_Print();
    }
  }

  virtual ~ArPyFunctor() {
    Py_DECREF(pyFunction);
  }

  virtual const char* getName() {
    return (const char*) PyString_AsString(PyObject_Str(pyFunction));
  }
};


class ArPyRetFunctor_Bool : 
  public ArRetFunctor<bool>,
  public ArPyFunctor
{
public:
  ArPyRetFunctor_Bool(PyObject* _m) : ArRetFunctor<bool>(), ArPyFunctor(_m) {
  }

  virtual bool invokeR() {
    PyObject* r = PyObject_CallObject(pyFunction, NULL);  
    if(!r) {
      fputs("** ArPyRetFunctor_Bool: Error calling Python function: ", stderr);
      PyErr_Print();
    }
    return(r == Py_True);
  }

  virtual const char* getName() {
    return (const char*) PyString_AsString(PyObject_Str(pyFunction));
  }
};

#endif // PYTHON



class ArRetFunctor_Bool:
  public ArRetFunctor<bool>
{
};

#endif // wrapperFunctors.h

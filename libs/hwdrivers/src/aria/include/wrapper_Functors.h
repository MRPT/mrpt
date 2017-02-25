/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


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

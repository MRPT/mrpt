/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

// pybind11
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT headers
#include <mrpt/rtti/CObject.h>

namespace py = pybind11;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_rtti";

  // pybind11 version used to build all MRPT bindings, to detect an
  // incompatible NumPy at import time (see mrpt/rtti/__init__.py).
  m.attr("_pybind11_version") =
      py::make_tuple(PYBIND11_VERSION_MAJOR, PYBIND11_VERSION_MINOR, PYBIND11_VERSION_PATCH);

  // Registered before TRuntimeClassId, whose createObject() returns it:
  py::class_<mrpt::rtti::CObject, std::shared_ptr<mrpt::rtti::CObject>> cObject(
      m, "CObject", "Virtual base to provide a compiler-independent RTTI system.");

  // Bind the TRuntimeClassId struct
  py::class_<mrpt::rtti::TRuntimeClassId>(
      m, "TRuntimeClassId",
      "Runtime type information of an MRPT class: name, base class and factory.")
      .def_readonly("className", &mrpt::rtti::TRuntimeClassId::className, "Name of the class")
      .def(
          "createObject", &mrpt::rtti::TRuntimeClassId::createObject,
          "Creates a new object of this class (default constructed), or returns None for virtual "
          "classes.")
      .def(
          "getBaseClass",
          [](const mrpt::rtti::TRuntimeClassId& self)
          { return self.getBaseClass ? self.getBaseClass() : nullptr; },
          py::return_value_policy::reference, "Gets the base class runtime id.")
      .def(
          "derivedFrom",
          static_cast<bool (mrpt::rtti::TRuntimeClassId::*)(const mrpt::rtti::TRuntimeClassId*)
                          const>(&mrpt::rtti::TRuntimeClassId::derivedFrom),
          "Returns true if this class derives from the given one.")
      .def(
          "derivedFrom",
          static_cast<bool (mrpt::rtti::TRuntimeClassId::*)(const char*) const>(
              &mrpt::rtti::TRuntimeClassId::derivedFrom),
          "Returns true if this class derives from the class with the given name.");

  // Bind the CObject class
  cObject.def(
      "GetRuntimeClass", &mrpt::rtti::CObject::GetRuntimeClass, py::return_value_policy::reference,
      "Returns information about the class of an object in runtime.");

  // Bind global functions for class registration and lookup
  m.def(
      "registerClass", &mrpt::rtti::registerClass,
      "Register a class into the MRPT internal list of \"CObject\" descendents.");
  m.def(
      "registerClassCustomName", &mrpt::rtti::registerClassCustomName,
      "Registers a class under an additional name (for backwards-compatible deserialization).");
  m.def(
      "getAllRegisteredClasses", &mrpt::rtti::getAllRegisteredClasses,
      py::return_value_policy::reference, "Returns all the registered classes.");
  m.def(
      "getAllRegisteredClassesChildrenOf", &mrpt::rtti::getAllRegisteredClassesChildrenOf,
      py::return_value_policy::reference,
      "Like getAllRegisteredClasses(), but filters the list to only include children clases of a "
      "given base one.");
  m.def(
      "findRegisteredClass", &mrpt::rtti::findRegisteredClass, py::return_value_policy::reference,
      py::arg("className"), py::arg("allow_ignore_namespace") = true,
      "Returns the class with the given name, or None if not registered.");
  m.def(
      "registerAllPendingClasses", &mrpt::rtti::registerAllPendingClasses,
      "Registers all pending classes (normally done automatically).");
  m.def(
      "classFactory", &mrpt::rtti::classFactory, "Creates an object given by its registered name.");
}
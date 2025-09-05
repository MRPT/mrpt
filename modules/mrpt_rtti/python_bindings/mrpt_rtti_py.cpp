/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
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

  // Bind the TRuntimeClassId struct
  py::class_<mrpt::rtti::TRuntimeClassId>(m, "TRuntimeClassId")
      .def_readonly("className", &mrpt::rtti::TRuntimeClassId::className)
      .def("createObject", &mrpt::rtti::TRuntimeClassId::createObject)
      .def(
          "getBaseClass",
          [](const mrpt::rtti::TRuntimeClassId& self)
          { return self.getBaseClass ? self.getBaseClass() : nullptr; },
          py::return_value_policy::reference)
      .def(
          "derivedFrom",
          static_cast<bool (mrpt::rtti::TRuntimeClassId::*)(const mrpt::rtti::TRuntimeClassId*)
                          const>(&mrpt::rtti::TRuntimeClassId::derivedFrom))
      .def(
          "derivedFrom", static_cast<bool (mrpt::rtti::TRuntimeClassId::*)(const char*) const>(
                             &mrpt::rtti::TRuntimeClassId::derivedFrom));

  // Bind the CObject class
  py::class_<mrpt::rtti::CObject, std::shared_ptr<mrpt::rtti::CObject>>(m, "CObject")
      .def(
          "GetRuntimeClass", &mrpt::rtti::CObject::GetRuntimeClass,
          py::return_value_policy::reference);

  // Bind global functions for class registration and lookup
  m.def("registerClass", &mrpt::rtti::registerClass);
  m.def("registerClassCustomName", &mrpt::rtti::registerClassCustomName);
  m.def(
      "getAllRegisteredClasses", &mrpt::rtti::getAllRegisteredClasses,
      py::return_value_policy::reference);
  m.def(
      "getAllRegisteredClassesChildrenOf", &mrpt::rtti::getAllRegisteredClassesChildrenOf,
      py::return_value_policy::reference);
  m.def(
      "findRegisteredClass", &mrpt::rtti::findRegisteredClass, py::return_value_policy::reference);
  m.def("registerAllPendingClasses", &mrpt::rtti::registerAllPendingClasses);
  m.def("classFactory", &mrpt::rtti::classFactory);
}
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

#include <mrpt/rtti/CObject.h>

#include <cstdio>

#include "internal_class_registry.h"

using namespace mrpt;
using namespace mrpt::rtti;

const mrpt::rtti::TRuntimeClassId& CObject::GetRuntimeClassIdStatic()
{
  return CObject::runtimeClassId;
}

bool TRuntimeClassId::derivedFrom(const TRuntimeClassId* pBaseClass) const
{
  ASSERT_(pBaseClass != nullptr);

  // The same class??
  if (pBaseClass == this) return true;

  // Automatically register all pending classes, just in case:
  registerAllPendingClasses();

  // Check heritage:
  const TRuntimeClassId* pClassThis = this;
  while (pClassThis != nullptr)
  {
    if (pClassThis == pBaseClass) return true;

    if (pClassThis->getBaseClass)
      pClassThis = (*pClassThis->getBaseClass)();
    else
      pClassThis = nullptr;  // The root class
  }

  // Parent class not found
  return false;
}

bool TRuntimeClassId::derivedFrom(const char* pBaseClass_name) const
{
  // Automatically register all pending classes, just in case:
  registerAllPendingClasses();

  const TRuntimeClassId* pBaseClass = findRegisteredClass(pBaseClass_name);
  ASSERTMSG_(pBaseClass != nullptr, format("Class %s not registered??", pBaseClass_name));

  // The same class??
  if (pBaseClass == this) return true;

  // Check heritage:
  const TRuntimeClassId* pClassThis = this;
  while (pClassThis != nullptr)
  {
    if (pClassThis == pBaseClass) return true;

    if (pClassThis->getBaseClass)
      pClassThis = (*pClassThis->getBaseClass)();
    else
      pClassThis = nullptr;  // The root class
  }

  // Parent class not found
  return false;
}

CObject::Ptr TRuntimeClassId::createObject() const
{
  if (!ptrCreateObject)
  {
    fprintf(
        stderr,
        "[TRuntimeClassId::createObject] Trying to create an object "
        "without dynamic constructor. classname=`%s`\n",
        className != nullptr ? className : "nullptr");
    return CObject::Ptr();
  }

  try
  {
    return this->ptrCreateObject();
  }
  catch (std::bad_alloc&)
  {
    throw;
  }
}

// For class CObject, special methods must be defined
// since it has no base class. These methods are defined
// automatically for derived classes.
TRuntimeClassId* CObject::_GetBaseClass() { return nullptr; }

mrpt::rtti::CObject::Ptr mrpt::rtti::classFactory(const std::string& className)
{
  auto pR = findRegisteredClass(className);
  if (!pR) return nullptr;
  return pR->createObject();
}

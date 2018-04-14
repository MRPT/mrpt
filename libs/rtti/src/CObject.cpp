/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "rtti-precomp.h"  // Precompiled headers

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
	ASSERTMSG_(
		pBaseClass != nullptr,
		format("Class %s not registered??", pBaseClass_name));

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

CObject* TRuntimeClassId::createObject() const
{
	if (!ptrCreateObject)
	{
		fprintf(
			stderr,
			"[TRuntimeClassId::createObject] Trying to create an object "
			"without dynamic constructor. classname=`%s`\n",
			className != nullptr ? className : "nullptr");
		return nullptr;
	}

	try
	{
		CObject* ret = (*ptrCreateObject)();
		return ret;
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
const struct TRuntimeClassId CObject::runtimeClassId = {"CObject", nullptr,
														nullptr};

mrpt::rtti::CObject* mrpt::rtti::classFactory(const std::string& className)
{
	auto pR = findRegisteredClass(className);
	if (!pR) return nullptr;
	return pR->createObject();
}

mrpt::rtti::CObject::Ptr mrpt::rtti::classFactoryPtr(
	const std::string& className)
{
	mrpt::rtti::CObject::Ptr ret;
	auto pR = findRegisteredClass(className);
	if (pR) ret.reset(pR->createObject());
	return ret;
}

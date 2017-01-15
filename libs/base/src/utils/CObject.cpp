/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/CObject.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/system/os.h>

#include "internal_class_registry.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;

#include <cstdio>

/*---------------------------------------------------------------
					TRuntimeClassId::derivedFrom
 ---------------------------------------------------------------*/
bool TRuntimeClassId::derivedFrom(const TRuntimeClassId* pBaseClass) const
{
	ASSERT_(this != NULL)
	ASSERT_(pBaseClass != NULL)

	// The same class??
	if (pBaseClass==this)
		return true;

	// Automatically register all pending classes, just in case:
	registerAllPendingClasses();

	// Check heritage:
	const TRuntimeClassId* pClassThis = this;
	while (pClassThis != NULL)
	{
		if (pClassThis == pBaseClass)	return true;

		if (pClassThis->getBaseClass)	pClassThis = (*pClassThis->getBaseClass)();
		else	pClassThis = NULL; // The root class
	}

	// Parent class not found
	return false;
}

/*---------------------------------------------------------------
					TRuntimeClassId::derivedFrom
 ---------------------------------------------------------------*/
bool TRuntimeClassId::derivedFrom(const char* pBaseClass_name) const
{
	ASSERT_(this != NULL)

	// Automatically register all pending classes, just in case:
	registerAllPendingClasses();

	const TRuntimeClassId* pBaseClass = findRegisteredClass(pBaseClass_name);
	ASSERTMSG_(pBaseClass != NULL, format("Class %s not registered??",pBaseClass_name) )

	// The same class??
	if (pBaseClass==this)
		return true;

	// Check heritage:
	const TRuntimeClassId* pClassThis = this;
	while (pClassThis != NULL)
	{
		if (pClassThis == pBaseClass)	return true;

		if (pClassThis->getBaseClass)	pClassThis = (*pClassThis->getBaseClass)();
		else	pClassThis = NULL; // The root class
	}

	// Parent class not found
	return false;
}


/*---------------------------------------------------------------
					TRuntimeClassId::createObject
 ---------------------------------------------------------------*/
CObject* TRuntimeClassId::createObject() const
{
	if (! ptrCreateObject )
	{
		perror("[TRuntimeClassId::createObject] Trying to create an object with not dynamic constructor\n");
		return NULL;
	}

	try
	{
		CObject* ret = (*ptrCreateObject)();
		return ret;
	}
	catch (std::bad_alloc &)
	{
		throw;
	}
}


/* -----------------------------------------------------------------------
		For class CObject, special methods must be defined
		 since it has no base class. These methods are defined
		 automatically for derived classes.
   ----------------------------------------------------------------------- */
TRuntimeClassId* CObject::_GetBaseClass()
{
	return NULL;
}

const struct TRuntimeClassId CObject::classCObject =
{
	"CObject",
	NULL,
	NULL
};

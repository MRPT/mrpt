/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/utils/CObject.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/utils/CStartUpClassesRegister.h>
#include <mrpt/system/os.h>

#include "internal_class_registry.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;

#include <iostream>

extern CStartUpClassesRegister  mrpt_base_class_reg;
const int dumm = mrpt_base_class_reg.do_nothing(); // Avoid compiler removing this class in static linking

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
		std::cerr << "[TRuntimeClassId::createObject] Trying to create an object with not dynamic constructor" << std::endl;
		return NULL;
	}

	try
	{
		CObject* ret = (*ptrCreateObject)();
		return ret;
	}
	catch (std::bad_alloc &e)
	{
		throw e;
	}
	catch (...)
	{
		return NULL;
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

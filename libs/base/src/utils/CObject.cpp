/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

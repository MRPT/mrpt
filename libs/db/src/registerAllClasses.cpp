/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "db-precomp.h"

#include <mrpt/db/CSimpleDatabase.h>
using namespace mrpt::db;

MRPT_INITIALIZER(registerAllClasses_mrpt_db)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass(CLASS_ID(CSimpleDatabase));
	registerClass(CLASS_ID(CSimpleDatabaseTable));
//	registerClass(CLASS_ID(CPropertiesValuesList));
//	registerClass(CLASS_ID(CMHPropertiesValuesList));
//	registerClass(CLASS_ID(CTypeSelector));
//	registerClass(CLASS_ID(CMemoryChunk));
#endif
}

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "db-precomp.h"

#include <mrpt/core/initializer.h>
#include <mrpt/db/CSimpleDatabase.h>
using namespace mrpt::db;

MRPT_INITIALIZER(registerAllClasses_mrpt_db)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass(CLASS_ID(CSimpleDatabase));
	registerClass(CLASS_ID(CSimpleDatabaseTable));
#endif
}

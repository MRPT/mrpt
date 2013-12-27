/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_core_unittests_H
#define  mrpt_core_unittests_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace utils
	{
		/** Run all the unit tests in mrpt::core classes. */
		int BASE_IMPEXP run_unittests(int argc, char **argv);
	} // End of namespace
} // End of namespace
#endif

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/core_defs.h>
#include <mrpt/tfest/link_pragmas.h>
//#include <functional>

namespace mrpt
{
	namespace tfest
	{
		/** \addtogroup mrpt_tfest_grp
		  * @{ */

		/** For each individual-compatibility (IC) test, the indices of the candidate match between elements in both reference frames. 
			* \sa TSE3RobustParams::user_individual_compat_callback , TSE2RobustParams::user_individual_compat_callback
			*/
		struct TFEST_IMPEXP TPotentialMatch {
			size_t idx_this, idx_other;
		};

		typedef bool (*TFunctorCheckPotentialMatch)(const TPotentialMatch &pm, void *user_data);

		/** @} */  // end of grouping
	}
} // End of namespace

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstddef>
#include <functional>

namespace mrpt::tfest
{
/** \addtogroup mrpt_tfest_grp
 * @{ */

/** For each individual-compatibility (IC) test, the indices of the candidate
 * match between elements in both reference frames.
 * \sa TSE3RobustParams::user_individual_compat_callback ,
 * TSE2RobustParams::user_individual_compat_callback
 */
struct TPotentialMatch
{
	size_t idx_this, idx_other;
};

using TFunctorCheckPotentialMatch = std::function<bool(const TPotentialMatch&)>;

/** @} */  // end of grouping
}  // namespace mrpt::tfest

/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
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
  size_t idx_this = 0;
  size_t idx_other = 0;
};

using TFunctorCheckPotentialMatch = std::function<bool(const TPotentialMatch&)>;

/** @} */  // end of grouping
}  // namespace mrpt::tfest

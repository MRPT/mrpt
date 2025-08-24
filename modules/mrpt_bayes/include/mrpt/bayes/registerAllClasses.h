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

#include <mrpt/config/registerAllClasses.h>
#include <mrpt/math/registerAllClasses.h>

namespace mrpt::bayes
{
/** Forces manual RTTI registration of all serializable classes in this
 * namespace. Should never be required to be explicitly called by users, except
 * if building MRPT as a static library.
 *
 * \ingroup mrpt_bayes_grp
 */
inline void registerAllClasses_mrpt_bayes()
{
  // None in this library
  mrpt::config::registerAllClasses_mrpt_config();
  mrpt::math::registerAllClasses_mrpt_math();
}

}  // namespace mrpt::bayes

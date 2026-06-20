/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <stdexcept>

/** MRPT_LIKELY(): disable the branch prediction and manually setting the
 * preference
 * for the positive case.
 * See:
 * https://akrzemi1.wordpress.com/2017/05/18/asserts-in-constexpr-functions */
#if defined __GNUC__
#define MRPT_LIKELY(EXPR) __builtin_expect(!!(EXPR), 1)
#else
#define MRPT_LIKELY(EXPR) (!!(EXPR))
#endif

/** MRPT_X_ASSERT(): build error if condition is known not to work at compile
 * time, throw an exception at runtime if the condition needs to be evaluated.
 */
#define MRPT_X_ASSERT(CHECK) \
  (MRPT_LIKELY(CHECK) ? void(0) : [] { throw std::runtime_error(#CHECK); }())

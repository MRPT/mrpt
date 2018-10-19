/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
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

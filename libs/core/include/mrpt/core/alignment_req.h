/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

// This is to match Eigen expectations on alignment of dynamic objects:
#if defined(__AVX2__) || defined(EIGEN_VECTORIZE_AVX512)
#define MRPT_MAX_ALIGN_BYTES 64
#elif defined(__AVX__)
#define MRPT_MAX_ALIGN_BYTES 32
#else
#define MRPT_MAX_ALIGN_BYTES 16
#endif

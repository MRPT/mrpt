/* +------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)            |
|                          https://www.mrpt.org/                         |
|                                                                        |
| Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
| See: https://www.mrpt.org/Authors - All rights reserved.               |
| Released under BSD License. See: https://www.mrpt.org/License          |
+------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/CVectorFixed.h>
#include "CMatrixFixed_impl.h"

#define DO_VECFIXED_INSTANTIATION_NM(T_, N_) \
	template class mrpt::math::CMatrixFixed<T_, N_, 1>;

#define DO_VECFIXED_INSTANTIATION(T_)   \
	DO_VECFIXED_INSTANTIATION_NM(T_, 2) \
	DO_VECFIXED_INSTANTIATION_NM(T_, 3) \
	DO_VECFIXED_INSTANTIATION_NM(T_, 4) \
	DO_VECFIXED_INSTANTIATION_NM(T_, 5) \
	DO_VECFIXED_INSTANTIATION_NM(T_, 6) \
	DO_VECFIXED_INSTANTIATION_NM(T_, 7)

DO_VECFIXED_INSTANTIATION(float);
DO_VECFIXED_INSTANTIATION(double);

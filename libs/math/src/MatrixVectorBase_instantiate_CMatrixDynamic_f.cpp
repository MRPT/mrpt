/* +------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)            |
|                          https://www.mrpt.org/                         |
|                                                                        |
| Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
| See: https://www.mrpt.org/Authors - All rights reserved.               |
| Released under BSD License. See: https://www.mrpt.org/License          |
+------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CVectorDynamic.h>
#include <Eigen/Dense>
#include "MatrixVectorBase_impl.h"

// Template instantiation:
#define DO_MATDYN_INSTANTIATION(T_)              \
	template class mrpt::math::MatrixVectorBase< \
		T_, mrpt::math::CMatrixDynamic<T_>>;

DO_MATDYN_INSTANTIATION(float)

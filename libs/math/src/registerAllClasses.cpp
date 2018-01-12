/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"

#include <mrpt/core/initializer.h>

#include <mrpt/math/CMatrix.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixB.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CSplineInterpolator1D.h>

using namespace mrpt::math;

MRPT_INITIALIZER(registerAllClasses_mrpt_math)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	// Abstract classes are not registered since they can not be
	//   instanciated, nor loaded from streams.
	registerClass(CLASS_ID(CMatrix));
	registerClass(CLASS_ID(CMatrixD));
	registerClass(CLASS_ID(CMatrixB));
	registerClass(CLASS_ID(CPolygon));
	registerClass(CLASS_ID(CSplineInterpolator1D));
#endif
}

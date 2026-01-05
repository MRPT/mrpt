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

#include <mrpt/core/initializer.h>
#include <mrpt/math/CMatrixB.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CSplineInterpolator1D.h>
#include <mrpt/math/registerAllClasses.h>
// deps:
#include <mrpt/serialization/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_math)
{
  using namespace mrpt::math;

#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
  // Abstract classes are not registered since they can not be
  //   instanciated, nor loaded from streams.
  registerClass(CLASS_ID(CMatrixF));
  // To support deserialization from rawlogs < mrpt 2.0.0
  registerClassCustomName("CMatrix", CLASS_ID(CMatrixF));

  registerClass(CLASS_ID(CMatrixD));
  registerClass(CLASS_ID(CMatrixB));
  registerClass(CLASS_ID(CPolygon));
  registerClass(CLASS_ID(CSplineInterpolator1D));
#endif
}

void mrpt::math::registerAllClasses_mrpt_math()
{
  ::registerAllClasses_mrpt_math();
  // deps:
  mrpt::serialization::registerAllClasses_mrpt_serialization();
}

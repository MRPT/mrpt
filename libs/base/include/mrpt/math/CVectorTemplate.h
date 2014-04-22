/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CVectorTemplate_H
#define CVectorTemplate_H

#include <mrpt/utils/types.h>

namespace mrpt
{
	namespace math
	{

		typedef mrpt::math::CVectorFloat  CVectorFloat;   //!< This is just another name for mrpt::math::CVectorFloat  (Backward compatibility with MRPT <=0.9.2); for a complete introduction to Matrices and vectors in MRPT, see: http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes

		typedef mrpt::math::CVectorDouble CVectorDouble;  //!< This is just another name for mrpt::math::CVectorDouble (Backward compatibility with MRPT <=0.9.2); for a complete introduction to Matrices and vectors in MRPT, see: http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes

	} // end namespace
} // end namespace

#endif

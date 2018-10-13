/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/math/CMatrixTemplate.h>

namespace mrpt::math
{
/**  This class is a "CSerializable" wrapper for "CMatrixBool".
 * \note For a complete introduction to Matrices and vectors in MRPT, see:
 * http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
 * \ingroup mrpt_math_grp
 */
class CMatrixB : public mrpt::serialization::CSerializable, public CMatrixBool
{
	DEFINE_SERIALIZABLE(CMatrixB)
   public:
	/** Constructor */
	CMatrixB(size_t row = 1, size_t col = 1) : CMatrixBool(row, col) {}
	/** Copy constructor */
	CMatrixB(const CMatrixBool& m) : CMatrixBool(m) {}
	/** Assignment operator for float matrixes */
	CMatrixB& operator=(const CMatrixBool& m)
	{
		CMatrixBool::operator=(m);
		return *this;
	}
};  // end of class definition

}  // namespace mrpt::math

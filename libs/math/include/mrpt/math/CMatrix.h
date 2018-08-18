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
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

namespace mrpt::math
{
/**  This class is a "CSerializable" wrapper for "CMatrixFloat".
 * \note For a complete introduction to Matrices and vectors in MRPT, see:
 * http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
 * \ingroup mrpt_math_grp
 */
class CMatrix : public mrpt::serialization::CSerializable, public CMatrixFloat
{
	DEFINE_SERIALIZABLE(CMatrix)
	DEFINE_SCHEMA_SERIALIZABLE()

   public:
	/** Constructor  */
	CMatrix() : CMatrixFloat(1, 1) {}
	/** Constructor  */
	CMatrix(size_t row, size_t col) : CMatrixFloat(row, col) {}
	/** Copy constructor
	 */
	CMatrix(const CMatrixFloat& m) : CMatrixFloat(m) {}
	/** Copy constructor
	 */
	CMatrix(const CMatrixTemplateNumeric<double>& m) : CMatrixFloat(0, 0)
	{
		*this = m.eval().cast<float>();
	}
	MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CMatrix)

	/** Assignment operator for float matrixes
	 */
	template <class OTHERMAT>
	inline CMatrix& operator=(const OTHERMAT& m)
	{
		CMatrixFloat::operator=(m);
		return *this;
	}

	/*! Assignment operator from any other Eigen class */
	template <typename OtherDerived>
	inline CMatrix& operator=(const Eigen::MatrixBase<OtherDerived>& other)
	{
		CMatrixTemplateNumeric<float>::operator=(other);
		return *this;
	}
	/*! Constructor from any other Eigen class */
	template <typename OtherDerived>
	inline CMatrix(const Eigen::MatrixBase<OtherDerived>& other)
		: CMatrixTemplateNumeric<float>(other)
	{
	}

};  // end of class definition
mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, CMatrix::Ptr& pObj);

}  // namespace mrpt::math

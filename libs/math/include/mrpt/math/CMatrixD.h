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

namespace mrpt::math
{
/**  This class is a "CSerializable" wrapper for
 * "CMatrixTemplateNumeric<double>".
 * \note For a complete introduction to Matrices and vectors in MRPT, see:
 * http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
 * \ingroup mrpt_math_grp
 */
class CMatrixD : public mrpt::serialization::CSerializable,
				 public CMatrixTemplateNumeric<double>
{
	DEFINE_SERIALIZABLE(CMatrixD)
	DEFINE_SCHEMA_SERIALIZABLE()
   public:
	/** Constructor */
	CMatrixD() : CMatrixTemplateNumeric<double>(1, 1) {}
	/** Constructor */
	CMatrixD(size_t row, size_t col) : CMatrixTemplateNumeric<double>(row, col)
	{
	}

	/** Copy constructor */
	CMatrixD(const CMatrixTemplateNumeric<double>& m)
		: CMatrixTemplateNumeric<double>(m)
	{
	}

	/** Copy constructor  */
	CMatrixD(const CMatrixFloat& m) : CMatrixTemplateNumeric<double>(0, 0)
	{
		*this = m.eval().cast<double>();
	}

	/*! Assignment operator from any other Eigen class */
	template <typename OtherDerived>
	inline CMatrixD& operator=(const Eigen::MatrixBase<OtherDerived>& other)
	{
		CMatrixTemplateNumeric<double>::operator=(other);
		return *this;
	}
	/*! Constructor from any other Eigen class */
	template <typename OtherDerived>
	inline CMatrixD(const Eigen::MatrixBase<OtherDerived>& other)
		: CMatrixTemplateNumeric<double>(other)
	{
	}

	MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CMatrixD)

};  // end of class definition
mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, CMatrixD::Ptr& pObj);

}  // namespace mrpt::math
